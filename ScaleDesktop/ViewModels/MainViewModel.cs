using System.Collections.ObjectModel;
using System.Globalization;
using Avalonia.Threading;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using ScaleDesktop.Models;
using ScaleDesktop.Services;

namespace ScaleDesktop.ViewModels;

public partial class MainViewModel : ViewModelBase, IDisposable
{
    private readonly ScaleConnection _connection = new();
    private uint _nextId;
    private uint? _activeOperationId;
    private string _activeOperation = string.Empty;
    private uint? _propertyRequestId;
    private readonly Dictionary<uint, DevicePropertyRow> _pendingPropertyWrites = [];
    private readonly Queue<DevicePropertyRow> _propertyWriteQueue = new();
    private bool _applyingProperties;

    public ObservableCollection<string> AvailablePorts { get; } = [];
    public ObservableCollection<string> Activity { get; } = [];
    public ObservableCollection<double> WeightHistory { get; } = [];
    public ObservableCollection<DevicePropertyRow> DeviceProperties { get; } = [];

    [ObservableProperty] private string? selectedPort;
    [ObservableProperty, NotifyPropertyChangedFor(nameof(CanStartMeasurement)), NotifyPropertyChangedFor(nameof(ConnectionButtonText))] private bool isConnected;
    [ObservableProperty, NotifyPropertyChangedFor(nameof(CanStartMeasurement))] private bool operationInProgress;
    [ObservableProperty, NotifyPropertyChangedFor(nameof(CanStartMeasurement))] private bool isTared;
    [ObservableProperty] private string connectionStatus = "Disconnected";
    [ObservableProperty] private string weightText = "0.00";
    [ObservableProperty] private string stabilityText = "WAITING";
    [ObservableProperty] private IBrush stabilityBrush = new SolidColorBrush(Color.Parse("#8792A2"));
    [ObservableProperty] private string modeText = "—";
    [ObservableProperty] private string tareText = "Not tared";
    [ObservableProperty] private string operationStatus = "Connect to the scale to begin.";
    [ObservableProperty] private string knownMass = "500";
    [ObservableProperty] private string calibrationResult = "No calibration this session";
    [ObservableProperty] private string characterisationResult = "No characterisation yet";
    [ObservableProperty] private string graphRangeText = "Waiting for readings";
    [ObservableProperty] private string propertyStatus = "Connect to load saved device properties.";
    [ObservableProperty] private int propertyCount;

    public bool CanStartMeasurement => IsConnected && IsTared && !OperationInProgress;
    public string ConnectionButtonText => IsConnected ? "Disconnect" : "Connect";
    public string PropertyCountText => $"{PropertyCount} device properties";
    public bool HasDirtyProperties => DeviceProperties.Any(property => property.IsDirty);
    private int CharacterisationCycles => int.TryParse(
        DeviceProperties.FirstOrDefault(property => property.Name == "characterisation.cycles")?.Value,
        NumberStyles.Integer, CultureInfo.InvariantCulture, out var cycles) ? cycles : 5;

    public MainViewModel()
    {
        _connection.MessageReceived += OnMessageReceived;
        _connection.ConnectionLost += OnConnectionLost;
        RefreshPorts();
    }

    [RelayCommand]
    private void RefreshPorts()
    {
        var current = SelectedPort;
        AvailablePorts.Clear();
        foreach (var port in ScaleConnection.GetPortNames())
            AvailablePorts.Add(port);

        if (current is not null && AvailablePorts.Contains(current))
            SelectedPort = current;
        else
            SelectedPort = AvailablePorts.FirstOrDefault();
    }

    [RelayCommand]
    private void GetProperties()
    {
        if (!IsConnected)
            return;
        DeviceProperties.Clear();
        PropertyCount = 0;
        OnPropertyChanged(nameof(HasDirtyProperties));
        _propertyRequestId = NextId();
        PropertyStatus = "Reading properties from device flash…";
        try { _connection.SendCommand(_propertyRequestId.Value, "GET"); }
        catch (Exception ex) { _propertyRequestId = null; PropertyStatus = ex.Message; }
    }

    [RelayCommand]
    private void ApplyPropertyChanges()
    {
        if (!IsConnected || _applyingProperties)
            return;
        _propertyWriteQueue.Clear();
        foreach (var row in DeviceProperties.Where(property => property.IsDirty && property.IsWritable))
            _propertyWriteQueue.Enqueue(row);
        if (_propertyWriteQueue.Count == 0)
        {
            PropertyStatus = "No property changes to save.";
            return;
        }
        _applyingProperties = true;
        PropertyStatus = $"Saving {_propertyWriteQueue.Count} changed value(s) to nonvolatile device storage…";
        SendNextPropertyWrite();
    }

    private void SendNextPropertyWrite()
    {
        if (_propertyWriteQueue.Count == 0)
        {
            _applyingProperties = false;
            PropertyStatus = "All changes saved to device flash.";
            return;
        }
        var row = _propertyWriteQueue.Dequeue();
        var id = NextId();
        _pendingPropertyWrites[id] = row;
        try { _connection.SendCommand(id, "SET", row.Name, row.Value); }
        catch (Exception ex)
        {
            _pendingPropertyWrites.Remove(id);
            _propertyWriteQueue.Clear();
            _applyingProperties = false;
            PropertyStatus = ex.Message;
        }
    }

    [RelayCommand]
    private void Connect()
    {
        if (IsConnected)
        {
            Disconnect();
            return;
        }

        IsTared = false;
        if (string.IsNullOrWhiteSpace(SelectedPort))
        {
            ConnectionStatus = "Select a COM port";
            return;
        }

        try
        {
            _connection.Connect(SelectedPort);
            IsConnected = true;
            WeightHistory.Clear();
            GraphRangeText = "Waiting for readings";
            ConnectionStatus = $"Opening {SelectedPort}…";
            OperationStatus = "Checking scale protocol…";
            var helloId = NextId();
            _connection.SendCommand(helloId, "HELLO");
            AddActivity($"Connected to {SelectedPort}");
        }
        catch (Exception ex)
        {
            _connection.Disconnect();
            IsConnected = false;
            ConnectionStatus = "Connection failed";
            OperationStatus = ex.Message;
        }
    }

    [RelayCommand]
    private void Tare()
    {
        if (!IsConnected)
            return;
        IsTared = false;
        StartOperation("TARE", "TARE");
        OperationStatus = "Tare started. Remove everything from the platform and keep it still.";
    }

    [RelayCommand]
    private void Calibrate()
    {
        if (!CanStartMeasurement)
            return;
        if (!decimal.TryParse(KnownMass, NumberStyles.Number, CultureInfo.CurrentCulture, out var mass) || mass <= 0)
        {
            OperationStatus = "Enter a known mass greater than zero.";
            return;
        }

        var argument = mass.ToString("0.###", CultureInfo.InvariantCulture);
        StartOperation("CAL", "CAL", argument);
        OperationStatus = $"Keep the {mass:0.###} g reference on the platform and still while calibration samples.";
    }

    [RelayCommand]
    private void Characterise()
    {
        if (!CanStartMeasurement)
            return;
        if (!decimal.TryParse(KnownMass, NumberStyles.Number, CultureInfo.CurrentCulture, out var mass) || mass <= 0)
        {
            OperationStatus = "Enter the known reference load in grams.";
            return;
        }

        var argument = mass.ToString("0.###", CultureInfo.InvariantCulture);
        StartOperation("CHARACTERISE", "CHARACTERISE", argument);
        OperationStatus = $"{CharacterisationCycles} load cycles started with the {mass:0.###} g reference. Follow the prompts and keep the scale still after each change.";
    }

    private void StartOperation(string operation, string command, string? argument = null)
    {
        var id = NextId();
        _activeOperationId = id;
        _activeOperation = operation;
        OperationInProgress = true;
        try
        {
            _connection.SendCommand(id, command, argument);
            AddActivity($"Sent {operation} (#{id})");
        }
        catch (Exception ex)
        {
            OperationInProgress = false;
            _activeOperationId = null;
            OperationStatus = ex.Message;
        }
    }

    private uint NextId()
    {
        _nextId = _nextId == uint.MaxValue ? 1 : _nextId + 1;
        return _nextId;
    }

    private void OnMessageReceived(object? sender, ScaleMessage message) =>
        Dispatcher.UIThread.Post(() => ApplyMessage(message));

    private void ApplyMessage(ScaleMessage message)
    {
        switch (message)
        {
            case HelloMessage hello:
                if (hello.Version == 2)
                {
                    ConnectionStatus = $"Connected · {hello.Device} · protocol v{hello.Version}";
                    OperationStatus = "Scale ready.";
                    GetProperties();
                }
                else
                {
                    ConnectionStatus = $"Unsupported protocol v{hello.Version}";
                    OperationStatus = "Update the scale firmware or desktop app.";
                }
                break;

            case ReadingMessage reading:
                WeightText = reading.WeightGrams.ToString("0.00", CultureInfo.CurrentCulture);
                WeightHistory.Add((double)reading.WeightGrams);
                while (WeightHistory.Count > 1200)
                    WeightHistory.RemoveAt(0);
                if (WeightHistory.Count > 0)
                {
                    var min = WeightHistory.Min();
                    var max = WeightHistory.Max();
                    GraphRangeText = $"{min:0.00}–{max:0.00} g";
                }
                StabilityText = reading.Stability;
                StabilityBrush = new SolidColorBrush(Color.Parse(reading.Stability switch
                {
                    "STABLE" => "#49D6A0",
                    "SETTLING" => "#F4C15D",
                    _ => "#F17D74"
                }));
                ModeText = reading.Mode;
                TareText = reading.TareComplete ? "Tare ready" : "Tare needed";
                IsTared = reading.TareComplete;
                break;

            case DevicePropertyMessage property when _propertyRequestId == property.Id:
                var existing = DeviceProperties.FirstOrDefault(item => item.Name == property.Name);
                if (existing is null)
                    DeviceProperties.Add(new DevicePropertyRow(property.Name, property.Value, property.Unit, property.IsWritable));
                else
                    existing.MarkSaved(property.Value);
                OnPropertyChanged(nameof(HasDirtyProperties));
                break;

            case PropertyEndMessage end when _propertyRequestId == end.Id:
                _propertyRequestId = null;
                PropertyCount = end.Count;
                PropertyStatus = DeviceProperties.Count == end.Count
                    ? $"Loaded {end.Count} properties from nonvolatile device storage."
                    : $"Device reported {end.Count} properties; received {DeviceProperties.Count}.";
                OnPropertyChanged(nameof(HasDirtyProperties));
                break;

            case PropertySetMessage set when _pendingPropertyWrites.Remove(set.Id, out var savedRow):
                savedRow.MarkSaved(set.Value);
                OnPropertyChanged(nameof(HasDirtyProperties));
                AddActivity($"Saved property: {savedRow.Name}");
                SendNextPropertyWrite();
                break;

            case PropertyErrorMessage propertyError:
                _pendingPropertyWrites.Remove(propertyError.Id);
                _propertyWriteQueue.Clear();
                _applyingProperties = false;
                PropertyStatus = $"Could not save {propertyError.Name}: {propertyError.Code.Replace('_', ' ')}.";
                break;

            case AcknowledgementMessage ack:
                if (_activeOperationId == ack.Id)
                {
                    OperationStatus = $"{_activeOperation} accepted; waiting for completion…";
                    AddActivity($"{_activeOperation} accepted");
                }
                break;

            case ErrorMessage error:
                if (_activeOperationId == error.Id || error.Id == 0)
                {
                    OperationInProgress = false;
                    _activeOperationId = null;
                    OperationStatus = $"Scale rejected the command: {error.Code.Replace('_', ' ')}.";
                }
                AddActivity($"Scale error: {error.Code}");
                break;

            case TareCompleteMessage tare:
                IsTared = true;
                if (_activeOperationId == tare.Id)
                    CompleteOperation("Tare complete. The platform is zeroed.");
                else
                    AddActivity("Tare complete");
                break;

            case CalibrationCompleteMessage calibration:
                CalibrationResult = $"{calibration.KnownMassGrams:0.###} g reference · {calibration.GramsPerCount:0.000000000} g/count";
                if (_activeOperationId == calibration.Id)
                    CompleteOperation("Calibration complete. The new factor is saved on the scale.");
                AddActivity("Calibration complete");
                break;

            case CharacterisationCompleteMessage characterisation:
                CharacterisationResult = $"Background noise σ {characterisation.BackgroundNoiseGrams:0.0000} g · loaded mean {characterisation.LoadedMeanGrams:0.000} g · loaded noise σ {characterisation.LoadedNoiseGrams:0.0000} g · settling avg/max {characterisation.AverageSettleMilliseconds / 1000m:0.00}/{characterisation.MaximumSettleMilliseconds / 1000m:0.00} s";
                if (_activeOperationId == characterisation.Id)
                    CompleteOperation($"{characterisation.Cycles}-cycle characterisation complete and saved.");
                AddActivity("Characterisation complete");
                break;

            case CharacterisationStepMessage step:
                if (_activeOperationId == step.Id)
                {
                    OperationStatus = step.Step == 0
                        ? "Baseline noise: remove the reference load and leave the empty platform still."
                        : $"Cycle {(step.Step + 1) / 2}/{CharacterisationCycles}: {(step.Action == "ADD_LOAD" ? "add" : "remove")} the reference load, then keep the scale still while it settles.";
                }
                break;

            case CancelledMessage cancelled:
                AddActivity($"Operation #{cancelled.Id} cancelled by {cancelled.ByOperation}");
                if (_activeOperationId == cancelled.Id)
                    CompleteOperation("Operation cancelled.");
                if (_activeOperation == "TARE")
                    IsTared = false;
                break;
        }
    }

    private void CompleteOperation(string status)
    {
        OperationInProgress = false;
        _activeOperationId = null;
        _activeOperation = string.Empty;
        OperationStatus = status;
    }

    private void OnConnectionLost(object? sender, string message) =>
        Dispatcher.UIThread.Post(() =>
        {
            _connection.Disconnect();
            IsConnected = false;
            OperationInProgress = false;
            _activeOperationId = null;
            ConnectionStatus = "Connection lost";
            OperationStatus = message;
        });

    private void Disconnect()
    {
        _connection.Disconnect();
        IsConnected = false;
        IsTared = false;
        OperationInProgress = false;
        _activeOperationId = null;
        ConnectionStatus = "Disconnected";
        OperationStatus = "Connect to the scale to begin.";
        _propertyRequestId = null;
        _pendingPropertyWrites.Clear();
        _propertyWriteQueue.Clear();
        _applyingProperties = false;
        DeviceProperties.Clear();
        PropertyCount = 0;
        PropertyStatus = "Connect to load saved device properties.";
        AddActivity("Disconnected");
    }

    private void AddActivity(string text)
    {
        Activity.Insert(0, $"{DateTime.Now:HH:mm:ss}  {text}");
        while (Activity.Count > 8)
            Activity.RemoveAt(Activity.Count - 1);
    }

    public void Dispose()
    {
        _connection.MessageReceived -= OnMessageReceived;
        _connection.ConnectionLost -= OnConnectionLost;
        _connection.Dispose();
    }
}
