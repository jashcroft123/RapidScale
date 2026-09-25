using System.IO.Ports;
using System.Text;
using ScaleDesktop.Models;

namespace ScaleDesktop.Services;

public sealed class ScaleConnection : IDisposable
{
    private readonly object _sync = new();
    private readonly StringBuilder _receiveBuffer = new();
    private SerialPort? _port;

    public event EventHandler<ScaleMessage>? MessageReceived;
    public event EventHandler<string>? ConnectionLost;

    public bool IsConnected => _port?.IsOpen == true;

    public static string[] GetPortNames() => SerialPort.GetPortNames().OrderBy(name => name).ToArray();

    public void Connect(string portName)
    {
        Disconnect();
        var port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One)
        {
            NewLine = "\n",
            Encoding = Encoding.ASCII,
            DtrEnable = false,
            RtsEnable = false,
            ReadTimeout = 1000,
            WriteTimeout = 1000
        };
        port.DataReceived += OnDataReceived;
        port.Open();
        _port = port;
        lock (_sync) _receiveBuffer.Clear();
    }

    public void SendCommand(uint id, string command, string? argument = null, string? secondArgument = null)
    {
        var port = _port;
        if (port?.IsOpen != true)
            throw new InvalidOperationException("The scale is not connected.");

        var line = argument is null
            ? $"@SCALE/2,CMD,{id},{command}"
            : secondArgument is null
                ? $"@SCALE/2,CMD,{id},{command},{argument}"
                : $"@SCALE/2,CMD,{id},{command},{argument},{secondArgument}";
        port.WriteLine(line);
    }

    public void Disconnect()
    {
        var port = _port;
        _port = null;
        if (port is null)
            return;

        port.DataReceived -= OnDataReceived;
        try
        {
            if (port.IsOpen)
                port.Close();
        }
        finally
        {
            port.Dispose();
        }
    }

    private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        if (sender is not SerialPort port)
            return;

        try
        {
            var chunk = port.ReadExisting();
            List<string> lines = [];
            lock (_sync)
            {
                _receiveBuffer.Append(chunk);
                while (true)
                {
                    var text = _receiveBuffer.ToString();
                    var newline = text.IndexOf('\n');
                    if (newline < 0)
                        break;
                    lines.Add(text[..newline].TrimEnd('\r'));
                    _receiveBuffer.Remove(0, newline + 1);
                }

                // Prevent unbounded growth if a disconnected/noisy device sends no line breaks.
                if (_receiveBuffer.Length > 2048)
                    _receiveBuffer.Clear();
            }

            foreach (var line in lines)
            {
                var message = ScaleProtocol.Parse(line);
                if (message is not null)
                    MessageReceived?.Invoke(this, message);
            }
        }
        catch (Exception ex) when (ex is IOException or InvalidOperationException or UnauthorizedAccessException)
        {
            ConnectionLost?.Invoke(this, ex.Message);
        }
    }

    public void Dispose() => Disconnect();
}
