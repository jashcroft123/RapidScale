using CommunityToolkit.Mvvm.ComponentModel;

namespace ScaleDesktop.Models;

public partial class DevicePropertyRow : ObservableObject
{
    private string _savedValue;

    public string Name { get; }
    public string DisplayName { get; }
    public string Unit { get; }
    public bool IsWritable { get; }
    public bool IsReadOnly => !IsWritable;
    public string AccessText => IsWritable ? "EDITABLE" : "DEVICE";
    public bool IsDirty => IsWritable && Value != _savedValue;

    [ObservableProperty]
    private string value = string.Empty;

    public DevicePropertyRow(string name, string value, string unit, bool isWritable)
    {
        Name = name;
        DisplayName = name.Replace('.', ' ').Replace('_', ' ');
        Unit = unit;
        IsWritable = isWritable;
        this.value = value;
        _savedValue = value;
    }

    partial void OnValueChanged(string value) => OnPropertyChanged(nameof(IsDirty));

    public void MarkSaved(string value)
    {
        Value = value;
        _savedValue = value;
        OnPropertyChanged(nameof(IsDirty));
    }
}
