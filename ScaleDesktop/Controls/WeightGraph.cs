using System.Collections.ObjectModel;
using System.Collections.Specialized;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;

namespace ScaleDesktop.Controls;

public sealed class WeightGraph : Control
{
    public static readonly StyledProperty<ObservableCollection<double>?> ValuesProperty =
        AvaloniaProperty.Register<WeightGraph, ObservableCollection<double>?>(nameof(Values));

    private static readonly Pen GridPen = new(new SolidColorBrush(Color.Parse("#263442")), 1);
    private static readonly Pen ReadingPen = new(new SolidColorBrush(Color.Parse("#65E0B2")), 2);

    static WeightGraph()
    {
        AffectsRender<WeightGraph>(ValuesProperty);
    }

    public ObservableCollection<double>? Values
    {
        get => GetValue(ValuesProperty);
        set => SetValue(ValuesProperty, value);
    }

    protected override void OnPropertyChanged(AvaloniaPropertyChangedEventArgs change)
    {
        base.OnPropertyChanged(change);
        if (change.Property != ValuesProperty)
            return;

        if (change.OldValue is ObservableCollection<double> oldValues)
            oldValues.CollectionChanged -= OnValuesChanged;
        if (change.NewValue is ObservableCollection<double> newValues)
            newValues.CollectionChanged += OnValuesChanged;
        InvalidateVisual();
    }

    private void OnValuesChanged(object? sender, NotifyCollectionChangedEventArgs e) => InvalidateVisual();

    public override void Render(DrawingContext context)
    {
        base.Render(context);

        var width = Bounds.Width;
        var height = Bounds.Height;
        if (width <= 0 || height <= 0)
            return;

        const double left = 4;
        const double right = 4;
        const double top = 7;
        const double bottom = 7;
        var plotWidth = width - left - right;
        var plotHeight = height - top - bottom;
        if (plotWidth <= 0 || plotHeight <= 0)
            return;

        for (var i = 0; i <= 4; i++)
        {
            var y = top + plotHeight * i / 4;
            context.DrawLine(GridPen, new Point(left, y), new Point(width - right, y));
        }

        var values = Values;
        if (values is null || values.Count == 0)
            return;

        var minimum = values[0];
        var maximum = values[0];
        foreach (var value in values)
        {
            minimum = Math.Min(minimum, value);
            maximum = Math.Max(maximum, value);
        }

        var span = maximum - minimum;
        if (span < 0.01)
        {
            var center = (maximum + minimum) / 2;
            span = 0.01;
            minimum = center - span / 2;
            maximum = center + span / 2;
        }
        else
        {
            var padding = span * 0.08;
            minimum -= padding;
            maximum += padding;
        }

        if (values.Count == 1)
        {
            var y = top + plotHeight * (maximum - values[0]) / (maximum - minimum);
            context.DrawEllipse(ReadingPen.Brush, null, new Point(left, y), 3, 3);
            return;
        }

        var lastIndex = values.Count - 1;
        var previous = new Point(left, top + plotHeight * (maximum - values[0]) / (maximum - minimum));
        for (var i = 1; i < values.Count; i++)
        {
            var point = new Point(
                left + plotWidth * i / lastIndex,
                top + plotHeight * (maximum - values[i]) / (maximum - minimum));
            context.DrawLine(ReadingPen, previous, point);
            previous = point;
        }
    }
}
