using System.Globalization;

namespace ScaleDesktop.Models;

public abstract record ScaleMessage;

public sealed record HelloMessage(uint Id, int Version, string Device, int SampleRate) : ScaleMessage;
public sealed record AcknowledgementMessage(uint Id, string Operation) : ScaleMessage;
public sealed record ErrorMessage(uint Id, string Code) : ScaleMessage;
public sealed record ReadingMessage(uint Sequence, decimal WeightGrams, string Stability, string Mode, bool TareComplete) : ScaleMessage;
public sealed record TareCompleteMessage(uint Id) : ScaleMessage;
public sealed record CalibrationCompleteMessage(uint Id, decimal KnownMassGrams, decimal RawDeltaCounts, decimal GramsPerCount) : ScaleMessage;
public sealed record CharacterisationStepMessage(uint Id, int Step, string Action) : ScaleMessage;
public sealed record CharacterisationCompleteMessage(uint Id, int Cycles, decimal BackgroundNoiseGrams, decimal LoadedMeanGrams, decimal LoadedNoiseGrams, uint AverageSettleMilliseconds, uint MaximumSettleMilliseconds) : ScaleMessage;
public sealed record DevicePropertyMessage(uint Id, string Name, string Value, string Unit, bool IsWritable) : ScaleMessage;
public sealed record PropertyEndMessage(uint Id, int Count) : ScaleMessage;
public sealed record PropertySetMessage(uint Id, string Name, string Value) : ScaleMessage;
public sealed record PropertyErrorMessage(uint Id, string Name, string Code) : ScaleMessage;
public sealed record CancelledMessage(uint Id, string ByOperation) : ScaleMessage;

public static class ScaleProtocol
{
    private const string Prefix = "@SCALE/2";

    public static ScaleMessage? Parse(string line)
    {
        var fields = line.Trim().Split(',');
        if (fields.Length < 3 || fields[0] != Prefix)
            return null;

        var invariant = CultureInfo.InvariantCulture;
        switch (fields[1])
        {
            case "HELLO" when fields.Length == 6
                && uint.TryParse(fields[2], invariant, out var helloId)
                && int.TryParse(fields[3], invariant, out var version)
                && int.TryParse(fields[5], invariant, out var sampleRate):
                return new HelloMessage(helloId, version, fields[4], sampleRate);

            case "ACK" when fields.Length == 4
                && uint.TryParse(fields[2], invariant, out var ackId):
                return new AcknowledgementMessage(ackId, fields[3]);

            case "ERROR" when fields.Length == 4
                && uint.TryParse(fields[2], invariant, out var errorId):
                return new ErrorMessage(errorId, fields[3]);

            case "PROPERTY" when fields.Length == 7
                && uint.TryParse(fields[2], invariant, out var propertyId)
                && (fields[6] == "RW" || fields[6] == "RO"):
                return new DevicePropertyMessage(propertyId, fields[3], fields[4], fields[5], fields[6] == "RW");

            case "PROPERTY_END" when fields.Length == 4
                && uint.TryParse(fields[2], invariant, out var endId)
                && int.TryParse(fields[3], invariant, out var propertyCount):
                return new PropertyEndMessage(endId, propertyCount);

            case "PROPERTY_SET" when fields.Length == 5
                && uint.TryParse(fields[2], invariant, out var setId):
                return new PropertySetMessage(setId, fields[3], fields[4]);

            case "PROPERTY_ERROR" when fields.Length == 5
                && uint.TryParse(fields[2], invariant, out var propertyErrorId):
                return new PropertyErrorMessage(propertyErrorId, fields[3], fields[4]);

            case "READING" when fields.Length == 7
                && uint.TryParse(fields[2], invariant, out var sequence)
                && decimal.TryParse(fields[3], NumberStyles.Float, invariant, out var weight)
                && (fields[6] == "0" || fields[6] == "1"):
                return new ReadingMessage(sequence, weight, fields[4], fields[5], fields[6] == "1");

            case "EVENT" when fields.Length == 4 && fields[2] == "TARE_DONE"
                && uint.TryParse(fields[3], invariant, out var tareId):
                return new TareCompleteMessage(tareId);

            case "EVENT" when fields.Length == 7 && fields[2] == "CAL_DONE"
                && uint.TryParse(fields[3], invariant, out var calId)
                && decimal.TryParse(fields[4], NumberStyles.Float, invariant, out var knownMass)
                && decimal.TryParse(fields[5], NumberStyles.Float, invariant, out var rawDelta)
                && decimal.TryParse(fields[6], NumberStyles.Float, invariant, out var gramsPerCount):
                return new CalibrationCompleteMessage(calId, knownMass, rawDelta, gramsPerCount);

            case "EVENT" when fields.Length == 6 && fields[2] == "CHAR_STEP"
                && uint.TryParse(fields[3], invariant, out var charId)
                && int.TryParse(fields[4], invariant, out var charStep):
                return new CharacterisationStepMessage(charId, charStep, fields[5]);

            case "EVENT" when fields.Length == 10 && fields[2] == "CHAR_DONE"
                && uint.TryParse(fields[3], invariant, out var charDoneId)
                && int.TryParse(fields[4], invariant, out var cycles)
                && decimal.TryParse(fields[5], NumberStyles.Float, invariant, out var backgroundNoise)
                && decimal.TryParse(fields[6], NumberStyles.Float, invariant, out var loadedMean)
                && decimal.TryParse(fields[7], NumberStyles.Float, invariant, out var loadedNoise)
                && uint.TryParse(fields[8], invariant, out var averageSettle)
                && uint.TryParse(fields[9], invariant, out var maximumSettle):
                return new CharacterisationCompleteMessage(charDoneId, cycles, backgroundNoise, loadedMean, loadedNoise, averageSettle, maximumSettle);

            case "EVENT" when fields.Length == 5 && fields[2] == "CANCELLED"
                && uint.TryParse(fields[3], invariant, out var cancelledId):
                return new CancelledMessage(cancelledId, fields[4]);

            default:
                return null;
        }
    }
}
