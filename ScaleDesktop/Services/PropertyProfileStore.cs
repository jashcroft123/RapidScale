using System.Text.Json;
using ScaleDesktop.Models;

namespace ScaleDesktop.Services;

public sealed class PropertyProfileStore
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true };
    private readonly string _path = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
        "RapidScale",
        "property-profiles.json");

    public List<PropertyProfile> Load()
    {
        if (!File.Exists(_path))
            return [];

        try
        {
            return JsonSerializer.Deserialize<List<PropertyProfile>>(File.ReadAllText(_path)) ?? [];
        }
        catch (JsonException)
        {
            return [];
        }
        catch (IOException)
        {
            return [];
        }
        catch (UnauthorizedAccessException)
        {
            return [];
        }
    }

    public void Save(IReadOnlyCollection<PropertyProfile> profiles)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(_path)!);
        var temporaryPath = _path + ".tmp";
        File.WriteAllText(temporaryPath, JsonSerializer.Serialize(profiles, JsonOptions));
        File.Move(temporaryPath, _path, overwrite: true);
    }
}
