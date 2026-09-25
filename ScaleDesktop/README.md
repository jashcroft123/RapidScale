# RapidScale Desktop

Windows desktop companion for the RP2350 scale. It uses Avalonia, CommunityToolkit.Mvvm, and the USB CDC serial connection.

## Requirements

- Windows 10 or newer
- .NET 10 SDK
- The scale flashed with USB protocol v2 firmware

## Build and run

From the repository root:

```powershell
dotnet build .\ScaleDesktop\ScaleDesktop.csproj -c Release
dotnet run --project .\ScaleDesktop\ScaleDesktop.csproj -c Release
```

Choose the scale's COM port and connect. The app sends a protocol `HELLO` request and listens for live readings. USB CDC is a virtual serial port; the baud rate does not affect the connection.

## Controls

- **Tare scale:** remove all load, then tare.
- **Calibrate:** place the known mass on the platform first, enter its mass in grams, and start calibration.
- **Characterise:** enter the known reference mass and follow the prompts to remove it, then add and remove it five times. The test estimates background noise on the empty platform, noise during stable loading, and average/maximum settling time for the current filters. Each settled plateau requires consecutive 0.5-second windows to agree within 0.01 g. It measures repeatability and filter response, not accuracy against the reference.
- **Weight trend:** view the most recent 60 seconds of readings, refreshed at 20 Hz (one update every 50 ms).
- **Device properties:** the app reads the full stored property table after connecting. Edit writable calibration, tare, and characterisation values, then save them to the scale. Each accepted edit is stored and verified in flash; measured results and validity flags are read-only.
- **Property profiles:** save a named snapshot of all writable values currently shown in the property table. Profiles are stored in `%LocalAppData%\RapidScale\property-profiles.json`; applying one writes its values to the connected scale, one at a time, and persists them in device flash.
- **Diagnostics:** show device/protocol identity, sensor sample rate, observed USB reading rate, missing reading sequence numbers, last-reading age, connection health, and tare state. Sequence gaps are counted since the current connection.

The device performs a fresh automatic tare at startup. The desktop app and firmware protocol are described in the repository's top-level README.
