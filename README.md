# RapidScale

Scale firmware for the Raspberry Pi Pico 2 (RP2350), written in Rust with Embassy.

## Build

```sh
cargo build --release
```

The Cargo configuration targets `thumbv8m.main-none-eabihf` and uses `picotool` to flash and run the firmware with `cargo run --release`.

The Windows desktop interface is in [`ScaleDesktop`](ScaleDesktop/README.md) and uses Avalonia with CommunityToolkit.Mvvm.

## USB serial commands

Connect to the scale's USB CDC serial port. USB CDC is virtual serial, so the baud rate setting is ignored. The same port carries firmware log output and accepts one command per line (CR, LF, or CRLF).

| Command | Action |
| --- | --- |
| `HELP` | List commands. |
| `TARE` | Start a new tare; remove all load and keep the platform still. |
| `CAL <known_mass_g>` | With a known mass already on the platform, average 640 consecutive stable samples and update the grams-per-count factor. For example: `CAL 500`. |
| `CHARACTERISE <known_mass_g>` | Run a guided five-cycle filter and noise characterisation using a known reference load. Follow the prompts to remove the load, then add and remove it five times. The report includes empty-platform background noise, steady loaded noise, and average/maximum time for the filtered reading to settle within 0.01 g. This measures repeatability and filter response, not weighing accuracy. |

Calibration, tare state/noise, characterisation settings, and the last characterisation results are stored in two alternating flash sectors. The firmware reserves the final 8 KiB of the 2 MiB flash for these values and uses a checksum plus generation counter to recover the previous copy if a write is interrupted. Startup still performs a fresh automatic tare on the empty platform.

## USB protocol v2

The desktop app should use the versioned protocol below. It shares the USB CDC stream with readable diagnostic logs; protocol records start with `@SCALE/2,` and are plain ASCII CSV terminated by CRLF. Ignore lines that do not start with that prefix. Fields use invariant decimal points. Command IDs are nonzero unsigned 32-bit integers chosen by the host; include the same ID in the matching completion event. Send one request per line.

### Requests

```text
@SCALE/2,CMD,<id>,HELLO
@SCALE/2,CMD,<id>,TARE
@SCALE/2,CMD,<id>,CAL,<known_mass_g>
@SCALE/2,CMD,<id>,CHARACTERISE,<known_mass_g>
@SCALE/2,CMD,<id>,GET
@SCALE/2,CMD,<id>,SET,<property>,<value>
```

`GET` streams the complete property table as `@SCALE/2,PROPERTY,<id>,<name>,<value>,<unit>,<RW|RO>` rows and ends with `@SCALE/2,PROPERTY_END,<id>,<count>`. `SET` updates one writable value and replies with `@SCALE/2,PROPERTY_SET,<id>,<name>,<value>` after it has been saved and verified in flash. Failures use `@SCALE/2,PROPERTY_ERROR,<id>,<name>,<code>`. Calibration factor, tare offset/noise, and characterisation reference/cycles/convergence/window size are writable. Recorded calibration and characterisation results and validity flags are read-only. Values are comma-free ASCII scalars; decimals use a dot. Valid ranges are checked by the device.

`HELLO` returns `@SCALE/2,HELLO,<id>,2,RP2350,320` (protocol version 2, 320 sensor samples/second). `TARE` requires an unloaded, still platform. For `CAL`, first place the known mass, then send the command; the firmware uses 640 consecutive stable raw samples. For `CHARACTERISE`, supply the known reference mass. The scale first asks you to remove it for a background-noise baseline, then guides five add/remove cycles. Each plateau is considered settled when two consecutive 0.5-second filtered windows differ by no more than 0.01 g. Each prompted step times out after two minutes; a detected transition that does not converge within 30 seconds is reported as an error.

### Responses

```text
@SCALE/2,ACK,<id>,<operation>
@SCALE/2,ERROR,<id>,<code>
@SCALE/2,READING,<sequence>,<weight_g>,<stability>,<mode>,<tare_complete>
@SCALE/2,EVENT,TARE_DONE,<id>
@SCALE/2,EVENT,CAL_DONE,<id>,<known_mass_g>,<raw_delta_counts>,<grams_per_count>
@SCALE/2,EVENT,CHAR_STEP,<id>,<step>,<ADD_LOAD|REMOVE_LOAD>
@SCALE/2,EVENT,CHAR_DONE,<id>,5,<background_noise_sd_g>,<loaded_mean_g>,<loaded_noise_sd_g>,<average_settle_ms>,<maximum_settle_ms>
@SCALE/2,EVENT,CANCELLED,<id>,TARE
@SCALE/2,PROPERTY,<id>,<name>,<value>,<unit>,<RW|RO>
@SCALE/2,PROPERTY_END,<id>,<count>
@SCALE/2,PROPERTY_SET,<id>,<name>,<value>
@SCALE/2,PROPERTY_ERROR,<id>,<name>,<code>
```

`ACK` means a valid operation started, not that it completed. `ERROR` codes include `BAD_REQUEST`, `BAD_ARGUMENT`, `NOT_TARED`, `BUSY`, `UNKNOWN_COMMAND`, `CALIBRATION_FAILED`, `CHAR_TIMEOUT`, and `CHAR_NOT_STABLE`. Readings are sent every 50 ms (20 Hz) after startup tare completes; the sensor is sampled internally at 320 Hz. Stability values are `UNSTABLE`, `SETTLING`, or `STABLE`; mode values are `FAST`, `SETTLING`, or `STABLE`; `tare_complete` is `0` or `1`. Completion events carry the request ID. A host should time out an operation if its completion event does not arrive and may send `HELLO` to confirm the connection. Sending `TARE` cancels a running calibration or characterisation, with a `CANCELLED` event for its ID.

Legacy terminal commands (`TARE`, `CAL 500`, and `CHARACTERISE 500`) remain accepted for interactive use. Legacy characterisation prompts and completion are available only in readable logs; the desktop app should use protocol v2.
