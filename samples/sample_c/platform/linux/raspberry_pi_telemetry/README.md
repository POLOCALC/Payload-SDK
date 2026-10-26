# psdk_telemetry

Telemetry bridge for the POLOCALC payload: connects to the DJI drone (Matrice 400) through the E-Port, subscribes flight-controller telemetry, and streams every sample as a framed binary record. It is meant to be started by lager (`UAVs/DJI_M400`, to be written), which reads the stream, decodes it and logs it, as it does for the Matrice 600.

- **stdout:** the binary stream, or a file with `--output`. The record format is described in [application/telemetry_stream.h](application/telemetry_stream.h).
- **stderr:** human-readable log, including the PSDK's own messages.

## Settings

It uses the Raspberry Pi sample's configuration, so both programs always share it (see the main [README](../../../../../README.md), section 4):
- `../raspberry_pi/hal/hal_uart.h`: the UART device (`/dev/ttyGS0`),
- `../raspberry_pi/application/dji_sdk_app_info.h`: the application credentials and baud rate.

## Build

From the repository root (the top-level `CMakeLists.txt` includes this folder):
```sh
mkdir -p build && cd build
cmake ..
make -j4 psdk_telemetry
```
The program is `build/bin/psdk_telemetry`. It only needs the PSDK library, without `libopus` or `libusb`.

## Options

| Option | Default | Meaning |
|---|---|---|
| `-o, --output FILE` | `-` (stdout) | where to write the binary stream |
| `--imu HZ` | 0 (off) | rate of the imu group (IMU navigation data; the aircraft only gives it at 50 Hz) |
| `--fast HZ` | 10 | rate of the fast group |
| `--status HZ` | 5 | rate of the status group |
| `--slow HZ` | 1 | rate of the slow group |
| `--stats S` | 5 | seconds between STATS records (0 = off) |
| `--serial SN` | `PSDK12345678XX` | payload serial number reported to the drone, the same as the DJI demo's, so an activation done with the demo stays valid |
| `--high-power V` | 24 | E-Port VCC voltage to request from the aircraft: `24`, `17`, `13.6`, or `off` to leave VCC unchanged |
| `--service` | off | service mode: telemetry and control sockets, RC widgets (see "Service mode") |
| `--socket PATH` | `/tmp/psdk_telemetry.sock` | service mode: telemetry socket |
| `--control PATH` | `/tmp/psdk_control.sock` | service mode: control socket for telemd |
| `--config-dir DIR` | none | service mode: porter's `config/` folder, listed on the RC |
| `--widget-icons DIR` | required with `--service` | service mode: the widget icons ([widget/](widget/)) |
| `-p, --print` | off | print the latest decoded values to stderr every second |
| `-v, --verbose` | off | PSDK debug logs |

Rates: `0` (group off), 1, 5, 10, 50, 100, 200 or 400 Hz. The PSDK allows at most 4 different rates and 242 bytes of topics per rate. Groups set to the same rate share those 242 bytes, so the program checks the sizes at start-up and refuses rates that don't fit.

**Topics:** every flight-controller topic the Matrice 400 provides, in four groups.

| Group (default rate) | Bytes | Topics |
|---|---|---|
| imu (off) | 64 | IMU navigation data with timestamp (position, velocity, acceleration, quaternion); the aircraft only gives it at 50 Hz |
| fast (10 Hz) | 168 | quaternion, velocity, acceleration (ground, body, raw), angular rate (fused, raw), compass, altitude (fused, barometer), fused height, fused position, obstacle distances, RC sticks |
| status (5 Hz) | 221 | GPS position and velocity, RTK position, velocity, yaw and their status (the Matrice 400 updates GPS and RTK at 5 Hz), visual-odometry position, RC sticks and link flags, GPS details and signal level, flight status, display mode, motor start error, control device, RTK connection, flight anomalies, ESC data |
| slow (1 Hz) | 62 | battery, GPS date and time, home-point altitude, status and position |

**Link bandwidth.** The link to the aircraft runs at 115200 baud, about 11.5 kB/s. At the defaults the telemetry takes about 3.6 kB/s; `--imu 50` adds about 4.7 kB/s. Don't go much higher: with all the groups at higher rates (about 10 kB/s), the aircraft stopped answering after about a minute (`Command async send retry ... 0x0503->0x0801` warnings), then dropped the link (`connect status async timeout`).

**Topics left out:**
- `BATTERY_SINGLE_INFO_INDEX2`: the Matrice 400 has a single battery, and this topic only repeats `INDEX1`.
- Rejected by the Matrice 400 (error `0x00000100`): `BATTERY_INFO`, `GIMBAL_STATUS`, `THREE_GIMBAL_DATA`, `GPS_CONTROL_LEVEL`, `STATUS_LANDINGGEAR`, `GIMBAL_CONTROL_MODE`, `HEIGHT_RELATIVE`, `HARD_SYNC`. `BATTERY_SINGLE_INFO_INDEX1` replaces `BATTERY_INFO`.
- For DJI gimbals on the aircraft's own gimbal ports, of which POLOCALC has none: `GIMBAL_ANGLES` (50 Hz only), `GIMBAL_ANGLES_ON_POS_NO1..7`. In `dji_fc_subscription.h`, `NO1` also has the same topic id as `IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP`.

The topics give data at up to their own maximum rate, whatever rate is requested. For example, the Matrice 400 sends GPS and RTK at 5 Hz even in the 10 Hz group. The STATS records show how many records each topic actually delivered.

A topic the aircraft doesn't support is reported as a warning and skipped; the others keep running.

**E-Port VCC (high power).** Right after connecting, the program asks the aircraft for high power on the adapter's VCC pin, at the `--high-power` voltage (24 V by default). This only works on adapters that can choose the voltage: the E-Port V2 ribbon cable (adapter type 5, the POLOCALC setup) and the SkyPort V3. On other adapters VCC is left unchanged, with a warning. The aircraft doesn't remember the request, so it is made at every connection; before that, VCC gives the E-Port's default output. On the Matrice 400 the HIGH_PWR_REQ pin (E-Port V2 pin 36) must be connected directly to GND, without any extra pull-down resistor. It's active low: when it's high or floating, the aircraft accepts the request but keeps VCC at its default 13.6 V. The result is a STATUS record: `high power requested: VCC 24 V`, or an ERROR with the PSDK error code. The telemetry keeps running either way.

**Exit codes:**

| Code | Meaning |
|---|---|
| 0 | stopped by SIGINT/SIGTERM |
| 1 | bad options |
| 2 | platform setup or credentials error |
| 3 | `/dev/ttyGS0` missing or in use by another process, or `DjiCore_Init` failed (no aircraft, wrong link, or authentication) |
| 4 | no topic could be subscribed |
| 5 | the reader went away (broken pipe) |
| 6 | link to the aircraft lost: no telemetry for 10 s (the PSDK doesn't recover by itself; lager starts the program again) |

## Service mode (`--service`): psdk-bridge.service

On the POLOCALC Pi the program normally runs from boot as `psdk-bridge.service` (porter's `services/` folder), with `--service`. It then owns the PSDK connection and shares it, instead of writing the stream to stdout:

| What | Where |
|---|---|
| telemetry stream (same records) | Unix socket `--socket`, default `/tmp/psdk_telemetry.sock`, any number of readers. A new reader first gets the CONNECTED status and the TOPIC_TABLE, then the live records. lager reads it when `socket:` is set in the `uav.connection` block of its config. |
| RC commands and status | Unix socket `--control`, default `/tmp/psdk_control.sock`, one text client (porter's telemd). Protocol in [application/service.h](application/service.h). |
| widget panel in DJI Pilot 2 | generated at start-up in `/tmp/polocalc_widget/`: **Start acquisition** and **Stop acquisition** buttons, a **POI tracking** switch, the **Configuration** list (the `.yml` files of `--config-dir`) and a **Shutdown Pi** button (press it twice within 5 s), all on the main panel; status line and command replies in the floating window. Icons from `--widget-icons` ([widget/](widget/)). |

The socket files appear once the topics are subscribed and are removed when the program stops, so readers notice at once. The list of configurations is read when the program starts: restart the service after adding a configuration (`sudo systemctl restart psdk-bridge`).

Run by hand for a test (stop the service first, only one program can hold the PSDK connection):
```sh
sudo systemctl stop psdk-bridge
./build/bin/psdk_telemetry --service --config-dir ~/flight/porter/config \
    --widget-icons samples/sample_c/platform/linux/raspberry_pi_telemetry/widget --print
```

## Testing on the Pi

With the gadget active (`/dev/ttyGS0` present) and the drone connected and powered:
```sh
# live, decoded on screen, with the program's own log in a file
./build/bin/psdk_telemetry --print 2> psdk.log | python3 samples/sample_c/platform/linux/raspberry_pi_telemetry/tools/psdk_stream_reader.py

# or record to a file and decode it afterwards
./build/bin/psdk_telemetry --output capture.bin --print
python3 samples/sample_c/platform/linux/raspberry_pi_telemetry/tools/psdk_stream_reader.py capture.bin --quiet
```
Stop with **Ctrl+C** (or `kill <pid>`): the program unsubscribes, closes the port, sends a final STATS record and exits with code 0.

**Don't use Ctrl+Z.** It only pauses the program, which keeps `/dev/ttyGS0` open and locked. The next start then fails, and the drone may keep waiting on the old session. If it happened: `kill %1` (or `pkill -f psdk_telemetry`), then power-cycle the drone. `psdk_telemetry` checks the lock before connecting, and names the process that holds the port.

Run it without `sudo`: with `sudo`, the SDK creates its files as root.

What a working run looks like:
1. STATUS records `STARTING`, then `CONNECTING`, then `CONNECTED: aircraft type 103 (Matrice 400) ...`,
2. a TOPIC TABLE with every topic marked `subscribed`,
3. `SUBSCRIBED`,
4. DATA records, and every 5 s a STATS line with the counts per topic and `dropped=0`.

`polocalc` must be able to open `/dev/ttyGS0` (the `dialout` group). The program doesn't need `sudo`.
