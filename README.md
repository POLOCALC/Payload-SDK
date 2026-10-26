# Payload-SDK (POLOCALC fork)

Fork of [DJI's Payload SDK (PSDK)](https://github.com/dji-sdk/Payload-SDK), currently at upstream **v3.16.0**, used by the **POLOCALC** payload to talk to the DJI drone through its **E-Port V2**. It is a git submodule of [lager](https://github.com/POLOCALC/lager) (`modules/Payload-SDK`), which uses it for the drone telemetry; lager is in turn a submodule of [porter](https://github.com/POLOCALC/porter), so in porter it is at `modules/lager/modules/Payload-SDK`.

The SDK library is unchanged from upstream. This fork sets the Raspberry Pi samples up for POLOCALC (section 4), builds them from the top-level `CMakeLists.txt` (section 5), and adds:
- `tools/dji_vcom_gadget/`: a script and a systemd unit that make the Raspberry Pi's USB-C port appear to the drone as a DJI USB serial device (see "Connection"),
- this README: registration, wiring, configuration and build for the POLOCALC Raspberry Pi 5.

For the SDK API, the samples and the full documentation, see DJI's [developer site](https://developer.dji.com/doc/payload-sdk-tutorial/en/) and the upstream repository.

## 1. Application registration

Using the PSDK needs an application registered with DJI:
1. Create or use a DJI developer account, and register a **PSDK application** for POLOCALC. DJI then verifies it.
2. DJI gives the application an **App Name, App ID, App Key and App License**, tied to the developer account.

These four values are compiled into the program (see "Configuration"). **Don't commit them to this repository.** Keep them in a local, uncommitted file, or in the team's password store. If the repository is public, anything committed here is public, including in the git history.

## 2. Activating the payload on the drone

Before first use, the drone must know the payload:
1. Build and run the PSDK program on the Pi (sections 4–5), with the Pi connected to the drone's E-Port. A sample program from section 5 is enough.
2. Connect the drone to **DJI Assistant 2** on a computer. Use the Enterprise edition for the M400.
3. Log in with **the same DJI account** the PSDK application is registered with.
4. In the **PSDK** menu, find the payload, i.e. the serial number of the Raspberry Pi + E-Port V2, and **activate** it. The payload and the drone must be communicating while the drone is connected to Assistant 2, so keep the program running during this step.

## 3. Connection: E-Port V2 over USB-C

The E-Port V2 talks to the payload **only over USB**, with the drone as the USB **host**. So the Raspberry Pi must act as a USB **device**, and the only Pi 5 port that can do that is the **USB-C** connector.

### 3.1 Put the USB-C port in device mode

In `/boot/firmware/config.txt` (porter's [config.txt](../../../../config.txt) already has it):
```
dtoverlay=dwc2,dr_mode=peripheral
```
Then reboot. `ls /sys/class/udc` should then list a controller (e.g. `1000480000.usb`).

### 3.2 Present the Pi as a DJI USB serial device

`tools/dji_vcom_gadget/dji_vcom_gadget.sh` configures the USB gadget through `configfs`:
- a **CDC-ACM serial function**: the drone sees a serial port, and the Pi gets **`/dev/ttyGS0`**,
- the **DJI VCOM USB identifiers** the E-Port V2 expects: vendor `0x2CA3`, product `0xF002`,
- product name "PSDK VCOM", serial number `PSDK0001`, maximum power 250 mA.

It removes a previous instance first, so it can be run again safely. It must run as root:
```sh
sudo bash tools/dji_vcom_gadget/dji_vcom_gadget.sh
# -> USB gadget bound to <controller>: /dev/ttyGS0
```

### 3.3 Run it at every boot

`tools/dji_vcom_gadget/dji-vcom-gadget.service` is a one-shot systemd unit that runs the script after `configfs` is mounted.

**Porter's installer sets it up.** `bash install_porter.sh`, also run at the end of `pi5_autoconfig.sh`:
- installs the unit as `/etc/systemd/system/dji-vcom-gadget.service`, with `ExecStart` set to the script's real location (`/bin/bash <porter>/modules/lager/modules/Payload-SDK/tools/dji_vcom_gadget/dji_vcom_gadget.sh`), so the path in the template doesn't matter,
- enables it, so it runs at every boot,
- starts it right away if the USB-C port is already in device mode (`/sys/class/udc` not empty). On a first install that's the case only after the reboot that applies `config.txt`, so the gadget starts at the next boot.

Check it with:
```sh
systemctl status dji-vcom-gadget.service
ls -l /dev/ttyGS0
```

To install it by hand instead (outside porter), set `ExecStart` in the unit to the script's location first, then:
```sh
sudo cp tools/dji_vcom_gadget/dji-vcom-gadget.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dji-vcom-gadget.service
```

## 4. Configuration (Raspberry Pi sample)

The Raspberry Pi samples are in `samples/sample_c/platform/linux/raspberry_pi/` (C) and `samples/sample_c++/platform/linux/raspberry_pi/` (C++). Both use the same three files with POLOCALC's settings, set identically in both samples.

**`hal/hal_uart.h`: serial devices.** The E-Port link goes through the USB serial device created by the gadget (section 3), not a hardware UART:
```c
#define LINUX_UART_DEV1    "/dev/ttyGS0"   // USB VCOM to the E-Port (sample default: /dev/ttyAMA1)
#define LINUX_UART_DEV2    "/dev/null"     // second UART not used (sample default: /dev/ttyAMA2)
```

In the same file, the USB IDs of the serial device the drone sees. The SDK sends them to the drone when the payload registers. They must match the device the drone actually sees, here the gadget's DJI VCOM IDs (section 3.2):
```c
#define USB_UART_CONNECTED_TO_UAV_VID (0x2CA3)   // DJI VCOM (sample default: 0x0403, FTDI FT232)
#define USB_UART_CONNECTED_TO_UAV_PID (0xF002)   // (sample default: 0x6001)
```
With the wrong IDs, the drone is identified (the log shows "Matrice 400 Series") but never answers the registration. The log then shows `Command async send retry ... 0x1300->0x1401` 20 times, then `add device sync send error`, then `Payload negotiate timeout`, and DJI Assistant 2 shows no payload. If you connect through a USB-to-serial cable instead of the USB-C gadget, use that cable's IDs: `0x0403:0x6001` for an FTDI FT232, or `0x10C4:0xEA60` for a CP2102.

**`application/dji_sdk_config.h`: connection type.** Serial only, over that VCOM device; no USB bulk or network link:
```c
#define CONFIG_HARDWARE_CONNECTION         DJI_USE_ONLY_UART
```
(the C sample's default is `DJI_USE_UART_AND_NETWORK_DEVICE`).

**`application/dji_sdk_app_info.h`: the registered application (section 1).**
```c
#define USER_APP_NAME               "polocalc"
#define USER_APP_ID                 "xxxxxx"
#define USER_APP_KEY                "<App Key of the registered application>"
#define USER_APP_LICENSE            "<App License of the registered application>"
#define USER_DEVELOPER_ACCOUNT      "developer.account"
#define USER_BAUD_RATE              "115200"
```
The App Key and App License are secret. They are kept in the team's password store, not in this repository, so that they can't leak if the repository (or a copy of it) is public. Paste them into the local copy of the file before building.

To keep the credentials out of git while still editing the file locally:
```sh
git update-index --skip-worktree \
    samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_app_info.h \
    samples/sample_c++/platform/linux/raspberry_pi/application/dji_sdk_app_info.h
```
Undo this with `--no-skip-worktree` before pulling an upstream change to those files.

## 5. Build

On the Raspberry Pi (aarch64). The samples need `cmake`, a C/C++ compiler, `libusb-1.0` and `libopus`; porter's `pi5_autoconfig.sh` installs all of them, building `libopus` from source.

In this fork, the top-level `CMakeLists.txt` builds the **Raspberry Pi** samples (upstream builds the Manifold 3 ones), so build from the repository root:
```sh
mkdir -p build && cd build
cmake ..
make -j4
```
This produces `dji_sdk_demo_on_rpi` (C), `dji_sdk_demo_on_rpi_cxx` (C++) and `psdk_telemetry`, the POLOCALC telemetry bridge for lager (see [its README](samples/sample_c/platform/linux/raspberry_pi_telemetry/README.md)), linked with the precompiled PSDK library for aarch64 (`psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a`).

To build only the C sample, run the same commands from `samples/sample_c/platform/linux/raspberry_pi/` instead.

## 6. Run

With the gadget active (`/dev/ttyGS0` present) and the Pi connected to the E-Port:
```sh
sudo ./build/bin/dji_sdk_demo_on_rpi          # C sample (from the repository root)
sudo ./build/bin/dji_sdk_demo_on_rpi_cxx      # or the C++ sample
```
On the first connection, activate the payload in DJI Assistant 2 (section 2).

## 7. Troubleshooting

| Symptom | Check |
|---|---|
| `ls /sys/class/udc` is empty | `dtoverlay=dwc2,dr_mode=peripheral` in `config.txt`, then reboot |
| `/dev/ttyGS0` missing | `systemctl status dji-vcom-gadget.service`, and `journalctl -u dji-vcom-gadget -b` for the script's errors; run `bash install_porter.sh` again to reinstall the unit |
| The drone is identified ("Matrice 400 Series"), then `add device sync send error` and `Payload negotiate timeout`; DJI Assistant 2 shows no payload | `USB_UART_CONNECTED_TO_UAV_VID/PID` in both `hal_uart.h` must be `0x2CA3`/`0xF002` for the USB-C gadget (section 4); rebuild after changing them |
| `/dev/ttyGS0 is in use by process ...`, or a connection that worked stops working | another PSDK program still holds the port, often one paused with Ctrl+Z: `pkill -f "psdk_telemetry\|dji_sdk_demo"`, then power-cycle the drone. Stop programs with Ctrl+C. |
| The drone doesn't see the payload | the USB-C cable carries data (not a charge-only cable); the USB IDs in the gadget script (`0x2CA3:0xF002`) |
| Authentication or activation errors | the four app values in `dji_sdk_app_info.h` match the registered application; the payload is activated in DJI Assistant 2 with the same account |
| Random resets or slow operation | undervoltage: `vcgencmd get_throttled` (`0x0` = none). On the Pi 5 the USB-C connector is also the power input. |

## Updating from upstream

To bring in a newer DJI release:
```sh
git remote add upstream https://github.com/dji-sdk/Payload-SDK   # once
git fetch upstream
git merge upstream/master
```
Then rebuild, and update the submodule pointers: in lager (`git add modules/Payload-SDK`), then in porter (`git add modules/lager`).

## License

The Payload SDK is DJI's software, distributed under the terms in `LICENSE.txt` and `EULA.txt`. The additions in `tools/dji_vcom_gadget/` and this README belong to the POLOCALC project.
