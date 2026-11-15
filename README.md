# actuator-driver

ESP32 based actuator driver. It controls hardware actuators and reports status to
the actuator controller.

This project also acts as a **Bluetooth Classic (BTC) host for a Sony gamepad**
— that is the input path feeding the actuator logic. Bluetooth is enabled in
dual mode (Classic + BLE), with the focus on Classic BT because that is what
gives the controllers their full HID report (analog sticks, triggers, rumble,
lightbar). Two controllers are supported, selected at build time:

- **Sony DualSense** (PS5) — `services/dualsense/`
- **Sony DualShock 4** (PS4) — `services/dualshock4/`

## Hardware target

- **ESP32-WROOM-32D** (`esp32` target in ESP-IDF). Classic BT requires a
  dual-mode chip; the ESP32-S3 is BLE-only and cannot do this.

## Layout

```
main/
  main.c                          # short: bring-up + wire the service to callbacks
  services/
    bluetooth/                    # generic BT stack + HID-host service (ESP-specific)
      bluetooth_service.[ch]
    dualsense/                    # DualSense controller support
      dualsense_types.h           # portable: state/output structs + report sizes
      dualsense_transport.[ch]    # portable: transport vtable + sink callbacks
      dualsense_report.[ch]       # portable: input/output report codec
      dualsense_service.[ch]      # portable: glue transport -> codec -> app callbacks
      esp32/                      # platform backend (only this dir may include ESP-IDF BT headers)
        dualsense_esp32_btc.[ch]
    dualshock4/                   # DualShock 4 controller support (same shape)
      dualshock4_types.h
      dualshock4_transport.[ch]
      dualshock4_report.[ch]
      dualshock4_service.[ch]
      esp32/
        dualshock4_esp32_btc.[ch]
```

Each controller has a portable core (all files except `esp32/`) with **no
ESP-IDF or RTOS includes**, so it can move to another MCU unchanged — you
implement its `*_transport_ops_t` on the new stack and reuse the rest.

## Choosing a controller

`idf.py menuconfig` → `Gamepad` → `Gamepad model`, then pick DualSense or
DualShock 4. Each model has its own Bluetooth device-name prefix used for
discovery:

- DualSense: `DualSense Wireless Controller`
- DualShock 4: `Wireless Controller`

The scan connects to the first device whose name starts with that prefix, so
only enable the model you actually have in range.

## Build (Ubuntu WSL)

```sh
source ~/.espressif/tools/activate_idf_v6.1.sh
idf.py build
```

The target (`esp32`) and Bluetooth configuration are pinned in
`sdkconfig.defaults`. A custom `partitions.csv` provides a 1700K app partition;
Bluedroid (Classic + BLE) does not fit in the default ~1 MB app partition.

Flash (port differs by machine):

```sh
idf.py -p /dev/ttyUSB0 flash monitor
```

## On Windows

1. Activate the ESP environment: `Windows + IDF_v6.1_Powershell`
2. Navigate to `build/`, e.g. `cd "\\wsl.localhost\Ubuntu-22.04\home\tomi\projects\actuator-driver\build"`
3. Flash:

```
python -m esptool --chip esp32 -p COM6 -b 460800 write-flash '@flash_args'
```

## Notes / TODO

- The DualSense and DualShock 4 report offsets are verified on hardware and match
  the Linux kernel `drivers/hid/hid-playstation.c`. Touchpad / motion (gyro/accel)
  decoding is not implemented yet.
- The DualShock 4 output report includes a CRC32 the controller validates
  (computed in `dualshock4_report.c`) and a BT poll-interval field in `hw_control`
  that must be non-zero (4 ms) to switch the controller from the minimal 0x01
  report to the full 0x11 report.
- Both controllers drop the Bluetooth link after ~20 s without periodic output
  reports; `main.c` keeps the connection alive with a 2 s output keepalive, and
  input logging is throttled to once per 5 s so the 250 Hz report stream doesn't
  starve the BT stack.
