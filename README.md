# actuator-driver

ESP32 based actuator driver. It controls hardware actuators and reports status to
the actuator controller.

This project also acts as a **Bluetooth Classic (BTC) host for a Sony DualSense
controller** — that is the input path feeding the actuator logic. Bluetooth is
enabled in dual mode (Classic + BLE), with the focus on Classic BT because that
is what gives the DualSense its full HID report (analog sticks, triggers,
rumble, lightbar).

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
```

The portable core (`dualsense_*` files except `esp32/`) has **no ESP-IDF or RTOS
includes**, so it can move to another MCU unchanged — you implement
`dualsense_transport_ops_t` on the new stack and reuse the rest.

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

- The DualSense Classic-BT report byte offsets in
  `dualsense_report.c` are **best-effort** and need validation against a real
  controller. Raise the log level to debug to see raw input reports; adjust the
  `IN_*` constants to match.
- Device discovery matches the name prefix `Wireless Controller`
  (`CONFIG_DUALSENSE_PEER_NAME`). The DualShock 4 uses the same name, so tighten
  this if you need to distinguish controllers (VID 0x054C, PID 0x0CE6 = DualSense).
- Touchpad / motion (gyro/accel) decoding is not implemented yet.
