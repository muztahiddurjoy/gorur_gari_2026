# rc_car — websocket RC test rig

Test scaffolding, not flight code. Joins wifi, hosts a mobile controller page on
`http://gorurgari.local` and drives the car over a websocket. It exists to shake
out the motor, encoder and steering on a bench before the real MAVLink link
takes over.

## Use it

```bash
pio run -e esp32doit-devkit-v1-rc -t upload   # rc build
pio run -e esp32doit-devkit-v1 -t upload      # normal build, rc car not compiled in
```

Put your network in `wifi_secrets.h` (gitignored, copy `wifi_secrets.example.h`).
Leave it empty and the car brings up its own AP `GorurGari-RC` / `gorurgari`
instead — join that and the controller is on `http://192.168.4.1`.

Hostname, AP name, failsafe and telemetry rate all live in `rc_car_config.h`.

## Protocol

Controller sends at 20 Hz over `/ws`:

```json
{"t":"drive","s":-1..1,"p":-1..1}   // s = steer, p = throttle
{"t":"zero"}                        // reset the odometer
```

Car replies every 100 ms:

```json
{"rpm":123.4,"ticks":5678,"dir":1,"rssi":-52}
```

Nothing is applied straight from the socket callback — that runs on the AsyncTCP
task. Commands are recorded and `update()` writes the hardware from `loop()`.
If commands stop for `RC_FAILSAFE_MS` the car brakes and centres itself.

## Remove it

Dependencies only point inward — `motor_control`, `encoder_reader` and
`steering_control` have never heard of this library. To rip it out:

1. `rm -rf firmware/lib/rc_car`
2. drop the `[env:esp32doit-devkit-v1-rc]` block from `platformio.ini`
3. delete the four `#ifdef ENABLE_RC_CAR` blocks in `src/main.cpp`

The default env never compiles any of this, so until then it costs nothing.
