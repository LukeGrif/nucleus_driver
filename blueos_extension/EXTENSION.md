# Nucleus DVL BlueOS extension with mounting rotation

An unofficial BlueOS extension for the Nortek Nucleus. It is rebuilt from the original Nortek Python
extension (removed from this repository in commit `56d6ce1`) and structured like the Water Linked DVL
extension. It adds a configurable mounting rotation, so the Nucleus can be mounted tilted instead of
only pointing straight down.

It is **not** the extension Nortek publishes in the BlueOS store (see [README.md](README.md) for that
one). Do not run both at the same time.

## What it does

* Connects to the Nucleus over TCP with the `nucleus_driver` library (port 9000, or 9002 for "stream only").
* On Start (not in stream-only mode) it sends the same setup as the original extension:
  `SETAHRS,DS="ON"` and `SETBT,WT="ON",DS="ON"`, then starts the measurement.
* Reads bottom track (`0xb4`), AHRS (`0xd2`) and altimeter (`0xaa`) packets.
* Sends to ArduSub through BlueOS mavlink2rest:
  * `VISION_POSITION_DELTA` (default, as before): velocity × dt and the AHRS roll/pitch/heading deltas, or
  * `VISION_SPEED_ESTIMATE`: the velocity, and
  * optionally `DISTANCE_SENSOR` from the altimeter (off by default; needs `RNGFND1_TYPE` = MAVLink, and the
    altimeter enabled on the Nucleus, e.g. `SETALTI,DS="ON"` plus a `SETTRIG,ALTI=...` setting, see the manual).

The ArduSub parameters in [README.md](README.md) (controller and PID parameters) still apply.

## Mounting rotation

Both frames are FRD (x forward, y right, z down).

* `mount_pitch_deg`: positive = DVL tilted **nose-up** relative to the vehicle.
  0° = pointing down (the normal mounting), 90° = pointing forward.
* `mount_roll_deg`: positive = DVL rolled **right side down**.
* Both must be finite and within [-180, 180].

```
R = Ry(pitch) @ Rx(roll)
v_vehicle = R @ v_dvl

Ry(p) = [[ cos p, 0, sin p],     Rx(r) = [[1,     0,      0],
         [     0, 1,     0],              [0, cos r, -sin r],
         [-sin p, 0, cos p]]              [0, sin r,  cos r]]
```

The rotation is applied to:

* the bottom-track velocity (both message types),
* the angle deltas in `VISION_POSITION_DELTA` (`R @ [droll, dpitch, dyaw]`),
* the altimeter range, projected onto the vehicle's down axis: `range × R[2][2]`. The raw range must be
  valid (flag set, finite, positive) before it is projected, and nothing is sent when the DVL is not
  looking down (`R[2][2] <= 0`, e.g. 90° pitch) or the result is 5 cm or less.

cos/sin are exact at multiples of 90°, so the presets (0°, 90°) reproduce the unrotated output and the
plain axis swap bit for bit.

The maths is in `nucleus_extension/mount_rotation.py` (no hardware or network dependencies). The code
that applies it asks `MountAngles.matrix_at(timestamp)` with the measurement's own Nucleus timestamp. A
live angle source (e.g. a gimbal) can be plugged in with `NucleusLink.set_angle_source(fn)`, where
`fn(timestamp) -> (pitch_deg, roll_deg)` or `None`; it overrides the fixed setting. If it returns `None`
the measurement is not sent (it does not fall back to the fixed angle). The timestamps are the
Nucleus clock (POSIX seconds), so a gimbal source must use the same time base.

### Rotation on the Nucleus itself

The Nucleus has its own `SETINST,ROTXY/ROTYZ/ROTXZ` settings. It is not yet confirmed whether these
also rotate the bottom-track velocities. **Keep them at 0 while using this extension's rotation**, so
the rotation cannot be applied twice. The extension reads them (`GETINST`) on Start, shows them on the
page, and warns if they are non-zero while a mount angle is set.

## Web page and REST API

The page has Status (connect / disconnect / start / stop), Connection and output settings, a
**Mounting** section (pitch, roll, Apply, presets Down = 0°, 30°, 45°, Forward = 90°), live raw vs.
rotated values, and message counters.

| Method | Path | Body / result |
| --- | --- | --- |
| GET | `/api/status` | state, counters, live raw (DVL frame) and rotated (vehicle frame) values, device rotation |
| GET | `/api/settings` | saved settings |
| POST | `/api/settings` | partial update, e.g. `{"message_type": "SPEED_ESTIMATE", "rangefinder": true}` |
| POST | `/api/mount` | `{"mount_pitch_deg": 30, "mount_roll_deg": 0}` |
| POST | `/api/connect`, `/api/disconnect`, `/api/start`, `/api/stop` | `{"ok": true, "status": "..."}` |

Invalid values get HTTP 400 with an `error` message and leave the setting unchanged.

## Settings

Saved in `/root/.config/nucleus_mount/settings.json` (bind-mounted from the host, so they survive
restarts and updates): `hostname`, `stream_only`, `autoconnect`, `autostart`, `enabled`, `message_type`,
`rangefinder`, `mount_pitch_deg`, `mount_roll_deg`.

On first start the hostname is imported from the original extension's `~/.config/nucleus/settings.json`
if it exists (that file is only read, never written). The original extension had no orientation
setting, so there is nothing else to convert; missing mount angles load as 0° (pointing down).

## Development

```
cd blueos_extension
pip install -r requirements-dev.txt
python -m pytest
black --check . && isort --check-only . && pylint nucleus_extension
```

CI (`.github/workflows/blueos_extension_ci.yml`) runs lint, tests and a Docker build as separate jobs.
The deploy workflow (`.github/workflows/blueos_extension_deploy.yml`) pushes
`<DOCKER_USERNAME>/blueos-nucleus-mount:<branch or tag>` for arm/v7, arm64 and amd64 on pushes to `main`,
on version tags (e.g. `0.1.0`, which also pushes `latest`) and manually. It needs the repository
secrets `DOCKER_USERNAME` and `DOCKER_PASSWORD` (a Docker Hub access token); without them it is skipped.

## Installing on BlueOS as a custom extension

1. Disable the store version first: **Extensions → Installed → Nortek Nucleus → Disable** (or uninstall).
   Two drivers would both feed the EKF and compete for the Nucleus connection.
2. **Extensions → Installed → "+"** (if there is no "+" button, enable Pirate Mode in the BlueOS settings).
3. Fill in:
   * Extension identifier: `lukegrif.nucleus-mount`
   * Extension name: `Nucleus DVL (mount rotation)`
   * Docker image: `<your Docker Hub user>/blueos-nucleus-mount`
   * Docker tag: `main` (or a version tag)
   * Custom settings (from the `permissions` label in the Dockerfile):

```json
{"ExposedPorts":{"8000/tcp":{}},"HostConfig":{"Binds":["/root/.config:/root/.config"],"ExtraHosts":["host.docker.internal:host-gateway"],"PortBindings":{"8000/tcp":[{"HostPort":""}]}}}
```

4. Open it from the BlueOS menu, set the Nucleus IP (e.g. `192.168.2.201`), Connect, Start.
