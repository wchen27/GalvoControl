# GalvoCam Target Streaming Protocol (v1)

One-way UDP protocol for streaming 3D target coordinates from a sender (the Linux
triangulation machine) to the GalvoCam motor-control app, which aims the mirrors at the point.

## Transport
- **UDP**, IPv4. Fire-and-forget: no ACK, no connection, no retransmit.
- Default **port 5005** (configurable in the app UI).
- One datagram = one complete target. **Latest wins**; lost or reordered packets are harmless
  (the next update supersedes them).

## Link / addressing
Direct Ethernet cable between the two machines; static IPs on a private /24 (no DHCP), e.g.:
- Windows (receiver / motor-control app): **192.168.50.1**
- Linux (sender / triangulation):         **192.168.50.2**

The sender transmits to the receiver's IP on port 5005. The app binds `INADDR_ANY:5005`.

## Packet — 48 bytes, little-endian, no padding
| off | size | type    | field         | meaning |
|-----|------|---------|---------------|---------|
| 0   | 4    | char[4] | magic         | `'G','C','T','1'` |
| 4   | 2    | uint16  | version       | protocol version = **1** |
| 6   | 2    | uint16  | flags         | bit0 = **target valid**; other bits reserved (0) |
| 8   | 4    | uint32  | seq           | monotonic sequence number (wraps at 2^32) |
| 12  | 4    | uint32  | target_id     | GalvoCam/target id (**0** = the single unit) |
| 16  | 8    | uint64  | timestamp_ns  | sender timestamp, ns (informational) |
| 24  | 8    | float64 | x             | world X, **millimeters** |
| 32  | 8    | float64 | y             | world Y, millimeters |
| 40  | 8    | float64 | z             | world Z, millimeters |

- **Little-endian** on the wire (both machines are x86-64 LE → no byte-swapping).
- Send exactly 48 bytes. Datagrams that aren't 48 bytes, or whose magic/version don't match,
  are ignored by the receiver.

## Semantics
- **Coordinate frame:** world coordinates (the GalvoCam applies base pose + world→angle calibration).
- **Units:** millimeters. Direction is scale-invariant for pointing; units matter later for focus.
- **seq:** increment by 1 per packet. The receiver keeps the highest seq seen and ignores older
  packets (handles out-of-order UDP; wraparound handled via signed 32-bit difference).
- **flags bit0 (valid):** `1` = a real triangulated target → the receiver aims at it. `0` = no
  valid target this frame (e.g. tracking lost) → the receiver **holds** (does not move).
- **timestamp_ns:** sender clock in ns (monotonic recommended). Informational — clocks are not
  synchronized across the link; useful for the sender's own logging / gap detection.
- **target_id:** `0` for the current single GalvoCam. Reserved so the same protocol scales to a
  6–8 unit array (each unit filters on its id).

## Rate
Send at your source rate (e.g. camera fps). The receiver aims at ~30 Hz using the latest packet,
so any rate up to ~1 kHz is fine and higher gives no benefit. Don't flood the link.

## Versioning
Bump `version` for incompatible changes. The receiver validates magic + version and ignores
mismatches, so a version skew fails safe (no motion) rather than misinterpreting bytes.

---

# Control Channel (GCC1 / GCS1)

Request/reply UDP channel on **port 5006** (configurable), used by orange to query
status, command raw mirror angles, toggle calib mode, and upload fitted calibration
parameters. The GCT1 target stream above is unchanged. Little-endian, packed, fixed
sizes; datagrams with the wrong size/magic/version are ignored (fail-safe).

The sender retries a request (same `seq`) after ~250 ms, up to 4 tries. Commands are
idempotent, so duplicate delivery is harmless. Every reply carries full status.

## GCC1 command (orange -> GalvoControl), 128 bytes
| off | size | type    | field    | meaning |
|-----|------|---------|----------|---------|
| 0   | 4    | char[4] | magic    | `'G','C','C','1'` |
| 4   | 2    | uint16  | version  | = **1** |
| 6   | 2    | uint16  | cmd      | see below |
| 8   | 4    | uint32  | seq      | echoed in the reply |
| 12  | 4    | uint32  | reserved | 0 |
| 16  | 112  | f64[14] | args     | per-command |

| cmd | name        | args |
|-----|-------------|------|
| 0   | PING        | - (status query; keepalive for calib mode) |
| 1   | SET_ANGLES  | `[0]`=pan, `[1]`=tilt - displayed motor degrees; clamped to travel limits (err=4 if clamped) |
| 2   | CALIB_MODE  | `[0]` = 0/1 |
| 3   | SET_CALIB   | `[0..2]`=base xyz mm, `[3..5]`=rot Euler XYZ deg (R=Rz*Ry*Rx, world->galvo), `[6..8]`=pan sign/scale/offset, `[9..11]`=tilt sign/scale/offset |
| 4   | SAVE_CONFIG | - (persist current config to motor_control.cfg) |
| 5   | STOP        | - (abort the pan/tilt axes; always allowed) |

## GCS1 reply (GalvoControl -> orange), 96 bytes
| off | size | type    | field     | meaning |
|-----|------|---------|-----------|---------|
| 0   | 4    | char[4] | magic     | `'G','C','S','1'` |
| 4   | 2    | uint16  | version   | = **1** |
| 6   | 2    | uint16  | cmd       | echoed |
| 8   | 4    | uint32  | seq       | echoed |
| 12  | 4    | uint32  | flags     | bit0 ok, bit1 in_position, bit2 calib_mode, bit3 motors_enabled, bit4 remote_allowed |
| 16  | 4    | int32   | err       | 0 ok, 1 bad args, 2 remote disabled, 3 not connected, 4 clamped |
| 20  | 4    | uint32  | reserved  | 0 |
| 24  | 8    | f64     | pan_deg   | current, displayed frame |
| 32  | 8    | f64     | tilt_deg  | current, displayed frame |
| 40  | 32   | f64[4]  | limits    | pan_min, pan_max, tilt_min, tilt_max (deg) |
| 72  | 24   | f64[3]  | reserved  | 0 |

## Semantics
- **Remote gating:** motion/config commands require the app's "allow remote control"
  checkbox; refused with err=2 otherwise. PING and STOP always work.
- **Calib mode:** while on, GCT1 target aiming and the test circle are suppressed so
  remote SET_ANGLES moves aren't fought. Auto-expires after **5 s** without control
  traffic (the sender's 2 Hz status poll is the keepalive) - a dead sender can't
  leave the unit paused.
- **in_position:** both targeting axes IDLE with amps enabled.
- **Angles** are in the displayed motor frame (raw minus the persisted zero_ref) -
  the same frame shown in the UI and used by the calibration fit.
