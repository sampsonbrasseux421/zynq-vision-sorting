# Host ↔ board interface

## 1. Transport

| Item | Value |
|---|---|
| Protocol | TCP, one command per connection, line terminated by `\n` |
| Port | 5000 (commands), 5002 (video) |
| Encoding | ASCII |

A command is a single line. The board replies with one or more lines and closes
the connection. The host treats `ERR` / `ERROR` prefixed lines as failures.

## 2. Commands

| Command | Reply | Meaning |
|---|---|---|
| `PING` | `PONG` | liveness |
| `<X> <Y> <Z>` | `OK GOTO X Y Z` | move the tool tip to a point, mm |
| `HOME` | `OK HOME` | fold all axes, re-enable drivers |
| `BOX <deg256>` | `OK BOX <deg256>` | rotate the base to a box angle, arm folded |
| `BOX <deg256> PLACE` | `OK PLACE X Y Z` | descend to the placement point at that angle |
| `LIFT` | `OK LIFT X Y Z` | raise vertically from the last box position |
| `FOLD` | `OK FOLD <deg256>` | fold the arm at the current box angle |
| `READY` | `OK READY X Y Z` | go to the standby pose |
| `CLAW <us>` | `OK CLAW <us>` | gripper pulse width, 500–2500 µs |
| `WAIT` | `OK ARRIVED` / `OK TIMEOUT` | wait for the frame burst to finish |
| `GAP <ms>` | `OK GAP <ms>` | set the inter-frame delay |
| `JOG ON` / `JOG OFF` | `OK JOG ON/OFF` | skip the enable burst on subsequent moves |
| `OSD <text>` | (empty) | write up to 6 lines to the overlay, `|` separated |

## 3. Register map

See `skill/pl-cmd-engine/register_map.md`.

## 4. Timing contract

| Item | Value |
|---|---|
| Command round trip (full) | ~1.05 s (17 frames × 60 ms) |
| Command round trip (fast) | ~0.22 s (5 frames × 60 ms) |
| Position resolution | 1 pulse; 418,400 pulses per base revolution |
| Angle unit | deg256 (360° = 92,160) |

The host must not send a second motion command before the first returns: two
concurrent `ik_zdt` processes would write the same registers.
