# Hardware setup

## 1. Bill of materials

| Item | Qty | Note |
|---|---|---|
| Zynq-7020 board (XC7Z020CLG400-2) | 1 | 50 MHz FCLK, PetaLinux 2023.1 |
| ZDT X42S closed-loop stepper driver | 4 (5 installed) | addresses 1–4 used; the 5th does not participate in motion |
| Stepper motor with encoder | 5 | base, upper arm, forearm A, forearm B, wrist |
| Gripper servo | 1 | 8 V supply, 50 Hz PWM |
| USB camera | 1 | 1280 × 720 |
| HDMI display | 1 | 1024 × 600 |
| Conveyor | 1 | 220 V, ~9.45 mm/s |
| 24 V PSU | 1 | motor drivers |

## 2. Power

| Rail | Load |
|---|---|
| 24 V | 4 × ZDT drivers |
| 8 V | gripper servo |
| 5 V | board, camera, display |

All control grounds are common. Motor power is kept separate from the board's
low-voltage rail.

## 3. Wiring

| Signal | Pin | Standard |
|---|---|---|
| UART TX to drivers | K14 | LVCMOS33 |
| UART RX from drivers | J14 | LVCMOS33 |
| Servo PWM | N15 | LVCMOS33 |
| HDMI | board connector | — |
| Ethernet | board connector | — |

The UART is **TTL level (3.3 V)**, not RS485.

## 4. Driver addressing

| Address | Joint |
|---|---|
| 1 | base |
| 2 | upper arm |
| 3 | forearm |
| 4 | wrist |

Addresses must match `/etc/ik.conf` and the `addr[]` table in `src/ps/ik_zdt.c`.
