# Kilin sbRIO FPGA Driver

`fpga_driver` is the userspace driver for the Kilin robot's NI sbRIO FPGA.
It opens the FPGA bitfile on `RIO0`, services CAN hip motors and RS-485 limb
modules, exchanges motor/power state over the gRPC core topics, and provides a
local ncurses console for supervised bring-up.

## Build and start

Build on the sbRIO image (or a compatible development environment with the
NI-RIO, gRPC/protobuf, yaml-cpp, Eigen, and ncurses dependencies installed).
The usual workspace workflow configures against the local `install` prefix:

```sh
cd build
cmake .. \
  -DCMAKE_PREFIX_PATH="$HOME/kilin_sbRIO_ws/install" \
  -DCMAKE_INSTALL_PREFIX="$HOME/kilin_sbRIO_ws/install" \
  -DOPENSSL_ROOT_DIR="$HOME/kilin_sbRIO_ws/install/ssl"
make -j16
make install
./fpga_driver
```

Run `sudo make install` only if the selected install prefix requires elevated
permissions. After the first configuration, the usual incremental rebuild is
simply `cd build && make -j16`.

The configuration is loaded from `config/config.yaml` through the compile-time
`CONFIG_PATH`. Verify the CAN ports, CAN IDs, motor gains, loop periods, and
power-board calibration values before enabling power.

Startup pauses twice for Enter: once before starting the server and once after
hardware/configuration initialization. This is intentional so that hardware
can be checked before the control loops run.

## Console

The program opens an ncurses status display and an input line at the bottom of
the terminal. The display shows power-board state, hip-motor CAN traffic and
feedback, RS-485 limb state, and hip `Diff` feedback. `Diff` is the MU150
encoder angle minus the motor electrical-angle estimate, in degrees.

To enter a command:

1. Press `:`.
2. Type one command, using spaces between fields.
3. Press Enter.

Commands are case-sensitive. Press `e` to leave ncurses display mode and `E`
to refresh it again.

### Switch console pages

Use these single-key shortcuts while the console is waiting for input (before
pressing `:` to type a command):

| Key | Page | Purpose |
| --- | --- | --- |
| `s` | **SAFE MONITOR** | Default page; displays heartbeat/progress only. |
| `d` | **DIRECT DEBUG** | Shows detailed power-board, CAN, and RS-485 panels. |

The default page is **SAFE MONITOR**. It never reads FPGA registers from the
console thread. Instead, it displays main-loop and CAN-loop heartbeat counters
and reports `RUNNING` while either counter advances; it reports `STALE / NOT
PROGRESSING` after both counters stop changing for about one second. This is
the recommended mode during normal operation.

Press `d` to enable **DIRECT DEBUG**, which restores the detailed power, CAN,
and RS-485 panels. That page directly reads FPGA registers and is intended
only for short, supervised diagnosis. Press `s` at any time to return to SAFE
MONITOR.

### Power-board commands

```text
P D <0|1>    Set the digital switch
P S <0|1>    Set the signal switch
P P <0|1>    Set the motor/power switch
```

Example:

```text
:P P 1
```

Setting the power switch to `1` permits CAN and RS-485 motor communication.
Treat this as a hardware-enable operation: ensure the robot is supported,
clear of people, and commanded to a safe/rest state first.

### Hip-motor commands

```text
H <id> A <position_rad>       Set position command
H <id> T <torque>             Set torque command
H <id> P <kp>                 Set proportional gain
H <id> I <ki>                 Set integral gain
H <id> D <kd>                 Set derivative gain
H <id> M <mode>               Request a motor mode transition
```

Motor IDs use the following fixed mapping:

| ID | Motor |
| -- | ----- |
| 0 | left front (LF) |
| 1 | left hind (LH) |
| 2 | right front (RF) |
| 3 | right hind (RH) |

Hip mode letters:

| Letter | Mode |
| ------ | ---- |
| `R` | REST |
| `C` | CONFIG |
| `Z` | SET_ZERO |
| `H` | HALL_CALIBRATE |
| `M` | MOTOR |
| `T` | CONTROL |

Examples:

```text
:H 0 M M
:H 0 P 85
:H 0 D 1.75
:H 0 A 0.20
:H 0 T 0.0
:H 0 M R
```

Mode changes synchronously exchange CAN frames and may take time or fail when
the motor is disconnected. Do not repeatedly submit mode changes.

### RS-485 limb commands

```text
T <module> <S|W> M <mode_number>
T <module> <S|W> P <position>
T <module> <S|W> V <velocity>
T <module> <S|W> T <torque>
```

`module` is the zero-based RS-485 port index: `0` through `3`, corresponding
to RS-485 ports 1 through 4. `S` selects the steering motor; `W` selects the
wheel/hub motor.

Accepted mode numbers are:

| Number | Mode |
| ------ | ---- |
| 0 | REST |
| 1 | CONFIG |
| 2 | SET_ZERO |
| 3 | HALL_CALIBRATE |
| 4 | POSITION |
| 5 | VELOCITY |
| 6 | TORQUE |

Examples:

```text
:T 0 S M 4
:T 0 S P 0.35
:T 0 W M 5
:T 0 W V 100
:T 0 W T 1.5
```

Steering currently implements position control in the RS-485 packet protocol;
use `S P` for steering commands. Wheel position is passed as the controller's
integer position unit, while gRPC position commands are converted separately.

## CAN backlash-angle feedback

Each hip motor returns a normalized 12-bit backlash/angle-difference field in
CAN bytes 6--7. The driver maps `0...4095` linearly to `-3...+3` degrees and
shows the result as `Diff` in the console. CAN byte 7's low nibble remains the
motor FSM state.

## Operational note

The console is a supervised local diagnostic interface. For long unattended
runs, prefer the gRPC command interface and minimize simultaneous local
console interaction with live hardware.
