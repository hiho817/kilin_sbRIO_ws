# kilin_sbRIO_ws

## Overview

This repository provides a driver for FPGA-based systems. It leverages the `yaml-cpp` and `Eigen` libraries for YAML configuration parsing and linear algebra computations, respectively. The project is built on sbRIO and integrates with gRPC.

---

## Workspace Structure

```
kilin_sbRIO_ws/
├── install/
├── kilin_fpga_driver/
│   ├── build/
│   ├── cmake/
│   ├── config/
│   ├── fpga_bitfile/
│   ├── include/
│   ├── log/
│   └── src/
└── third_party/
    ├── eigen/
    ├── grpc_core/
    └── yaml-cpp/
```

---

## Prerequisites

### 1. **yaml-cpp**
```bash
cd kilin_sbRIO_ws/third_party/yaml-cpp/
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/kilin_sbRIO_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/kilin_sbRIO_ws/install
make -j16
sudo make install
```

### 2. **Eigen**
```bash
cd kilin_sbRIO_ws/third_party/eigen/
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/kilin_sbRIO_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/kilin_sbRIO_ws/install
make -j16
sudo make install
```

### 3. **grpc**
For grpc_core installation, refer to the [grpc_core repository](https://github.com/hiho817/grpc_core.git).

---

## Path Configuration

Due to path limitations on sbRIO, certain paths are hard-coded in the code. Update the following files as needed:

1. `fpga_server.hpp` (update `CONFIG_PATH` and `FPGA_PATH`)
2. `config.yaml` (update `log_path`)
3. `NiFpga_FPGA_CANBus_4module_v3_steering.h` (update `NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile`)

---

## Compilation

```bash
cd kilin/kilin_fpga_driver/
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/kilin_sbRIO_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/kilin_sbRIO_ws/install -DOPENSSL_ROOT_DIR=$HOME/kilin_sbRIO_ws/install/ssl
make -j16
```

---

## FPGA Console Interface

This project provides a curses-based console interface for interacting with an FPGA-driven legged robot system. Users can visualize module status in real time and issue control commands via keyboard input.

### Features

- Real-time display of Power Board switch states and voltage/current readings
- Real-time display of motor module status, including CAN ID, position, torque, PID gains, and telemetry
- Live command input for Power Board, FSM mode switching, and motor parameter updates
- Thread-safe operation using `std::mutex` and `std::thread`

---

### Screen Layout

```
+-------------------------+    +------------------------+
| [F] FPGA Server         |    | [L] L_Module           |
| Behavior: TCP_SLAVE     |    | F/H Motor TxRx Status  |
| FSM Mode: MOTOR         |    +------------------------+
+-------------------------+

+-------------------------+    +------------------------+
| [P] Power Board         |    | [R] R_Module           |
| Digital / Signal / PWR  |    | F/H Motor TxRx Status  |
| Voltage / Current ADC   |    +------------------------+
+-------------------------+

+--------------------------------------------------------+
| : [Command Input]                                       |
+--------------------------------------------------------+
```

---

### Key Bindings (Non-Command Mode)

- `r` – Clear and reset the input window
- `e` – Exit ncurses mode (switch to stdout)
- `E` – Re-enter ncurses refresh mode
- `:` – Enter full command input mode

---

### Command Mode Syntax

Enter `:` then type one of the following commands and press Enter.

#### 1. Power Board Control
```
P <D|S|P> <0|1>
  D — Digital switch on/off
  S — Signal switch on/off
  P — Main power switch on/off
```

Example:
```
P D 1   # Turn Digital switch ON
```

#### 2. FSM Mode Switching
```
F M <R|M|S|H>
  R — REST mode
  M — MOTOR mode
  S — SET_ZERO mode
  H — HALL_CALIBRATE mode
```

Example:
```
F M H   # Switch to HALL_CALIBRATE mode
```

### Deprecated Feature Notice

The motor module parameter update functionality has been deprecated. Instead, use the `grpc_core` interface to transmit commands. For more details, refer to the [grpc_core repository](https://github.com/hiho817/grpc_core.git).

### Command Responses

- `Command Send !` – Command executed successfully
- `Syntax Error !` – Invalid command format
- `Switch Mode Timeout !` – FSM mode transition timed out


## coding guideline
should use public and private. put params in private and access them through public functions.
Trailing Underscore for private params.
hope to add doxygen document comments in hpp.
don't capitalize first word of variables and function params, use snake_case instead of cammelCase.
use PascalCase for class and struct.
don't add tail underscore to function input parameters