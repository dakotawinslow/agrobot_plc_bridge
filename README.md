# agrobot_plc_bridge

ROS2 ↔ OPC UA bridge node for the agrobot physical operator panel.

Connects to a **Phoenix Contact AXC F 2152 PLCnext** PLC over OPC UA
(Basic256Sha256 / SignAndEncrypt) and exposes panel outputs (lights, buzzer)
as ROS2 services and panel buttons as outgoing ROS2 service calls.

```
         ┌─────────────┐     SetOutputs      ┌───────────────────┐
         │  ROS2 nodes  │ ─────────────────► │                   │
         │  (your code) │     QueryOutputs   │  PlcBridgeNode    │   asyncua
         │              │ ◄───────────────── │  (this package)   │ ◄────────► PLC
         │              │                    │                   │
         │  Trigger /   │ ◄── button press ─ │   poll @ 20 Hz   │
         │  ButtonState │                    └───────────────────┘
         └─────────────┘
```

## Hardware

- PLC: Phoenix Contact AXC F 2152 PLCnext
- Default endpoint: `opc.tcp://172.16.0.151:4840`
- Security: Basic256Sha256 / SignAndEncrypt (self-signed client cert, auto-generated)
- Panel: 7 buttons, 8 lights, 1 buzzer (9 addressable outputs)

## Dependencies

- ROS2 Jazzy
- [`agrobot_plc_bridge_msgs`](https://github.com/dakotawinslow/agrobot_plc_bridge_msgs)
- `asyncua` ≥ 1.1.8 — `pip install asyncua` (or `uv pip install asyncua`)
- `std_srvs`

## Setup

```bash
# Create workspace
mkdir -p ~/opcua_ws/src && cd ~/opcua_ws

# Clone packages
git clone https://github.com/dakotawinslow/agrobot_plc_bridge_msgs src/agrobot_plc_bridge_msgs
git clone https://github.com/dakotawinslow/agrobot_plc_bridge      src/agrobot_plc_bridge

# Optional — demo node (lights/buttons showcase):
git clone https://github.com/dakotawinslow/agrobot_plc_bridge_demo src/agrobot_plc_bridge_demo

# Python venv with system-site-packages so rclpy is visible
uv venv --system-site-packages   # or: python3 -m venv --system-site-packages .venv
source .venv/bin/activate
uv pip install asyncua            # or: pip install asyncua

# Copy the workspace build helper (patches entry-point shebangs to use the venv)
cp src/agrobot_plc_bridge/build.sh .

# Build (use build.sh instead of colcon build — see note below)
./build.sh
source install/setup.bash
```

> **Why `build.sh` instead of `colcon build`?**
> colcon's `ament_python` task always generates entry-point scripts with
> `#!/usr/bin/python3`, even when a venv is active.  `build.sh` runs
> `colcon build` and then rewrites those shebangs to point at the project
> venv, so that `ros2 run` and `ros2 launch` find `asyncua` without
> installing it system-wide.  It forwards any extra flags to colcon, so
> `./build.sh --packages-select agrobot_plc_bridge` works as expected.

## Configuration

Edit `config/plc_bridge.yaml` (or supply your own via the `config_file` launch arg).

```yaml
plc_bridge:
  ros__parameters:
    opcua_endpoint: "opc.tcp://172.16.0.151:4840"
    opcua_username: "admin"
    opcua_password: "changeme"      # ⚠ use a secrets manager in production

    trigger_rearm_timeout:  5.0     # seconds before a held trigger re-arms
    state_change_debounce:  0.1     # dead-time between state_change calls (s)

    output_nodes:
      running_tower: "ns=6;s=Arp.Plc.Eclr/ExposeIO1.RunningTowerLight_Mode"
      # ... (see config/plc_bridge.yaml for full list)

    button_nodes:
      halt: "ns=6;s=Arp.Plc.Eclr/ExposeIO1.HaltButton"
      # ...

    buttons:
      halt:
        mode: trigger               # trigger | state_change | disabled
        service: /agrobot/plc/halt  # Trigger service to call on press
```

### Button modes

| Mode | Behaviour |
|---|---|
| `trigger` | Calls a `std_srvs/Trigger` service once per press; re-arms after response (or timeout) |
| `state_change` | Calls a `ButtonStateChange` service on every press **and** release, with debounce |
| `disabled` | Button ignored |

## Running

```bash
# Default config (from installed package):
ros2 launch agrobot_plc_bridge plc_bridge.launch.py

# Custom config:
ros2 launch agrobot_plc_bridge plc_bridge.launch.py \
    config_file:=/path/to/my_config.yaml
```

## Services

| Service | Type | Description |
|---|---|---|
| `/agrobot/plc/set_outputs` | `SetOutputs` | Write one or more panel outputs |
| `/agrobot/plc/query_outputs` | `QueryOutputs` | Read all 9 output states |

Example — flash the halt light:
```bash
ros2 service call /agrobot/plc/set_outputs agrobot_plc_bridge_msgs/srv/SetOutputs \
  "{outputs: [{output_id: 6, state: 2}]}"
```

## Scripts

| Script | Purpose |
|---|---|
| `scripts/browse_plc.py` | Discover OPC UA node tree on the PLC |
| `scripts/live_button_test.py` | Interactive 15-second button callback test |
| `scripts/smoke_test.py` | End-to-end SetOutputs/QueryOutputs smoke test |

```bash
# Run smoke test (needs live PLC):
python3 src/agrobot_plc_bridge/scripts/smoke_test.py

# Watch for button presses (30-second window):
python3 src/agrobot_plc_bridge/scripts/live_button_test.py --wait 30
```

## Testing

```bash
python3 -m pytest src/agrobot_plc_bridge/test/ -v
```

63 unit tests covering edge detection, debounce logic, error code
classification, and service handler behaviour — all run without a live PLC.

## Architecture notes

- `OpcUaClient` runs `asyncua` in a **dedicated background thread** with its
  own `asyncio` event loop.  ROS2 service callbacks submit coroutines via
  `asyncio.run_coroutine_threadsafe()` and block until resolved.
- Button callbacks run in a **thread-pool executor** so blocking ROS2 service
  calls don't stall the asyncua event loop.
- If the PLC drops offline, the client detects sustained poll failures
  (default: 5 consecutive), marks itself disconnected, and starts a
  reconnection loop (retries every 5 s).
- `OpcUaClientBase` is an ABC that lets unit tests inject a mock client into
  `PlcBridgeNode` without any live PLC connection.

## Security note

`config/plc_bridge.yaml` stores OPC UA credentials in plaintext — acceptable
for bench/development use.  Before field deployment, replace with environment
variables or a secrets manager and remove credentials from version control.
