#!/usr/bin/env python3
"""
Live integration test for trigger-mode button monitoring.

Starts the bridge node with button polling active against the real PLC.
A mock Trigger service is advertised for each button; pressing any
panel button causes the corresponding service to be called exactly once.

Usage (from the workspace root):
    source .venv/bin/activate && source install/setup.bash
    python3 src/agrobot_plc_bridge/scripts/live_button_test.py [--wait SECONDS]
"""

import argparse
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from agrobot_plc_bridge.opcua_client import (
    DEFAULT_CERT_DIR,
    OpcUaClient,
    OpcUaConfig,
)
from agrobot_plc_bridge.plc_bridge import BUTTON_NAMES, PlcBridgeNode


# ---------------------------------------------------------------------------
# Config — matches config/plc_bridge.yaml
# ---------------------------------------------------------------------------

def _make_config() -> OpcUaConfig:
    return OpcUaConfig(
        endpoint  = 'opc.tcp://172.16.0.151:4840',
        username  = 'admin',
        password  = 'botbotbot',
        cert_file = DEFAULT_CERT_DIR / 'client_cert.der',
        key_file  = DEFAULT_CERT_DIR / 'client_key.pem',
        output_node_ids = {
            0: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.RunningTowerLight_Mode',
            1: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.FaultTowerLight_Mode',
            2: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.AutoPanelLight_Mode',
            3: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.CyclePanelLight_Mode',
            4: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.StepPanelLight_Mode',
            5: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.PrevPanelLight_Mode',
            6: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.HaltPanelLight_Mode',
            7: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.SlowPanelLight_Mode',
            8: 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.Buzzer_Mode',
        },
        button_node_ids = {
            'slow':  'ns=6;s=Arp.Plc.Eclr/ExposeIO1.SlowButton',
            'emote': 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.EmoteButton',
            'halt':  'ns=6;s=Arp.Plc.Eclr/ExposeIO1.HaltButton',
            'auto':  'ns=6;s=Arp.Plc.Eclr/ExposeIO1.AutoButton',
            'cycle': 'ns=6;s=Arp.Plc.Eclr/ExposeIO1.CycleButton',
            'step':  'ns=6;s=Arp.Plc.Eclr/ExposeIO1.StepButton',
            'prev':  'ns=6;s=Arp.Plc.Eclr/ExposeIO1.PrevButton',
        },
        trigger_rearm_timeout = 5.0,
    )


# ---------------------------------------------------------------------------
# Mock downstream services
# ---------------------------------------------------------------------------

class MockServiceNode(rclpy.node.Node):
    """Advertises a Trigger service for every button and logs calls."""

    def __init__(self, call_log: dict):
        super().__init__('mock_trigger_services')
        self._call_log = call_log
        for name in BUTTON_NAMES:
            self.create_service(
                Trigger,
                f'/agrobot/plc/{name}',
                lambda req, resp, n=name: self._cb(req, resp, n),
            )

    def _cb(self, req, resp, name: str):
        self._call_log[name].append(time.monotonic())
        self.get_logger().info(f'[mock] {name} called')
        resp.success = True
        resp.message = 'ack'
        return resp


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait', type=float, default=15.0,
                        help='Seconds to wait for button presses (default: 15)')
    args = parser.parse_args()

    rclpy.init()

    call_log: dict[str, list] = {n: [] for n in BUTTON_NAMES}
    mock_node = MockServiceNode(call_log)

    config = _make_config()
    opcua  = OpcUaClient(config)
    bridge = PlcBridgeNode(opcua_client=opcua)

    # Wire callbacks before starting the client
    for name in BUTTON_NAMES:
        client = bridge.create_client(Trigger, f'/agrobot/plc/{name}')
        opcua.set_button_callback(
            name,
            bridge._make_trigger_callback(name, f'/agrobot/plc/{name}', client),
        )

    opcua.start()

    executor = MultiThreadedExecutor()
    executor.add_node(mock_node)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f'\nNode started.  Press any button on the panel (waiting {args.wait:.0f} s)...\n')
    try:
        time.sleep(args.wait)
    except KeyboardInterrupt:
        print('\n(interrupted)')

    executor.shutdown()
    bridge.destroy_node()
    mock_node.destroy_node()
    rclpy.shutdown()

    # Report
    print('\n--- Button call log ---')
    fired = {n: len(v) for n, v in call_log.items() if v}
    if not fired:
        print('  No buttons were pressed.')
    else:
        for name, count in fired.items():
            print(f'  {name}: called {count} time(s)')
    print()


if __name__ == '__main__':
    main()
