"""
PLC Bridge node — ROS2 ↔ OPC UA bridge for the agrobot operator panel.

Step 4: SetOutputs wired to the real PLC via OpcUaClient.
The node accepts an optional injected OpcUaClientBase so unit tests can
supply a mock without starting a real OPC UA connection.
"""

from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

from agrobot_plc_bridge_msgs.msg import OutputState
from agrobot_plc_bridge_msgs.srv import ButtonStateChange, QueryOutputs, SetOutputs

from agrobot_plc_bridge.opcua_client import (
    DEFAULT_CERT_DIR,
    OpcUaClientBase,
    OpcUaClient,
    OpcUaConfig,
)

# Maps OutputState output_id constants → YAML parameter names.
# Order matches OutputState constants 0-8.
_OUTPUT_PARAMS: list[tuple[int, str]] = [
    (OutputState.OUTPUT_RUNNING_TOWER, 'output_nodes.running_tower'),
    (OutputState.OUTPUT_FAULT_TOWER,   'output_nodes.fault_tower'),
    (OutputState.OUTPUT_AUTO,          'output_nodes.auto'),
    (OutputState.OUTPUT_CYCLE,         'output_nodes.cycle'),
    (OutputState.OUTPUT_STEP,          'output_nodes.step'),
    (OutputState.OUTPUT_PREV,          'output_nodes.prev'),
    (OutputState.OUTPUT_HALT,          'output_nodes.halt'),
    (OutputState.OUTPUT_SLOW,          'output_nodes.slow'),
    (OutputState.OUTPUT_BUZZER,        'output_nodes.buzzer'),
]

BUTTON_NAMES = ['slow', 'emote', 'halt', 'auto', 'cycle', 'step', 'prev']


class PlcBridgeNode(Node):

    def __init__(self, opcua_client: Optional[OpcUaClientBase] = None):
        super().__init__('plc_bridge')

        if opcua_client is not None:
            # Injected path (tests): skip param declaration and OPC UA connect.
            self._opcua = opcua_client
        else:
            # Normal path: load params, create and connect the real client.
            config = self._declare_and_load_params()
            self._opcua = OpcUaClient(config, logger=self.get_logger())
            self._setup_button_clients()
            self._opcua.start()

        self._set_outputs_srv = self.create_service(
            SetOutputs,
            '/agrobot/plc/set_outputs',
            self._handle_set_outputs,
        )
        self._query_outputs_srv = self.create_service(
            QueryOutputs,
            '/agrobot/plc/query_outputs',
            self._handle_query_outputs,
        )

        self.get_logger().info('PLC Bridge node ready.')

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------

    def _declare_and_load_params(self) -> OpcUaConfig:
        self.declare_parameter('opcua_endpoint', 'opc.tcp://172.16.0.151:4840')
        self.declare_parameter('opcua_username', '')
        self.declare_parameter('opcua_password', '')
        self.declare_parameter(
            'opcua_cert_file',
            str(DEFAULT_CERT_DIR / 'client_cert.der'),
        )
        self.declare_parameter(
            'opcua_key_file',
            str(DEFAULT_CERT_DIR / 'client_key.pem'),
        )
        for _, pname in _OUTPUT_PARAMS:
            self.declare_parameter(pname, '')

        self.declare_parameter('trigger_rearm_timeout', 5.0)
        self.declare_parameter('state_change_debounce', 0.1)
        for name in BUTTON_NAMES:
            self.declare_parameter(f'button_nodes.{name}', '')
            self.declare_parameter(f'buttons.{name}.mode',    'trigger')
            self.declare_parameter(f'buttons.{name}.service', f'/agrobot/plc/{name}')

        return OpcUaConfig(
            endpoint=self.get_parameter('opcua_endpoint').value,
            username=self.get_parameter('opcua_username').value,
            password=self.get_parameter('opcua_password').value,
            cert_file=Path(self.get_parameter('opcua_cert_file').value),
            key_file=Path(self.get_parameter('opcua_key_file').value),
            output_node_ids={
                output_id: self.get_parameter(pname).value
                for output_id, pname in _OUTPUT_PARAMS
            },
            button_node_ids={
                name: self.get_parameter(f'button_nodes.{name}').value
                for name in BUTTON_NAMES
            },
            trigger_rearm_timeout=self.get_parameter('trigger_rearm_timeout').value,
            state_change_debounce=self.get_parameter('state_change_debounce').value,
        )

    # ------------------------------------------------------------------
    # Button setup
    # ------------------------------------------------------------------

    def _setup_button_clients(self) -> None:
        """Create ROS2 service clients and register callbacks for each button."""
        for name in BUTTON_NAMES:
            mode     = self.get_parameter(f'buttons.{name}.mode').value
            svc_name = self.get_parameter(f'buttons.{name}.service').value

            if mode == 'disabled':
                self.get_logger().info(f'Button [{name}]: disabled')
                continue

            if mode == 'trigger':
                client = self.create_client(Trigger, svc_name)
                self._opcua.set_button_callback(
                    name,
                    self._make_trigger_callback(name, svc_name, client),
                )
                self.get_logger().info(
                    f'Button [{name}]: trigger mode → {svc_name}'
                )
                continue

            if mode == 'state_change':
                client = self.create_client(ButtonStateChange, svc_name)
                self._opcua.set_state_change_callback(
                    name,
                    self._make_state_change_callback(name, svc_name, client),
                )
                self.get_logger().info(
                    f'Button [{name}]: state_change mode → {svc_name}'
                )
                continue

    def _make_state_change_callback(self, name: str, svc_name: str, client) -> callable:
        """Return a blocking callable(pressed: bool) for state_change mode buttons.

        Calls the ButtonStateChange service on every press and release.
        Runs in a thread-pool executor — safe to block on client.call().
        """
        def callback(pressed: bool) -> None:
            if not client.service_is_ready():
                self.get_logger().warn(
                    f'Button [{name}]: service {svc_name} not available'
                )
                return
            try:
                req         = ButtonStateChange.Request()
                req.pressed = pressed
                response    = client.call(req)
                label       = 'press' if pressed else 'release'
                if response.success:
                    self.get_logger().debug(
                        f'Button [{name}]: {label} ACK'
                    )
                else:
                    self.get_logger().debug(
                        f'Button [{name}]: {label} rejected — {response.message}'
                    )
            except Exception as e:
                self.get_logger().warn(f'Button [{name}]: call error: {e}')

        return callback

    def _make_trigger_callback(self, name: str, svc_name: str, client) -> callable:
        """Return a blocking callable that calls the downstream Trigger service.

        Runs in a thread-pool executor — safe to block on client.call().
        The ROS2 executor (rclpy.spin) must be running in the main thread for
        the service response to be delivered.
        """
        def callback() -> None:
            if not client.service_is_ready():
                self.get_logger().warn(
                    f'Button [{name}]: service {svc_name} not available, re-arming'
                )
                return
            try:
                response = client.call(Trigger.Request())
                if response.success:
                    self.get_logger().debug(f'Button [{name}]: ACK success')
                else:
                    self.get_logger().debug(
                        f'Button [{name}]: ACK rejected — {response.message}'
                    )
            except Exception as e:
                self.get_logger().warn(f'Button [{name}]: call error: {e}')

        return callback

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_set_outputs(
        self,
        request:  SetOutputs.Request,
        response: SetOutputs.Response,
    ) -> SetOutputs.Response:

        result = self._opcua.write_outputs(
            [(e.output_id, e.state) for e in request.outputs]
        )

        response.success       = result.success
        response.error_code    = result.error_code
        response.error_message = result.error_message
        response.actual_states = [
            OutputState(output_id=oid, state=state)
            for oid, state in result.actual_states
        ]
        return response

    def _handle_query_outputs(
        self,
        request:  QueryOutputs.Request,
        response: QueryOutputs.Response,
    ) -> QueryOutputs.Response:

        result = self._opcua.read_outputs()

        response.success       = result.success
        response.opc_ua_sync   = result.opc_ua_sync
        response.error_code    = result.error_code
        response.error_message = result.error_message
        response.states = [
            OutputState(output_id=oid, state=state)
            for oid, state in result.states
        ]
        return response

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._opcua.stop()
        super().destroy_node()


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PlcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
