"""
Unit tests for the SetOutputs service — mock OPC UA backend.

MockOpcUaClient is injected into PlcBridgeNode so these tests run with no
live PLC connection.  They verify the service handler's logic: response
shape, partial writes, state persistence, and overwriting.
"""

import pytest
import rclpy

from agrobot_plc_bridge_msgs.msg import OutputState
from agrobot_plc_bridge_msgs.srv import SetOutputs

from agrobot_plc_bridge.opcua_client import (
    ERROR_NONE,
    NUM_OUTPUTS,
    OpcUaClientBase,
    ReadResult,
    WriteResult,
)
from agrobot_plc_bridge.plc_bridge import PlcBridgeNode


# ---------------------------------------------------------------------------
# Mock client
# ---------------------------------------------------------------------------

class MockOpcUaClient(OpcUaClientBase):
    """Stateful in-memory output store; no OPC UA connection needed."""

    def __init__(self):
        self._state: dict[int, int] = {
            i: OutputState.STATE_OFF for i in range(NUM_OUTPUTS)
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def write_outputs(self, outputs, timeout=5.0) -> WriteResult:
        for output_id, state in outputs:
            self._state[output_id] = state
        return WriteResult(
            success=True,
            error_code=ERROR_NONE,
            error_message='',
            actual_states=sorted(self._state.items()),
        )

    def read_outputs(self, timeout=5.0) -> ReadResult:
        return ReadResult(
            success=True,
            opc_ua_sync=True,
            error_code=ERROR_NONE,
            error_message='',
            states=sorted(self._state.items()),
        )

    def set_button_callback(self, button_name, callback) -> None:
        pass  # Not needed for output service tests

    def set_state_change_callback(self, button_name, callback) -> None:
        pass  # Not needed for output service tests


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(_rclpy):
    """Fresh node with a fresh MockOpcUaClient for every test."""
    n = PlcBridgeNode(opcua_client=MockOpcUaClient())
    yield n
    n.destroy_node()


def _call(node, *outputs):
    """Build a Request, invoke the handler, return the Response."""
    req          = SetOutputs.Request()
    req.outputs  = list(outputs)
    resp         = SetOutputs.Response()
    return node._handle_set_outputs(req, resp)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_returns_success(node):
    resp = _call(node, OutputState(
        output_id=OutputState.OUTPUT_BUZZER,
        state=OutputState.STATE_BLINK,
    ))
    assert resp.success is True
    assert resp.error_code == SetOutputs.Response.ERROR_NONE
    assert resp.error_message == ''


def test_actual_states_contains_all_nine_outputs(node):
    resp = _call(node, OutputState(
        output_id=OutputState.OUTPUT_AUTO,
        state=OutputState.STATE_SOLID,
    ))
    assert len(resp.actual_states) == 9
    assert {s.output_id for s in resp.actual_states} == set(range(9))


def test_requested_output_is_written(node):
    resp   = _call(node, OutputState(
        output_id=OutputState.OUTPUT_FAULT_TOWER,
        state=OutputState.STATE_FAST_BLINK,
    ))
    states = {s.output_id: s.state for s in resp.actual_states}
    assert states[OutputState.OUTPUT_FAULT_TOWER] == OutputState.STATE_FAST_BLINK


def test_unrequested_outputs_are_untouched(node):
    resp   = _call(node, OutputState(
        output_id=OutputState.OUTPUT_BUZZER,
        state=OutputState.STATE_BLINK,
    ))
    states = {s.output_id: s.state for s in resp.actual_states}
    for output_id, state in states.items():
        if output_id != OutputState.OUTPUT_BUZZER:
            assert state == OutputState.STATE_OFF, (
                f'Output {output_id} should be OFF but is {state}'
            )


def test_multiple_outputs_written_in_one_call(node):
    resp   = _call(
        node,
        OutputState(output_id=OutputState.OUTPUT_AUTO,          state=OutputState.STATE_SOLID),
        OutputState(output_id=OutputState.OUTPUT_RUNNING_TOWER, state=OutputState.STATE_BLINK),
        OutputState(output_id=OutputState.OUTPUT_HALT,          state=OutputState.STATE_FAST_BLINK),
    )
    states = {s.output_id: s.state for s in resp.actual_states}
    assert states[OutputState.OUTPUT_AUTO]          == OutputState.STATE_SOLID
    assert states[OutputState.OUTPUT_RUNNING_TOWER] == OutputState.STATE_BLINK
    assert states[OutputState.OUTPUT_HALT]          == OutputState.STATE_FAST_BLINK


def test_state_persists_across_calls(node):
    _call(node, OutputState(
        output_id=OutputState.OUTPUT_SLOW,
        state=OutputState.STATE_SOLID,
    ))
    resp   = _call(node, OutputState(
        output_id=OutputState.OUTPUT_BUZZER,
        state=OutputState.STATE_BLINK,
    ))
    states = {s.output_id: s.state for s in resp.actual_states}
    assert states[OutputState.OUTPUT_SLOW]   == OutputState.STATE_SOLID
    assert states[OutputState.OUTPUT_BUZZER] == OutputState.STATE_BLINK


def test_output_can_be_overwritten(node):
    _call(node, OutputState(output_id=OutputState.OUTPUT_AUTO, state=OutputState.STATE_SOLID))
    resp   = _call(node, OutputState(output_id=OutputState.OUTPUT_AUTO, state=OutputState.STATE_OFF))
    states = {s.output_id: s.state for s in resp.actual_states}
    assert states[OutputState.OUTPUT_AUTO] == OutputState.STATE_OFF
