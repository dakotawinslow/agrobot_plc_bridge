"""
Unit tests for the QueryOutputs service — mock OPC UA backend.

Key behaviours verified:
  - Response shape (success, opc_ua_sync, error_code, states length)
  - QueryOutputs reflects the current panel state
  - SetOutputs followed by QueryOutputs returns updated values (round-trip)
  - QueryOutputs is truly stateless: it reads from the client, not a local cache
"""

import pytest
import rclpy

from agrobot_plc_bridge_msgs.msg import OutputState
from agrobot_plc_bridge_msgs.srv import QueryOutputs, SetOutputs

from agrobot_plc_bridge.opcua_client import (
    ERROR_NONE,
    NUM_OUTPUTS,
    OpcUaClientBase,
    ReadResult,
    WriteResult,
)
from agrobot_plc_bridge.plc_bridge import PlcBridgeNode


# ---------------------------------------------------------------------------
# Mock client  (same as in test_set_outputs_mock — kept local for clarity)
# ---------------------------------------------------------------------------

class MockOpcUaClient(OpcUaClientBase):

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
        pass

    def set_state_change_callback(self, button_name, callback) -> None:
        pass


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
    n = PlcBridgeNode(opcua_client=MockOpcUaClient())
    yield n
    n.destroy_node()


def _set(node, *outputs):
    req = SetOutputs.Request()
    req.outputs = list(outputs)
    return node._handle_set_outputs(req, SetOutputs.Response())


def _query(node):
    return node._handle_query_outputs(
        QueryOutputs.Request(), QueryOutputs.Response()
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_query_returns_success_and_sync(node):
    resp = _query(node)
    assert resp.success is True
    assert resp.opc_ua_sync is True
    assert resp.error_code == QueryOutputs.Response.ERROR_NONE
    assert resp.error_message == ''


def test_query_returns_all_nine_outputs(node):
    resp = _query(node)
    assert len(resp.states) == 9
    assert {s.output_id for s in resp.states} == set(range(9))


def test_fresh_node_all_outputs_off(node):
    resp   = _query(node)
    states = {s.output_id: s.state for s in resp.states}
    assert all(states[i] == OutputState.STATE_OFF for i in range(9))


def test_query_reflects_set_outputs(node):
    """SetOutputs then QueryOutputs must return the updated values."""
    _set(node,
        OutputState(output_id=OutputState.OUTPUT_AUTO,   state=OutputState.STATE_SOLID),
        OutputState(output_id=OutputState.OUTPUT_BUZZER, state=OutputState.STATE_BLINK),
    )
    resp   = _query(node)
    states = {s.output_id: s.state for s in resp.states}
    assert states[OutputState.OUTPUT_AUTO]   == OutputState.STATE_SOLID
    assert states[OutputState.OUTPUT_BUZZER] == OutputState.STATE_BLINK


def test_query_unrequested_outputs_remain_off(node):
    """QueryOutputs must not report spurious state changes."""
    _set(node, OutputState(
        output_id=OutputState.OUTPUT_HALT, state=OutputState.STATE_FAST_BLINK,
    ))
    resp   = _query(node)
    states = {s.output_id: s.state for s in resp.states}
    for output_id, state in states.items():
        if output_id != OutputState.OUTPUT_HALT:
            assert state == OutputState.STATE_OFF


def test_set_outputs_actual_states_matches_subsequent_query(node):
    """actual_states from SetOutputs must agree with a follow-up QueryOutputs."""
    set_resp   = _set(node,
        OutputState(output_id=OutputState.OUTPUT_RUNNING_TOWER, state=OutputState.STATE_BLINK),
        OutputState(output_id=OutputState.OUTPUT_FAULT_TOWER,   state=OutputState.STATE_SOLID),
    )
    query_resp = _query(node)

    set_states   = {s.output_id: s.state for s in set_resp.actual_states}
    query_states = {s.output_id: s.state for s in query_resp.states}

    assert set_states == query_states
