"""
Unit tests for OPC UA error handling.

Covers:
  - _classify_error(): UNREACHABLE vs WRITE_FAILED classification
  - PlcBridgeNode service handlers propagate error codes and messages faithfully
  - QueryOutputs reports opc_ua_sync=False on error
  - SetOutputs returns empty actual_states on error
"""

import pytest
import rclpy

from agrobot_plc_bridge_msgs.msg import OutputState
from agrobot_plc_bridge_msgs.srv import QueryOutputs, SetOutputs

from agrobot_plc_bridge.opcua_client import (
    ERROR_NONE,
    ERROR_OPC_UA_UNREACHABLE,
    ERROR_OPC_UA_WRITE_FAILED,
    NUM_OUTPUTS,
    OpcUaClientBase,
    ReadResult,
    WriteResult,
    _classify_error,
    _UNREACHABLE_HINTS,
)
from agrobot_plc_bridge.plc_bridge import PlcBridgeNode


# ---------------------------------------------------------------------------
# _classify_error — pure function, no mocks needed
# ---------------------------------------------------------------------------

class TestClassifyError:

    def test_unknown_exception_is_write_failed(self):
        assert _classify_error(Exception('SomeRandomError')) == ERROR_OPC_UA_WRITE_FAILED

    def test_empty_message_is_write_failed(self):
        assert _classify_error(Exception('')) == ERROR_OPC_UA_WRITE_FAILED

    @pytest.mark.parametrize('hint', _UNREACHABLE_HINTS)
    def test_each_unreachable_hint_detected(self, hint):
        assert _classify_error(Exception(hint)) == ERROR_OPC_UA_UNREACHABLE

    def test_hint_embedded_in_longer_message(self):
        e = Exception('asyncua.ua.uaerrors._auto.BadConnectionClosed: session lost')
        assert _classify_error(e) == ERROR_OPC_UA_UNREACHABLE

    def test_unreachable_and_write_failed_are_distinct(self):
        assert ERROR_OPC_UA_UNREACHABLE != ERROR_OPC_UA_WRITE_FAILED

    def test_error_none_is_zero(self):
        assert ERROR_NONE == 0


# ---------------------------------------------------------------------------
# Failing mock clients
# ---------------------------------------------------------------------------

class _FailingWriteMock(OpcUaClientBase):
    """write_outputs always fails; read_outputs succeeds."""

    def __init__(self, error_code: int, error_message: str):
        self._code = error_code
        self._msg  = error_message
        self._state = {i: OutputState.STATE_OFF for i in range(NUM_OUTPUTS)}

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def write_outputs(self, outputs, timeout=5.0) -> WriteResult:
        return WriteResult(
            success=False,
            error_code=self._code,
            error_message=self._msg,
            actual_states=[],   # empty on failure
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


class _FailingReadMock(OpcUaClientBase):
    """read_outputs always fails; write_outputs succeeds."""

    def __init__(self, error_code: int, error_message: str):
        self._code = error_code
        self._msg  = error_message
        self._state = {i: OutputState.STATE_OFF for i in range(NUM_OUTPUTS)}

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
            success=False,
            opc_ua_sync=False,
            error_code=self._code,
            error_message=self._msg,
            states=[],   # empty on failure
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


def _make_write_failing_node(rclpy_fixture, error_code, message):
    n = PlcBridgeNode(opcua_client=_FailingWriteMock(error_code, message))
    return n


def _make_read_failing_node(rclpy_fixture, error_code, message):
    n = PlcBridgeNode(opcua_client=_FailingReadMock(error_code, message))
    return n


def _set(node, *outputs):
    req = SetOutputs.Request()
    req.outputs = list(outputs)
    return node._handle_set_outputs(req, SetOutputs.Response())


def _query(node):
    return node._handle_query_outputs(
        QueryOutputs.Request(), QueryOutputs.Response()
    )


# ---------------------------------------------------------------------------
# SetOutputs error propagation
# ---------------------------------------------------------------------------

class TestSetOutputsErrorPropagation:

    def test_unreachable_returns_failure(self, _rclpy):
        node = _make_write_failing_node(
            _rclpy, ERROR_OPC_UA_UNREACHABLE, 'BadConnectionClosed'
        )
        try:
            resp = _set(node, OutputState(
                output_id=OutputState.OUTPUT_HALT,
                state=OutputState.STATE_SOLID,
            ))
            assert resp.success is False
        finally:
            node.destroy_node()

    def test_unreachable_error_code_propagated(self, _rclpy):
        node = _make_write_failing_node(
            _rclpy, ERROR_OPC_UA_UNREACHABLE, 'BadConnectionClosed'
        )
        try:
            resp = _set(node, OutputState(
                output_id=OutputState.OUTPUT_HALT,
                state=OutputState.STATE_SOLID,
            ))
            assert resp.error_code == SetOutputs.Response.ERROR_OPC_UA_UNREACHABLE
        finally:
            node.destroy_node()

    def test_write_failed_error_code_propagated(self, _rclpy):
        node = _make_write_failing_node(
            _rclpy, ERROR_OPC_UA_WRITE_FAILED, 'BadTypeMismatch'
        )
        try:
            resp = _set(node, OutputState(
                output_id=OutputState.OUTPUT_AUTO,
                state=OutputState.STATE_BLINK,
            ))
            assert resp.error_code == SetOutputs.Response.ERROR_OPC_UA_WRITE_FAILED
        finally:
            node.destroy_node()

    def test_error_message_propagated(self, _rclpy):
        msg = 'BadConnectionClosed: session lost'
        node = _make_write_failing_node(_rclpy, ERROR_OPC_UA_UNREACHABLE, msg)
        try:
            resp = _set(node, OutputState(
                output_id=OutputState.OUTPUT_BUZZER,
                state=OutputState.STATE_BLINK,
            ))
            assert resp.error_message == msg
        finally:
            node.destroy_node()

    def test_actual_states_empty_on_failure(self, _rclpy):
        node = _make_write_failing_node(
            _rclpy, ERROR_OPC_UA_UNREACHABLE, 'not connected'
        )
        try:
            resp = _set(node, OutputState(
                output_id=OutputState.OUTPUT_HALT,
                state=OutputState.STATE_SOLID,
            ))
            assert resp.actual_states == []
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# QueryOutputs error propagation
# ---------------------------------------------------------------------------

class TestQueryOutputsErrorPropagation:

    def test_unreachable_returns_failure(self, _rclpy):
        node = _make_read_failing_node(
            _rclpy, ERROR_OPC_UA_UNREACHABLE, 'BadCommunicationError'
        )
        try:
            resp = _query(node)
            assert resp.success is False
        finally:
            node.destroy_node()

    def test_opc_ua_sync_false_on_failure(self, _rclpy):
        node = _make_read_failing_node(
            _rclpy, ERROR_OPC_UA_UNREACHABLE, 'BadCommunicationError'
        )
        try:
            resp = _query(node)
            assert resp.opc_ua_sync is False
        finally:
            node.destroy_node()

    def test_error_code_propagated(self, _rclpy):
        node = _make_read_failing_node(
            _rclpy, ERROR_OPC_UA_UNREACHABLE, 'BadCommunicationError'
        )
        try:
            resp = _query(node)
            assert resp.error_code == QueryOutputs.Response.ERROR_OPC_UA_UNREACHABLE
        finally:
            node.destroy_node()

    def test_states_empty_on_failure(self, _rclpy):
        node = _make_read_failing_node(
            _rclpy, ERROR_OPC_UA_UNREACHABLE, 'TimeoutError'
        )
        try:
            resp = _query(node)
            assert resp.states == []
        finally:
            node.destroy_node()

    def test_error_message_propagated(self, _rclpy):
        msg = 'TimeoutError: read timed out'
        node = _make_read_failing_node(_rclpy, ERROR_OPC_UA_UNREACHABLE, msg)
        try:
            resp = _query(node)
            assert resp.error_message == msg
        finally:
            node.destroy_node()
