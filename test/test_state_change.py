"""
Unit tests for state_change mode: falling-edge detection and debounce.

These tests exercise pure-logic helpers (_falling_edges, _Debouncer) and the
callback dispatch mechanism (MockStateChangeClient) without any OPC UA or
ROS2 infrastructure.
"""

import pytest

from agrobot_plc_bridge.opcua_client import _falling_edges, _rising_edges, _Debouncer


# ---------------------------------------------------------------------------
# Falling-edge detection — pure logic, symmetric with rising-edge tests
# ---------------------------------------------------------------------------

class TestFallingEdges:

    def test_falling_edge_detected(self):
        edges = _falling_edges({'halt': False}, {'halt': True})
        assert 'halt' in edges

    def test_rising_not_falling(self):
        assert _falling_edges({'halt': True}, {'halt': False}) == []

    def test_held_down_not_falling(self):
        assert _falling_edges({'halt': True}, {'halt': True}) == []

    def test_held_released_not_falling(self):
        assert _falling_edges({'halt': False}, {'halt': False}) == []

    def test_absent_from_previous_not_falling(self):
        # First poll: previous is empty; False button is NOT a falling edge
        # (it was never True, so it can't have transitioned True → False).
        assert _falling_edges({'halt': False}, {}) == []

    def test_true_absent_from_previous_not_falling(self):
        # A button that reads True on first poll is a rising edge, not falling.
        assert _falling_edges({'halt': True}, {}) == []

    def test_multiple_simultaneous_releases(self):
        edges = set(_falling_edges(
            {'a': False, 'b': False, 'c': True},
            {'a': True,  'b': True,  'c': True},
        ))
        assert edges == {'a', 'b'}

    def test_rising_and_falling_in_same_poll(self):
        """One button just pressed, another just released."""
        current  = {'a': True,  'b': False}
        previous = {'a': False, 'b': True}
        assert _falling_edges(current, previous) == ['b']
        assert _rising_edges(current, previous)  == ['a']


# ---------------------------------------------------------------------------
# Debouncer — pure timing logic, no I/O
# ---------------------------------------------------------------------------

class TestDebouncer:

    def test_first_call_always_allowed(self):
        d = _Debouncer(0.1)
        assert d.allow('x', 0.0) is True

    def test_immediate_repeat_blocked(self):
        d = _Debouncer(0.1)
        d.allow('x', 0.0)
        assert d.allow('x', 0.0) is False

    def test_repeat_within_window_blocked(self):
        d = _Debouncer(0.1)
        d.allow('x', 0.0)
        assert d.allow('x', 0.05) is False   # 50 ms < 100 ms window

    def test_repeat_after_window_allowed(self):
        d = _Debouncer(0.1)
        d.allow('x', 0.0)
        assert d.allow('x', 0.11) is True    # just past the 100 ms window

    def test_exactly_at_boundary_allowed(self):
        d = _Debouncer(0.1)
        d.allow('x', 0.0)
        assert d.allow('x', 0.10) is True    # >= interval → allowed

    def test_debounce_independent_per_key(self):
        d = _Debouncer(0.1)
        d.allow('a', 0.0)
        # 'b' has its own timer; first call for 'b' should be allowed.
        assert d.allow('b', 0.0) is True

    def test_debounce_resets_after_second_allowed_call(self):
        d = _Debouncer(0.1)
        d.allow('x', 0.0)
        d.allow('x', 0.1)   # second allowed call resets the timer
        assert d.allow('x', 0.15) is False   # only 50 ms since last allowed


# ---------------------------------------------------------------------------
# State-change callback dispatch — MockStateChangeClient
# ---------------------------------------------------------------------------

class MockStateChangeClient:
    """Minimal stand-in for OpcUaClientBase supporting state_change callbacks."""

    def __init__(self):
        self._sc_callbacks: dict[str, callable] = {}

    def set_state_change_callback(self, name: str, callback) -> None:
        self._sc_callbacks[name] = callback

    def simulate_press(self, name: str) -> None:
        if name in self._sc_callbacks:
            self._sc_callbacks[name](True)

    def simulate_release(self, name: str) -> None:
        if name in self._sc_callbacks:
            self._sc_callbacks[name](False)


class TestStateChangeCallbackDispatch:

    def test_callback_receives_true_on_press(self):
        received = []
        mock = MockStateChangeClient()
        mock.set_state_change_callback('slow', lambda p: received.append(p))
        mock.simulate_press('slow')
        assert received == [True]

    def test_callback_receives_false_on_release(self):
        received = []
        mock = MockStateChangeClient()
        mock.set_state_change_callback('slow', lambda p: received.append(p))
        mock.simulate_release('slow')
        assert received == [False]

    def test_press_then_release_order(self):
        received = []
        mock = MockStateChangeClient()
        mock.set_state_change_callback('halt', lambda p: received.append(p))
        mock.simulate_press('halt')
        mock.simulate_release('halt')
        assert received == [True, False]

    def test_unregistered_button_does_not_raise(self):
        mock = MockStateChangeClient()
        mock.simulate_press('cycle')   # no callback registered — should not raise

    def test_multiple_buttons_independent(self):
        log: dict[str, list] = {'a': [], 'b': []}
        mock = MockStateChangeClient()
        mock.set_state_change_callback('a', lambda p: log['a'].append(p))
        mock.set_state_change_callback('b', lambda p: log['b'].append(p))
        mock.simulate_press('a')
        mock.simulate_press('b')
        mock.simulate_release('a')
        assert log['a'] == [True, False]
        assert log['b'] == [True]
