#!/usr/bin/env python3
"""
End-to-end integration smoke test — requires a live PLC at 172.16.0.151.

Exercises the full stack:
    test script → PlcBridgeNode → OpcUaClient → PLC → OpcUaClient → response

Test sequence
─────────────
1. Connect and query: verify initial state is readable
2. Write a known pattern:  running_tower=SOLID, fault_tower=BLINK, buzzer=BLINK
3. Query and assert the written values came back correctly
4. Overwrite one output: buzzer=OFF, auto=FAST_BLINK
5. Query and assert only the changed outputs differ
6. Write all outputs OFF and assert clean
7. (Cleanup always runs — outputs left OFF even if test aborts mid-run)

Exit code: 0 = all checks passed, 1 = one or more checks failed.

Usage (from workspace root):
    source .venv/bin/activate && source install/setup.bash
    python3 src/agrobot_plc_bridge/scripts/smoke_test.py
"""

import sys
import time

import rclpy
from agrobot_plc_bridge_msgs.msg import OutputState
from agrobot_plc_bridge_msgs.srv import QueryOutputs, SetOutputs

from agrobot_plc_bridge.opcua_client import (
    DEFAULT_CERT_DIR,
    OpcUaClient,
    OpcUaConfig,
)
from agrobot_plc_bridge.plc_bridge import PlcBridgeNode


# ---------------------------------------------------------------------------
# Config — mirrors config/plc_bridge.yaml
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
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

S = OutputState   # short alias

_STATE_NAMES = {
    S.STATE_OFF:        'OFF',
    S.STATE_SOLID:      'SOLID',
    S.STATE_BLINK:      'BLINK',
    S.STATE_FAST_BLINK: 'FAST_BLINK',
}

_OUTPUT_NAMES = {
    S.OUTPUT_RUNNING_TOWER: 'running_tower',
    S.OUTPUT_FAULT_TOWER:   'fault_tower',
    S.OUTPUT_AUTO:          'auto',
    S.OUTPUT_CYCLE:         'cycle',
    S.OUTPUT_STEP:          'step',
    S.OUTPUT_PREV:          'prev',
    S.OUTPUT_HALT:          'halt',
    S.OUTPUT_SLOW:          'slow',
    S.OUTPUT_BUZZER:        'buzzer',
}


def _fmt(output_id: int, state: int) -> str:
    return f'{_OUTPUT_NAMES.get(output_id, output_id)}={_STATE_NAMES.get(state, state)}'


class SmokeTest:

    def __init__(self, node: PlcBridgeNode):
        self._node    = node
        self._passed  = 0
        self._failed  = 0
        self._errors: list[str] = []

    # ------------------------------------------------------------------
    # Service wrappers
    # ------------------------------------------------------------------

    def _set(self, *outputs) -> SetOutputs.Response:
        req = SetOutputs.Request()
        req.outputs = list(outputs)
        return self._node._handle_set_outputs(req, SetOutputs.Response())

    def _query(self) -> dict[int, int]:
        resp = self._node._handle_query_outputs(
            QueryOutputs.Request(), QueryOutputs.Response()
        )
        if not resp.success:
            raise RuntimeError(
                f'QueryOutputs failed: [{resp.error_code}] {resp.error_message}'
            )
        return {s.output_id: s.state for s in resp.states}

    def _set_all_off(self) -> SetOutputs.Response:
        return self._set(*[
            S(output_id=oid, state=S.STATE_OFF)
            for oid in range(9)
        ])

    # ------------------------------------------------------------------
    # Assertion helpers
    # ------------------------------------------------------------------

    def _check(self, label: str, ok: bool, detail: str = '') -> None:
        status = '✓ PASS' if ok else '✗ FAIL'
        print(f'  {status}  {label}' + (f' — {detail}' if detail else ''))
        if ok:
            self._passed += 1
        else:
            self._failed += 1
            self._errors.append(label)

    def _assert_states(self, label: str, states: dict[int, int],
                       expected: dict[int, int]) -> None:
        for oid, exp_state in expected.items():
            got = states.get(oid, -1)
            self._check(
                f'{label}: {_fmt(oid, exp_state)}',
                got == exp_state,
                f'got {_STATE_NAMES.get(got, got)}' if got != exp_state else '',
            )

    def _assert_all_off(self, label: str, states: dict[int, int]) -> None:
        all_off = all(v == S.STATE_OFF for v in states.values())
        self._check(label, all_off,
                    '' if all_off else
                    str({_OUTPUT_NAMES[k]: _STATE_NAMES[v]
                         for k, v in states.items() if v != S.STATE_OFF}))

    # ------------------------------------------------------------------
    # Test steps
    # ------------------------------------------------------------------

    def run(self) -> int:
        print('\n══════════════════════════════════════════')
        print('  Agrobot PLC Bridge — Smoke Test')
        print('══════════════════════════════════════════\n')

        try:
            # ── Step 1: initial query ──────────────────────────────────
            print('Step 1: Initial query')
            states = self._query()
            self._check(
                'QueryOutputs returns 9 states',
                len(states) == 9,
                f'got {len(states)}',
            )
            print()

            # ── Step 2: write a known pattern ─────────────────────────
            print('Step 2: Write running_tower=SOLID, fault_tower=BLINK, buzzer=BLINK')
            resp = self._set(
                S(output_id=S.OUTPUT_RUNNING_TOWER, state=S.STATE_SOLID),
                S(output_id=S.OUTPUT_FAULT_TOWER,   state=S.STATE_BLINK),
                S(output_id=S.OUTPUT_BUZZER,         state=S.STATE_BLINK),
            )
            self._check('SetOutputs succeeds',           resp.success)
            self._check('SetOutputs actual_states has 9', len(resp.actual_states) == 9)
            time.sleep(0.2)   # brief pause so changes settle on PLC

            # ── Step 3: query and verify ───────────────────────────────
            print('\nStep 3: Query and verify written pattern')
            states = self._query()
            self._assert_states('Query after write', states, {
                S.OUTPUT_RUNNING_TOWER: S.STATE_SOLID,
                S.OUTPUT_FAULT_TOWER:   S.STATE_BLINK,
                S.OUTPUT_BUZZER:        S.STATE_BLINK,
            })
            print()

            # ── Step 4: partial overwrite ──────────────────────────────
            print('Step 4: Overwrite buzzer=OFF, set auto=FAST_BLINK')
            self._set(
                S(output_id=S.OUTPUT_BUZZER, state=S.STATE_OFF),
                S(output_id=S.OUTPUT_AUTO,   state=S.STATE_FAST_BLINK),
            )
            time.sleep(0.2)

            # ── Step 5: verify only changed outputs differ ─────────────
            print('\nStep 5: Verify partial overwrite')
            states = self._query()
            self._assert_states('After partial overwrite', states, {
                S.OUTPUT_RUNNING_TOWER: S.STATE_SOLID,       # unchanged
                S.OUTPUT_FAULT_TOWER:   S.STATE_BLINK,       # unchanged
                S.OUTPUT_BUZZER:        S.STATE_OFF,          # cleared
                S.OUTPUT_AUTO:          S.STATE_FAST_BLINK,  # new
            })
            print()

        finally:
            # ── Step 6: cleanup — always runs ──────────────────────────
            print('Step 6: Reset all outputs OFF (cleanup)')
            cleanup_resp = self._set_all_off()
            time.sleep(0.2)
            states = self._query()
            self._check('SetOutputs cleanup succeeds', cleanup_resp.success)
            self._assert_all_off('All outputs OFF after cleanup', states)

        # ── Summary ───────────────────────────────────────────────────
        total = self._passed + self._failed
        print(f'\n{"─" * 42}')
        print(f'  {self._passed}/{total} checks passed')
        if self._errors:
            print('  FAILED:')
            for e in self._errors:
                print(f'    • {e}')
        print(f'{"─" * 42}\n')

        return 0 if self._failed == 0 else 1


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    rclpy.init()
    config = _make_config()
    opcua  = OpcUaClient(config)
    node   = PlcBridgeNode(opcua_client=opcua)

    print('Connecting to PLC…')
    opcua.start()
    time.sleep(1.0)   # allow _connect() to complete

    try:
        test   = SmokeTest(node)
        result = test.run()
    finally:
        opcua.stop()
        node.destroy_node()
        rclpy.shutdown()

    return result


if __name__ == '__main__':
    sys.exit(main())
