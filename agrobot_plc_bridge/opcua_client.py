"""
opcua_client.py — asyncua connection manager for the agrobot PLC bridge.

The real client (OpcUaClient) runs asyncua in a dedicated background thread
with its own event loop.  ROS2 service callbacks submit coroutines to that
loop via asyncio.run_coroutine_threadsafe() and block until they resolve.

OpcUaClientBase is an ABC shared by the real client and test doubles so that
PlcBridgeNode can be unit-tested without a live PLC.
"""

import asyncio
import socket
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from asyncua import Client, Node, ua
from asyncua.crypto.cert_gen import setup_self_signed_certificate
from asyncua.crypto.security_policies import SecurityPolicyBasic256Sha256
from cryptography.x509.oid import ExtendedKeyUsageOID

# Application URI embedded in the client certificate.
APP_URI = 'urn:agrobot:plc_bridge'

# Default location for the auto-generated client certificate.
DEFAULT_CERT_DIR = Path.home() / '.config' / 'agrobot_plc_bridge' / 'certs'

# Total number of physical outputs on the operator panel.
NUM_OUTPUTS = 9

# Error code constants — mirror the values defined in SetOutputs.srv.
ERROR_NONE                = 0
ERROR_OPC_UA_UNREACHABLE  = 1
ERROR_OPC_UA_WRITE_FAILED = 2

# How often to poll button states (seconds).  Not a ROS2 parameter — this is
# an implementation detail that users don't normally need to tune.
POLL_INTERVAL = 0.05   # 20 Hz

# Consecutive all-failed polls before the client declares the connection lost
# and starts the reconnection loop.  At 20 Hz this is ~250 ms.
POLL_FAIL_THRESHOLD = 5

# Seconds between reconnection attempts after a connection loss.
RECONNECT_INTERVAL = 5.0

# Error substrings that indicate a connectivity failure rather than a
# rejected-write failure.
_UNREACHABLE_HINTS = (
    'BadConnectionClosed',
    'BadCommunicationError',
    'BadNoCommunication',
    'BadServerNotConnected',
    'ConnectionError',
    'TimeoutError',
)


def _classify_error(exc: Exception) -> int:
    """Return ERROR_OPC_UA_UNREACHABLE or ERROR_OPC_UA_WRITE_FAILED.

    Inspects the exception message for known OPC UA connectivity-error
    substrings so callers can distinguish a transient network loss from a
    rejected or malformed write operation.
    """
    s = str(exc)
    return (
        ERROR_OPC_UA_UNREACHABLE
        if any(hint in s for hint in _UNREACHABLE_HINTS)
        else ERROR_OPC_UA_WRITE_FAILED
    )


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class OpcUaConfig:
    endpoint:               str
    username:               str
    password:               str
    cert_file:              Path
    key_file:               Path
    output_node_ids:        dict    # {output_id (int):   node_id_str (str)}
    button_node_ids:        dict    # {button_name (str): node_id_str (str)}
    trigger_rearm_timeout:  float = 5.0
    state_change_debounce:  float = 0.1   # seconds dead-time between calls


@dataclass
class WriteResult:
    success:       bool
    error_code:    int
    error_message: str
    actual_states: list     # [(output_id, state), …] for all 9 outputs


@dataclass
class ReadResult:
    success:       bool
    opc_ua_sync:   bool     # True = values are current; False = read failed
    error_code:    int      # ERROR_NONE or ERROR_OPC_UA_UNREACHABLE
    error_message: str
    states:        list     # [(output_id, state), …] for all 9 outputs; empty on failure


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def _rising_edges(
    current:  dict,   # {button_name: bool}
    previous: dict,   # {button_name: bool}  (may be empty on first poll)
) -> list:
    """Return names of buttons that transitioned False → True.

    A button absent from `previous` is treated as previously False, so the
    very first poll correctly detects any button that is already held down at
    startup as a press (edge).
    """
    return [
        name for name in current
        if current[name] and not previous.get(name, False)
    ]


def _falling_edges(
    current:  dict,   # {button_name: bool}
    previous: dict,   # {button_name: bool}  (may be empty on first poll)
) -> list:
    """Return names of buttons that transitioned True → False.

    A button absent from `previous` is treated as previously False, so no
    falling edge is generated on the very first poll even if the button reads
    False (that would be a spurious release).
    """
    return [
        name for name in current
        if not current[name] and previous.get(name, False)
    ]


class _Debouncer:
    """Per-key dead-time gate.

    Tracks the last accepted timestamp for each key.  ``allow()`` returns
    True (and records the timestamp) only when *interval* seconds have passed
    since the last accepted call for that key.
    """

    def __init__(self, interval: float):
        self._interval = interval
        self._last: dict[str, float] = {}

    def allow(self, key: str, now: float) -> bool:
        if now - self._last.get(key, float('-inf')) >= self._interval:
            self._last[key] = now
            return True
        return False


# ---------------------------------------------------------------------------
# Abstract base — implemented by OpcUaClient and by test doubles
# ---------------------------------------------------------------------------

class OpcUaClientBase(ABC):

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def write_outputs(
        self,
        outputs: list,          # [(output_id, state), …]
        timeout: float = 5.0,
    ) -> WriteResult: ...

    @abstractmethod
    def read_outputs(
        self,
        timeout: float = 5.0,
    ) -> ReadResult: ...

    @abstractmethod
    def set_button_callback(
        self,
        button_name: str,
        callback,       # callable() → None  (blocking; runs in thread pool)
    ) -> None: ...

    @abstractmethod
    def set_state_change_callback(
        self,
        button_name: str,
        callback,       # callable(pressed: bool) → None  (blocking)
    ) -> None: ...


# ---------------------------------------------------------------------------
# Real client
# ---------------------------------------------------------------------------

class OpcUaClient(OpcUaClientBase):
    """asyncua client running in a dedicated background thread."""

    def __init__(self, config: OpcUaConfig, logger=None):
        self._config  = config
        self._logger  = logger
        self._loop    = asyncio.new_event_loop()
        self._thread  = threading.Thread(
            target=self._run_loop,
            name='plc-bridge-opcua',
            daemon=True,
        )
        self._client:                Optional[Client]           = None
        self._output_nodes:         dict[int, Node]            = {}
        self._output_variant_types: dict[int, ua.VariantType]  = {}
        # Button monitoring state
        self._button_nodes:      dict[str, Node]     = {}
        self._button_states:     dict[str, bool]     = {}   # last known states
        self._locked_buttons:    set[str]            = set()
        self._button_callbacks:  dict[str, callable] = {}   # trigger mode
        self._button_sc_callbacks: dict[str, callable] = {} # state_change mode
        self._sc_debouncer:      _Debouncer = _Debouncer(config.state_change_debounce)
        self._poll_task:         Optional[asyncio.Task] = None
        self._connected = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._thread.start()
        try:
            self._submit(self._connect(), timeout=15.0)
        except Exception as e:
            self._err(f'OPC UA connect failed at startup: {e}')

    def stop(self) -> None:
        if self._connected:
            try:
                self._submit(self._disconnect(), timeout=5.0)
            except Exception:
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)

    # ------------------------------------------------------------------
    # Public API  (called from ROS2 service callbacks)
    # ------------------------------------------------------------------

    def write_outputs(
        self,
        outputs: list,
        timeout: float = 5.0,
    ) -> WriteResult:
        if not self._connected:
            return WriteResult(
                success=False,
                error_code=ERROR_OPC_UA_UNREACHABLE,
                error_message='Not connected to OPC UA server',
                actual_states=[],
            )
        try:
            return self._submit(self._async_write(outputs), timeout=timeout)
        except TimeoutError:
            return WriteResult(
                success=False,
                error_code=ERROR_OPC_UA_UNREACHABLE,
                error_message='OPC UA write timed out',
                actual_states=[],
            )
        except Exception as e:
            return WriteResult(
                success=False,
                error_code=ERROR_OPC_UA_WRITE_FAILED,
                error_message=str(e),
                actual_states=[],
            )

    def read_outputs(
        self,
        timeout: float = 5.0,
    ) -> ReadResult:
        if not self._connected:
            return ReadResult(
                success=False,
                opc_ua_sync=False,
                error_code=ERROR_OPC_UA_UNREACHABLE,
                error_message='Not connected to OPC UA server',
                states=[],
            )
        try:
            return self._submit(self._async_read(), timeout=timeout)
        except TimeoutError:
            return ReadResult(
                success=False,
                opc_ua_sync=False,
                error_code=ERROR_OPC_UA_UNREACHABLE,
                error_message='OPC UA read timed out',
                states=[],
            )
        except Exception as e:
            return ReadResult(
                success=False,
                opc_ua_sync=False,
                error_code=ERROR_OPC_UA_UNREACHABLE,
                error_message=str(e),
                states=[],
            )

    def set_button_callback(self, button_name: str, callback) -> None:
        """Register a blocking callable to be called on a trigger-mode press.

        Must be called before start().  The callback runs in a thread-pool
        executor so it may block (e.g. on a synchronous ROS2 service call)
        without stalling the asyncua event loop.
        """
        self._button_callbacks[button_name] = callback

    def set_state_change_callback(self, button_name: str, callback) -> None:
        """Register a callable(pressed: bool) for state-change mode.

        Called on every press AND release transition, subject to debounce.
        Must be called before start().
        """
        self._button_sc_callbacks[button_name] = callback

    # ------------------------------------------------------------------
    # Internal — run on the asyncua background thread
    # ------------------------------------------------------------------

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _submit(self, coro, timeout: float):
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=timeout)

    async def _connect(self) -> None:
        # Generate (or reuse) a self-signed client certificate.
        self._config.cert_file.parent.mkdir(parents=True, exist_ok=True)
        await setup_self_signed_certificate(
            self._config.key_file,
            self._config.cert_file,
            app_uri=APP_URI,
            host_name=socket.gethostname(),
            cert_use=[ExtendedKeyUsageOID.CLIENT_AUTH],
            subject_attrs={'organizationName': 'Agrobot Robotics Club'},
        )

        self._client = Client(url=self._config.endpoint)
        self._client.set_user(self._config.username)
        self._client.set_password(self._config.password)
        self._client.application_uri = APP_URI

        await self._client.set_security(
            SecurityPolicyBasic256Sha256,
            certificate=str(self._config.cert_file),
            private_key=str(self._config.key_file),
            mode=ua.MessageSecurityMode.SignAndEncrypt,
        )

        await self._client.connect()

        # Cache node handles and discover each output's data type once.
        for output_id, node_id_str in self._config.output_node_ids.items():
            node = self._client.get_node(node_id_str)
            dv   = await node.read_data_value()
            self._output_nodes[output_id]         = node
            self._output_variant_types[output_id] = dv.Value.VariantType

        # Cache button node handles; initialise previous state to False.
        for name, node_id_str in self._config.button_node_ids.items():
            if not node_id_str:
                self._warn(f'Button [{name}]: no node ID configured, skipping')
                continue
            self._button_nodes[name]  = self._client.get_node(node_id_str)
            self._button_states[name] = False

        self._connected = True
        self._info(f'Connected to {self._config.endpoint}')

        # Start button polling if any buttons are configured.
        if self._button_nodes:
            self._poll_task = self._loop.create_task(self._poll_buttons())

    async def _disconnect(self) -> None:
        if self._poll_task is not None:
            self._poll_task.cancel()
            try:
                await self._poll_task
            except asyncio.CancelledError:
                pass
            self._poll_task = None

        if self._client:
            await self._client.disconnect()
        self._client    = None
        self._connected = False

    async def _async_write(self, outputs: list) -> WriteResult:
        try:
            # Write only the requested outputs.
            for output_id, state in outputs:
                node  = self._output_nodes[output_id]
                vtype = self._output_variant_types[output_id]
                await node.write_value(ua.DataValue(ua.Variant(state, vtype)))

            # Read back all 9 to populate actual_states.
            actual = []
            for output_id in range(NUM_OUTPUTS):
                value = await self._output_nodes[output_id].read_value()
                actual.append((output_id, int(value)))

            return WriteResult(
                success=True,
                error_code=ERROR_NONE,
                error_message='',
                actual_states=actual,
            )

        except Exception as e:
            return WriteResult(
                success=False,
                error_code=_classify_error(e),
                error_message=str(e),
                actual_states=[],
            )

    async def _poll_buttons(self) -> None:
        """Continuously read button states; fire callbacks on edge transitions.

        Tracks consecutive all-failed polls.  When POLL_FAIL_THRESHOLD is
        reached the connection is declared lost: _connected is set to False and
        _reconnect_loop is spawned to restore it.
        """
        consecutive_failures = 0
        try:
            while True:
                # Read all configured button nodes.
                current: dict[str, bool] = {}
                any_succeeded = False
                for name, node in self._button_nodes.items():
                    try:
                        current[name] = bool(await node.read_value())
                        any_succeeded = True
                    except Exception:
                        # Keep the last known state on a transient read failure.
                        current[name] = self._button_states.get(name, False)

                # Sustained-failure detection.
                if self._button_nodes:
                    if any_succeeded:
                        consecutive_failures = 0
                    else:
                        consecutive_failures += 1
                        if consecutive_failures >= POLL_FAIL_THRESHOLD:
                            self._err(
                                f'PLC unreachable: {consecutive_failures} consecutive '
                                f'poll failures — starting reconnect loop'
                            )
                            self._connected = False
                            self._loop.create_task(self._reconnect_loop())
                            return  # Exit; _reconnect_loop restarts this task.

                now = self._loop.time()

                # Trigger mode — rising edges only, with per-button lock.
                for name in _rising_edges(current, self._button_states):
                    if name in self._locked_buttons:
                        continue
                    cb = self._button_callbacks.get(name)
                    if cb is None:
                        continue
                    self._locked_buttons.add(name)
                    self._loop.create_task(self._fire_trigger(name, cb))

                # State-change mode — both edges, with shared debounce per button.
                edges: list[tuple[str, bool]] = (
                    [(n, True)  for n in _rising_edges(current, self._button_states)] +
                    [(n, False) for n in _falling_edges(current, self._button_states)]
                )
                for name, pressed in edges:
                    cb = self._button_sc_callbacks.get(name)
                    if cb is None:
                        continue
                    if not self._sc_debouncer.allow(name, now):
                        continue
                    self._loop.create_task(self._fire_state_change(name, cb, pressed))

                self._button_states = current
                await asyncio.sleep(POLL_INTERVAL)

        except asyncio.CancelledError:
            pass   # Normal shutdown path.

    async def _fire_trigger(self, name: str, callback) -> None:
        """Call the downstream Trigger service in a thread-pool executor.

        Re-arms the button when the service responds OR when the rearm timeout
        expires — whichever comes first.
        """
        try:
            await asyncio.wait_for(
                self._loop.run_in_executor(None, callback),
                timeout=self._config.trigger_rearm_timeout,
            )
        except asyncio.TimeoutError:
            self._warn(
                f'Button [{name}]: trigger service timed out after '
                f'{self._config.trigger_rearm_timeout}s, re-arming'
            )
        except Exception as e:
            self._warn(f'Button [{name}]: trigger service error: {e}')
        finally:
            self._locked_buttons.discard(name)

    async def _fire_state_change(self, name: str, callback, pressed: bool) -> None:
        """Call the downstream ButtonStateChange service in a thread-pool executor.

        Uses the same rearm timeout as trigger mode to bound how long the
        executor thread can block.
        """
        try:
            await asyncio.wait_for(
                self._loop.run_in_executor(None, lambda: callback(pressed)),
                timeout=self._config.trigger_rearm_timeout,
            )
        except asyncio.TimeoutError:
            self._warn(
                f'Button [{name}]: state_change service timed out after '
                f'{self._config.trigger_rearm_timeout}s'
            )
        except Exception as e:
            self._warn(f'Button [{name}]: state_change service error: {e}')

    async def _reconnect_loop(self) -> None:
        """Retry connecting after a mid-session connection loss.

        Waits RECONNECT_INTERVAL seconds between attempts.  On success,
        _connect() automatically restarts the button poll task.
        """
        attempt = 0
        while not self._connected:
            await asyncio.sleep(RECONNECT_INTERVAL)
            attempt += 1
            self._info(
                f'Reconnect attempt {attempt} to {self._config.endpoint}…'
            )
            try:
                # Close the stale client before reconnecting.
                if self._client is not None:
                    try:
                        await self._client.disconnect()
                    except Exception:
                        pass
                    self._client = None

                await self._connect()

            except Exception as e:
                self._warn(f'Reconnect attempt {attempt} failed: {e}')

        self._info('Reconnected to PLC successfully.')

    async def _async_read(self) -> ReadResult:
        try:
            states = []
            for output_id in range(NUM_OUTPUTS):
                value = await self._output_nodes[output_id].read_value()
                states.append((output_id, int(value)))

            return ReadResult(
                success=True,
                opc_ua_sync=True,
                error_code=ERROR_NONE,
                error_message='',
                states=states,
            )

        except Exception as e:
            return ReadResult(
                success=False,
                opc_ua_sync=False,
                error_code=ERROR_OPC_UA_UNREACHABLE,
                error_message=str(e),
                states=[],
            )

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------

    def _info(self, msg: str) -> None:
        if self._logger:
            self._logger.info(msg)

    def _warn(self, msg: str) -> None:
        if self._logger:
            self._logger.warn(msg)

    def _err(self, msg: str) -> None:
        if self._logger:
            self._logger.error(msg)
