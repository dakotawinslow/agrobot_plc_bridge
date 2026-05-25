#!/usr/bin/env python3
"""
browse_plc.py  —  Discover OPC UA nodes on the agrobot operator-panel PLC.

Connects to the PLC using Basic256Sha256 / SignAndEncrypt with a self-signed
client certificate (auto-generated on first run and cached for reuse), then
browses the full node tree.  Nodes matching expected output/button names are
highlighted with ◄.

The generated certificate lives in CERT_DIR (default shown below).  If the
PLC requires manual certificate trust you will get a BadCertificateUntrusted
error on the first run — approve the certificate in the PLC's OPC UA trusted-
clients store, then re-run this script.

Usage
-----
    python3 scripts/browse_plc.py --user USER --password PASS
    python3 scripts/browse_plc.py --user USER --password PASS --depth 6
    python3 scripts/browse_plc.py --endpoint opc.tcp://... --user USER --password PASS
    python3 scripts/browse_plc.py --user USER --password PASS --cert-dir /path/to/certs
"""

import argparse
import asyncio
import socket
from pathlib import Path

from asyncua import Client, ua
from asyncua.crypto.cert_gen import setup_self_signed_certificate
from asyncua.crypto.security_policies import SecurityPolicyBasic256Sha256
from cryptography.x509.oid import ExtendedKeyUsageOID

DEFAULT_ENDPOINT = 'opc.tcp://172.16.0.151:4840'
DEFAULT_DEPTH    = 5
DEFAULT_CERT_DIR = Path.home() / '.config' / 'agrobot_plc_bridge' / 'certs'
APP_URI          = 'urn:agrobot:plc_bridge'

# Node browse names the bridge needs — highlighted in output.
EXPECTED_NAMES = {
    'RunningTower', 'FaultTower', 'Auto', 'Cycle', 'Step',
    'Prev', 'Halt', 'Slow', 'Buzzer', 'Emote',
}


# ---------------------------------------------------------------------------
# Certificate helpers
# ---------------------------------------------------------------------------

async def _ensure_cert(cert_dir: Path) -> tuple[Path, Path]:
    """Generate (or reuse) a self-signed client certificate in cert_dir."""
    cert_dir.mkdir(parents=True, exist_ok=True)
    key_file  = cert_dir / 'client_key.pem'
    cert_file = cert_dir / 'client_cert.der'

    await setup_self_signed_certificate(
        key_file,
        cert_file,
        app_uri=APP_URI,
        host_name=socket.gethostname(),
        cert_use=[ExtendedKeyUsageOID.CLIENT_AUTH],
        subject_attrs={'organizationName': 'Agrobot Robotics Club'},
    )

    action = 'Reusing' if cert_file.exists() else 'Generated'
    print(f'{action} client certificate: {cert_file}')
    return cert_file, key_file


async def _build_client(
    endpoint: str, user: str, password: str, cert_dir: Path
) -> Client:
    cert_file, key_file = await _ensure_cert(cert_dir)

    client = Client(url=endpoint)
    client.set_user(user)
    client.set_password(password)
    client.application_uri = APP_URI

    await client.set_security(
        SecurityPolicyBasic256Sha256,
        certificate=str(cert_file),
        private_key=str(key_file),
        mode=ua.MessageSecurityMode.SignAndEncrypt,
    )
    return client


# ---------------------------------------------------------------------------
# Node tree browsing
# ---------------------------------------------------------------------------

async def _browse(node, depth: int, max_depth: int, prefix: str = '') -> None:
    """Recursively print the node tree, skipping type/method nodes."""
    try:
        browse_name = await node.read_browse_name()
        node_class  = await node.read_node_class()
        node_id_str = node.nodeid.to_string()
    except Exception as e:
        print(f'{prefix}<unreadable: {e}>')
        return

    if node_class not in (ua.NodeClass.Object, ua.NodeClass.Variable):
        return

    highlight = (
        '  ◄  ← copy this node-ID to config/plc_bridge.yaml'
        if browse_name.Name in EXPECTED_NAMES else ''
    )

    if node_class == ua.NodeClass.Variable:
        try:
            value = await node.read_value()
            print(f'{prefix}{browse_name.Name}  [{node_id_str}]  =  {value!r}{highlight}')
        except Exception:
            print(f'{prefix}{browse_name.Name}  [{node_id_str}]{highlight}')
        return

    # Object node — print and recurse
    print(f'{prefix}{browse_name.Name}/  [{node_id_str}]{highlight}')

    if depth >= max_depth:
        print(f'{prefix}  <max depth — re-run with --depth {max_depth + 1} to go deeper>')
        return

    try:
        children = await node.get_children()
    except Exception as e:
        print(f'{prefix}  <cannot get children: {e}>')
        return

    for child in children:
        await _browse(child, depth + 1, max_depth, prefix + '  ')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def _run(
    endpoint: str, user: str, password: str, depth: int, cert_dir: Path
) -> None:
    client = await _build_client(endpoint, user, password, cert_dir)
    print(f'Connecting to {endpoint} ...')

    try:
        async with client:
            ns_array = await client.get_namespace_array()
            print(f'Connected.  Namespaces: {list(enumerate(ns_array))}\n')
            await _browse(client.nodes.objects, depth=0, max_depth=depth)
    except Exception as e:
        msg = str(e)
        if 'BadCertificateUntrusted' in msg or 'BadCertificateInvalid' in msg:
            print(
                f'\nERROR: The PLC rejected our certificate ({e})\n\n'
                f'Action required:\n'
                f'  1. The rejected certificate has been placed in the PLC\'s\n'
                f'     "rejected" certificate store.\n'
                f'  2. Open the PLC\'s OPC UA configuration and move\n'
                f'     the certificate to the "trusted" store.\n'
                f'  3. Re-run this script.\n\n'
                f'Client certificate: {cert_dir / "client_cert.der"}\n'
            )
        else:
            print(f'\nERROR: {e}')
        raise SystemExit(1)

    print('\nDone.  Copy the node-ID strings marked with ◄ into config/plc_bridge.yaml.')


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Browse OPC UA node tree on the agrobot PLC',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--endpoint', default=DEFAULT_ENDPOINT,
        help=f'OPC UA endpoint URL  (default: {DEFAULT_ENDPOINT})',
    )
    parser.add_argument('--user',     required=True, help='OPC UA username')
    parser.add_argument('--password', required=True, help='OPC UA password')
    parser.add_argument(
        '--depth', type=int, default=DEFAULT_DEPTH,
        help=f'Maximum browse depth  (default: {DEFAULT_DEPTH})',
    )
    parser.add_argument(
        '--cert-dir', type=Path, default=DEFAULT_CERT_DIR,
        help=f'Directory for client certificate/key  (default: {DEFAULT_CERT_DIR})',
    )
    args = parser.parse_args()
    asyncio.run(_run(args.endpoint, args.user, args.password, args.depth, args.cert_dir))


if __name__ == '__main__':
    main()
