#!/bin/bash
# build.sh — build the opcua_ws workspace and rewrite entry-point shebangs
# so that `ros2 run` / `ros2 launch` use the project venv (and can find asyncua)
# instead of the system Python.
#
# Usage:
#   ./build.sh                          # build all packages
#   ./build.sh --packages-select foo    # any extra colcon flags are forwarded
#
# Why this is needed:
#   colcon's ament_python task always invokes `setup.py install` with
#   /usr/bin/python3, so the generated entry-point scripts always get the
#   shebang #!/usr/bin/python3 regardless of which Python is active in the
#   shell.  This script patches that shebang after the build.

set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="$WORKSPACE/.venv/bin/python3"

if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "ERROR: venv not found at $WORKSPACE/.venv"
    echo "Create it first:"
    echo "  uv venv --system-site-packages && uv pip install asyncua"
    exit 1
fi

# Activate the venv so colcon can find the right Python site-packages.
# shellcheck source=/dev/null
source "$WORKSPACE/.venv/bin/activate"

colcon build "$@"

echo ""
echo "Patching shebangs → $VENV_PYTHON"
while IFS= read -r -d '' script; do
    first_line="$(head -1 "$script")"
    if [[ "$first_line" == "#!/usr/bin/python3" ]]; then
        sed -i "1s|^#!/usr/bin/python3|#!$VENV_PYTHON|" "$script"
        echo "  ✓  $(realpath --relative-to="$WORKSPACE" "$script")"
    fi
done < <(find "$WORKSPACE/install" -path "*/lib/agrobot_*/*" -maxdepth 5 -type f -print0)

echo "Done.  Source install/setup.bash and you're ready."
