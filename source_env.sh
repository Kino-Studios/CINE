# Source this file (`source source_env.sh`); do not execute it.
#
# Sets the ROS environment for CINE: ROS 2 Jazzy, then the colcon workspace that holds
# this repository. The workspace is found from CINE_WS when set, otherwise as the
# directory two levels above this file (<ws>/src/CINE).
#
# WSL tip: for RViz through an X server on the Windows host, also
# `export DISPLAY=localhost:0`.

CINE_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CINE_WS="${CINE_WS:-$(cd "$CINE_REPO/../.." && pwd)}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset ROS_LOCALHOST_ONLY

# Optional virtual environment for the pip-only packages (requirements.txt). The depth
# publisher looks for one beside <ws>/src/Depth-Anything, so the workspace is tried first.
if [ -f "$CINE_WS/venv/bin/activate" ]; then
    source "$CINE_WS/venv/bin/activate"
elif [ -f "$CINE_REPO/venv/bin/activate" ]; then
    source "$CINE_REPO/venv/bin/activate"
fi

# The ROS setup scripts read unset variables, so they fail under `set -u`.
if set -o | grep -q 'nounset *on'; then
    _CINE_RESTORE_NOUNSET=1
    set +u
fi

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "source_env.sh: /opt/ros/jazzy/setup.bash not found; install ROS 2 Jazzy first." >&2
fi

if [ -f "$CINE_WS/install/setup.bash" ]; then
    source "$CINE_WS/install/setup.bash"
else
    echo "source_env.sh: $CINE_WS/install/setup.bash not found; run colcon build in $CINE_WS." >&2
fi

if [ "${_CINE_RESTORE_NOUNSET-}" = "1" ]; then
    set -u
    unset _CINE_RESTORE_NOUNSET
fi
