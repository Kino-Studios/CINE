#!/usr/bin/env bash
# Rebuild the Source One's Gazebo meshes from the CAD in a CINE-CAD checkout.
#
# Usage: build.sh <cine-cad-checkout> <scratch-directory> [mesh-directory]
#
# OCP_PYTHON names a Python with cadquery-ocp installed (default python3); BLENDER names
# a Blender 5 binary (default blender). Meshes go to the model's meshes/ folder unless a
# third argument says otherwise.
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 <cine-cad-checkout> <scratch-directory> [mesh-directory]" >&2
  exit 2
fi

readonly SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SOURCES="$1/sources"
readonly PARTS="$2"
readonly MESHES="${3:-${SCRIPT_DIRECTORY}/../../cine_gazebo/models/cine_source_one/meshes}"
readonly OCP_PYTHON="${OCP_PYTHON:-python3}"
readonly BLENDER="${BLENDER:-blender}"

split_step() {
  local step_file="$1" part="$2" deflection_mm="$3"
  "${OCP_PYTHON}" -I "${SCRIPT_DIRECTORY}/split_step_by_colour.py" \
    "${step_file}" "${PARTS}/${part}" "${deflection_mm}" >/dev/null
}

"${OCP_PYTHON}" -I "${SCRIPT_DIRECTORY}/split_iges_bodies.py" \
  "${SOURCES}/tbs_v5/SO_V5.iges" "${PARTS}/tbs_v5"
split_step "${SOURCES}/grabcad/motor.stp" motor 0.05
split_step "${SOURCES}/grabcad/prop.step" prop 0.05
split_step "${SOURCES}/grabcad/stack.step" stack 0.05
split_step "${SOURCES}/camera/Camera_module_3_wide_model_simple.stp" camera 0.03
split_step "${SOURCES}/pi5/rpi-5b_no_graphics.step" pi5 0.1

"${BLENDER}" -b -P "${SCRIPT_DIRECTORY}/build_meshes.py" -- \
  "${PARTS}" "${SOURCES}/guards" "${MESHES}"
