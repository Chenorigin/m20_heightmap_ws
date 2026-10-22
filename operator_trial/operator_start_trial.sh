#!/usr/bin/env bash
set -euo pipefail
bundle=$(cd "$(dirname "$0")" && pwd)
root=/home/user/m20_heightmap/releases/20261015_1d75c14e4ab5
[[ "$EUID" == 0 && "$(cat /opt/robot/os_type)" == CA9B_AOS ]] || exit 2
[[ -t 0 ]] || exit 2
[[ "$(readlink -f /home/user/m20_heightmap/current)" == "$root" ]] || exit 2
cd "$root"
exec 9>runtime/student_control.lock
flock -n 9 || { echo 'An instance already owns this control lock'; exit 2; }
source scripts/board_env.sh
export M20_TRIAL_RELEASE="$root"
export M20_GAMEPAD_TRANSPORT=passive M20_GAMEPAD_INTERFACE=eth0
install/m20_student_sdk/lib/m20_student_sdk/gamepad_observe "$M20_GAMEPAD_INTERFACE" >runtime/operator_handset_final.json
export M20_VERIFIED_VENDOR_PID=$(python3 scripts/check_vendor_owner.py --pid-only)
[[ "$M20_VERIFIED_VENDOR_PID" =~ ^[1-9][0-9]*$ ]] || exit 2
python3 "$bundle/sdk_init_preflight.py" --profile v1 --control --seconds 8 >runtime/operator_sdk_init_preflight.json
# This preflight authorizes initialization only. The unchanged runner validates
# every terrain input and holds its current joints when policy inputs fail.
export M20_OPERATOR_START=START_STUDENT M20_IMU_SOURCE=direct M20_REMOTE_CONTROL=gamepad
export M20_SDK_MODE_CONFIRMED=I_CONFIRMED
export M20_POLICY_PATH="$root/models/v1/policy.onnx"
export M20_ROBOT_CONFIG="$root/deploy/robot.json" M20_HEIGHTMAP_CONFIG="$root/runtime/heightmap_v1.local.json"
export M20_EXPECTED_GEOMETRY_HASH=$(python3 -c 'import os;from m20_heightmap.commission import geometry_hash;print(geometry_hash(os.getcwd(),os.environ["M20_HEIGHTMAP_CONFIG"]))')
python3 - "$root" <<'PY'
import json,sys
from pathlib import Path
from m20_heightmap.commission import commission_matches
root=Path(sys.argv[1]);receipt=json.loads((root/'runtime/operator_authorization.json').read_text())
assert receipt['user_requested_all_start'] is True and receipt['user_requested_ignore_terrain_quality_rejections'] is True
assert receipt['user_confirmed_prone_supported'] is True and receipt['user_confirmed_sdk_mode'] is True
assert receipt['user_confirmed_estop_attendant'] is True
assert receipt['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert commission_matches(root,root/'runtime/heightmap_v1.local.json','v1')
PY
echo 'USER AUTHORIZED SDK INITIALIZATION. Robot confirmed prone/supported. Starting Idle: zero gain output; this is not standing hold. No automatic standing or policy transition.'
exec taskset -c 4,5 install/m20_student_sdk/lib/m20_student_sdk/student_control
