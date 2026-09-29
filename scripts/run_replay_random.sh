#!/bin/bash
# Run tb_bingo_hw_manager_replay_random with several seeds (after scripts/sim.sh
# has compiled the design into build/). Usage: scripts/run_replay_random.sh [N_SEEDS] [FIRST_SEED]
set -uo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
N=${1:-20}
FIRST=${2:-1}
command -v vsim >/dev/null || source ~micasusr/design/scripts/questasim_2025.2.rc
export MTI_VCO_MODE=64
cd "$ROOT/build" || exit 1
fail=0
for ((s = FIRST; s < FIRST + N; s++)); do
    log="sim_replay_random_seed$s.log"
    vsim -c -sv_seed "$s" tb_bingo_hw_manager_replay_random -t 1ns -voptargs=+acc \
        -do "run -all; quit -f" > "$log" 2>&1
    info=$(grep -m1 "\[RANDOM\] [0-9]* tasks" "$log" | sed 's/^# //')
    if grep -q "SIMULATION PASSED" "$log" && ! grep -q "Error:\|Fatal:" "$log"; then
        echo "seed $s PASS  $info"
    else
        echo "seed $s FAIL  $info  (build/$log)"
        fail=1
    fi
done
exit $fail
