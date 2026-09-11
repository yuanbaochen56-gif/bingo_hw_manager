#!/bin/bash
set -e
source ~micasusr/design/scripts/questasim_2025.2.rc
cd ~/thesis/bingo_hw_manager
mkdir -p build

podman run --rm \
  -v "$PWD:$PWD" \
  -w "$PWD/build" \
  ghcr.io/kuleuven-micas/hemaia:main \
  bash -lc 'VSIM=true ../scripts/compile_vsim.sh'

cd build
vsim -c -do 'exit -code [source compile.tcl]'
../scripts/run_vsim.sh bingo_hw_manager_dep_matrix

