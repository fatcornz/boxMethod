#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
GHDL=${GHDL:-ghdl}
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/boxmethod-test.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT
"$GHDL" -a --std=08 --workdir="$build_dir" spi_master.vhd ina229_interface.vhd tests/tb_ina229_interface.vhd
"$GHDL" -a --std=08 --workdir="$build_dir" controller_stub.vhd frequency_generator.vhd deadtime_generator.vhd fault_controller.vhd top.vhd
"$GHDL" -e --std=08 --workdir="$build_dir" top
"$GHDL" -e --std=08 --workdir="$build_dir" tb_ina229_interface
for half_period in 5 50; do
  for scenario in 0 1 2 3 4 5; do
    "$GHDL" -r --std=08 --workdir="$build_dir" tb_ina229_interface -gSCENARIO="$scenario" -gHALF_PERIOD="$half_period" --assert-level=error
  done
done
"$GHDL" --synth --std=08 --workdir="$build_dir" ina229_interface > "$build_dir/ina229_synth.vhd"
