#!/bin/sh
set -eu

run_step() {
	printf '\n=== %s ===\n' "$1"
}

poke() {
	addr="$1"
	val="$2"
	sysctl "dev.rk_cdn_dp.0.debug_reg_addr=$addr" >/dev/null
	sysctl "dev.rk_cdn_dp.0.debug_reg_value=$val" >/dev/null
	sysctl dev.rk_cdn_dp.0.debug_reg_write_now=1 >/dev/null
	sysctl dev.rk_cdn_dp.0.debug_reg_read_now=1 >/dev/null
}

run_step "baseline-read"
for a in 0x220c 0x2210 0x2290 0x2294; do
	sysctl "dev.rk_cdn_dp.0.debug_reg_addr=$a" >/dev/null
	sysctl dev.rk_cdn_dp.0.debug_reg_read_now=1 >/dev/null
	sleep 1
done

sleep 3
run_step "stream-off"
poke 0x2294 0x00000000

sleep 3
run_step "stream-on"
poke 0x2294 0x00000001

sleep 3
run_step "msa-misc-zero"
poke 0x2290 0x00000000

sleep 3
run_step "msa-misc-rgb8"
poke 0x2290 0x00000020

sleep 3
run_step "pxl-repr-6bpc"
poke 0x220c 0x00000100

sleep 3
run_step "pxl-repr-rgb8"
poke 0x220c 0x00000102

sleep 3
run_step "sp-zero"
poke 0x2210 0x00000000

sleep 3
run_step "sp-vsp"
poke 0x2210 0x00000001

sleep 3
sysctl dev.rk_cdn_dp.0.aux_probe_now=1 >/dev/null
