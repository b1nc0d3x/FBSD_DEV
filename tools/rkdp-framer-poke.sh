#!/bin/sh
#
# Poke a Cadence cdn-dp framer register via the rk_cdn_dp debug
# sysctls.  Drives the running target board over ssh, using sshpass
# for the noninteractive password feed required by the dev-board
# user account.
#
# Configure via environment, not in source:
#   RP64_HOST=admin@192.168.1.10  (default)
#   RP64_PORT=60022               (default)
#   RP64_PASS=<password>          (REQUIRED — exported, not hardcoded)
#
# Example:
#   RP64_PASS=secret ./rkdp-framer-poke.sh 0x2200 0x000000cb 0x000000eb
#
set -eu

if [ "$#" -lt 2 ]; then
	echo "usage: $0 <addr_hex> <value_hex> [value_hex ...]" >&2
	echo "example: $0 0x2200 0x000000cb 0x000000eb" >&2
	exit 64
fi

: "${RP64_HOST:=admin@192.168.1.10}"
: "${RP64_PORT:=60022}"
if [ -z "${RP64_PASS:-}" ]; then
	echo "$0: set RP64_PASS in the environment (not hardcoded)" >&2
	exit 65
fi

ADDR="$1"
shift

run_sysctl() {
	sshpass -p "$RP64_PASS" ssh -p "$RP64_PORT" \
		-o StrictHostKeyChecking=no "$RP64_HOST" "$@"
}

for VALUE in "$@"; do
	printf '=== poke addr=%s value=%s ===\n' "$ADDR" "$VALUE"
	run_sysctl "echo \"$RP64_PASS\" | sudo -S sysctl dev.rk_cdn_dp.0.debug_reg_addr=$ADDR"
	run_sysctl "echo \"$RP64_PASS\" | sudo -S sysctl dev.rk_cdn_dp.0.debug_reg_value=$VALUE"
	run_sysctl "echo \"$RP64_PASS\" | sudo -S sysctl dev.rk_cdn_dp.0.debug_reg_write_now=1"
	sleep 0.5
	run_sysctl "echo \"$RP64_PASS\" | sudo -S sysctl dev.rk_cdn_dp.0.debug_reg_read_now=1"
	run_sysctl "echo \"$RP64_PASS\" | sudo -S sysctl dev.rk_cdn_dp.0.aux_probe_now=1"
	sleep 0.5
done
