#!/usr/bin/env bash
# Sweep controller configurations over an identical address stream.
# One RTL source; only NBANKS and QDEPTH change, so any difference in the
# numbers is attributable to the microarchitecture and not the workload.
set -u
cd "$(dirname "$0")/.."

NREQ=${NREQ:-500}
CONFIGS=("1 1" "2 1" "4 1" "4 8" "4 16" "8 16")

printf '\n'
printf 'Identical %s-request stream through every configuration\n' "$NREQ"
printf '%s\n' "-------------------------------------------------------------------------------"
printf '%-7s %-7s %10s %10s %10s %10s %8s\n' \
       "banks" "queue" "cycles" "cyc/req" "avg lat" "worst" "result"
printf '%s\n' "-------------------------------------------------------------------------------"

BASE=""
for cfg in "${CONFIGS[@]}"; do
    set -- $cfg
    NB=$1; QD=$2
    iverilog -g2012 -DNB="$NB" -DQD="$QD" -DNREQ="$NREQ" \
             -o sim/mb_${NB}_${QD}.vvp tb/tb_dram_ctrl_mb.v rtl/dram_ctrl_mb.v 2>/dev/null
    OUT=$(vvp sim/mb_${NB}_${QD}.vvp 2>/dev/null)
    CSV=$(echo "$OUT" | grep '^CSV,')
    IFS=, read -r _ nb qd cycles avglat worst hits errs <<< "$CSV"

    CPR=$(awk "BEGIN{printf \"%.1f\", $cycles/$NREQ}")
    if [ -z "$BASE" ]; then BASE=$cycles; fi
    SPD=$(awk "BEGIN{printf \"%.2fx\", $BASE/$cycles}")

    if [ "$errs" = "0" ]; then RES="PASS"; else RES="FAIL($errs)"; fi
    printf '%-7s %-7s %10s %10s %10s %10s %8s   %s\n' \
           "$nb" "$qd" "$cycles" "$CPR" "$avglat" "$worst" "$RES" "$SPD"
done
printf '%s\n' "-------------------------------------------------------------------------------"
printf 'Speedup is total cycles relative to the 1-bank in-order baseline.\n\n'
