#!/usr/bin/env bash
# Usage: ./check_bag.sh <bag_dir>

set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 <bag_dir>"
  exit 1
fi

BAG="$1"
INFO="$(ros2 bag info "$BAG")"
# INFO="$(cat /home/alan/NeuROAM/util/record/sample_output.info)"

# the percentage of the expected count that is acceptable
# e.g., 1.1 means 10% more than expected is acceptable
# 0.9 means 10% less than expected is acceptable
EPS=0.01 # 1% tolerance
LB_TOLER=$(echo "1 - $EPS" | bc -l)  # Lower bound tolerance
UB_TOLER=$(echo "1 + $EPS" | bc -l)  #

echo "Allowing ${LB_TOLER}x to ${UB_TOLER}x of expected message counts."

# Extract duration in seconds (strip trailing 's')
duration=$(awk '
  BEGIN{FS="[[:space:]]+"}
  /^Duration:/ {
    gsub(/s$/,"",$2);
    print $2; exit
  }' <<<"$INFO")

if [ -z "$duration" ]; then
  echo "Could not parse Duration from ros2 bag info."
  exit 2
fi

# Robust count extractor: works for both one-line (| delimited) and block formats
count_for () {
  local topic="$1"
  awk -v t="$topic" '
    # Match exact topic after "Topic:"
    # Case A: one-line, pipe-delimited
    $0 ~ ("Topic:[[:space:]]*" t "[[:space:]]*\\|") {
      n=split($0, parts, /\|/)
      for (i=1;i<=n;i++) {
        if (parts[i] ~ /[[:space:]]*Count:[[:space:]]*/) {
          sub(/^.*Count:[[:space:]]*/,"",parts[i])
          gsub(/[[:space:]]+/,"",parts[i])
          print parts[i]; exit
        }
      }
    }
    # Case B: multi-line block starting with Topic: <name>
    $0 ~ ("Topic:[[:space:]]*" t "$") { inblk=1; next }
    inblk && /^[[:space:]]*Count:[[:space:]]*/ {
      c=$0; sub(/^.*Count:[[:space:]]*/,"",c); gsub(/[[:space:]]+/,"",c)
      print c; exit
    }
    # If we hit a new Topic: before finding Count:, stop tracking
    inblk && /^Topic:/ { inblk=0 }
  ' <<<"$INFO"
}

declare -A expected=( \
  ["/cam_sync/cam0/image_raw"]=20 \
  ["/cam_sync/cam1/image_raw"]=20 \
  ["/ouster/points"]=10 \
)

echo "Bag: $BAG"
echo "Duration: ${duration}s"
echo

for t in "${!expected[@]}"; do
  count="$(count_for "$t" || true)"
  if [[ -z "$count" ]]; then
    echo "❌ $t: not found"
    continue
  fi
  # Compute observed rate
  rate=$(awk -v c="$count" -v d="$duration" 'BEGIN{printf "%.2f", (d>0)?c/d:0}')
  exp_hz="${expected[$t]}"
  exp_msgs=$(echo "$exp_hz * $duration" | bc -l)
  msgs_missing=$(awk -v c="$count" -v e="$exp_msgs" 'BEGIN{printf "%.0f", (e-c)}')
  lo=$(echo "$exp_hz * $LB_TOLER" | bc -l)
  hi=$(echo "$exp_hz * $UB_TOLER" | bc -l)
  within=$(awk -v r="$rate" -v lo="$lo" -v hi="$hi" 'BEGIN{print (r>=lo && r<=hi)?"1":"0"}')
  if [[ "$within" == "1" ]]; then
    echo "✅ $t: count=$count, rate=${rate} Hz (target ~${exp_hz} Hz, ${percent}%)"
  else
    echo "❌ $t: count=$count, rate=${rate} Hz (target ~${exp_hz} Hz, ${percent}%)"
  fi
done

