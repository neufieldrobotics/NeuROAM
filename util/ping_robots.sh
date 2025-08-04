#!/bin/bash

# Ordered payload list
payloads=("payload0" "payload1" "payload2" "payload3" "payload4")

# Mapping from payload names to IPs
declare -A payload_ips=(
  ["payload0"]="10.19.30.100"
  ["payload1"]="10.19.30.101"
  ["payload2"]="10.19.30.102"
  ["payload3"]="10.19.30.103"
  ["payload4"]="10.19.30.104"
)

# Optional: iperf duration (seconds)
IPERF_DURATION=3

echo "🔌 Running connectivity and bandwidth check..."
echo "Ping timeout: 0.75s | iperf3 duration: ${IPERF_DURATION}s"
echo

for payload in "${payloads[@]}"; do
  ip="${payload_ips[$payload]}"
  echo "🔎 Checking $payload ($ip)..."

  # Ping
  output=$(ping -c 1 -W 1 "$ip" 2>/dev/null)
  if echo "$output" | grep -q "1 received"; then
    rtt=$(echo "$output" | grep -oP 'time=\K[0-9.]+')
    echo "  ✅ Ping OK — RTT: ${rtt} ms"

    # iperf3
    bw=$(iperf3 -c "$ip" -t $IPERF_DURATION -J 2>/dev/null \
          | jq '.end.sum_received.bits_per_second // empty' \
          | awk '{printf "%.2f Mbps\n", $1 / 1e6}')
    if [ -n "$bw" ]; then
      echo "  📶 Bandwidth: $bw"
    else
      echo "  ⚠️  iperf3 failed or timed out"
    fi

  else
    echo "  ❌ Ping failed — host unreachable"
  fi

  echo
done

