#!/bin/bash
# Usage: ./check_bag.sh <bag_name>

if [ $# -ne 1 ]; then
    echo "Usage: $0 <bag_name>"
    exit 1
fi

BAG=$1

# Get duration in seconds (as float)
DURATION=$(ros2 bag info "$BAG" | grep "Duration" | awk '{print $2}' | sed 's/s//')

# Topics to check
declare -A EXPECTED_RATES=(
    ["/cam_sync/cam0/image_raw"]=20
    ["/cam_sync/cam1/image_raw"]=20
    ["/ouster/points"]=10
)

echo "Bag duration: ${DURATION}s"
echo

# Loop over topics and check rates
for TOPIC in "${!EXPECTED_RATES[@]}"; do
    COUNT=$(ros2 bag info "$BAG" | \
        awk -v topic="$TOPIC" '
            $0 ~ topic {
                getline; # next line has message count
                if ($1 == "Message" && $2 == "count:") print $3
            }'
    )

    if [ -z "$COUNT" ]; then
        echo "❌ Topic '$TOPIC' not found in bag."
        continue
    fi

    RATE=$(awk -v count="$COUNT" -v duration="$DURATION" 'BEGIN {printf "%.2f", count/duration}')
    EXPECTED=${EXPECTED_RATES[$TOPIC]}

    # Check if within ±10% of expected
    LOWER=$(awk -v exp="$EXPECTED" 'BEGIN {print exp*0.9}')
    UPPER=$(awk -v exp="$EXPECTED" 'BEGIN {print exp*1.1}')

    STATUS="❌"
    if (( $(echo "$RATE >= $LOWER && $RATE <= $UPPER" | bc -l) )); then
        STATUS="✅"
    fi

    echo "$STATUS $TOPIC: count=$COUNT, rate=${RATE}Hz (expected ~${EXPECTED}Hz)"
done
