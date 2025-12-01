#!/bin/bash

# Configuration
DEVICE="/dev/ovis0"

echo "Starting Resource Monitor -> $DEVICE"

while true; do
    # 1. Get the top CPU process
    # -A: All processes
    # -o comm,pcpu: Output Command Name and CPU Percentage only
    # --sort=-pcpu: Sort by CPU (descending)
    # head -n 2: Get the first 2 lines (Header + Top Process)
    # tail -n 1: Drop the header, keep the process
    RAW_DATA=$(ps -Ao comm,pcpu --sort=-pcpu | head -n 2 | tail -n 1)

    # 2. Format it using awk (Remove extra spaces, add %)
    # Input: "    stress-ng  99.5"
    # Output: "stress-ng:99.5%"
    FORMATTED_MSG=$(echo "$RAW_DATA" | awk '{print $1 ":" $2 "%"}')

    # 3. Write to the USB Driver
    # We use 'tee' to handle the write gracefully.
    # echo automatically adds the '\n' your STM32 needs.
    echo "$FORMATTED_MSG" | sudo tee "$DEVICE" > /dev/null

    # 4. Debug print to console so you know it's working
    echo "Sent: $FORMATTED_MSG"

    # 5. Wait before next update (don't spam the USB bus)
    sleep 1
done
