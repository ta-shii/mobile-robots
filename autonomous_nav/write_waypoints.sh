#!/bin/bash
# write_waypoints.sh
# Interactively write 3 waypoints to waypoints.json before Phase 2.
# Run this inside the container after mapping, before pressing Square.
#
# Usage:  ./write_waypoints.sh

WP_FILE="/workspace/autonomous_nav/outputs/markers/waypoints.json"

LETTERS=("Alpha" "Beta" "Gamma" "Delta" "Epsilon" "Zeta" "Eta" "Theta"
         "Iota" "Kappa" "Lambda" "Mu" "Nu" "Xi" "Omicron" "Pi"
         "Rho" "Sigma" "Tau" "Upsilon" "Phi" "Chi" "Psi" "Omega")

CODES=("A" "B" "G" "D" "E" "Z" "H" "Q"
       "I" "K" "L" "M" "N" "C" "O" "P"
       "R" "S" "T" "U" "F" "X" "Y" "W")

echo ""
echo "=========================================="
echo "  Waypoint Writer"
echo "  Output: $WP_FILE"
echo "=========================================="
echo ""
echo "Get coordinates by running in another terminal:"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo ""

entries=()

for i in 1 2 3; do
    echo "------------------------------------------"
    echo "Waypoint $i of 3"
    echo ""

    # Show Greek letter menu
    echo "Greek letters:"
    for j in "${!LETTERS[@]}"; do
        printf "  %2d) %-10s" "$((j+1))" "${LETTERS[$j]}"
        if (( (j+1) % 4 == 0 )); then echo ""; fi
    done
    echo ""
    echo ""

    while true; do
        read -rp "  Select letter (1-24): " choice
        if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= 24 )); then
            greek_name="${LETTERS[$((choice-1))]}"
            letter="${CODES[$((choice-1))]}"
            break
        fi
        echo "  Invalid — enter a number between 1 and 24."
    done

    while true; do
        read -rp "  x coordinate: " x
        if [[ "$x" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then break; fi
        echo "  Invalid — enter a number (e.g. 4.258 or -6.812)."
    done

    while true; do
        read -rp "  y coordinate: " y
        if [[ "$y" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then break; fi
        echo "  Invalid — enter a number."
    done

    entries+=("{\"letter\": \"$letter\", \"greek_name\": \"$greek_name\", \"x\": $x, \"y\": $y}")
    echo "  -> $greek_name ($letter)  map=($x, $y)"
    echo ""
done

echo "------------------------------------------"
echo ""
echo "Waypoints to write:"
for i in "${!entries[@]}"; do
    echo "  $((i+1)). ${entries[$i]}"
done
echo ""
read -rp "Write to $WP_FILE? (y/n): " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "Cancelled."
    exit 0
fi

mkdir -p "$(dirname "$WP_FILE")"

cat > "$WP_FILE" << EOF
[
  ${entries[0]},
  ${entries[1]},
  ${entries[2]}
]
EOF

echo ""
echo "Written successfully:"
cat "$WP_FILE"
echo ""
echo "Press Square (□) on the gamepad to start Phase 2."
