# Test Run – Part 3 End-to-End

---

## Before Going Outside

```bash
# On robot PC (host)
cd ~/Desktop/group9/mobile-robots/autonomous_nav
./run_docker.sh

# Inside container – rebuild only if code changed
colcon build --packages-select part3_explore --build-base ros2_ws/build --install-base install
source install/setup.bash

# Clear old waypoints
echo '[]' > /workspace/autonomous_nav/outputs/markers/waypoints.json

# Launch everything
ros2 launch part3_explore robot.launch.py
```

**Expected within ~10 seconds:**
```
[lifecycle_manager]      Managed nodes are active
[mission_manager]        MissionManager ready  state=IDLE
[display_node]           DisplayNode ready.
[waypoint_driver]        WaypointDriver ready.
[marker_detector]        MarkerDetector ready.
```

**Connect Foxglove on laptop:**
- Chrome → `https://app.foxglove.dev` → connect to `ws://192.168.2.215:8765`
- Add Image panel → topic: `/display/image/compressed`

---

## Phase 1 – Mapping

### Option A: Manual Mapping

Press **Triangle (△)** → state becomes `MANUAL_MAPPING`

Drive with **L1 (deadman) + left stick (forward/back) + right stick (turn)**. Cover the full area.

While mapping, drive to each marker, stop, and note the coordinates:
```bash
# In a second terminal inside the container:
ros2 run tf2_ros tf2_echo map base_link
# Read x and y from the Translation line
```

### Option B: Autonomous Mapping

Press **X (Cross)** → state becomes `MAPPING`

Robot drives itself via m-explore frontier exploration. Wait until the terminal shows:
```
[explore] No frontiers found, stopping.
```
This means the area is fully mapped.

---

## Write Waypoints (if OCR did not detect markers automatically)

```bash
cd /workspace/autonomous_nav
./write_waypoints.sh
```

Follow the prompts — select Greek letter, enter x/y coordinates from the TF output above.

**Verify the file:**
```bash
python3 -c "import json; print(json.load(open('/workspace/autonomous_nav/outputs/markers/waypoints.json')))"
```

---

## Phase 2 – Waypoint Navigation

Press **Square (□)** → state becomes `RAPID_NAV`

The robot will:
1. Compute the shortest visit order from current position
2. Navigate to each waypoint via Nav2 (obstacle-avoiding)
3. Return to the starting position

**Watch in Foxglove:** waypoint progress list, planned path on map, robot position (red dot).

**If something goes wrong:**
- E-stop triggered → wait 3 seconds for auto-clear, or press **O (Circle)**
- Robot stuck → press **O** → IDLE, then restart Phase 1 or 2
- Wrong waypoints → Ctrl+C, re-run `write_waypoints.sh`, relaunch

---

## After the Run

```bash
cat /workspace/autonomous_nav/outputs/markers/waypoints.json
cat /workspace/autonomous_nav/outputs/markers/obstacles.json
ls  /workspace/autonomous_nav/outputs/markers/photos/
```
