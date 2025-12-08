# Wraith Robot - Quick Setup Guide

**Complete Flask API Control System**

---

## 📦 What You Have

### ✅ New Files Created:

1. **Flask Server** (Raspberry Pi):
   ```
   wraithphase2/SLAM/slam_ws/src/slam_robot/slam_robot/flask_control_server.py
   ```

2. **Updated Flutter Service**:
   ```
   lib/services/mapping_service.dart
   ```

3. **Documentation**:
   - `FLASK_API_COMPLETE_GUIDE.md` - Full API reference
   - `WRAITH_SLAM_SYSTEM_DOCUMENTATION.md` - System architecture
   - `QUICK_SETUP_GUIDE.md` - This file

---

## 🚀 Setup Steps

### Step 1: Copy Flask Server to Raspberry Pi

On your **development machine**, the file is at:
```
D:\FYP\wraith-app-mapping\wraithphase2\SLAM\slam_ws\src\slam_robot\slam_robot\flask_control_server.py
```

**Transfer to Raspberry Pi:**

```bash
# Option A: Using SCP (from Windows)
scp "D:\FYP\wraith-app-mapping\wraithphase2\SLAM\slam_ws\src\slam_robot\slam_robot\flask_control_server.py" pi@raspberrypi.local:~/SLAM/slam_ws/src/slam_robot/slam_robot/

# Option B: Manually copy via USB/network drive
```

### Step 2: Install Dependencies on Raspberry Pi

SSH into Raspberry Pi:

```bash
ssh pi@raspberrypi.local
```

Install Flask:

```bash
pip3 install flask flask-cors pyyaml
```

### Step 3: Register Flask Server in Package

Edit `setup.py`:

```bash
cd ~/SLAM/slam_ws/src/slam_robot
nano setup.py
```

Add this line to `entry_points['console_scripts']`:

```python
'flask_control_server = slam_robot.flask_control_server:main',
```

Full example:
```python
entry_points={
    'console_scripts': [
        'motor_controller = slam_robot.motor_controller:main',
        'odometry_publisher = slam_robot.odometry_publisher:main',
        'imu_publisher = slam_robot.imu_publisher:main',
        'obstacle_avoidance = slam_robot.obstacle_avoidance:main',
        'auto_waypoint_generator = slam_robot.auto_waypoint_generator:main',
        'waypoint_navigator = slam_robot.waypoint_navigator:main',
        'flask_control_server = slam_robot.flask_control_server:main',  # ← ADD THIS
    ],
},
```

### Step 4: Build ROS2 Package

```bash
cd ~/SLAM/slam_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select slam_robot --symlink-install
source install/setup.bash
```

### Step 5: Start Flask Server

```bash
ros2 run slam_robot flask_control_server
```

You should see:

```
======================================================================
     WRAITH ROBOT - COMPLETE FLASK CONTROL SERVER
======================================================================
  Robot IP: 192.168.1.100
  Port: 5000
  Base URL: http://192.168.1.100:5000
======================================================================
[✓] Flask API server started
[⏳] Listening on http://192.168.1.100:5000
======================================================================
```

**Note the IP address** - you'll enter this in your Flutter app!

### Step 6: Update and Run Flutter App

The Flutter app is **already updated** with the new API endpoints.

Just rebuild:

```bash
cd D:\FYP\wraith-app-mapping
flutter pub get
flutter run -d windows   # or chrome, or your device
```

---

## 🎮 How to Use

### Complete Mapping Workflow

**In Flutter App:**

1. **Connect to Robot**
   - Enter IP: `192.168.1.100`
   - Port: `5000`
   - Tap "Activate Robot" ✅

2. **Start Mapping**
   - Tap "Start Mapping"
   - Wait 3-5 seconds
   - Tap "Activate SLAM" ✅

3. **Drive Robot**
   - Use D-pad to manually drive
   - OR tap "Enable Auto Mode" for autonomous exploration

4. **Save Map**
   - Drive for 3-5 minutes
   - Tap "Save Map"
   - Enter map name: `my_office`
   - Tap "Confirm" ✅

5. **Stop Mapping**
   - Tap "Stop Mapping" ✅

### Complete Navigation Workflow

**In Flutter App:**

1. **View Maps**
   - Switch to "Maps" tab
   - See list of saved maps with thumbnails

2. **Select Map**
   - Tap on a map (e.g., "my_office")
   - View waypoint list

3. **Start Navigation**
   - Tap "Start Navigation"
   - Wait 10-15 seconds for localization

4. **Navigate to Waypoints**
   - Tap a waypoint (e.g., "Point_B")
   - Tap "Navigate to Waypoint"
   - Robot drives autonomously! ✅

5. **Stop Navigation**
   - Tap "Stop Navigation"

---

## 🔧 API Endpoints Overview

### Connection
- `GET /status` - Robot status
- `GET /get_ip` - Get IP address
- `GET /get_distance` - Ultrasonic sensor

### Mapping
- `POST /mapping/start` - Start SLAM mapping (launches ROS2 subprocess)
- `POST /slam/activate` - Activate SLAM Toolbox
- `POST /mapping/stop` - Stop mapping
- `POST /mapping/save` - Save map + waypoints

### Control
- `POST /control` - Manual D-pad control
  - Body: `forward_start`, `forward_stop`, etc.
  - Body: `auto_start`, `auto_stop`
  - Body: `speed+`, `speed-`

### Navigation
- `POST /navigation/start` - Start navigation (launches ROS2 subprocess)
- `POST /navigate` - Navigate to waypoint
- `POST /navigation/stop` - Stop navigation

### Maps & Waypoints
- `GET /maps/list` - List all saved maps
- `GET /maps/<name>/waypoints` - Get waypoints
- `GET /maps/<name>/image` - Get map PNG

---

## 🎯 What Changed from Old System

### Before (Manual Terminal Commands):

```bash
# Terminal 1
ros2 launch slam_robot mapping.launch.py control_mode:=manual

# Terminal 2
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# Drive around...

# Terminal 3
python3 scripts/save_map.py my_map

# Terminal 4
ros2 launch slam_robot navigation.launch.py map:=my_map

# Terminal 5
ros2 run slam_robot go_to Point_B
```

### After (Flutter App Buttons):

```
1. Tap "Start Mapping"
2. Tap "Activate SLAM"
3. Drive with D-pad
4. Tap "Save Map"
5. Tap "Start Navigation"
6. Tap waypoint → "Navigate"
```

---

## 🐛 Troubleshooting

### Issue: Flask server won't start

```bash
# Install missing dependencies
pip3 install flask flask-cors pyyaml

# Check if port 5000 is available
netstat -tuln | grep 5000
```

### Issue: Can't connect from Flutter app

```bash
# Check Flask server is running
ros2 run slam_robot flask_control_server

# Check IP address
hostname -I

# Test from browser
# Open: http://<robot-ip>:5000/status
# Should show: {"status": "ok", ...}
```

### Issue: Mapping fails to start

```bash
# Check workspace exists
ls ~/SLAM/slam_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/SLAM/slam_ws/install/setup.bash

# Test launch manually
ros2 launch slam_robot mapping.launch.py control_mode:=manual
```

### Issue: Maps not saving

```bash
# Create maps directory
mkdir -p ~/slam_maps

# Check permissions
chmod 755 ~/slam_maps

# Test save script
cd ~/SLAM/slam_ws
python3 src/slam_robot/scripts/save_map.py test_map
```

---

## 📱 Flutter App Features

### Mapping Screen

- ✅ Robot connection with IP/port input
- ✅ Start/Stop mapping buttons
- ✅ SLAM activation button
- ✅ D-pad manual control (4 directions + center stop)
- ✅ Auto mode toggle
- ✅ Speed control (+/-)
- ✅ Save map with custom name
- ✅ Real-time status display

### Maps Screen

- ✅ Grid view of all saved maps
- ✅ Map thumbnails (PNG images)
- ✅ Waypoint count per map
- ✅ Tap to view details

### Map Detail Screen

- ✅ Waypoint list with coordinates
- ✅ Start/stop navigation
- ✅ Navigate to waypoint button
- ✅ Real-time navigation status

---

## 🎓 Key Concepts

### Subprocess Management

The Flask server manages ROS2 processes using Python `subprocess`:

```python
# Start mapping
mapping_process = subprocess.Popen(
    "ros2 launch slam_robot mapping.launch.py control_mode:=manual",
    shell=True,
    preexec_fn=os.setsid  # Create process group
)

# Stop mapping
os.killpg(os.getpgid(mapping_process.pid), signal.SIGINT)
```

This allows **one-button control** of entire ROS2 launch files from Flask API!

### Waypoint Format

Waypoints are stored in YAML:

```yaml
waypoints:
  - label: Point_A
    x: 0.0
    y: 0.0
    theta: 0.0
    environmental_features:
      north: 0.81
      northeast: 16.0
      # ... 8 directions
```

Environmental features enable **automatic localization** without manual pose setting!

### Map Files

Each saved map creates 4 files:

1. `map.pgm` - Grayscale occupancy grid (for Nav2)
2. `map.png` - Color image (for Flutter display)
3. `map.yaml` - Metadata (resolution, origin)
4. `map_waypoints.yaml` - Waypoint list

---

## ✅ Testing Checklist

### After Setup:

- [ ] Flask server starts without errors
- [ ] Browser can access `http://<robot-ip>:5000/status`
- [ ] Flutter app connects successfully
- [ ] D-pad controls move robot
- [ ] Mapping starts and SLAM activates
- [ ] Waypoints are generated (check logs)
- [ ] Map saves successfully
- [ ] Maps list shows saved maps
- [ ] Navigation starts with map
- [ ] Robot navigates to waypoint

---

## 📚 Documentation Files

- **`FLASK_API_COMPLETE_GUIDE.md`** ← Full API reference with examples
- **`WRAITH_SLAM_SYSTEM_DOCUMENTATION.md`** ← Complete system architecture
- **`QUICK_SETUP_GUIDE.md`** ← This file

---

## 🎉 You're Ready!

Your complete system is now set up:

1. ✅ Flask server with subprocess management
2. ✅ Complete HTTP API for all operations
3. ✅ Updated Flutter app with all endpoints
4. ✅ Full mapping + navigation workflow
5. ✅ Map storage and retrieval
6. ✅ Waypoint-based navigation

**Start the Flask server and enjoy controlling your robot from your phone!**

---

*Wraith Robot - Flask API Control System v2.0*
*Setup Guide - December 2025*
