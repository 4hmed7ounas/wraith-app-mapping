# Wraith Robot - Complete Flask API Guide

**Version:** 2.0
**Platform:** Raspberry Pi 5 + ROS2 Jazzy
**Date:** December 2025

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [Setup Instructions](#setup-instructions)
3. [Complete API Reference](#complete-api-reference)
4. [Flutter App Integration](#flutter-app-integration)
5. [Workflow Examples](#workflow-examples)
6. [Troubleshooting](#troubleshooting)

---

## Overview

### What Changed?

**New Flask Server:** `flask_control_server.py`
- **Replaces:** `manual_control_server.py`
- **New Features:**
  - ✅ Full subprocess management for ROS2 launches
  - ✅ Complete mapping lifecycle control (start/activate/stop/save)
  - ✅ Navigation system control
  - ✅ Map and waypoint management API
  - ✅ Base64 map image delivery for Flutter display

### Architecture

```
Flutter App (Mobile/Desktop)
      ↓ HTTP API
Flask Control Server (Port 5000)
      ↓ Subprocess Management
ROS2 Processes (Mapping, Navigation, SLAM)
      ↓ Hardware Control
Robot (Motors, Sensors, Lidar)
```

---

## Setup Instructions

### 1. Install Flask Server Dependencies

On Raspberry Pi:

```bash
# Install Flask and CORS support
pip3 install flask flask-cors pyyaml

# Install ROS2 dependencies (if not already installed)
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-rplidar-ros \
                 ros-jazzy-nav2-bringup ros-jazzy-robot-localization
```

### 2. Add Flask Server to Package

The new server is located at:
```
D:\FYP\wraith-app-mapping\wraithphase2\SLAM\slam_ws\src\slam_robot\slam_robot\flask_control_server.py
```

**On Raspberry Pi**, copy this file to:
```bash
~/SLAM/slam_ws/src/slam_robot/slam_robot/flask_control_server.py
```

### 3. Register in setup.py

Edit `~/SLAM/slam_ws/src/slam_robot/setup.py`:

```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'flask_control_server = slam_robot.flask_control_server:main',  # ADD THIS LINE
    ],
},
```

### 4. Build Package

```bash
cd ~/SLAM/slam_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select slam_robot --symlink-install
source install/setup.bash
```

### 5. Start Flask Server

```bash
# Start the Flask control server
ros2 run slam_robot flask_control_server

# Or specify custom port
ros2 run slam_robot flask_control_server --ros-args -p port:=5000
```

**Output:**
```
======================================================================
     WRAITH ROBOT - COMPLETE FLASK CONTROL SERVER
======================================================================
  Robot IP: 192.168.1.100
  Port: 5000
  Base URL: http://192.168.1.100:5000
======================================================================
Available API Endpoints:
  Connection:
    GET  /status                  - Robot status
    GET  /get_ip                  - Get IP address
    GET  /get_distance            - Ultrasonic sensor

  Mapping:
    POST /mapping/start           - Start SLAM mapping
    POST /slam/activate           - Activate SLAM Toolbox
    POST /mapping/stop            - Stop mapping
    POST /mapping/save            - Save map + waypoints

  Control:
    POST /control                 - Manual D-pad control

  Navigation:
    POST /navigation/start        - Start navigation (JSON: map_name)
    POST /navigation/stop         - Stop navigation
    POST /navigate                - Go to waypoint (JSON: waypoint)

  Maps & Waypoints:
    GET  /maps/list               - List all saved maps
    GET  /maps/<name>/waypoints   - Get waypoint list
    GET  /maps/<name>/image       - Get map PNG image
======================================================================
```

### 6. Update Flutter App

The `mapping_service.dart` has already been updated with all new endpoints.

Just rebuild your Flutter app:

```bash
cd ~/wraith-app-mapping
flutter pub get
flutter run
```

---

## Complete API Reference

### Base URL

```
http://<raspberry-pi-ip>:5000
```

Example: `http://192.168.1.100:5000`

---

## 📡 Connection & Status Endpoints

### GET `/status`

Check robot status and system state.

**Response:**
```json
{
  "status": "ok",
  "connected": true,
  "mapping_active": false,
  "navigation_active": false,
  "slam_activated": false,
  "auto_mode": false,
  "speed": 30.0
}
```

**Flutter Usage:**
```dart
final status = await mappingService.getStatus();
if (status['mapping_active']) {
  print('Mapping is running');
}
```

---

### GET `/get_ip`

Get robot's IP address.

**Response:**
```json
{
  "ip": "192.168.1.100"
}
```

---

### GET `/get_distance`

Get ultrasonic sensor distance reading.

**Response:**
```json
{
  "distance": 45.23
}
```

Distance in centimeters.

---

## 🗺️ Mapping Control Endpoints

### POST `/mapping/start`

Start SLAM mapping session. This launches `mapping.launch.py` in subprocess.

**Request:** No body required

**Response:**
```json
{
  "status": "ok",
  "message": "Mapping started successfully",
  "pid": 12345,
  "note": "Now activate SLAM using /slam/activate endpoint"
}
```

**What Happens:**
1. Launches ROS2 mapping launch file in background subprocess
2. Starts: RPLidar, IMU, odometry, motor controller, auto-waypoint generator
3. **Does NOT start SLAM yet** - requires activation (next step)

**Flutter Usage:**
```dart
final result = await mappingService.startMapping();
if (result['status'] == 'ok') {
  print('Mapping process started, PID: ${result['pid']}');
}
```

---

### POST `/slam/activate`

Activate SLAM Toolbox lifecycle (configure + activate). Call this **AFTER** `/mapping/start`.

**Request:** No body required

**Response:**
```json
{
  "status": "ok",
  "message": "SLAM activated successfully",
  "mapping_active": true
}
```

**What Happens:**
1. Waits 3 seconds for SLAM Toolbox node to initialize
2. Runs: `ros2 lifecycle set /slam_toolbox configure`
3. Runs: `ros2 lifecycle set /slam_toolbox activate`
4. Robot now actively mapping environment

**Flutter Usage:**
```dart
// Wait a few seconds after startMapping()
await Future.delayed(Duration(seconds: 3));

final result = await mappingService.activateSlam();
if (result['status'] == 'ok') {
  print('SLAM is now active - robot is mapping!');
}
```

---

### POST `/mapping/stop`

Stop SLAM mapping session. Kills mapping subprocess.

**Request:** No body required

**Response:**
```json
{
  "status": "ok",
  "message": "Mapping stopped successfully"
}
```

**What Happens:**
1. Sends SIGINT to mapping process group
2. Terminates all ROS2 nodes (SLAM, lidar, motors, etc.)
3. Resets slam_activation_done flag

**Flutter Usage:**
```dart
final result = await mappingService.stopMapping();
if (result['status'] == 'ok') {
  print('Mapping stopped');
}
```

---

### POST `/mapping/save`

Save current map with waypoints to disk.

**Request (JSON):**
```json
{
  "map_name": "my_office_map"
}
```

**Response:**
```json
{
  "status": "ok",
  "message": "Map saved successfully",
  "map_name": "my_office_map",
  "files": {
    "pgm": "/home/pi/slam_maps/my_office_map.pgm",
    "png": "/home/pi/slam_maps/my_office_map.png",
    "yaml": "/home/pi/slam_maps/my_office_map.yaml",
    "waypoints": "/home/pi/slam_maps/my_office_map_waypoints.yaml"
  },
  "png_base64": "iVBORw0KGgoAAAANSUhEUgA..."
}
```

**What Happens:**
1. Runs `save_map.py` script
2. Generates 4 files:
   - `.pgm` - Grayscale occupancy grid
   - `.png` - Visualization image
   - `.yaml` - Map metadata
   - `_waypoints.yaml` - Waypoint list with coordinates
3. Returns PNG as base64 for Flutter to display

**Flutter Usage:**
```dart
final result = await mappingService.saveMapping(
  mapName: 'my_office_map'
);

if (result['status'] == 'ok') {
  final pngBase64 = result['png_base64'];
  // Display in Flutter Image.memory(base64Decode(pngBase64))

  final waypoints = result['files']['waypoints'];
  print('Waypoints saved to: $waypoints');
}
```

---

## 🎮 Manual Control Endpoints

### POST `/control`

Send control command to robot motors.

**Request:** Plain text body with command

**Commands:**
- `forward_start` - Start moving forward
- `forward_stop` - Stop forward movement
- `backward_start` - Start moving backward
- `backward_stop` - Stop backward movement
- `left_start` - Start turning left
- `left_stop` - Stop turning left
- `right_start` - Start turning right
- `right_stop` - Stop turning right
- `speed+` - Increase speed by 10%
- `speed-` - Decrease speed by 10%
- `auto_start` - Enable autonomous exploration
- `auto_stop` - Disable autonomous mode

**Response:**
```json
{
  "status": "ok",
  "message": "forward_start executed"
}
```

**Flutter Usage:**
```dart
// Move forward
await mappingService.sendCommand('forward_start');
await Future.delayed(Duration(seconds: 2));
await mappingService.sendCommand('forward_stop');

// Or use convenience methods
await mappingService.moveForward();
await Future.delayed(Duration(seconds: 2));
await mappingService.stopForward();
```

**D-Pad Implementation:**
```dart
GestureDetector(
  onTapDown: (_) => mappingService.sendCommand('forward_start'),
  onTapUp: (_) => mappingService.sendCommand('forward_stop'),
  onTapCancel: () => mappingService.sendCommand('forward_stop'),
  child: Icon(Icons.arrow_upward),
)
```

---

## 🧭 Navigation Control Endpoints

### POST `/navigation/start`

Start navigation system with saved map. Launches `navigation.launch.py`.

**Request (JSON):**
```json
{
  "map_name": "my_office_map"
}
```

**Response:**
```json
{
  "status": "ok",
  "message": "Navigation started successfully",
  "map_name": "my_office_map",
  "pid": 23456,
  "note": "Feature-based localizer will auto-initialize robot pose"
}
```

**What Happens:**
1. Checks if map exists
2. Launches navigation stack in subprocess
3. Loads: map_server, AMCL, Nav2, waypoint navigator, feature-based localizer
4. Auto-initializes robot pose (no manual RViz needed!)

**Flutter Usage:**
```dart
final result = await mappingService.startNavigation(
  mapName: 'my_office_map'
);

if (result['status'] == 'ok') {
  print('Navigation started, PID: ${result['pid']}');
  // Wait 10-15 seconds for localization to converge
  await Future.delayed(Duration(seconds: 15));
}
```

---

### POST `/navigation/stop`

Stop navigation system. Kills navigation subprocess.

**Request:** No body required

**Response:**
```json
{
  "status": "ok",
  "message": "Navigation stopped successfully"
}
```

**Flutter Usage:**
```dart
final result = await mappingService.stopNavigation();
```

---

### POST `/navigate`

Navigate to specific waypoint. Executes `ros2 run slam_robot go_to` command.

**Request (JSON):**
```json
{
  "waypoint": "Point_B"
}
```

**Response:**
```json
{
  "status": "ok",
  "message": "Navigation to Point_B started",
  "waypoint": "Point_B"
}
```

**What Happens:**
1. Runs `ros2 run slam_robot go_to Point_B` in background
2. Waypoint navigator publishes goal to Nav2
3. Nav2 plans path and executes navigation
4. Robot autonomously drives to waypoint

**Flutter Usage:**
```dart
final result = await mappingService.navigateToWaypoint(
  waypoint: 'Point_B'
);

if (result['status'] == 'ok') {
  print('Navigating to ${result['waypoint']}');
}
```

---

## 📂 Maps & Waypoints Endpoints

### GET `/maps/list`

Get list of all saved maps.

**Response:**
```json
{
  "status": "ok",
  "maps": [
    {
      "name": "my_office_map",
      "has_pgm": true,
      "has_png": true,
      "has_waypoints": true,
      "waypoint_count": 15,
      "png_base64": "iVBORw0KGgoAAAANSUhEUgA...",
      "created": 1701234567.89
    },
    {
      "name": "warehouse_map",
      "has_pgm": true,
      "has_png": true,
      "has_waypoints": true,
      "waypoint_count": 22,
      "png_base64": "iVBORw0KGgoAAAANSUhEUgA...",
      "created": 1701123456.78
    }
  ],
  "total": 2
}
```

**Flutter Usage:**
```dart
final maps = await mappingService.listMaps();

for (var map in maps) {
  print('Map: ${map['name']}');
  print('Waypoints: ${map['waypoint_count']}');

  // Display image
  if (map['png_base64'] != null) {
    final imageBytes = base64Decode(map['png_base64']);
    Image.memory(imageBytes);
  }
}
```

---

### GET `/maps/<map_name>/waypoints`

Get waypoints for specific map.

**Example:** `/maps/my_office_map/waypoints`

**Response:**
```json
{
  "status": "ok",
  "map_name": "my_office_map",
  "total_waypoints": 3,
  "waypoints": [
    {
      "label": "Point_A",
      "x": 0.0,
      "y": 0.0,
      "theta": 0.0,
      "timestamp": 1701234567,
      "distance_from_previous": 0.0,
      "environmental_features": {
        "north": 0.81,
        "northeast": 16.0,
        "east": 16.0,
        "southeast": 16.0,
        "south": 16.0,
        "southwest": 16.0,
        "west": 16.0,
        "northwest": 16.0
      }
    },
    {
      "label": "Point_B",
      "x": 0.08,
      "y": -0.44,
      "theta": 2.21,
      "timestamp": 1701234646,
      "distance_from_previous": 0.44,
      "environmental_features": {
        "north": 16.0,
        "northeast": 1.18,
        "east": 1.18,
        // ...
      }
    }
  ]
}
```

**Flutter Usage:**
```dart
final result = await mappingService.getWaypoints(
  mapName: 'my_office_map'
);

if (result['status'] == 'ok') {
  final waypoints = result['waypoints'];
  for (var wp in waypoints) {
    print('${wp['label']}: (${wp['x']}, ${wp['y']})');
  }
}
```

---

### GET `/maps/<map_name>/image`

Get map PNG image as base64.

**Example:** `/maps/my_office_map/image`

**Response:**
```json
{
  "status": "ok",
  "map_name": "my_office_map",
  "png_base64": "iVBORw0KGgoAAAANSUhEUgA..."
}
```

**Flutter Usage:**
```dart
final pngBase64 = await mappingService.getMapImage(
  mapName: 'my_office_map'
);

if (pngBase64 != null) {
  final imageBytes = base64Decode(pngBase64);
  return Image.memory(imageBytes);
}
```

---

## Flutter App Integration

### Updated mapping_service.dart

The service now includes all new endpoints:

```dart
// Connection
await mappingService.checkConnection();
await mappingService.getStatus();
await mappingService.getRobotIp();
await mappingService.getDistance();

// Mapping workflow
await mappingService.startMapping();
await mappingService.activateSlam();
// ... drive around ...
await mappingService.saveMapping(mapName: 'my_map');
await mappingService.stopMapping();

// Manual control
await mappingService.sendCommand('forward_start');
await mappingService.startAutoMode();
await mappingService.increaseSpeed();

// Navigation workflow
await mappingService.startNavigation(mapName: 'my_map');
await mappingService.navigateToWaypoint(waypoint: 'Point_B');
await mappingService.stopNavigation();

// Maps & waypoints
final maps = await mappingService.listMaps();
final waypoints = await mappingService.getWaypoints(mapName: 'my_map');
final image = await mappingService.getMapImage(mapName: 'my_map');
```

---

## Workflow Examples

### Complete Mapping Workflow

```dart
class MappingWorkflow {
  final MappingService service;

  Future<void> performMapping() async {
    try {
      // 1. Start mapping process
      print('Starting mapping...');
      final startResult = await service.startMapping();
      if (startResult['status'] != 'ok') {
        throw Exception(startResult['message']);
      }

      // 2. Wait for nodes to initialize
      await Future.delayed(Duration(seconds: 5));

      // 3. Activate SLAM
      print('Activating SLAM...');
      final slamResult = await service.activateSlam();
      if (slamResult['status'] != 'ok') {
        throw Exception(slamResult['message']);
      }

      print('SLAM active! Now drive the robot around.');
      print('You can use:');
      print('- Manual control (D-pad)');
      print('- Auto mode: service.startAutoMode()');

      // 4. Drive for 3-5 minutes (manual or auto)
      // User drives robot using D-pad or auto mode

      // 5. Save map when done
      print('Saving map...');
      final mapName = 'map_${DateTime.now().millisecondsSinceEpoch}';
      final saveResult = await service.saveMapping(mapName: mapName);

      if (saveResult['status'] == 'ok') {
        print('Map saved: $mapName');
        print('Waypoints: ${saveResult['files']['waypoints']}');

        // Display map image
        final pngBase64 = saveResult['png_base64'];
        // Show in UI: Image.memory(base64Decode(pngBase64))
      }

      // 6. Stop mapping
      await service.stopMapping();
      print('Mapping session complete!');

    } catch (e) {
      print('Error: $e');
    }
  }
}
```

---

### Complete Navigation Workflow

```dart
class NavigationWorkflow {
  final MappingService service;

  Future<void> performNavigation(String mapName) async {
    try {
      // 1. Start navigation system
      print('Starting navigation with map: $mapName');
      final navResult = await service.startNavigation(mapName: mapName);

      if (navResult['status'] != 'ok') {
        throw Exception(navResult['message']);
      }

      // 2. Wait for localization
      print('Waiting for AMCL localization...');
      await Future.delayed(Duration(seconds: 15));

      // 3. Get available waypoints
      final waypointsResult = await service.getWaypoints(mapName: mapName);
      final waypoints = waypointsResult['waypoints'] as List;

      print('Available waypoints:');
      for (var wp in waypoints) {
        print('  - ${wp['label']}: (${wp['x']}, ${wp['y']})');
      }

      // 4. Navigate to waypoints
      for (var wp in waypoints) {
        final label = wp['label'];
        print('Navigating to $label...');

        final result = await service.navigateToWaypoint(waypoint: label);

        if (result['status'] == 'ok') {
          // Wait for robot to reach destination
          // In real app, you'd monitor /goal_status topic
          await Future.delayed(Duration(seconds: 30));
          print('Reached $label');
        }
      }

      // 5. Stop navigation when done
      await service.stopNavigation();
      print('Navigation complete!');

    } catch (e) {
      print('Error: $e');
    }
  }
}
```

---

## Troubleshooting

### Issue: Flask server won't start

**Error:** `ModuleNotFoundError: No module named 'flask_cors'`

**Solution:**
```bash
pip3 install flask flask-cors
```

---

### Issue: Mapping process fails to start

**Error:** `{'status': 'error', 'message': 'Mapping process failed to start'}`

**Checks:**
1. Verify workspace path exists:
   ```bash
   ls ~/SLAM/slam_ws
   ```

2. Check if ROS2 is sourced:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/SLAM/slam_ws/install/setup.bash
   ```

3. Test launch manually:
   ```bash
   ros2 launch slam_robot mapping.launch.py control_mode:=manual
   ```

---

### Issue: SLAM activation fails

**Error:** `Configure failed` or `Activate failed`

**Solution:**
- SLAM Toolbox needs time to initialize
- Increase wait time before activation (currently 3 seconds)
- Check SLAM Toolbox node is running:
  ```bash
  ros2 node list | grep slam_toolbox
  ```

---

### Issue: Map not saving

**Error:** `Map files not created properly`

**Checks:**
1. Verify maps directory exists:
   ```bash
   mkdir -p ~/slam_maps
   ```

2. Check permissions:
   ```bash
   chmod 755 ~/slam_maps
   ```

3. Test save_map.py manually:
   ```bash
   cd ~/SLAM/slam_ws
   python3 src/slam_robot/scripts/save_map.py test_map
   ```

---

### Issue: Navigation can't find map

**Error:** `{'status': 'error', 'message': 'Map not found: my_map'}`

**Solution:**
- Check map file exists:
  ```bash
  ls ~/slam_maps/my_map.yaml
  ```

- List all maps:
  ```bash
  ls ~/slam_maps/*.yaml
  ```

- Use exact map name (case-sensitive)

---

### Issue: Waypoint navigation not working

**Error:** Navigation system running but robot doesn't move

**Checks:**
1. Verify AMCL localized robot:
   ```bash
   ros2 topic echo /particlecloud
   # Particles should be clustered, not scattered
   ```

2. Check waypoint exists:
   ```bash
   cat ~/slam_maps/my_map_waypoints.yaml | grep Point_B
   ```

3. Verify Nav2 received goal:
   ```bash
   ros2 topic echo /goal_pose
   ```

---

## Summary

### Key Changes from Old System

| Feature | Old System | New System |
|---------|-----------|------------|
| **Mapping Control** | Manual terminal commands | ✅ HTTP API endpoints |
| **SLAM Activation** | Manual `ros2 lifecycle` | ✅ `/slam/activate` endpoint |
| **Map Saving** | Manual script execution | ✅ `/mapping/save` with auto-naming |
| **Navigation Launch** | Manual terminal | ✅ `/navigation/start` endpoint |
| **Waypoint Navigation** | Manual `ros2 run` | ✅ `/navigate` endpoint |
| **Map Listing** | Manual file browsing | ✅ `/maps/list` with base64 images |
| **Subprocess Management** | None | ✅ Full process lifecycle control |

### Workflow Comparison

**Old Way:**
```bash
# Terminal 1
ros2 launch slam_robot mapping.launch.py control_mode:=manual

# Terminal 2
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# ... drive around ...

# Terminal 3
python3 scripts/save_map.py my_map

# Back to Terminal 1
Ctrl+C

# Terminal 4
ros2 launch slam_robot navigation.launch.py map:=my_map

# Terminal 5
ros2 run slam_robot go_to Point_B
```

**New Way (from Flutter app):**
```dart
// One tap: Start Mapping
await service.startMapping();
await service.activateSlam();

// Drive around with D-pad

// One tap: Save Map
await service.saveMapping(mapName: 'my_map');

// One tap: Stop Mapping
await service.stopMapping();

// One tap: Start Navigation
await service.startNavigation(mapName: 'my_map');

// One tap per waypoint
await service.navigateToWaypoint(waypoint: 'Point_B');
```

---

**🎉 You now have complete Flask API control over your entire SLAM system from your Flutter app!**

---

*Wraith Robot - Flask Control Server v2.0*
*Complete API Documentation*
