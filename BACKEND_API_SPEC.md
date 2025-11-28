# Robot Mapping Control App - Backend API Specification

## Overview

This document provides a comprehensive guide for backend developers implementing the Flask API for the **Robot Mapping Control App**. The app is a Flutter-based mobile/desktop application that controls a robot's mapping, navigation, and movement through REST API endpoints.

---

## Table of Contents

1. [Application Architecture](#application-architecture)
2. [API Endpoints](#api-endpoints)
3. [Data Models](#data-models)
4. [Request/Response Format](#requestresponse-format)
5. [Error Handling](#error-handling)
6. [Implementation Examples](#implementation-examples)
7. [Connection Requirements](#connection-requirements)
8. [Map File Formats](#map-file-formats)
9. [Expected Changes & Future Data](#expected-changes--future-data)

---

## Application Architecture

### Frontend Stack
- **Framework**: Flutter (Dart)
- **State Management**: Provider (for global robot URL)
- **HTTP Client**: dart:io (http package)
- **Navigation**: Bottom Navigation Bar (2 tabs)

### App Structure

```
┌─────────────────────────────────────────┐
│         Flutter Application             │
│  (AppShell with Bottom Navigation)      │
└─────────────────────────────────────────┘
         │                    │
         ▼                    ▼
    MappingScreen        MapsScreen
  (Control Tab)          (Maps Tab)
         │                    │
    Connect Robot         View Maps
    Start Mapping         Select Map
    Manual/Auto Mode      Select Waypoint
    Save Map              Navigate
```

### Key Components

1. **MappingScreen** - Main control interface
   - Robot connection (IP + Port input)
   - Mapping operations (start, stop, save)
   - Manual/Auto mode control
   - Real-time directional control

2. **MapsScreen** - Map management
   - Display saved maps
   - Delete maps
   - View map details (modal)

3. **MapDetailScreen** - Waypoint navigation
   - Show map information
   - List waypoints
   - Select waypoint for navigation

---

## API Endpoints

All endpoints follow this base URL format:
```
http://{IP_ADDRESS}:{PORT}
```

### 1. Connection Status
**Endpoint**: `GET /status`
- **Purpose**: Verify robot connection
- **Response Code**: `200 OK` = Connected, `5xx` or timeout = Disconnected
- **Timeout**: 5 seconds
- **Response Body** (optional):
```json
{
  "status": "online",
  "battery": 85,
  "robot_name": "WRAITH"
}
```

### 2. Start Mapping
**Endpoint**: `POST /mapping/start`
- **Purpose**: Initialize mapping session
- **Request**: Empty body
- **Response Code**: `200 OK`
- **Response Body**:
```json
{
  "status": "mapping_started",
  "message": "Mapping started successfully"
}
```

### 3. Stop Mapping
**Endpoint**: `POST /mapping/stop`
- **Purpose**: End mapping session
- **Request**: Empty body
- **Response Code**: `200 OK`
- **Response Body**:
```json
{
  "status": "mapping_stopped",
  "message": "Mapping stopped successfully"
}
```

### 4. Save Mapping
**Endpoint**: `POST /mapping/save`
- **Purpose**: Save generated map with PNG image and XML data
- **Request**: Empty body
- **Response Code**: `200 OK`
- **Response**: Return the generated map data
  - **PNG Image**: Map visualization (required)
  - **Map XML**: Map structure data (required)
  - **Waypoints XML**: Waypoint definitions (required)

**Expected Response Format** (can be multipart or JSON with base64):
```json
{
  "status": "map_saved",
  "message": "Map saved successfully",
  "map_name": "map_2024_01_15_143022",
  "image_data": "base64_encoded_png_image",
  "map_xml": "<?xml version='1.0'?>...</map_xml>",
  "waypoints_xml": "<?xml version='1.0'?>...</waypoints_xml>"
}
```

### 5. Robot Movement Control
**Endpoint**: `POST /control`
- **Purpose**: Send movement commands to robot
- **Request Body**: Plain text command
  - `forward_start` - Begin forward movement
  - `forward_stop` - Stop forward movement
  - `backward_start` - Begin backward movement
  - `backward_stop` - Stop backward movement
  - `left_start` - Begin left turn
  - `left_stop` - Stop left turn
  - `right_start` - Begin right turn
  - `right_stop` - Stop right turn
  - `stop` - Emergency stop all movement

**Example Request**:
```
POST /control HTTP/1.1
Host: 192.168.1.100:5000
Content-Type: text/plain
Content-Length: 15

forward_start
```

- **Response Code**: `200 OK`
- **Response Body**:
```json
{
  "status": "command_executed",
  "command": "forward_start"
}
```

### 6. Auto Mode Start
**Endpoint**: `POST /mode/auto/start`
- **Purpose**: Enable autonomous mapping mode
- **Request**: Empty body
- **Response Code**: `200 OK`
- **Response Body**:
```json
{
  "status": "auto_mode_started",
  "message": "Auto mode started"
}
```

### 7. Auto Mode Stop
**Endpoint**: `POST /mode/auto/stop`
- **Purpose**: Disable autonomous mapping mode
- **Request**: Empty body
- **Response Code**: `200 OK`
- **Response Body**:
```json
{
  "status": "auto_mode_stopped",
  "message": "Auto mode stopped"
}
```

### 8. Navigate to Waypoint
**Endpoint**: `POST /navigate`
- **Purpose**: Command robot to navigate to specific waypoint on a saved map
- **Request Body**: Form data or JSON
```json
{
  "map": "map_2024_01_15_143022",
  "waypoint": "wp_001"
}
```

- **Response Code**: `200 OK`
- **Response Body**:
```json
{
  "status": "navigation_started",
  "message": "Navigation started",
  "destination": "wp_001",
  "estimated_time": 45
}
```

---

## Data Models

### SavedMap (Frontend Model)
```dart
class SavedMap {
  String id;                    // Unique identifier (timestamp-based)
  String name;                  // Human-readable map name
  String imagePath;             // Path to PNG preview
  String mapXmlPath;            // Path to map XML file
  String waypointsXmlPath;      // Path to waypoints XML file
  DateTime createdAt;           // Creation timestamp
  List<Waypoint> waypoints;     // List of waypoints
}
```

### Waypoint (Frontend Model)
```dart
class Waypoint {
  String id;                    // Unique ID (e.g., "wp_001")
  String name;                  // Human-readable name (e.g., "Start Point")
  double x;                     // X coordinate in meters
  double y;                     // Y coordinate in meters
  double theta;                 // Orientation angle in degrees (0-360)
}
```

---

## Request/Response Format

### General Response Format

All successful responses should follow this pattern:

```json
{
  "status": "success_or_status_code",
  "message": "Human readable message",
  "data": {
    "additional": "fields as needed"
  }
}
```

### Error Response Format

```json
{
  "status": "error",
  "message": "Error description",
  "error_code": "ERROR_CODE"
}
```

### Common Error Codes

| Code | Meaning |
|------|---------|
| `400` | Bad Request |
| `500` | Internal Server Error |
| `503` | Service Unavailable |
| `ROBOT_OFFLINE` | Robot disconnected |
| `MAPPING_NOT_ACTIVE` | Mapping not in progress |
| `INVALID_WAYPOINT` | Waypoint not found |

---

## Error Handling

### Frontend Error Handling Strategy

The frontend handles errors as follows:

1. **Connection Errors** (5-second timeout)
   - Displays red error message
   - Disconnects user
   - Shows "Robot not connected" status

2. **Command Failures**
   - HTTP status code != 200
   - Shows SnackBar with error message
   - Maintains current state

3. **Exception Handling**
   - Catches all exceptions
   - Logs error details
   - Displays user-friendly message

### Backend Recommendations

1. **Always return HTTP 200** for valid requests (even if operation has internal issues)
2. **Include status field** in JSON response body explaining what happened
3. **Return descriptive messages** that frontend can display to user
4. **Handle timeout gracefully** (respond within 5 seconds)
5. **Log all errors** on backend for debugging

---

## Implementation Examples

### Python/Flask Example - Status Endpoint

```python
from flask import Flask, jsonify

app = Flask(__name__)

@app.route('/status', methods=['GET'])
def get_status():
    try:
        # Check robot hardware connection
        is_connected = check_robot_connection()

        if is_connected:
            return jsonify({
                'status': 'online',
                'battery': 85,
                'robot_name': 'WRAITH'
            }), 200
        else:
            return jsonify({
                'status': 'offline',
                'message': 'Robot is offline'
            }), 503
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500
```

### Python/Flask Example - Start Mapping

```python
@app.route('/mapping/start', methods=['POST'])
def start_mapping():
    try:
        # Initialize mapping process
        start_lidar_scan()
        start_odometry_tracking()

        return jsonify({
            'status': 'mapping_started',
            'message': 'Mapping started successfully'
        }), 200
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500
```

### Python/Flask Example - Save Mapping

```python
from PIL import Image
import io
import base64

@app.route('/mapping/save', methods=['POST'])
def save_mapping():
    try:
        # Generate map image
        map_image = generate_occupancy_grid_image()

        # Convert PNG to base64
        img_bytes = io.BytesIO()
        map_image.save(img_bytes, format='PNG')
        img_base64 = base64.b64encode(img_bytes.getvalue()).decode()

        # Generate XML files
        map_xml = generate_map_xml()
        waypoints_xml = generate_waypoints_xml()

        return jsonify({
            'status': 'map_saved',
            'message': 'Map saved successfully',
            'map_name': f'map_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
            'image_data': img_base64,
            'map_xml': map_xml,
            'waypoints_xml': waypoints_xml
        }), 200
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500
```

### Python/Flask Example - Robot Control

```python
@app.route('/control', methods=['POST'])
def control_robot():
    command = request.get_data(as_text=True).strip()

    valid_commands = [
        'forward_start', 'forward_stop',
        'backward_start', 'backward_stop',
        'left_start', 'left_stop',
        'right_start', 'right_stop',
        'stop'
    ]

    if command not in valid_commands:
        return jsonify({
            'status': 'error',
            'message': f'Invalid command: {command}'
        }), 400

    try:
        execute_motor_command(command)
        return jsonify({
            'status': 'command_executed',
            'command': command
        }), 200
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500
```

### Python/Flask Example - Navigate to Waypoint

```python
@app.route('/navigate', methods=['POST'])
def navigate_to_waypoint():
    data = request.get_json()
    map_name = data.get('map')
    waypoint_id = data.get('waypoint')

    if not map_name or not waypoint_id:
        return jsonify({
            'status': 'error',
            'message': 'map and waypoint parameters required'
        }), 400

    try:
        # Load map and find waypoint coordinates
        waypoint = load_waypoint(map_name, waypoint_id)

        # Start navigation
        start_navigation(waypoint.x, waypoint.y, waypoint.theta)

        return jsonify({
            'status': 'navigation_started',
            'message': 'Navigation started',
            'destination': waypoint_id,
            'estimated_time': 45
        }), 200
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500
```

---

## Connection Requirements

### Network Configuration
- **Protocol**: HTTP (not HTTPS in current implementation)
- **Port**: Configurable (default 5000)
- **Base URL Format**: `http://{IP}:{PORT}`
- **Timeout**: All requests have 5-second timeout
- **URL Validation**: Frontend automatically adds `http://` prefix if missing

### Example Connection Attempts
```
User Input: "192.168.1.100:5000"
Frontend converts to: "http://192.168.1.100:5000"

User Input: "localhost:8000"
Frontend converts to: "http://localhost:8000"

User Input: "http://192.168.1.100:5000"
Frontend keeps as: "http://192.168.1.100:5000"
```

---

## Map File Formats

### PNG Image Format
- **Format**: PNG (8-bit or 24-bit)
- **Purpose**: Visual preview of map
- **Dimensions**: Recommended 512x512 or 1024x1024 pixels
- **Encoding**: Base64 string in JSON response (for `/mapping/save`)

### Map XML Format

Structure for map file (e.g., `map_20240115_143022.xml`):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<map>
  <metadata>
    <name>map_20240115_143022</name>
    <creation_date>2024-01-15T14:30:22Z</creation_date>
    <resolution>0.05</resolution>
    <width>100</width>
    <height>100</height>
  </metadata>
  <origin>
    <x>0.0</x>
    <y>0.0</y>
    <theta>0.0</theta>
  </origin>
  <occupancy_grid>
    <!-- 2D grid data representing obstacles and free space -->
    <!-- 0 = free, 100 = occupied, -1 = unknown -->
  </occupancy_grid>
</map>
```

### Waypoints XML Format

Structure for waypoints file (e.g., `waypoints_20240115_143022.xml`):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<waypoints>
  <metadata>
    <map_name>map_20240115_143022</map_name>
    <creation_date>2024-01-15T14:30:22Z</creation_date>
    <waypoint_count>3</waypoint_count>
  </metadata>
  <waypoint>
    <id>wp_001</id>
    <name>Start Point</name>
    <x>0.0</x>
    <y>0.0</y>
    <theta>0.0</theta>
    <description>Initial position</description>
  </waypoint>
  <waypoint>
    <id>wp_002</id>
    <name>Charging Station</name>
    <x>10.5</x>
    <y>5.2</y>
    <theta>90.0</theta>
    <description>Charging dock location</description>
  </waypoint>
  <waypoint>
    <id>wp_003</id>
    <name>Inspection Point</name>
    <x>15.0</x>
    <y>20.0</y>
    <theta>180.0</theta>
    <description>Area to inspect</description>
  </waypoint>
</waypoints>
```

### Waypoint Parsing (Frontend)

Frontend parses waypoints using regex pattern:
```regex
<waypoint\s+id="([^"]+)"\s+name="([^"]+)"\s+x="([^"]+)"\s+y="([^"]+)"\s+theta="([^"]+)"
```

This extracts: `id`, `name`, `x`, `y`, `theta`

---

## App User Flows

### Flow 1: Mapping and Saving

```
User enters IP:Port
    ↓
Click "Activate Robot" → GET /status
    ↓ (Connection successful)
Click "Start Mapping" → POST /mapping/start
    ↓ (Mapping active)
Select Manual/Auto mode
    ↓
If Manual: Use directional buttons → POST /control
    ↓
Click "Stop Mapping" → POST /mapping/stop
    ↓
Click "Save Map" → POST /mapping/save
    ↓ (Receive PNG + XML files)
Map stored locally in app
```

### Flow 2: Navigation to Waypoint

```
Go to Maps tab
    ↓
Select saved map
    ↓
View map details (shows waypoints)
    ↓
Select waypoint
    ↓
Click "Navigate to Waypoint" → POST /navigate
    ↓ (Robot starts navigating)
Status updates in real-time
```

---

## Development Checklist

### Required Endpoints
- [ ] `GET /status` - Connection check
- [ ] `POST /mapping/start` - Start mapping
- [ ] `POST /mapping/stop` - Stop mapping
- [ ] `POST /mapping/save` - Save map (with PNG + XML)
- [ ] `POST /control` - Robot movement
- [ ] `POST /mode/auto/start` - Enable auto mode
- [ ] `POST /mode/auto/stop` - Disable auto mode
- [ ] `POST /navigate` - Navigate to waypoint

### Testing Requirements
- [ ] Test connection timeout (5 seconds)
- [ ] Test all movement commands
- [ ] Test manual vs auto mode switching
- [ ] Test map saving with proper XML format
- [ ] Test waypoint navigation
- [ ] Test error responses (return proper error codes)

### Performance Requirements
- [ ] All endpoints respond within 5 seconds
- [ ] No blocking operations on main thread
- [ ] Handle concurrent requests safely

---

## Troubleshooting Guide

### Frontend Connection Issues

**Symptom**: "Failed to connect to robot"
- **Check 1**: Verify robot IP and port are correct
- **Check 2**: Ensure `/status` endpoint returns 200
- **Check 3**: Check network connectivity to robot

**Symptom**: "Connection timeout"
- **Check 1**: Endpoint takes > 5 seconds to respond
- **Check 2**: Firewall blocking port access
- **Check 3**: Robot hardware issue

### Mapping Issues

**Symptom**: "Map not saving"
- **Check 1**: Verify `/mapping/save` returns 200
- **Check 2**: Ensure XML files are valid
- **Check 3**: Check file permissions

**Symptom**: "Waypoints not showing"
- **Check 1**: Verify waypoints XML matches expected format
- **Check 2**: Check waypoint XML regex pattern matches actual XML
- **Check 3**: Ensure all required fields (id, name, x, y, theta) are present

---

## Future Enhancements

### Potential Backend Improvements
1. Add authentication/authorization
2. Implement WebSocket for real-time updates
3. Add map visualization endpoint
4. Implement map comparison/merging
5. Add robot diagnostics endpoint
6. Implement task scheduling
7. Add map versioning/history

### Potential Frontend Improvements
1. Real-time map visualization during mapping
2. Live robot status dashboard
3. Map comparison view
4. Waypoint editing interface
5. Multi-map navigation planning

---

## Expected Changes & Future Data

### Important Note

This application specification covers the **initial implementation** of the Robot Mapping Control App. As the robot hardware and backend systems evolve, the robot may return additional data types and sensor information beyond what is currently specified. This section outlines potential changes and how the app should adapt.

### Potential New Data Fields from Robot

#### 1. Battery & Power Management
**Current Implementation**: `/status` optionally returns battery percentage
**Potential New Data**:
```json
{
  "battery_percentage": 85,
  "battery_voltage": 24.5,
  "battery_temperature": 35.2,
  "charging_status": "idle",
  "estimated_runtime_minutes": 120,
  "power_consumption_watts": 45.0
}
```

**App Changes Required**:
- Add battery status widget on mapping screen
- Display battery warning at < 20%
- Show charging status indicator
- Add battery history/graph on maps screen
- Implement battery level cutoff to prevent low-power operations

#### 2. Sensor Data & Diagnostics
**Potential New Endpoints**:
```
GET /sensors/status
GET /diagnostics
GET /lidar/scan (real-time)
```

**Example Response**:
```json
{
  "sensors": {
    "lidar": {
      "status": "operational",
      "range_min": 0.15,
      "range_max": 10.0,
      "error": null
    },
    "odometry": {
      "status": "operational",
      "error_rate": 0.02
    },
    "imu": {
      "status": "operational",
      "temperature": 28.5
    },
    "camera": {
      "status": "operational",
      "resolution": "1920x1080"
    }
  },
  "diagnostics": {
    "last_calibration": "2024-01-10T12:00:00Z",
    "wheel_odometry_error": 0.02,
    "imu_drift": 0.5
  }
}
```

**App Changes Required**:
- Create diagnostics screen in maps tab
- Display sensor health indicators
- Show last calibration date and recommend recalibration
- Add sensor error alerts
- Implement real-time lidar visualization (if scan data available)

#### 3. Real-Time Position Tracking
**Potential New Endpoint**:
```
GET /robot/position (or WebSocket for real-time)
POST /robot/position/history
```

**Example Response**:
```json
{
  "position": {
    "x": 5.2,
    "y": 3.8,
    "theta": 45.3,
    "timestamp": "2024-01-15T14:30:22.123Z"
  },
  "velocity": {
    "linear": 0.5,
    "angular": 0.2
  },
  "map_certainty": 0.95
}
```

**App Changes Required**:
- Add real-time position overlay on map during navigation
- Display current robot coordinates
- Show velocity indicators
- Implement breadcrumb trail visualization
- Add position history tracking for troubleshooting

#### 4. Mapping Progress & Statistics
**Potential New Data in `/mapping/status`**:
```json
{
  "mapping_active": true,
  "elapsed_time_seconds": 45,
  "area_explored_sqm": 125.5,
  "map_coverage_percentage": 35,
  "wall_count": 12,
  "obstacle_count": 5,
  "current_scan_points": 2000
}
```

**App Changes Required**:
- Add progress indicator during mapping
- Display area explored in real-time
- Show coverage percentage
- Add estimated time to completion
- Display scan quality metrics

#### 5. Navigation Status & Obstacle Detection
**Potential New Endpoint**:
```
GET /navigation/status
POST /navigation/abort
```

**Example Response**:
```json
{
  "navigation_active": true,
  "destination": "wp_001",
  "progress_percentage": 45,
  "estimated_time_remaining": 30,
  "current_obstacle": {
    "detected": true,
    "distance": 0.5,
    "type": "dynamic_obstacle"
  },
  "path_replanning_count": 2
}
```

**App Changes Required**:
- Add navigation progress indicator
- Display obstacle detection warnings
- Show estimated time to destination
- Add obstacle avoidance status
- Implement emergency stop button for navigation
- Add navigation history/log

#### 6. Map Quality & Stitching Information
**Potential New Data in `/mapping/save`**:
```json
{
  "map_quality_score": 0.92,
  "stitching_success": true,
  "loop_closures_detected": 3,
  "scan_count": 450,
  "total_distance_traveled": 85.3,
  "map_creation_time_seconds": 120,
  "warnings": [
    "Low light conditions detected",
    "High noise in lidar data"
  ]
}
```

**App Changes Required**:
- Display map quality score
- Show warnings about map reliability
- Add map statistics in detail screen
- Log scan count and travel distance
- Add quality warnings before saving

#### 7. Environmental & Context Data
**Potential New Fields**:
```json
{
  "environment": {
    "lighting_level": "bright",
    "ambient_temperature": 25.2,
    "humidity": 45,
    "floor_type": "tiles",
    "reflective_surfaces": true
  }
}
```

**App Changes Required**:
- Add environment context to saved maps
- Display environment notes
- Add filtering by environment type
- Implement environmental warnings

### Recommended App Adaptations

#### 1. Make All Parsers Flexible
```dart
// Instead of:
final batteryLevel = response['battery']; // Will crash if missing

// Use:
final batteryLevel = response['battery'] ?? 0; // Default value
final batteryVoltage = response['battery_voltage']; // May be null
```

#### 2. Create Feature Flags
```dart
class RobotCapabilities {
  bool supportsBattery = false;
  bool supportsRealTimePosition = false;
  bool supportsNavigationStatus = false;
  bool supportsSensorDiagnostics = false;

  // Query robot capabilities on connect
  Future<void> detectCapabilities() async { }
}
```

#### 3. Versioning in API Responses
```json
{
  "api_version": "2.0",
  "supported_features": ["battery", "position", "diagnostics"],
  "deprecated_fields": []
}
```

**App Should**:
- Parse API version on connection
- Only use features advertised as supported
- Handle gracefully when features are missing
- Log unsupported feature requests

#### 4. Extend Data Models Easily
```dart
// Instead of fixed SavedMap class, use:
class SavedMap {
  final Map<String, dynamic> metadata; // Extensible
  final String id;
  final String name;
  // ... required fields
}

// Can add fields without modifying model:
final mapQuality = savedMap.metadata['quality_score'] ?? 'unknown';
const environmentType = savedMap.metadata['environment'] ?? 'unknown';
```

#### 5. Create Settings Panel
Add a settings screen where users can:
- Enable/disable sensor monitoring
- Configure alert thresholds
- Set battery low warning level
- Choose visualization preferences
- Enable/disable real-time tracking

#### 6. Logging & Debugging
Implement detailed logging for:
- All API responses (with timestamps)
- Any fields not matching expected schema
- Sensor values and statistics
- Navigation events and obstacles

This helps quickly identify when new data arrives from robot.

### Migration Path Example

**Phase 1 (Current)**: Basic mapping and navigation
- Required endpoints: `/status`, `/mapping/*`, `/control`, `/navigate`
- Data: Position, map, waypoints

**Phase 2 (Next)**: Real-time monitoring
- Add endpoints: `/sensors/status`, `/robot/position`
- Add battery, sensor, and position tracking

**Phase 3**: Advanced autonomy
- Add endpoints: `/navigation/status`, `/diagnostics`
- Real-time navigation updates, obstacle detection

**Phase 4**: Analytics & Intelligence
- Add endpoints: `/analytics`, `/maps/compare`
- Map quality analysis, performance metrics

### Summary

The app is built with a solid foundation that can adapt to additional data. As the backend evolves:

1. ✅ **Always return 200** for valid requests
2. ✅ **Include optional fields** in responses (don't break if missing)
3. ✅ **Version your API** to track features
4. ✅ **Document new fields** when adding them
5. ✅ **Test backward compatibility** before deploying

The frontend will automatically utilize any new data that arrives, and the user will have an increasingly rich interface as more robot capabilities are exposed through the API.

---

## Contact & Support

For questions regarding this specification, refer to:
1. **Frontend Repo**: CLAUDE.md for architecture details
2. **Navigation Docs**: NAVIGATION.md for UI flow
3. **Code Comments**: Check individual files for implementation details

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.1 | 2024-01-15 | Added "Expected Changes & Future Data" section with extensibility guidelines |
| 1.0 | 2024-01-15 | Initial specification |

---

**Document Generated**: 2024-01-15
**Last Updated**: 2024-01-15
**Status**: Complete and Ready for Backend Implementation with Future Evolution Path
