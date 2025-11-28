# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Robot Mapping Control App** - A Flutter application that provides a complete interface to control a robot's mapping functionality via Flask-based REST endpoints. The app allows users to:
- Connect to a robot by entering IP address and port
- Start/stop mapping sessions with real-time manual or autonomous control
- Save generated maps with PNG preview and XML data (map + waypoints)
- View all saved maps with waypoint information
- Navigate the robot to selected waypoints on previously saved maps

## Architecture

### Layered Structure

```
lib/
├── main.dart                          # App entry point, Provider setup
├── screens/
│   ├── app_shell.dart                # Bottom navigation bar with page switching
│   ├── mapping_screen.dart           # Main mapping control interface
│   ├── maps_screen.dart              # List of saved maps
│   └── map_detail_screen.dart        # Map details and waypoint navigation
├── services/
│   ├── mapping_service.dart          # HTTP client for robot communication
│   └── map_storage_service.dart      # In-memory map storage and parsing
├── models/
│   └── map_model.dart                # SavedMap and Waypoint data models
└── providers/
    └── robot_url_provider.dart       # Global robot URL state management
```

### Key Concepts

0. **AppShell** (`lib/screens/app_shell.dart`)
   - Root navigation widget with bottom navigation bar
   - Two tabs: "Control" (MappingScreen) and "Maps" (MapsScreen)
   - Manages page switching with `_selectedIndex`
   - Provides persistent navigation across the app

1. **MappingService** (`lib/services/mapping_service.dart`)
   - Centralized HTTP client with URL validation (`_ensureUrl()`)
   - All Flask endpoints with 5-second timeout
   - Methods: `checkConnection()`, `startMapping()`, `stopMapping()`, `saveMapping()`, `startAutoMode()`, `stopAutoMode()`, `sendCommand()`, `navigateToWaypoint()`

2. **MapStorageService** (`lib/services/map_storage_service.dart`)
   - Singleton pattern for managing saved maps
   - In-memory storage (can be extended to use SharedPreferences/Hive)
   - Waypoint XML parsing with regex patterns
   - Methods: `saveMap()`, `loadMaps()`, `deleteMap()`, `parseWaypoints()`, `getSortedMaps()`

3. **Data Models** (`lib/models/map_model.dart`)
   - `SavedMap`: Contains map metadata, file paths, and waypoints
   - `Waypoint`: Coordinates (x, y) and orientation (theta) data
   - Both support JSON serialization for potential persistence

4. **State Management**
   - `RobotUrlProvider`: Global robot URL using Provider pattern
   - MappingScreen uses `setState()` for local UI state
   - MapDetailScreen uses Provider to access robot URL

5. **Control Flow**
   - **Mapping Flow**: Connect → Start Mapping → Manual/Auto Control → Stop Mapping → Save Map
   - **Navigation Flow**: View Maps → Select Map → Select Waypoint → Navigate
   - Stopping mapping resets both manual and auto modes to prevent conflicts

## Flask Endpoints Expected

The app communicates with the following Flask endpoints:

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/status` | GET | Check robot connection status |
| `/mapping/start` | POST | Start mapping session |
| `/mapping/stop` | POST | Stop mapping session |
| `/mapping/save` | POST | Save generated map |
| `/control` | POST | Send movement commands (body: "forward", "backward", "left", "right", "stop") |
| `/mode/auto/start` | POST | Start autonomous mode |
| `/mode/auto/stop` | POST | Stop autonomous mode |

All requests use `http://IP:PORT` base URL from user input and have a 5-second timeout.

## Development Commands

```bash
# Install dependencies
flutter pub get

# Run the app (specify device)
flutter run -d windows      # Windows desktop
flutter run -d chrome       # Web browser
flutter run -d <device_id>  # Android/iOS

# Analyze code for issues
flutter analyze

# Format code
dart format lib/

# Run tests
flutter test

# Build for release
flutter build windows       # Windows
flutter build web          # Web
flutter build apk          # Android
flutter build ios          # iOS
```

## Dependencies

- **http** (^1.1.0): HTTP client for Flask communication
- **provider** (^6.0.0): State management (imported but not actively used in current implementation)
- **flutter_lints** (^5.0.0): Code linting

## Key Implementation Details

### State Management
Currently uses `setState()` for local state management. The `provider` package is imported but not utilized; future refactoring could migrate to Provider for more scalable state management.

### Error Handling
All async operations use try-catch blocks with user-facing error messages via `_showMessage()` SnackBar notifications.

### UI Patterns
- **Conditional Rendering**: Buttons appear/disappear based on app state
- **Button States**: Most buttons are disabled when prerequisites aren't met
- **Color-Coded Feedback**:
  - Green for success
  - Red for errors
  - Orange for warnings
  - Blue for info messages

### Manual Control D-Pad
- Press-and-hold directional buttons to move (via `onTapDown`/`onTapUp`/`onTapCancel`)
- Sends `{direction}_start` command when pressed, `{direction}_stop` when released
- Center red button stops all movement immediately
- Only active in manual mode

## Common Tasks

### Adding a New Flask Endpoint
1. Add method to `MappingService` in `lib/services/mapping_service.dart`
2. Call the method from appropriate handler in `MappingScreen`
3. Update UI as needed based on response

### Modifying UI Layout
- Edit `lib/screens/mapping_screen.dart`'s `build()` method
- Use conditional `if (_condition)` for state-dependent visibility
- Wrap button state logic in `onPressed: _isConnected ? _functionName : null`

### Testing the UI Flow Without Backend
Temporarily remove API calls from handler methods (e.g., `_handleConnect()`, `_startMapping()`) and just update state directly to test UI flows.

## Notes

- The app uses Material Design 3 (`useMaterial3: true`)
- All user feedback happens through SnackBar notifications at the bottom of the screen
- The connection check on app load is intentionally removed; connection happens when user clicks "Activate Robot"
- Manual and Auto modes are designed to be mutually exclusive to prevent conflicting commands
