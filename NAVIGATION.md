# Navigation Structure

## Bottom Navigation Bar

The app uses a persistent bottom navigation bar in `AppShell` to switch between two main screens:

### Tab 1: Control (Icon: videogame_asset)
- **Screen**: `MappingScreen`
- **Features**:
  - Connect to robot (IP + Port)
  - Start/Stop mapping
  - Manual control with D-Pad (4 directional buttons)
  - Auto mode toggle
  - Save map when mapping is stopped
  - Real-time status updates

### Tab 2: Maps (Icon: map)
- **Screen**: `MapsScreen`
- **Features**:
  - View all saved maps
  - Display map creation date
  - Show waypoint count
  - Delete maps with confirmation
  - Tap map to see details

## Screen Flow

```
AppShell (Bottom Nav Bar)
├── MappingScreen (Control Tab)
│   └── Robot Connection
│   └── Mapping Operations
│   └── Manual/Auto Control
│
└── MapsScreen (Maps Tab)
    └── MapDetailScreen (Modal Navigation)
        └── Waypoint Selection
        └── Navigation to Waypoint
```

## Navigation Between Screens

- **Bottom Navigation**: Tap "Control" or "Maps" in the bottom nav bar to switch
- **Detail View**: From MapsScreen, tap a map card to open MapDetailScreen
- **Back**: Use device back button or AppBar back button to return to MapsScreen

## State Management Across Screens

- **Robot URL**: Stored globally in `RobotUrlProvider` for access from any screen
- **Maps Storage**: `MapStorageService` singleton maintains maps list
- **Local State**: Each screen manages its own UI state with `setState()`

## Navigation Benefits

1. **Persistent State**: Switching tabs keeps the previous screen's state
2. **Easy Access**: Quick tab switching without modal dialogs
3. **Clear Intent**: Visual indication of which section you're in
4. **Consistent**: Standard bottom navigation pattern
