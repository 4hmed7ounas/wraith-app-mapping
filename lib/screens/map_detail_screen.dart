import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/map_model.dart';
import '../services/mapping_service.dart';
import '../providers/robot_url_provider.dart';

class MapDetailScreen extends StatefulWidget {
  const MapDetailScreen({super.key});

  @override
  State<MapDetailScreen> createState() => _MapDetailScreenState();
}

class _MapDetailScreenState extends State<MapDetailScreen> {
  late MappingService _mappingService;

  List<SavedMap> _maps = [];
  SavedMap? _selectedMap;
  Waypoint? _selectedWaypoint;

  bool _isLoadingMaps = false;
  bool _isLoadingWaypoints = false;
  bool _isNavigationActive = false;
  bool _isStartingNavigation = false;
  bool _isNavigatingToWaypoint = false;

  @override
  void initState() {
    super.initState();
    final robotUrl = Provider.of<RobotUrlProvider>(context, listen: false).robotUrl;
    _mappingService = MappingService(robotUrl: robotUrl);
    _loadAllMaps();
  }

  Future<void> _loadAllMaps() async {
    setState(() {
      _isLoadingMaps = true;
    });

    try {
      final mapsList = await _mappingService.listMaps();

      List<SavedMap> loadedMaps = [];
      for (var mapData in mapsList) {
        loadedMaps.add(SavedMap(
          id: mapData['name'],
          name: mapData['name'],
          imagePath: '',
          mapXmlPath: '',
          waypointsXmlPath: '',
          createdAt: DateTime.fromMillisecondsSinceEpoch(
            (mapData['created'] * 1000).toInt(),
          ),
          waypoints: [],
        ));
      }

      setState(() {
        _maps = loadedMaps;
        _isLoadingMaps = false;
      });
    } catch (e) {
      setState(() {
        _isLoadingMaps = false;
      });
      _showMessage('Error loading maps: $e', Colors.red);
    }
  }

  Future<void> _loadWaypointsForMap(SavedMap map) async {
    setState(() {
      _isLoadingWaypoints = true;
      _selectedMap = map;
      _selectedWaypoint = null;
    });

    try {
      final waypointsData = await _mappingService.getWaypoints(mapName: map.name);

      List<Waypoint> waypoints = [];
      if (waypointsData['status'] == 'ok' && waypointsData['waypoints'] != null) {
        waypoints = (waypointsData['waypoints'] as List).map((wp) {
          return Waypoint(
            id: wp['label'] ?? 'unknown',
            name: wp['label'] ?? 'Waypoint',
            x: (wp['x'] as num).toDouble(),
            y: (wp['y'] as num).toDouble(),
            theta: (wp['theta'] as num).toDouble(),
          );
        }).toList();
      }

      final updatedMap = SavedMap(
        id: map.id,
        name: map.name,
        imagePath: map.imagePath,
        mapXmlPath: map.mapXmlPath,
        waypointsXmlPath: map.waypointsXmlPath,
        createdAt: map.createdAt,
        waypoints: waypoints,
      );

      setState(() {
        _selectedMap = updatedMap;
        _isLoadingWaypoints = false;
      });

      _showMessage('Loaded ${waypoints.length} waypoints', Colors.green);
    } catch (e) {
      setState(() {
        _isLoadingWaypoints = false;
      });
      _showMessage('Error loading waypoints: $e', Colors.red);
    }
  }

  Future<void> _startNavigation() async {
    if (_selectedMap == null) return;

    setState(() {
      _isStartingNavigation = true;
    });

    try {
      final result = await _mappingService.startNavigation(mapName: _selectedMap!.name);

      if (result['status'] == 'ok') {
        setState(() {
          _isNavigationActive = true;
          _isStartingNavigation = false;
        });
        _showMessage('Navigation started! Wait 10-15 seconds for localization', Colors.green);
      } else {
        setState(() {
          _isStartingNavigation = false;
        });
        _showMessage(result['message'] ?? 'Failed to start navigation', Colors.red);
      }
    } catch (e) {
      setState(() {
        _isStartingNavigation = false;
      });
      _showMessage('Error: $e', Colors.red);
    }
  }

  Future<void> _stopNavigation() async {
    try {
      final result = await _mappingService.stopNavigation();
      setState(() {
        _isNavigationActive = false;
      });
      _showMessage(result['message'] ?? 'Navigation stopped', Colors.orange);
    } catch (e) {
      _showMessage('Error: $e', Colors.red);
    }
  }

  Future<void> _navigateToWaypoint(Waypoint waypoint) async {
    if (!_isNavigationActive) {
      _showMessage('Start Navigation first!', Colors.orange);
      return;
    }

    setState(() {
      _selectedWaypoint = waypoint;
      _isNavigatingToWaypoint = true;
    });

    try {
      await _mappingService.navigateToWaypoint(waypoint: waypoint.name);

      setState(() {
        _isNavigatingToWaypoint = false;
      });

      _showMessage('Navigating to ${waypoint.name}', Colors.green);
    } catch (e) {
      setState(() {
        _isNavigatingToWaypoint = false;
      });
      _showMessage('Navigation error: $e', Colors.red);
    }
  }

  void _showMessage(String message, Color color) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: color,
        duration: const Duration(seconds: 2),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Navigation'),
        centerTitle: true,
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _loadAllMaps,
          ),
        ],
      ),
      body: _isLoadingMaps
          ? const Center(child: CircularProgressIndicator())
          : Row(
              children: [
                // Left side - Maps list
                Expanded(
                  flex: 1,
                  child: Container(
                    decoration: BoxDecoration(
                      border: Border(
                        right: BorderSide(color: Colors.grey[300]!),
                      ),
                    ),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Padding(
                          padding: const EdgeInsets.all(16),
                          child: Text(
                            'Maps (${_maps.length})',
                            style: Theme.of(context).textTheme.titleLarge,
                          ),
                        ),
                        Expanded(
                          child: _maps.isEmpty
                              ? Center(
                                  child: Text(
                                    'No maps found',
                                    style: TextStyle(color: Colors.grey[600]),
                                  ),
                                )
                              : ListView.builder(
                                  itemCount: _maps.length,
                                  itemBuilder: (context, index) {
                                    final map = _maps[index];
                                    final isSelected = _selectedMap?.id == map.id;

                                    return ListTile(
                                      selected: isSelected,
                                      selectedTileColor: Colors.blue[50],
                                      leading: Icon(
                                        Icons.map,
                                        color: isSelected ? Colors.blue : Colors.grey,
                                      ),
                                      title: Text(
                                        map.name,
                                        style: TextStyle(
                                          fontWeight: isSelected
                                              ? FontWeight.bold
                                              : FontWeight.normal,
                                        ),
                                      ),
                                      subtitle: Text(
                                        _formatDate(map.createdAt),
                                        style: const TextStyle(fontSize: 12),
                                      ),
                                      onTap: () => _loadWaypointsForMap(map),
                                    );
                                  },
                                ),
                        ),
                      ],
                    ),
                  ),
                ),

                // Right side - Waypoints and navigation controls
                Expanded(
                  flex: 2,
                  child: _selectedMap == null
                      ? Center(
                          child: Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              Icon(Icons.touch_app, size: 64, color: Colors.grey[400]),
                              const SizedBox(height: 16),
                              Text(
                                'Select a map to view waypoints',
                                style: TextStyle(color: Colors.grey[600]),
                              ),
                            ],
                          ),
                        )
                      : _isLoadingWaypoints
                          ? const Center(child: CircularProgressIndicator())
                          : SingleChildScrollView(
                              padding: const EdgeInsets.all(16),
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  // Map info
                                  Text(
                                    _selectedMap!.name,
                                    style: Theme.of(context).textTheme.headlineSmall,
                                  ),
                                  const SizedBox(height: 8),
                                  Text(
                                    '${_selectedMap!.waypoints.length} waypoints',
                                    style: TextStyle(color: Colors.grey[600]),
                                  ),
                                  const SizedBox(height: 20),

                                  // Navigation control buttons
                                  if (!_isNavigationActive)
                                    SizedBox(
                                      width: double.infinity,
                                      child: ElevatedButton.icon(
                                        onPressed: _isStartingNavigation ? null : _startNavigation,
                                        icon: _isStartingNavigation
                                            ? const SizedBox(
                                                width: 20,
                                                height: 20,
                                                child: CircularProgressIndicator(strokeWidth: 2),
                                              )
                                            : const Icon(Icons.play_arrow),
                                        label: Padding(
                                          padding: const EdgeInsets.all(12),
                                          child: Text(_isStartingNavigation
                                              ? 'Starting...'
                                              : 'Start Navigation'),
                                        ),
                                        style: ElevatedButton.styleFrom(
                                          backgroundColor: Colors.green,
                                          foregroundColor: Colors.white,
                                        ),
                                      ),
                                    )
                                  else
                                    SizedBox(
                                      width: double.infinity,
                                      child: ElevatedButton.icon(
                                        onPressed: _stopNavigation,
                                        icon: const Icon(Icons.stop),
                                        label: const Padding(
                                          padding: EdgeInsets.all(12),
                                          child: Text('Stop Navigation'),
                                        ),
                                        style: ElevatedButton.styleFrom(
                                          backgroundColor: Colors.red,
                                          foregroundColor: Colors.white,
                                        ),
                                      ),
                                    ),

                                  const SizedBox(height: 20),

                                  // Waypoints list
                                  Text(
                                    'Waypoints',
                                    style: Theme.of(context).textTheme.titleLarge,
                                  ),
                                  const SizedBox(height: 12),

                                  if (_selectedMap!.waypoints.isEmpty)
                                    Center(
                                      child: Padding(
                                        padding: const EdgeInsets.all(32),
                                        child: Text(
                                          'No waypoints found',
                                          style: TextStyle(color: Colors.grey[600]),
                                        ),
                                      ),
                                    )
                                  else
                                    ListView.separated(
                                      shrinkWrap: true,
                                      physics: const NeverScrollableScrollPhysics(),
                                      itemCount: _selectedMap!.waypoints.length,
                                      separatorBuilder: (context, index) =>
                                          const SizedBox(height: 8),
                                      itemBuilder: (context, index) {
                                        final waypoint = _selectedMap!.waypoints[index];
                                        final isSelected = _selectedWaypoint?.id == waypoint.id;

                                        return Card(
                                          elevation: isSelected ? 4 : 1,
                                          color: isSelected ? Colors.blue[100] : Colors.white,
                                          child: ListTile(
                                            leading: Icon(
                                              Icons.location_on,
                                              color: isSelected ? Colors.blue : Colors.grey,
                                            ),
                                            title: Text(
                                              waypoint.name,
                                              style: TextStyle(
                                                fontWeight: isSelected
                                                    ? FontWeight.bold
                                                    : FontWeight.normal,
                                              ),
                                            ),
                                            subtitle: Text(
                                              'x: ${waypoint.x.toStringAsFixed(2)}, y: ${waypoint.y.toStringAsFixed(2)}',
                                            ),
                                            trailing: _isNavigatingToWaypoint && isSelected
                                                ? const SizedBox(
                                                    width: 20,
                                                    height: 20,
                                                    child: CircularProgressIndicator(strokeWidth: 2),
                                                  )
                                                : Icon(
                                                    Icons.arrow_forward_ios,
                                                    size: 16,
                                                    color: _isNavigationActive
                                                        ? Colors.blue
                                                        : Colors.grey,
                                                  ),
                                            onTap: () => _navigateToWaypoint(waypoint),
                                          ),
                                        );
                                      },
                                    ),
                                ],
                              ),
                            ),
                ),
              ],
            ),
    );
  }

  String _formatDate(DateTime date) {
    return '${date.day}/${date.month}/${date.year}';
  }
}
