import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/map_model.dart';
import '../services/mapping_service.dart';
import '../services/map_storage_service.dart';
import '../providers/robot_url_provider.dart';
import 'dart:io';

class MapDetailScreen extends StatefulWidget {
  const MapDetailScreen({super.key});

  @override
  State<MapDetailScreen> createState() => _MapDetailScreenState();
}

class _MapDetailScreenState extends State<MapDetailScreen> {
  final TextEditingController _mapNameController = TextEditingController();
  final MapStorageService _storageService = MapStorageService();

  SavedMap? _currentMap;
  Waypoint? _selectedWaypoint;
  bool _isNavigating = false;
  bool _isLoading = false;
  late MappingService _mappingService;

  @override
  void initState() {
    super.initState();
    final robotUrl = Provider.of<RobotUrlProvider>(
      context,
      listen: false,
    ).robotUrl;
    _mappingService = MappingService(robotUrl: robotUrl);
  }

  @override
  void dispose() {
    _mapNameController.dispose();
    super.dispose();
  }

  Future<void> _loadMapByName() async {
    final mapName = _mapNameController.text.trim();

    if (mapName.isEmpty) {
      _showMessage('Please enter a map name', Colors.orange);
      return;
    }

    setState(() {
      _isLoading = true;
    });

    try {
      // Search for map by name
      final maps = _storageService.searchMaps(mapName);

      if (maps.isEmpty) {
        if (mounted) {
          setState(() {
            _isLoading = false;
            _currentMap = null;
          });
          _showMessage('No map found with name: $mapName', Colors.red);
        }
        return;
      }

      // Get the first matching map
      final foundMap = maps.first;

      // Try to load waypoints from XML file if it exists
      List<Waypoint> waypoints = foundMap.waypoints;

      if (foundMap.waypointsXmlPath.isNotEmpty) {
        try {
          final file = File(foundMap.waypointsXmlPath);
          if (await file.exists()) {
            final xmlContent = await file.readAsString();
            waypoints = await _storageService.parseWaypoints(xmlContent);

            // Update map with parsed waypoints
            final updatedMap = SavedMap(
              id: foundMap.id,
              name: foundMap.name,
              imagePath: foundMap.imagePath,
              mapXmlPath: foundMap.mapXmlPath,
              waypointsXmlPath: foundMap.waypointsXmlPath,
              createdAt: foundMap.createdAt,
              waypoints: waypoints,
            );

            if (mounted) {
              setState(() {
                _currentMap = updatedMap;
                _isLoading = false;
              });
              _showMessage('Map loaded with ${waypoints.length} waypoints', Colors.green);
            }
            return;
          }
        } catch (e) {
          // XML parsing failed, use existing waypoints
          // Silently continue with existing waypoints
        }
      }

      // Use existing waypoints if XML loading failed or doesn't exist
      if (mounted) {
        setState(() {
          _currentMap = foundMap;
          _isLoading = false;
        });
        _showMessage('Map loaded with ${waypoints.length} waypoints', Colors.green);
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _isLoading = false;
        });
        _showMessage('Error loading map: $e', Colors.red);
      }
    }
  }

  Future<void> _navigateToWaypoint() async {
    if (_selectedWaypoint == null) {
      _showMessage('Please select a waypoint', Colors.orange);
      return;
    }

    if (_currentMap == null) {
      _showMessage('No map loaded', Colors.orange);
      return;
    }

    setState(() {
      _isNavigating = true;
    });

    try {
      await _mappingService.navigateToWaypoint(
        waypoint: _selectedWaypoint!.id,
      );

      if (mounted) {
        setState(() {
          _isNavigating = false;
        });

        _showMessage('Navigation started to ${_selectedWaypoint!.name}', Colors.green);
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _isNavigating = false;
        });

        _showMessage('Navigation error: $e', Colors.red);
      }
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
        title: const Text('Map Waypoint Navigator'),
        centerTitle: true,
      ),
      body: SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              // Map name input section
              Card(
                elevation: 2,
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Load Map',
                        style: Theme.of(context).textTheme.titleLarge,
                      ),
                      const SizedBox(height: 12),
                      TextField(
                        controller: _mapNameController,
                        decoration: InputDecoration(
                          labelText: 'Enter Map Name',
                          hintText: 'e.g., Map_2024-01-01',
                          border: OutlineInputBorder(
                            borderRadius: BorderRadius.circular(8),
                          ),
                          prefixIcon: const Icon(Icons.map),
                        ),
                        onSubmitted: (_) => _loadMapByName(),
                      ),
                      const SizedBox(height: 12),
                      SizedBox(
                        width: double.infinity,
                        child: ElevatedButton.icon(
                          onPressed: _isLoading ? null : _loadMapByName,
                          icon: _isLoading
                              ? const SizedBox(
                                  width: 20,
                                  height: 20,
                                  child: CircularProgressIndicator(
                                    strokeWidth: 2,
                                  ),
                                )
                              : const Icon(Icons.search),
                          label: Text(_isLoading ? 'Loading...' : 'Load Map'),
                          style: ElevatedButton.styleFrom(
                            padding: const EdgeInsets.symmetric(vertical: 12),
                            backgroundColor: Colors.blue,
                            foregroundColor: Colors.white,
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 20),

              // Map preview and details
              if (_currentMap != null) ...[
                // Map preview placeholder
                Container(
                  height: 250,
                  width: double.infinity,
                  decoration: BoxDecoration(
                    color: Colors.grey[300],
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(
                          Icons.map_outlined,
                          size: 80,
                          color: Colors.grey[600],
                        ),
                        const SizedBox(height: 8),
                        Text(
                          _currentMap!.name,
                          style: TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                            color: Colors.grey[700],
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 20),

                // Map information
                Card(
                  elevation: 2,
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          'Map Information',
                          style: Theme.of(context).textTheme.titleLarge,
                        ),
                        const SizedBox(height: 12),
                        _buildInfoRow('Name', _currentMap!.name),
                        const SizedBox(height: 8),
                        _buildInfoRow(
                          'Created',
                          _formatDateTime(_currentMap!.createdAt),
                        ),
                        const SizedBox(height: 8),
                        _buildInfoRow(
                          'Waypoints',
                          '${_currentMap!.waypoints.length}',
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 20),

                // Waypoints section
                Text(
                  'Available Waypoints',
                  style: Theme.of(context).textTheme.titleLarge,
                ),
                const SizedBox(height: 12),

                if (_currentMap!.waypoints.isEmpty)
                  Center(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(vertical: 32),
                      child: Column(
                        children: [
                          Icon(
                            Icons.location_off,
                            size: 48,
                            color: Colors.grey[400],
                          ),
                          const SizedBox(height: 12),
                          Text(
                            'No waypoints found for this map',
                            style: Theme.of(context).textTheme.bodyMedium?.copyWith(
                                  color: Colors.grey[600],
                                ),
                          ),
                        ],
                      ),
                    ),
                  )
                else
                  // Waypoint buttons list (scrollable)
                  Container(
                    constraints: const BoxConstraints(maxHeight: 400),
                    child: ListView.separated(
                      shrinkWrap: true,
                      itemCount: _currentMap!.waypoints.length,
                      separatorBuilder: (context, index) =>
                          const SizedBox(height: 8),
                      itemBuilder: (context, index) {
                        final waypoint = _currentMap!.waypoints[index];
                        final isSelected = _selectedWaypoint?.id == waypoint.id;

                        return Card(
                          elevation: isSelected ? 4 : 1,
                          color: isSelected
                              ? Colors.blue[100]
                              : Colors.white,
                          child: InkWell(
                            onTap: () {
                              setState(() {
                                _selectedWaypoint = waypoint;
                              });
                            },
                            child: Padding(
                              padding: const EdgeInsets.all(12),
                              child: Row(
                                children: [
                                  // Waypoint icon
                                  Container(
                                    padding: const EdgeInsets.all(8),
                                    decoration: BoxDecoration(
                                      color: isSelected
                                          ? Colors.blue[600]
                                          : Colors.grey[300],
                                      shape: BoxShape.circle,
                                    ),
                                    child: Icon(
                                      Icons.location_on,
                                      color: isSelected
                                          ? Colors.white
                                          : Colors.grey[700],
                                      size: 24,
                                    ),
                                  ),
                                  const SizedBox(width: 12),

                                  // Waypoint details
                                  Expanded(
                                    child: Column(
                                      crossAxisAlignment:
                                          CrossAxisAlignment.start,
                                      children: [
                                        Text(
                                          waypoint.name,
                                          style: Theme.of(context)
                                              .textTheme
                                              .titleMedium
                                              ?.copyWith(
                                                fontWeight: isSelected
                                                    ? FontWeight.bold
                                                    : FontWeight.normal,
                                              ),
                                        ),
                                        const SizedBox(height: 4),
                                        Text(
                                          'ID: ${waypoint.id}',
                                          style: Theme.of(context)
                                              .textTheme
                                              .bodySmall
                                              ?.copyWith(
                                                color: Colors.grey[600],
                                              ),
                                        ),
                                        const SizedBox(height: 4),
                                        Text(
                                          'Position: (${waypoint.x.toStringAsFixed(2)}, ${waypoint.y.toStringAsFixed(2)}) | θ: ${waypoint.theta.toStringAsFixed(2)}°',
                                          style: Theme.of(context)
                                              .textTheme
                                              .bodySmall,
                                        ),
                                      ],
                                    ),
                                  ),

                                  // Selection indicator
                                  if (isSelected)
                                    Icon(
                                      Icons.check_circle,
                                      color: Colors.blue[600],
                                      size: 28,
                                    ),
                                ],
                              ),
                            ),
                          ),
                        );
                      },
                    ),
                  ),
                const SizedBox(height: 20),

                // Navigate button
                SizedBox(
                  width: double.infinity,
                  child: ElevatedButton.icon(
                    onPressed: (_isNavigating || _selectedWaypoint == null)
                        ? null
                        : _navigateToWaypoint,
                    icon: _isNavigating
                        ? const SizedBox(
                            width: 20,
                            height: 20,
                            child: CircularProgressIndicator(
                              strokeWidth: 2,
                              valueColor: AlwaysStoppedAnimation<Color>(
                                Colors.white,
                              ),
                            ),
                          )
                        : const Icon(Icons.navigation),
                    label: Padding(
                      padding: const EdgeInsets.all(12),
                      child: Text(
                        _isNavigating
                            ? 'Starting Navigation...'
                            : 'Start Navigation to Waypoint',
                      ),
                    ),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.green,
                      foregroundColor: Colors.white,
                      disabledBackgroundColor: Colors.grey,
                    ),
                  ),
                ),
              ] else
                // Empty state when no map is loaded
                Center(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(vertical: 64),
                    child: Column(
                      children: [
                        Icon(
                          Icons.map_outlined,
                          size: 80,
                          color: Colors.grey[400],
                        ),
                        const SizedBox(height: 16),
                        Text(
                          'Enter a map name to load waypoints',
                          style: Theme.of(context).textTheme.bodyLarge?.copyWith(
                                color: Colors.grey[600],
                              ),
                        ),
                      ],
                    ),
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        Text(
          label,
          style: Theme.of(context).textTheme.bodyMedium,
        ),
        Text(
          value,
          style: Theme.of(context)
              .textTheme
              .bodyMedium
              ?.copyWith(fontWeight: FontWeight.bold),
        ),
      ],
    );
  }

  String _formatDateTime(DateTime dateTime) {
    return '${dateTime.day}/${dateTime.month}/${dateTime.year} '
        '${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}';
  }
}
