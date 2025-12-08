import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/map_model.dart';
import '../services/mapping_service.dart';
import '../providers/robot_url_provider.dart';

class MapsScreen extends StatefulWidget {
  const MapsScreen({super.key});

  @override
  State<MapsScreen> createState() => _MapsScreenState();
}

class _MapsScreenState extends State<MapsScreen> {
  MappingService? _mappingService;
  List<SavedMap> _maps = [];
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _loadMaps();
  }

  Future<void> _loadMaps() async {
    setState(() {
      _isLoading = true;
    });

    try {
      // Get robot URL from provider
      final robotUrl = Provider.of<RobotUrlProvider>(context, listen: false).robotUrl;

      if (robotUrl.isEmpty) {
        setState(() {
          _isLoading = false;
          _maps = [];
        });
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
              content: Text('Please connect to robot first from the Control tab'),
              backgroundColor: Colors.orange,
            ),
          );
        }
        return;
      }

      // Create MappingService with robot URL
      _mappingService = MappingService(robotUrl: robotUrl);

      // Fetch maps from Flask API
      final mapsList = await _mappingService!.listMaps();

      // Convert API response to SavedMap objects
      List<SavedMap> loadedMaps = [];
      for (var mapData in mapsList) {
        try {
          // Get waypoints for this map
          final waypointsData = await _mappingService!.getWaypoints(
            mapName: mapData['name'],
          );

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

          // Create SavedMap object
          loadedMaps.add(SavedMap(
            id: mapData['name'],
            name: mapData['name'],
            imagePath: '', // Image will be loaded on demand
            mapXmlPath: '',
            waypointsXmlPath: '',
            createdAt: DateTime.fromMillisecondsSinceEpoch(
              (mapData['created'] * 1000).toInt(),
            ),
            waypoints: waypoints,
          ));
        } catch (e) {
          debugPrint('Error loading map ${mapData['name']}: $e');
        }
      }

      setState(() {
        _maps = loadedMaps;
        _isLoading = false;
      });
    } catch (e) {
      setState(() {
        _isLoading = false;
      });
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Error loading maps: $e'),
            backgroundColor: Colors.red,
          ),
        );
      }
    }
  }

  void _viewMapInfo(SavedMap map) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(map.name),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Created: ${_formatDateTime(map.createdAt)}'),
            const SizedBox(height: 8),
            Text('Waypoints: ${map.waypoints.length}'),
            const SizedBox(height: 8),
            Text('Map ID: ${map.id}'),
            const SizedBox(height: 16),
            const Text(
              'To navigate to waypoints, go to the Navigate tab and enter this map name.',
              style: TextStyle(fontStyle: FontStyle.italic, fontSize: 12),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Close'),
          ),
        ],
      ),
    );
  }


  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Saved Maps'),
        centerTitle: true,
        elevation: 0,
      ),
      body: _isLoading
          ? const Center(
              child: CircularProgressIndicator(),
            )
          : _maps.isEmpty
              ? Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(
                        Icons.map_outlined,
                        size: 64,
                        color: Colors.grey[400],
                      ),
                      const SizedBox(height: 16),
                      Text(
                        'No maps saved yet',
                        style: Theme.of(context).textTheme.titleLarge,
                      ),
                      const SizedBox(height: 8),
                      Text(
                        'Create a map by using the mapping screen',
                        style: Theme.of(context).textTheme.bodyMedium?.copyWith(
                              color: Colors.grey[600],
                            ),
                      ),
                    ],
                  ),
                )
              : ListView.builder(
                  padding: const EdgeInsets.all(16),
                  itemCount: _maps.length,
                  itemBuilder: (context, index) {
                    final map = _maps[index];
                    return _buildMapCard(map);
                  },
                ),
    );
  }

  Widget _buildMapCard(SavedMap map) {
    return Card(
      margin: const EdgeInsets.only(bottom: 16),
      child: InkWell(
        onTap: () => _viewMapInfo(map),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Map preview image placeholder
            Container(
              height: 200,
              width: double.infinity,
              decoration: BoxDecoration(
                color: Colors.grey[300],
                borderRadius: const BorderRadius.only(
                  topLeft: Radius.circular(4),
                  topRight: Radius.circular(4),
                ),
              ),
              child: Center(
                child: Icon(
                  Icons.map_outlined,
                  size: 64,
                  color: Colors.grey[600],
                ),
              ),
            ),
            // Map info
            Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    map.name,
                    style: Theme.of(context).textTheme.titleLarge,
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                  ),
                  const SizedBox(height: 8),
                  Row(
                    children: [
                      Icon(
                        Icons.location_on_outlined,
                        size: 16,
                        color: Colors.grey[600],
                      ),
                      const SizedBox(width: 4),
                      Text(
                        '${map.waypoints.length} waypoints',
                        style: Theme.of(context).textTheme.bodySmall?.copyWith(
                              color: Colors.grey[600],
                            ),
                      ),
                      const SizedBox(width: 16),
                      Icon(
                        Icons.access_time_outlined,
                        size: 16,
                        color: Colors.grey[600],
                      ),
                      const SizedBox(width: 4),
                      Text(
                        _formatDate(map.createdAt),
                        style: Theme.of(context).textTheme.bodySmall?.copyWith(
                              color: Colors.grey[600],
                            ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  String _formatDate(DateTime date) {
    return '${date.day}/${date.month}/${date.year}';
  }

  String _formatDateTime(DateTime dateTime) {
    return '${dateTime.day}/${dateTime.month}/${dateTime.year} '
        '${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}';
  }
}
