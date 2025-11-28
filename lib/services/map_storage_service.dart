import '../models/map_model.dart';

class MapStorageService {
  static final MapStorageService _instance = MapStorageService._internal();

  final List<SavedMap> _maps = [];

  factory MapStorageService() {
    return _instance;
  }

  MapStorageService._internal();

  // Load maps from storage (in-memory for now)
  Future<List<SavedMap>> loadMaps() async {
    // In production, load from SharedPreferences or local database
    // For now, return the in-memory list
    return _maps;
  }

  // Save a new map
  Future<void> saveMap(SavedMap map) async {
    _maps.add(map);
    // In production, persist to SharedPreferences or local database
    await _persistMaps();
  }

  // Update existing map
  Future<void> updateMap(SavedMap map) async {
    final index = _maps.indexWhere((m) => m.id == map.id);
    if (index != -1) {
      _maps[index] = map;
      await _persistMaps();
    }
  }

  // Delete a map
  Future<void> deleteMap(String mapId) async {
    _maps.removeWhere((m) => m.id == mapId);
    await _persistMaps();
  }

  // Get map by ID
  SavedMap? getMapById(String mapId) {
    try {
      return _maps.firstWhere((m) => m.id == mapId);
    } catch (e) {
      return null;
    }
  }

  // Get all maps
  List<SavedMap> getAllMaps() {
    return _maps;
  }

  // Parse XML waypoints from string
  Future<List<Waypoint>> parseWaypoints(String xmlContent) async {
    try {
      final waypoints = <Waypoint>[];
      // Simple regex-based XML parsing for waypoints
      final regex = RegExp(
        r'<waypoint\s+id="([^"]+)"\s+name="([^"]+)"\s+x="([^"]+)"\s+y="([^"]+)"\s+theta="([^"]+)"',
        caseSensitive: false,
      );

      final matches = regex.allMatches(xmlContent);
      for (final match in matches) {
        waypoints.add(
          Waypoint(
            id: match.group(1) ?? '',
            name: match.group(2) ?? '',
            x: double.tryParse(match.group(3) ?? '0') ?? 0,
            y: double.tryParse(match.group(4) ?? '0') ?? 0,
            theta: double.tryParse(match.group(5) ?? '0') ?? 0,
          ),
        );
      }
      return waypoints;
    } catch (e) {
      return [];
    }
  }

  // Persist maps to storage (placeholder)
  Future<void> _persistMaps() async {
    // In production, use SharedPreferences or Hive
    // For now, just keep in memory
  }

  // Clear all maps
  Future<void> clearAllMaps() async {
    _maps.clear();
    await _persistMaps();
  }

  // Search maps by name
  List<SavedMap> searchMaps(String query) {
    return _maps
        .where((map) => map.name.toLowerCase().contains(query.toLowerCase()))
        .toList();
  }

  // Sort maps by creation date
  List<SavedMap> getSortedMaps({bool descending = true}) {
    final sorted = [..._maps];
    sorted.sort((a, b) {
      if (descending) {
        return b.createdAt.compareTo(a.createdAt);
      } else {
        return a.createdAt.compareTo(b.createdAt);
      }
    });
    return sorted;
  }
}
