class SavedMap {
  final String id;
  final String name;
  final String imagePath; // Path to PNG image
  final String mapXmlPath; // Path to map XML
  final String waypointsXmlPath; // Path to waypoints XML
  final DateTime createdAt;
  final List<Waypoint> waypoints;

  SavedMap({
    required this.id,
    required this.name,
    required this.imagePath,
    required this.mapXmlPath,
    required this.waypointsXmlPath,
    required this.createdAt,
    required this.waypoints,
  });

  factory SavedMap.fromJson(Map<String, dynamic> json) {
    return SavedMap(
      id: json['id'] as String,
      name: json['name'] as String,
      imagePath: json['imagePath'] as String,
      mapXmlPath: json['mapXmlPath'] as String,
      waypointsXmlPath: json['waypointsXmlPath'] as String,
      createdAt: DateTime.parse(json['createdAt'] as String),
      waypoints: (json['waypoints'] as List?)
          ?.map((w) => Waypoint.fromJson(w))
          .toList() ??
          [],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'name': name,
      'imagePath': imagePath,
      'mapXmlPath': mapXmlPath,
      'waypointsXmlPath': waypointsXmlPath,
      'createdAt': createdAt.toIso8601String(),
      'waypoints': waypoints.map((w) => w.toJson()).toList(),
    };
  }
}

class Waypoint {
  final String id;
  final String name;
  final double x;
  final double y;
  final double theta; // Orientation angle

  Waypoint({
    required this.id,
    required this.name,
    required this.x,
    required this.y,
    required this.theta,
  });

  factory Waypoint.fromJson(Map<String, dynamic> json) {
    return Waypoint(
      id: json['id'] as String,
      name: json['name'] as String,
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
      theta: (json['theta'] as num).toDouble(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'name': name,
      'x': x,
      'y': y,
      'theta': theta,
    };
  }
}
