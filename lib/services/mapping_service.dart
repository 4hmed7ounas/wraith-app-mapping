import 'dart:convert';
import 'package:http/http.dart' as http;

/// Wraith Robot Mapping Service
/// Complete HTTP client for Flask control server
class MappingService {
  final String robotUrl;

  MappingService({required this.robotUrl});

  String _ensureUrl(String url) {
    String finalUrl = url.trim();
    if (!finalUrl.startsWith('http://') && !finalUrl.startsWith('https://')) {
      finalUrl = 'http://$finalUrl';
    }
    if (finalUrl.endsWith('/')) {
      finalUrl = finalUrl.substring(0, finalUrl.length - 1);
    }
    return finalUrl;
  }

  // ========== CONNECTION & STATUS ==========

  /// Check if robot is online and responding
  Future<bool> checkConnection() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/status'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        return data['status'] == 'ok';
      }
      return false;
    } catch (e) {
      return false;
    }
  }

  /// Get detailed robot status
  Future<Map<String, dynamic>> getStatus() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/status'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      return {'status': 'error', 'message': 'Failed to get status'};
    } catch (e) {
      throw Exception('Error getting status: $e');
    }
  }

  /// Get robot IP address
  Future<String> getRobotIp() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/get_ip'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        return data['ip'] ?? 'Unknown';
      }
      return 'Unknown';
    } catch (e) {
      throw Exception('Error getting IP: $e');
    }
  }

  /// Get ultrasonic sensor distance (cm)
  Future<double> getDistance() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/get_distance'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        return (data['distance'] ?? 0.0).toDouble();
      }
      return 0.0;
    } catch (e) {
      throw Exception('Error getting distance: $e');
    }
  }

  // ========== MAPPING CONTROL ==========

  /// Start SLAM mapping session
  /// This launches ROS2 mapping.launch.py in subprocess
  Future<Map<String, dynamic>> startMapping() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mapping/start'),
      ).timeout(const Duration(seconds: 10));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Failed to start mapping'
        };
      }
    } catch (e) {
      throw Exception('Error starting mapping: $e');
    }
  }

  /// Activate SLAM Toolbox lifecycle
  /// Call this AFTER startMapping() to begin actual SLAM
  Future<Map<String, dynamic>> activateSlam() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/slam/activate'),
      ).timeout(const Duration(seconds: 15));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Failed to activate SLAM'
        };
      }
    } catch (e) {
      throw Exception('Error activating SLAM: $e');
    }
  }

  /// Stop SLAM mapping session
  Future<Map<String, dynamic>> stopMapping() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mapping/stop'),
      ).timeout(const Duration(seconds: 10));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Failed to stop mapping'
        };
      }
    } catch (e) {
      throw Exception('Error stopping mapping: $e');
    }
  }

  /// Save current map with waypoints
  Future<Map<String, dynamic>> saveMapping({String? mapName}) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mapping/save'),
        headers: {'Content-Type': 'application/json'},
        body: json.encode({
          'map_name': mapName ?? 'wraith_map_${DateTime.now().millisecondsSinceEpoch}'
        }),
      ).timeout(const Duration(seconds: 15));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Failed to save map'
        };
      }
    } catch (e) {
      throw Exception('Error saving mapping: $e');
    }
  }

  // ========== MANUAL CONTROL ==========

  /// Send control command to robot
  /// Commands: forward_start, forward_stop, backward_start, backward_stop,
  ///          left_start, left_stop, right_start, right_stop,
  ///          speed+, speed-, auto_start, auto_stop
  Future<String> sendCommand(String command) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/control'),
        body: command,
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        return data['message'] ?? 'Command executed';
      } else {
        return 'Command failed';
      }
    } catch (e) {
      throw Exception('Error sending command: $e');
    }
  }

  /// Start autonomous exploration mode
  Future<String> startAutoMode() async {
    try {
      return await sendCommand('auto_start');
    } catch (e) {
      throw Exception('Error starting auto mode: $e');
    }
  }

  /// Stop autonomous exploration mode
  Future<String> stopAutoMode() async {
    try {
      return await sendCommand('auto_stop');
    } catch (e) {
      throw Exception('Error stopping auto mode: $e');
    }
  }

  /// Increase robot speed by 10%
  Future<String> increaseSpeed() async {
    try {
      return await sendCommand('speed+');
    } catch (e) {
      throw Exception('Error increasing speed: $e');
    }
  }

  /// Decrease robot speed by 10%
  Future<String> decreaseSpeed() async {
    try {
      return await sendCommand('speed-');
    } catch (e) {
      throw Exception('Error decreasing speed: $e');
    }
  }

  // Convenience methods for Flutter D-pad
  Future<String> moveForward() => sendCommand('forward_start');
  Future<String> stopForward() => sendCommand('forward_stop');
  Future<String> moveBackward() => sendCommand('backward_start');
  Future<String> stopBackward() => sendCommand('backward_stop');
  Future<String> turnLeft() => sendCommand('left_start');
  Future<String> stopLeft() => sendCommand('left_stop');
  Future<String> turnRight() => sendCommand('right_start');
  Future<String> stopRight() => sendCommand('right_stop');
  Future<String> stopMovement() => sendCommand('forward_stop'); // Stop all

  // ========== NAVIGATION CONTROL ==========

  /// Start navigation system with saved map
  Future<Map<String, dynamic>> startNavigation({required String mapName}) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/navigation/start'),
        headers: {'Content-Type': 'application/json'},
        body: json.encode({'map_name': mapName}),
      ).timeout(const Duration(seconds: 15));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Failed to start navigation'
        };
      }
    } catch (e) {
      throw Exception('Error starting navigation: $e');
    }
  }

  /// Stop navigation system
  Future<Map<String, dynamic>> stopNavigation() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/navigation/stop'),
      ).timeout(const Duration(seconds: 10));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Failed to stop navigation'
        };
      }
    } catch (e) {
      throw Exception('Error stopping navigation: $e');
    }
  }

  /// Navigate to specific waypoint
  Future<Map<String, dynamic>> navigateToWaypoint({required String waypoint}) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/navigate'),
        headers: {'Content-Type': 'application/json'},
        body: json.encode({'waypoint': waypoint}),
      ).timeout(const Duration(seconds: 10));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Navigation failed'
        };
      }
    } catch (e) {
      throw Exception('Error navigating to waypoint: $e');
    }
  }

  // ========== MAPS & WAYPOINTS ==========

  /// Get list of all saved maps
  Future<List<Map<String, dynamic>>> listMaps() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/maps/list'),
      ).timeout(const Duration(seconds: 10));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        if (data['status'] == 'ok') {
          return List<Map<String, dynamic>>.from(data['maps']);
        }
      }
      return [];
    } catch (e) {
      throw Exception('Error listing maps: $e');
    }
  }

  /// Get waypoints for specific map
  Future<Map<String, dynamic>> getWaypoints({required String mapName}) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/maps/$mapName/waypoints'),
      ).timeout(const Duration(seconds: 10));

      if (response.statusCode == 200) {
        return json.decode(response.body);
      } else {
        final error = json.decode(response.body);
        return {
          'status': 'error',
          'message': error['message'] ?? 'Failed to get waypoints'
        };
      }
    } catch (e) {
      throw Exception('Error getting waypoints: $e');
    }
  }

  /// Get map image as base64
  Future<String?> getMapImage({required String mapName}) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/maps/$mapName/image'),
      ).timeout(const Duration(seconds: 10));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        return data['png_base64'];
      }
      return null;
    } catch (e) {
      throw Exception('Error getting map image: $e');
    }
  }
}
