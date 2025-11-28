import 'package:http/http.dart' as http;

class MappingService {
  final String robotUrl;

  MappingService({required this.robotUrl});

  String _ensureUrl(String url) {
    String finalUrl = url.trim();
    if (!finalUrl.startsWith('http://') && !finalUrl.startsWith('https://')) {
      finalUrl = 'http://$finalUrl';
    }
    // Remove trailing slash
    if (finalUrl.endsWith('/')) {
      finalUrl = finalUrl.substring(0, finalUrl.length - 1);
    }
    return finalUrl;
  }

  Future<bool> checkConnection() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.get(
        Uri.parse('$url/status'),
      ).timeout(const Duration(seconds: 5));
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<String> startMapping() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mapping/start'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return 'Mapping started successfully';
      } else {
        return 'Failed to start mapping';
      }
    } catch (e) {
      throw Exception('Error starting mapping: $e');
    }
  }

  Future<String> stopMapping() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mapping/stop'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return 'Mapping stopped successfully';
      } else {
        return 'Failed to stop mapping';
      }
    } catch (e) {
      throw Exception('Error stopping mapping: $e');
    }
  }

  Future<Map<String, dynamic>> saveMapping() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mapping/save'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        // Expecting response with map image and XML data
        return {
          'success': true,
          'message': 'Map saved successfully',
          'data': response.body,
        };
      } else {
        return {
          'success': false,
          'message': 'Failed to save mapping',
        };
      }
    } catch (e) {
      throw Exception('Error saving mapping: $e');
    }
  }

  Future<String> moveForward() async {
    return sendCommand('forward');
  }

  Future<String> moveBackward() async {
    return sendCommand('backward');
  }

  Future<String> turnLeft() async {
    return sendCommand('left');
  }

  Future<String> turnRight() async {
    return sendCommand('right');
  }

  Future<String> stopMovement() async {
    return sendCommand('stop');
  }

  Future<String> startAutoMode() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mode/auto/start'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return 'Auto mode started';
      } else {
        return 'Failed to start auto mode';
      }
    } catch (e) {
      throw Exception('Error starting auto mode: $e');
    }
  }

  Future<String> stopAutoMode() async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/mode/auto/stop'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return 'Auto mode stopped';
      } else {
        return 'Failed to stop auto mode';
      }
    } catch (e) {
      throw Exception('Error stopping auto mode: $e');
    }
  }

  Future<String> sendCommand(String command) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/control'),
        body: command,
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return 'Command executed';
      } else {
        return 'Command failed';
      }
    } catch (e) {
      throw Exception('Error sending command: $e');
    }
  }

  Future<String> navigateToWaypoint(String mapName, String waypointId) async {
    try {
      final url = _ensureUrl(robotUrl);
      final response = await http.post(
        Uri.parse('$url/navigate'),
        body: {'map': mapName, 'waypoint': waypointId},
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return 'Navigation started';
      } else {
        return 'Navigation failed';
      }
    } catch (e) {
      throw Exception('Error navigating: $e');
    }
  }
}
