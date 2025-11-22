import 'package:http/http.dart' as http;

class MappingService {
  final String robotUrl;

  MappingService({required this.robotUrl});

  Future<bool> checkConnection() async {
    try {
      final response = await http.get(
        Uri.parse('$robotUrl/status'),
      ).timeout(const Duration(seconds: 5));
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<String> startMapping() async {
    try {
      final response = await http.post(
        Uri.parse('$robotUrl/mapping/start'),
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
      final response = await http.post(
        Uri.parse('$robotUrl/mapping/stop'),
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

  Future<String> saveMapping() async {
    try {
      final response = await http.post(
        Uri.parse('$robotUrl/mapping/save'),
      ).timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        return 'Map saved successfully';
      } else {
        return 'Failed to save mapping';
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
      final response = await http.post(
        Uri.parse('$robotUrl/mode/auto/start'),
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
      final response = await http.post(
        Uri.parse('$robotUrl/mode/auto/stop'),
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
      final response = await http.post(
        Uri.parse('$robotUrl/control'),
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
}
