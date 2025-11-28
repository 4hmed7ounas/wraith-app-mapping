import 'package:flutter/foundation.dart';

class RobotUrlProvider extends ChangeNotifier {
  String _robotUrl = '';

  String get robotUrl => _robotUrl;

  void setRobotUrl(String url) {
    _robotUrl = url;
    notifyListeners();
  }

  bool isConfigured() {
    return _robotUrl.isNotEmpty;
  }

  void clear() {
    _robotUrl = '';
    notifyListeners();
  }
}
