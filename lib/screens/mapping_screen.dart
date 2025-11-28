import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/mapping_service.dart';
import '../services/map_storage_service.dart';
import '../models/map_model.dart';
import '../providers/robot_url_provider.dart';

class MappingScreen extends StatefulWidget {
  const MappingScreen({super.key});

  @override
  State<MappingScreen> createState() => _MappingScreenState();
}

class _MappingScreenState extends State<MappingScreen> {
  final TextEditingController _ipController = TextEditingController();
  final TextEditingController _portController = TextEditingController();

  late MappingService _mappingService;
  late MapStorageService _storageService;

  bool _isConnected = false;
  bool _isMappingStarted = false;
  bool _isAutoMode = false;
  bool _isManualMode = false;
  bool _isSaving = false;
  String _statusMessage = 'Ready to connect';

  String? _currentDirection;
  bool _isHoldingButton = false;

  @override
  void initState() {
    super.initState();
    _portController.text = '5000';
    _storageService = MapStorageService();
  }

  @override
  void dispose() {
    _ipController.dispose();
    _portController.dispose();
    super.dispose();
  }

  String get _robotUrl {
    final ip = _ipController.text.trim();
    final port = _portController.text.trim();
    return 'http://$ip:$port';
  }

  Future<void> _handleConnect() async {
    final ip = _ipController.text.trim();
    final port = _portController.text.trim();

    if (ip.isEmpty || port.isEmpty) {
      _showMessage('Please enter IP address and port number', Colors.orange);
      return;
    }

    try {
      _mappingService = MappingService(robotUrl: _robotUrl);
      final isConnected = await _mappingService.checkConnection();

      if (!mounted) return;

      if (isConnected) {
        setState(() {
          _isConnected = true;
          _statusMessage = 'Connected to robot at $_robotUrl';
        });
        _showMessage('Connected to robot', Colors.green);

        if (mounted) {
          Provider.of<RobotUrlProvider>(context, listen: false)
              .setRobotUrl(_robotUrl);
        }
      } else {
        _showMessage('Failed to connect to robot', Colors.red);
      }
    } catch (e) {
      _showMessage('Connection error: $e', Colors.red);
    }
  }

  void _handleDisconnect() {
    setState(() {
      _isConnected = false;
      _isMappingStarted = false;
      _isAutoMode = false;
      _isManualMode = false;
      _statusMessage = 'Disconnected';
    });
    _showMessage('Disconnected from robot', Colors.blue);
  }

  Future<void> _startMapping() async {
    if (!_isConnected) {
      _showMessage('Robot not connected', Colors.red);
      return;
    }

    try {
      final result = await _mappingService.startMapping();
      setState(() {
        _isMappingStarted = true;
        _statusMessage = result;
      });
      _showMessage(result, Colors.green);
    } catch (e) {
      _showMessage('Error: $e', Colors.red);
    }
  }

  Future<void> _stopMapping() async {
    if (!_isConnected) {
      _showMessage('Robot not connected', Colors.red);
      return;
    }

    try {
      final result = await _mappingService.stopMapping();
      setState(() {
        _isMappingStarted = false;
        _isManualMode = false;
        _isAutoMode = false;
        _statusMessage = result;
      });
      _showMessage(result, Colors.green);
    } catch (e) {
      _showMessage('Error: $e', Colors.red);
    }
  }

  Future<void> _saveMapping() async {
    if (!_isConnected) {
      _showMessage('Robot not connected', Colors.red);
      return;
    }

    setState(() {
      _isSaving = true;
      _statusMessage = 'Saving map...';
    });

    try {
      final result = await _mappingService.saveMapping();

      if (!mounted) return;

      if (result['success'] == true) {
        final waypoints = [
          Waypoint(
            id: 'wp_1',
            name: 'Start Point',
            x: 0.0,
            y: 0.0,
            theta: 0.0,
          ),
          Waypoint(
            id: 'wp_2',
            name: 'Mid Point',
            x: 5.0,
            y: 5.0,
            theta: 45.0,
          ),
        ];

        final savedMap = SavedMap(
          id: DateTime.now().millisecondsSinceEpoch.toString(),
          name: 'Map_${DateTime.now().toString().split('.')[0]}',
          imagePath: 'assets/maps/map_${DateTime.now().millisecondsSinceEpoch}.png',
          mapXmlPath: 'assets/maps/map_${DateTime.now().millisecondsSinceEpoch}.xml',
          waypointsXmlPath: 'assets/maps/waypoints_${DateTime.now().millisecondsSinceEpoch}.xml',
          createdAt: DateTime.now(),
          waypoints: waypoints,
        );

        await _storageService.saveMap(savedMap);

        setState(() {
          _isSaving = false;
          _statusMessage = 'Map saved successfully!';
        });

        _showMessage('Map saved successfully!', Colors.green);

      } else {
        setState(() {
          _isSaving = false;
        });
        _showMessage(result['message'] ?? 'Failed to save map', Colors.red);
      }
    } catch (e) {
      setState(() {
        _isSaving = false;
      });
      _showMessage('Error: $e', Colors.red);
    }
  }

  void _toggleManualMode() {
    if (!_isMappingStarted) {
      _showMessage('Start mapping first', Colors.orange);
      return;
    }

    setState(() {
      _isManualMode = !_isManualMode;
      if (_isManualMode) {
        _isAutoMode = false;
        _statusMessage = 'Manual mode enabled';
      } else {
        _statusMessage = 'Manual mode disabled';
      }
    });
  }

  void _toggleAutoMode() {
    if (!_isMappingStarted) {
      _showMessage('Start mapping first', Colors.orange);
      return;
    }

    setState(() {
      if (_isAutoMode) {
        _stopAutoMode();
      } else {
        _startAutoMode();
      }
    });
  }

  Future<void> _startAutoMode() async {
    try {
      final result = await _mappingService.startAutoMode();
      setState(() {
        _isAutoMode = true;
        _isManualMode = false;
        _statusMessage = result;
      });
      _showMessage(result, Colors.green);
    } catch (e) {
      _showMessage('Error: $e', Colors.red);
    }
  }

  Future<void> _stopAutoMode() async {
    try {
      final result = await _mappingService.stopAutoMode();
      setState(() {
        _isAutoMode = false;
        _statusMessage = result;
      });
      _showMessage(result, Colors.green);
    } catch (e) {
      _showMessage('Error: $e', Colors.red);
    }
  }

  Future<void> _startMovement(String direction) async {
    if (!_isConnected || !_isManualMode || _isHoldingButton) return;

    setState(() {
      _currentDirection = direction;
      _isHoldingButton = true;
    });

    try {
      await _mappingService.sendCommand('${direction}_start');
    } catch (e) {
      _showMessage('Movement error: $e', Colors.red);
    }
  }

  Future<void> _stopMovement() async {
    if (!_isHoldingButton || _currentDirection == null) return;

    final direction = _currentDirection;
    setState(() {
      _isHoldingButton = false;
      _currentDirection = null;
    });

    try {
      await _mappingService.sendCommand('${direction}_stop');
    } catch (e) {
      _showMessage('Stop error: $e', Colors.red);
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
        title: const Text('Robot Mapping Control'),
        centerTitle: true,
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // Status Indicator
            Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: _isConnected ? Colors.green[50] : Colors.red[50],
                border: Border.all(
                  color: _isConnected ? Colors.green : Colors.red,
                ),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Column(
                children: [
                  Row(
                    children: [
                      Container(
                        width: 12,
                        height: 12,
                        decoration: BoxDecoration(
                          color: _isConnected ? Colors.green : Colors.red,
                          shape: BoxShape.circle,
                        ),
                      ),
                      const SizedBox(width: 12),
                      Text(
                        _isConnected ? 'Connected' : 'Disconnected',
                        style: TextStyle(
                          fontWeight: FontWeight.bold,
                          color: _isConnected ? Colors.green : Colors.red,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Text(
                    _statusMessage,
                    style: const TextStyle(fontSize: 12),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 20),

            // Connection Panel
            if (!_isConnected)
              Column(
                children: [
                  TextField(
                    controller: _ipController,
                    decoration: InputDecoration(
                      labelText: 'Robot IP Address',
                      hintText: '192.168.1.100',
                      border: OutlineInputBorder(
                        borderRadius: BorderRadius.circular(8),
                      ),
                    ),
                  ),
                  const SizedBox(height: 12),
                  TextField(
                    controller: _portController,
                    decoration: InputDecoration(
                      labelText: 'Port Number',
                      hintText: '5000',
                      border: OutlineInputBorder(
                        borderRadius: BorderRadius.circular(8),
                      ),
                    ),
                    keyboardType: TextInputType.number,
                  ),
                  const SizedBox(height: 16),
                  ElevatedButton(
                    onPressed: _handleConnect,
                    child: const Padding(
                      padding: EdgeInsets.all(12),
                      child: Text('Activate Robot'),
                    ),
                  ),
                ],
              )
            else
              ElevatedButton(
                onPressed: _handleDisconnect,
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.red,
                ),
                child: const Padding(
                  padding: EdgeInsets.all(12),
                  child: Text(
                    'Disconnect',
                    style: TextStyle(color: Colors.white),
                  ),
                ),
              ),
            const SizedBox(height: 20),

            // Mapping Controls
            if (_isConnected)
              Column(
                children: [
                  const Text(
                    'Mapping Controls',
                    style: TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 12),
                  Row(
                    children: [
                      Expanded(
                        child: ElevatedButton(
                          onPressed: _isMappingStarted ? null : _startMapping,
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.blue,
                            disabledBackgroundColor: Colors.grey,
                          ),
                          child: const Padding(
                            padding: EdgeInsets.all(12),
                            child: Text(
                              'Start Mapping',
                              style: TextStyle(color: Colors.white),
                            ),
                          ),
                        ),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: ElevatedButton(
                          onPressed: _isMappingStarted ? _stopMapping : null,
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.orange,
                            disabledBackgroundColor: Colors.grey,
                          ),
                          child: const Padding(
                            padding: EdgeInsets.all(12),
                            child: Text(
                              'Stop Mapping',
                              style: TextStyle(color: Colors.white),
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  if (!_isMappingStarted)
                    ElevatedButton(
                      onPressed: _isSaving ? null : _saveMapping,
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.green,
                        disabledBackgroundColor: Colors.grey,
                      ),
                      child: Padding(
                        padding: const EdgeInsets.all(12),
                        child: _isSaving
                            ? const SizedBox(
                                height: 20,
                                width: 20,
                                child: CircularProgressIndicator(
                                  strokeWidth: 2,
                                  valueColor: AlwaysStoppedAnimation<Color>(
                                    Colors.white,
                                  ),
                                ),
                              )
                            : const Text(
                                'Save Map',
                                style: TextStyle(color: Colors.white),
                              ),
                      ),
                    ),
                ],
              ),
            const SizedBox(height: 20),

            // Mode Selection
            if (_isConnected && _isMappingStarted)
              Column(
                children: [
                  const Text(
                    'Control Mode',
                    style: TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 12),
                  Row(
                    children: [
                      Expanded(
                        child: ElevatedButton(
                          onPressed: _toggleManualMode,
                          style: ElevatedButton.styleFrom(
                            backgroundColor: _isManualMode
                                ? Colors.purple
                                : Colors.grey,
                          ),
                          child: const Padding(
                            padding: EdgeInsets.all(12),
                            child: Text(
                              'Manual',
                              style: TextStyle(color: Colors.white),
                            ),
                          ),
                        ),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: ElevatedButton(
                          onPressed: _toggleAutoMode,
                          style: ElevatedButton.styleFrom(
                            backgroundColor:
                                _isAutoMode ? Colors.cyan : Colors.grey,
                          ),
                          child: const Padding(
                            padding: EdgeInsets.all(12),
                            child: Text(
                              'Auto',
                              style: TextStyle(color: Colors.white),
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            const SizedBox(height: 20),

            // Manual Control D-Pad
            if (_isConnected && _isMappingStarted && _isManualMode)
              Column(
                children: [
                  const Text(
                    'Manual Control',
                    style: TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 20),
                  Center(
                    child: SizedBox(
                      width: 250,
                      height: 250,
                      child: Stack(
                        children: [
                          Positioned(
                            top: 0,
                            left: 0,
                            right: 0,
                            child: Center(
                              child: _buildDirectionButton(
                                icon: Icons.arrow_upward,
                                direction: 'forward',
                              ),
                            ),
                          ),
                          Positioned(
                            bottom: 0,
                            left: 0,
                            right: 0,
                            child: Center(
                              child: _buildDirectionButton(
                                icon: Icons.arrow_downward,
                                direction: 'backward',
                              ),
                            ),
                          ),
                          Positioned(
                            left: 0,
                            top: 0,
                            bottom: 0,
                            child: Center(
                              child: _buildDirectionButton(
                                icon: Icons.arrow_back,
                                direction: 'left',
                              ),
                            ),
                          ),
                          Positioned(
                            right: 0,
                            top: 0,
                            bottom: 0,
                            child: Center(
                              child: _buildDirectionButton(
                                icon: Icons.arrow_forward,
                                direction: 'right',
                              ),
                            ),
                          ),
                          Center(
                            child: GestureDetector(
                              onTap: () =>
                                  _mappingService.stopMovement(),
                              child: Container(
                                width: 60,
                                height: 60,
                                decoration: BoxDecoration(
                                  color: Colors.red,
                                  shape: BoxShape.circle,
                                ),
                                child: const Icon(
                                  Icons.stop,
                                  color: Colors.white,
                                  size: 30,
                                ),
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildDirectionButton({
    required IconData icon,
    required String direction,
  }) {
    return GestureDetector(
      onTapDown: (_) => _startMovement(direction),
      onTapUp: (_) => _stopMovement(),
      onTapCancel: () => _stopMovement(),
      child: Container(
        width: 60,
        height: 60,
        decoration: BoxDecoration(
          color: Colors.blue,
          shape: BoxShape.circle,
          boxShadow: [
            BoxShadow(
              color: Colors.blue.withValues(alpha: 0.5),
              blurRadius: 8,
            ),
          ],
        ),
        child: Icon(
          icon,
          color: Colors.white,
          size: 28,
        ),
      ),
    );
  }
}
