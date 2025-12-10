import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/robot_url_provider.dart';

class CameraStreamScreen extends StatefulWidget {
  const CameraStreamScreen({super.key});

  @override
  State<CameraStreamScreen> createState() => _CameraStreamScreenState();
}

class _CameraStreamScreenState extends State<CameraStreamScreen> {
  String? _streamUrl;
  bool _isLoading = true;
  String? _error;

  @override
  void initState() {
    super.initState();
    _initializeStream();
  }

  void _initializeStream() {
    try {
      final robotUrl =
          Provider.of<RobotUrlProvider>(context, listen: false).robotUrl;
      setState(() {
        _streamUrl = '$robotUrl/camera/stream';
        _isLoading = false;
      });
    } catch (e) {
      setState(() {
        _error = e.toString();
        _isLoading = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Live Camera Feed'),
        centerTitle: true,
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: () {
              setState(() {
                _isLoading = true;
                _error = null;
              });
              _initializeStream();
            },
          ),
        ],
      ),
      body: _isLoading
          ? const Center(child: CircularProgressIndicator())
          : _error != null
              ? Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      const Icon(Icons.error_outline,
                          size: 64, color: Colors.red),
                      const SizedBox(height: 16),
                      Text('Error: $_error'),
                      const SizedBox(height: 16),
                      ElevatedButton(
                        onPressed: _initializeStream,
                        child: const Text('Retry'),
                      ),
                    ],
                  ),
                )
              : Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Expanded(
                        child: InteractiveViewer(
                          minScale: 0.5,
                          maxScale: 4.0,
                          child: Image.network(
                            _streamUrl!,
                            fit: BoxFit.contain,
                            loadingBuilder:
                                (context, child, loadingProgress) {
                              if (loadingProgress == null) return child;
                              return const Center(
                                child: CircularProgressIndicator(),
                              );
                            },
                            errorBuilder:
                                (context, error, stackTrace) {
                              return Column(
                                mainAxisAlignment: MainAxisAlignment.center,
                                children: [
                                  const Icon(Icons.videocam_off, size: 64),
                                  const SizedBox(height: 16),
                                  const Text('Camera stream unavailable'),
                                  const SizedBox(height: 8),
                                  Text(
                                    error.toString(),
                                    style:
                                        const TextStyle(fontSize: 12),
                                  ),
                                ],
                              );
                            },
                          ),
                        ),
                      ),
                      Container(
                        padding: const EdgeInsets.all(16),
                        color: Colors.black87,
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            const Icon(Icons.videocam,
                                color: Colors.white),
                            const SizedBox(width: 8),
                            Text(
                              'Streaming: $_streamUrl',
                              style: const TextStyle(
                                  color: Colors.white70, fontSize: 12),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
    );
  }
}
