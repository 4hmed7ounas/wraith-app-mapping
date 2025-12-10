import 'package:flutter/material.dart';
import 'mapping_screen.dart';
import 'maps_screen.dart';
import 'map_detail_screen.dart';
import 'streaming_screen.dart';

class AppShell extends StatefulWidget {
  const AppShell({super.key});

  @override
  State<AppShell> createState() => _AppShellState();
}

class _AppShellState extends State<AppShell> {
  int _selectedIndex = 0;

  late List<Widget> _pages;

  @override
  void initState() {
    super.initState();
    _pages = [
      const MappingScreen(),
      const MapsScreen(),
      const MapDetailScreen(),
      const CameraStreamScreen(),
    ];
  }

  void _onItemTapped(int index) {
    setState(() {
      _selectedIndex = index;

      // Recreate the streaming screen whenever its tab is selected
      // so that initState runs again and the Flask streaming endpoint
      // is hit each time the user navigates to this screen.
      if (index == 3) {
        _pages[3] = CameraStreamScreen(key: UniqueKey());
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: _pages[_selectedIndex],
      bottomNavigationBar: BottomNavigationBar(
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(
            icon: Icon(Icons.videogame_asset),
            label: 'Control',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.map),
            label: 'Maps',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.navigation),
            label: 'Navigate',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.videocam),
            label: 'Stream',
          ),
        ],
        currentIndex: _selectedIndex,
        selectedItemColor: Colors.blue,
        unselectedItemColor: Colors.grey,
        onTap: _onItemTapped,
      ),
    );
  }
}
