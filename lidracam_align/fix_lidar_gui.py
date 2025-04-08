#!/usr/bin/env python3
"""
LiDAR-Camera Alignment GUI Launcher
This script sets the correct Qt environment variables before launching the GUI.
"""

import os
import sys
import subprocess

# Set Qt environment variables
os.environ["QT_QPA_PLATFORM"] = "xcb"  # Force XCB platform
os.environ["QT_DEBUG_PLUGINS"] = "0"   # Disable verbose debugging

# Clear any conflicting plugin paths
if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ:
    del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]

# Point to PyQt5's plugins directly
pyqt_path = None
try:
    import PyQt5
    pyqt_dir = os.path.dirname(PyQt5.__file__)
    plugin_path = os.path.join(pyqt_dir, "Qt5", "plugins")
    if os.path.exists(plugin_path):
        os.environ["QT_PLUGIN_PATH"] = plugin_path
        print(f"Using Qt plugins from: {plugin_path}")
except ImportError:
    print("PyQt5 not found.")
    sys.exit(1)

# Launch the GUI
print("Launching LiDAR-Camera Alignment GUI...")
gui_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lidar_camera_gui.py")
result = subprocess.run([sys.executable, gui_script])
sys.exit(result.returncode)