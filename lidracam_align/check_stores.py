#!/usr/bin/env python3
"""
Check available typestores in rosbags.
"""

from rosbags.typesys import Stores

# Print all available stores
print("Available stores in rosbags.typesys.Stores:")
for store in dir(Stores):
    if not store.startswith('_'):
        print(f"  - {store}")
