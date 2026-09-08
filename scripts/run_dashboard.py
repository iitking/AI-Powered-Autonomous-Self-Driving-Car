#!/usr/bin/env python3
"""
Standalone Dashboard Server Launcher.
"""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from scripts.run_simulation import main

if __name__ == "__main__":
    main()
