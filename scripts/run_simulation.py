#!/usr/bin/env python3
"""
Autonomous Self-Driving Car - Simulation & Web Cockpit Launcher.
Runs the complete physics simulation, perception pipeline, decision state machine,
and serves the real-time interactive cockpit dashboard.

Usage:
    python3 scripts/run_simulation.py [--port 8080]
"""

import argparse
import os
import sys
import time
import webbrowser

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.config import DEFAULT_CONFIG
from src.dashboard.web_server import DashboardServer
from src.simulation.simulator import AutonomousCarSimulator


def main():
    parser = argparse.ArgumentParser(description="Autonomous Self-Driving Car Simulator")
    parser.add_argument("--port", type=int, default=8080, help="Web dashboard port (default: 8080)")
    parser.add_argument("--no-browser", action="store_true", help="Do not auto-open web browser")
    args = parser.parse_args()

    print("=" * 65)
    print("🚗 AI-POWERED AUTONOMOUS SELF-DRIVING CAR SIMULATOR")
    print("Author: Nivesh Kumar Meena")
    print("=" * 65)

    simulator = AutonomousCarSimulator(DEFAULT_CONFIG)
    server = DashboardServer(simulator, port=args.port)
    server.start(run_sim_loop=True)

    url = f"http://localhost:{args.port}"
    print(f"\n[Ready] Interactive Dashboard active at: {url}")
    print("[Info]  Open this URL in your browser to view the live cockpit,")
    print("        monitor telemetry, and test autonomous driving!\n")
    print("Press Ctrl+C to terminate the simulation.\n")

    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Shutdown] Stopping simulation and server...")
        server.stop()
        print("[Shutdown] Completed cleanly. Goodbye!")


if __name__ == "__main__":
    main()
