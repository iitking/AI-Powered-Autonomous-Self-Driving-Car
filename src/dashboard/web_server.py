"""
Lightweight Web Dashboard Server for Autonomous Vehicle Cockpit.
Built using standard library http.server (no external dependencies required).
Provides real-time JSON telemetry endpoints and serves interactive HTML5 canvas UI.
"""

import json
import os
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from typing import Optional
from urllib.parse import parse_qs, urlparse

from src.decision.state_machine import DrivingMode
from src.simulation.simulator import AutonomousCarSimulator


class DashboardRequestHandler(SimpleHTTPRequestHandler):
    """Handles HTTP requests for static assets and simulation REST API."""

    simulator: Optional[AutonomousCarSimulator] = None
    static_dir: str = os.path.join(os.path.dirname(__file__), "static")

    def translate_path(self, path):
        """Maps root or /static paths to dashboard static directory."""
        parsed = urlparse(path).path
        if parsed == "/" or parsed == "/index.html":
            return os.path.join(self.static_dir, "index.html")
        elif parsed.startswith("/static/"):
            rel_path = parsed[len("/static/"):]
            return os.path.join(self.static_dir, rel_path)
        return super().translate_path(path)

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path == "/api/state":
            if self.simulator is None:
                self.send_error(503, "Simulator not initialized")
                return

            state = self.simulator.get_full_state()
            payload = json.dumps(state).encode("utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            self.wfile.write(payload)

        elif parsed.path == "/" or parsed.path.startswith("/static/"):
            super().do_GET()
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        parsed = urlparse(self.path)

        if parsed.path == "/api/control":
            if self.simulator is None:
                self.send_error(503, "Simulator not initialized")
                return

            content_length = int(self.headers.get("Content-Length", 0))
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode("utf-8"))
            except Exception:
                data = {}

            action = data.get("action")

            if action == "set_mode":
                mode_str = data.get("mode", "AUTONOMOUS").upper()
                new_mode = DrivingMode.MANUAL if mode_str == "MANUAL" else DrivingMode.AUTONOMOUS
                self.simulator.set_driving_mode(new_mode)

            elif action == "manual_drive":
                steer = float(data.get("steer_deg", 0.0))
                throttle = float(data.get("throttle", 0.0))
                self.simulator.set_manual_inputs(steer, throttle)

            elif action == "spawn_obstacle":
                # Spawn in front of car's current position on track
                car_pos = self.simulator.vehicle.state
                # 1.5m ahead along car yaw
                import math
                ox = car_pos.x + 1.5 * math.cos(car_pos.yaw)
                oy = car_pos.y + 1.5 * math.sin(car_pos.yaw)
                self.simulator.add_obstacle(ox, oy, data.get("class", "box"))

            elif action == "clear_obstacles":
                self.simulator.clear_obstacles()

            elif action == "toggle_traffic_light":
                self.simulator.toggle_traffic_light()

            elif action == "toggle_traffic_car":
                self.simulator.toggle_traffic_car()

            elif action == "stall_traffic_car":
                self.simulator.stall_traffic_car()

            elif action == "trigger_pedestrian":
                self.simulator.trigger_pedestrian()

            elif action == "clear_all_traffic":
                self.simulator.clear_all_traffic()

            elif action == "tune_pid":
                kp = data.get("kp_steer")
                ki = data.get("ki_steer")
                kd = data.get("kd_steer")
                self.simulator.steering_ctrl.pid.set_gains(kp=kp, ki=ki, kd=kd)

            elif action == "reset":
                self.simulator.reset()

            response = json.dumps({"status": "ok", "action": action}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(response)
        else:
            self.send_error(404, "Unknown action")

    def log_message(self, format, *args):
        # Suppress routine log messages to keep terminal clean
        pass


class DashboardServer:
    """HTTP Server runner for the interactive web dashboard."""

    def __init__(self, simulator: AutonomousCarSimulator, port: int = 8080):
        self.simulator = simulator
        self.port = port
        self.httpd = None
        self.server_thread = None
        self.sim_thread = None
        self.running = False

    def _sim_loop(self):
        """Runs the continuous simulation loop at 20 Hz."""
        dt = 0.05
        while self.running:
            start_t = time.time()
            self.simulator.step(dt)
            elapsed = time.time() - start_t
            sleep_time = max(0.001, dt - elapsed)
            time.sleep(sleep_time)

    def start(self, run_sim_loop: bool = True):
        """Starts web server and simulation background worker."""
        DashboardRequestHandler.simulator = self.simulator
        self.httpd = HTTPServer(("0.0.0.0", self.port), DashboardRequestHandler)
        self.running = True

        if run_sim_loop:
            self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
            self.sim_thread.start()

        self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.server_thread.start()
        print(f"[DashboardServer] 🚗 Autonomous Vehicle Cockpit running at: http://localhost:{self.port}")

    def stop(self):
        self.running = False
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()
