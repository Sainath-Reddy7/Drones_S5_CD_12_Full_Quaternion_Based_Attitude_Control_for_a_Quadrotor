"""stdlib-only ground station server: Server-Sent Events push telemetry
snapshots from the bus at ~50 Hz; POST endpoints accept manual stick input
and commands (arm/disarm/mode/gains). Deliberately avoids `websockets` /
Flask / etc so `pip install -e .` stays exactly what it was for the paper
reproduction -- zero new dependencies.

The physics runs in its own background thread via Scheduler.run_for(...,
realtime=True); this server thread only ever reads bus snapshots (already
thread-safe, see bus.py) and writes sim.manual_input / calls sim.commander
methods, which the physics thread reads at the start of each tick. Plain
attribute writes are fine here -- we only need "eventually consistent," not
exact-tick ordering, the same way a real RC receiver feeds a flight
controller asynchronously.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

from ..control.manual import ManualInput
from ..modes import FlightMode
from ..sim import Simulation

STATIC_DIR = Path(__file__).parent / "static"
STREAM_HZ = 50.0

_MIME = {".html": "text/html", ".js": "application/javascript", ".css": "text/css"}


def _snapshot(sim: Simulation) -> dict:
    status = sim.bus.get_value("vehicle_status")
    setpoint = sim.bus.get_value("vehicle_attitude_setpoint")
    torque = sim.bus.get_value("vehicle_torque_setpoint")
    actuators = sim.bus.get_value("actuator_outputs")
    return {
        "t": round(sim.scheduler.sim_time, 4),
        "q": [round(c, 5) for c in sim.plant.quaternion],
        "omega": [round(c, 4) for c in sim.plant.omega],
        "pos": [round(c, 4) for c in sim.plant.position],
        "vel": [round(c, 4) for c in sim.plant.velocity],
        "altitude": round(sim.plant.altitude, 4),
        "armed": status.armed if status else False,
        "mode": status.mode if status else "?",
        "failsafe": status.failsafe_reason if status else None,
        "q_ref": [round(c, 5) for c in setpoint.q_ref] if setpoint else [1, 0, 0, 0],
        "tau": [round(c, 4) for c in torque.tau] if torque else [0, 0, 0],
        "torque_limit": sim.gains.torque_limit,
        "rotor_thrust": [round(c, 4) for c in actuators.rotor_thrust] if actuators else [0, 0, 0, 0],
        "rotor_thrust_max": round(sim.vehicle.rotor_thrust_max, 4),
        "target_altitude": round(sim.target_altitude, 3),
        "control_hz": sim.scheduler.base_hz,
        "gains": {"Pq": sim.attitude_ctrl.gains.Pq, "Pw": sim.attitude_ctrl.gains.Pw},
    }


def _make_handler(sim: Simulation):
    class Handler(BaseHTTPRequestHandler):
        server_version = "pysitl-gcs/0.1"

        def log_message(self, fmt, *args):  # quiet: default logs every request to stderr
            pass

        def _send_json(self, obj: dict, status: int = 200) -> None:
            body = json.dumps(obj).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path == "/" or self.path == "/index.html":
                self._serve_file(STATIC_DIR / "index.html", "text/html")
            elif self.path == "/events":
                self._stream_events()
            elif self.path.startswith("/static/"):
                self._serve_file(STATIC_DIR / self.path[len("/static/") :], None)
            else:
                self.send_error(404)

        def _serve_file(self, path: Path, mime: Optional[str]) -> None:
            if not path.is_file():
                self.send_error(404)
                return
            mime = mime or _MIME.get(path.suffix, "application/octet-stream")
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream_events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            period = 1.0 / STREAM_HZ
            try:
                while True:
                    payload = json.dumps(_snapshot(sim))
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self) -> None:
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                body = json.loads(raw)
            except json.JSONDecodeError:
                self._send_json({"error": "invalid JSON"}, status=400)
                return

            if self.path == "/stick":
                sim.manual_input = ManualInput(
                    roll=float(body.get("roll", 0.0)),
                    pitch=float(body.get("pitch", 0.0)),
                    yaw_rate=float(body.get("yaw_rate", 0.0)),
                    throttle=float(body.get("throttle", 0.0)),
                ).clamped()
                self._send_json({"ok": True})

            elif self.path == "/command":
                action = body.get("action")
                if action == "arm":
                    ok = sim.commander.try_arm(sim.plant.altitude)
                    self._send_json({"ok": ok, "checks": [c.__dict__ for c in sim.commander.last_checks]})
                elif action == "disarm":
                    sim.commander.disarm(reason="manual disarm")
                    self._send_json({"ok": True})
                elif action == "set_mode":
                    try:
                        sim.set_flight_mode(FlightMode(body["mode"]))
                        self._send_json({"ok": True})
                    except (KeyError, ValueError):
                        self._send_json({"ok": False, "error": "unknown mode"}, status=400)
                elif action == "set_gains":
                    if "Pq" in body:
                        sim.attitude_ctrl.gains.Pq = float(body["Pq"])
                    if "Pw" in body:
                        sim.attitude_ctrl.gains.Pw = float(body["Pw"])
                    self._send_json({"ok": True})
                elif action == "reset":
                    sim.reset()
                    self._send_json({"ok": True})
                else:
                    self._send_json({"ok": False, "error": "unknown action"}, status=400)
            else:
                self.send_error(404)

    return Handler


def run_server(sim: Simulation, host: str = "127.0.0.1", port: int = 8765) -> ThreadingHTTPServer:
    """Starts the HTTP server on a background daemon thread and returns the
    server object (call .shutdown() to stop it)."""
    httpd = ThreadingHTTPServer((host, port), _make_handler(sim))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd
