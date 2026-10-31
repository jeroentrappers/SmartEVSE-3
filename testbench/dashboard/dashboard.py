"""Testbench dashboard — Flask sidecar for the SmartEVSE firmware harness.

Spawns the device binary (Python mock today, C++ native binary later),
streams its stdout events to the browser over SSE, and forwards UI
commands onto its stdin.
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
import queue
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque

from flask import Flask, Response, jsonify, render_template, request

HERE = Path(__file__).resolve().parent

# -- device subprocess --------------------------------------------------


class Device:
    """Spawn + bridge the backend process."""

    def __init__(self, argv: list[str]) -> None:
        self.argv = argv
        self.proc: subprocess.Popen | None = None
        self.recent: Deque[dict] = deque(maxlen=500)
        self.subscribers: list[queue.Queue] = []
        self._sub_lock = threading.Lock()
        self._recorded: list[dict] = []
        self._recording = False

    def start(self) -> None:
        self.proc = subprocess.Popen(  # noqa: S603
            self.argv,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._read_stdout, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.stdin and self.proc.stdin.close()
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()

    def send(self, cmd: dict) -> None:
        assert self.proc and self.proc.stdin
        self.proc.stdin.write(json.dumps(cmd) + "\n")
        self.proc.stdin.flush()
        if self._recording:
            self._recorded.append({"at_ms": None, "cmd": cmd})

    def _broadcast(self, event: dict) -> None:
        self.recent.append(event)
        with self._sub_lock:
            dead = []
            for q in self.subscribers:
                try:
                    q.put_nowait(event)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                self.subscribers.remove(q)

    def _read_stdout(self) -> None:
        assert self.proc and self.proc.stdout
        for line in self.proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                event = {"t": 0, "kind": "log", "level": "error", "msg": f"bad event: {line}"}
            self._broadcast(event)

    def _read_stderr(self) -> None:
        assert self.proc and self.proc.stderr
        for line in self.proc.stderr:
            line = line.rstrip()
            if line:
                self._broadcast({"t": 0, "kind": "log", "level": "stderr", "msg": line})

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=1000)
        with self._sub_lock:
            self.subscribers.append(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._sub_lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    # -- recording -----------------------------------------------------

    def start_recording(self) -> None:
        self._recorded = []
        self._recording = True

    def stop_recording(self) -> list[dict]:
        self._recording = False
        return list(self._recorded)


# -- flask app ----------------------------------------------------------


def make_app(device: Device) -> Flask:
    app = Flask(
        __name__,
        template_folder=str(HERE / "templates"),
        static_folder=str(HERE / "static"),
    )

    @app.get("/")
    def home():
        return render_template("index.html")

    @app.get("/events")
    def events():
        def gen():
            q = device.subscribe()
            # Replay recent events to a new subscriber so the UI hydrates.
            for ev in list(device.recent):
                yield f"data: {json.dumps(ev)}\n\n"
            try:
                while True:
                    try:
                        ev = q.get(timeout=15)
                        yield f"data: {json.dumps(ev)}\n\n"
                    except queue.Empty:
                        yield ": keepalive\n\n"
            finally:
                device.unsubscribe(q)

        return Response(gen(), mimetype="text/event-stream")

    @app.post("/command")
    def command():
        cmd = request.get_json(force=True)
        device.send(cmd)
        return ("", 204)

    @app.post("/recording/start")
    def rec_start():
        device.start_recording()
        return ("", 204)

    @app.post("/recording/stop")
    def rec_stop():
        recorded = device.stop_recording()
        return jsonify({"commands": recorded})

    @app.get("/healthz")
    def healthz():
        ok = device.proc is not None and device.proc.poll() is None
        return jsonify({"ok": ok})

    return app


# -- main ---------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--backend",
        default=str(HERE / "mock_device.py"),
        help="Path to the device backend (mock or C++ native binary).",
    )
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", 7777)))
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args()

    if args.backend.endswith(".py"):
        argv = [sys.executable, "-u", args.backend]
    else:
        argv = [args.backend]

    device = Device(argv)
    device.start()
    atexit.register(device.stop)

    # Give the backend a moment to emit its boot/snapshot events.
    time.sleep(0.1)

    app = make_app(device)
    print(f"Dashboard on http://{args.host}:{args.port}", file=sys.stderr)
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
