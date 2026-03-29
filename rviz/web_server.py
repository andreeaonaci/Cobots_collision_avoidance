#!/usr/bin/env python3
"""
web_server.py  —  single-port demo server  (port 8000)
=======================================================
• GET /       → serves ur5e_simulator.html
• GET /*      → static files from the rviz/ directory
• WS  /ws     → HTTP Upgrade → proxies transparently to rosbridge:9090

Launched automatically by simulation.launch.py.
Can also be run standalone for development:
    python3 rviz/web_server.py
"""

import asyncio
import os

import tornado.ioloop
import tornado.web
import tornado.websocket

# ── Configuration (all overridable via environment) ───────────────
RVIZ_DIR      = os.environ.get(
    "RVIZ_DIR",
    os.path.dirname(os.path.realpath(__file__)),   # directory this file lives in
)
ROSBRIDGE_URL = os.environ.get("ROSBRIDGE_URL", "ws://localhost:9090")
PORT          = int(os.environ.get("WEB_PORT", 8000))

# How many times to retry connecting to rosbridge (×0.5 s each = 15 s total)
_ROSBRIDGE_RETRIES = 30


class WSProxyHandler(tornado.websocket.WebSocketHandler):
    """
    Transparent WebSocket proxy.

        Browser  ←──ws──→  WSProxyHandler  ←──ws──→  rosbridge:9090

    The browser sends its HTTP Upgrade request to port 8000 (/ws).
    Tornado handles the upgrade; then we open a second WS connection
    to rosbridge and forward frames in both directions.

    We retry the rosbridge connection while the browser WS is alive,
    so the page works even when rosbridge is still starting up.
    """

    def check_origin(self, origin: str) -> bool:
        return True   # local demo – accept any origin

    async def open(self) -> None:
        self._ros_ws = None
        for attempt in range(_ROSBRIDGE_RETRIES):
            try:
                self._ros_ws = await tornado.websocket.websocket_connect(
                    ROSBRIDGE_URL,
                    on_message_callback=self._from_ros,
                )
                return   # connected ✓
            except Exception:
                await asyncio.sleep(0.5)
        # Could not reach rosbridge after 15 s – tell the browser
        self.close(1011, "rosbridge unreachable – is simulation running?")

    def _from_ros(self, msg: object) -> None:
        """rosbridge → browser"""
        if msg is None:          # rosbridge closed
            self.close()
            return
        try:
            self.write_message(msg)
        except tornado.websocket.WebSocketClosedError:
            pass

    def on_message(self, msg: object) -> None:
        """browser → rosbridge"""
        if self._ros_ws:
            self._ros_ws.write_message(msg)

    def on_close(self) -> None:
        if self._ros_ws:
            self._ros_ws.close()
            self._ros_ws = None


def make_app() -> tornado.web.Application:
    return tornado.web.Application(
        [
            (r"/ws",   WSProxyHandler),
            (r"/(.*)", tornado.web.StaticFileHandler, {
                "path":             RVIZ_DIR,
                "default_filename": "ur5e_simulator.html",
            }),
        ],
        # Disable tornado's own debug logging that clutters the terminal
        debug=False,
    )


if __name__ == "__main__":
    app = make_app()
    app.listen(PORT)
    print(
        f"[web_server] Serving  http://localhost:{PORT}/\n"
        f"[web_server] ROS WS   ws://localhost:{PORT}/ws  →  {ROSBRIDGE_URL}",
        flush=True,
    )
    tornado.ioloop.IOLoop.current().start()
