#!/usr/bin/env python3
"""
web_server.py
=============
Single-port server (default: 8000) that:
  • Serves the rviz/ directory over plain HTTP  (GET /)
  • Upgrades WebSocket connections via HTTP Upgrade → WebSocket
    and proxies them transparently to rosbridge on ws://localhost:9090

Run:
  python3 web_server.py              # port 8000, rosbridge on 9090
  WEB_PORT=8080 python3 web_server.py

Or launched automatically by simulation.launch.py.
"""

import os
import sys
import tornado.web
import tornado.websocket
import tornado.ioloop

RVIZ_DIR      = os.path.join(os.path.dirname(__file__), '..', '..', 'rviz')
ROSBRIDGE_URL = os.environ.get('ROSBRIDGE_URL', 'ws://localhost:9090')
PORT          = int(os.environ.get('WEB_PORT', 8000))


class WSProxyHandler(tornado.websocket.WebSocketHandler):
    """
    Transparent WebSocket proxy.
    Browser  <──ws──>  this handler  <──ws──>  rosbridge:9090
    The HTTP Upgrade: websocket handshake is handled by tornado.
    """

    def check_origin(self, origin):
        return True  # allow requests from any origin (local demo)

    async def open(self):
        self._ros_ws = None
        try:
            self._ros_ws = await tornado.websocket.websocket_connect(
                ROSBRIDGE_URL,
                on_message_callback=self._from_ros,
            )
        except Exception as exc:
            self.close(1011, f'rosbridge unreachable: {exc}')

    def _from_ros(self, msg):
        """Forward a message from rosbridge → browser."""
        if msg is None:          # rosbridge closed
            self.close()
            return
        try:
            self.write_message(msg)
        except tornado.websocket.WebSocketClosedError:
            pass

    def on_message(self, msg):
        """Forward a message from browser → rosbridge."""
        if self._ros_ws:
            self._ros_ws.write_message(msg)

    def on_close(self):
        if self._ros_ws:
            self._ros_ws.close()


def make_app():
    return tornado.web.Application([
        (r'/ws',   WSProxyHandler),
        (r'/(.*)', tornado.web.StaticFileHandler, {
            'path':             RVIZ_DIR,
            'default_filename': 'ur5e_simulator.html',
        }),
    ])


if __name__ == '__main__':
    app = make_app()
    app.listen(PORT)
    print(f'[web_server] http://localhost:{PORT}  —  WebSocket: ws://localhost:{PORT}/ws → {ROSBRIDGE_URL}',
          flush=True)
    tornado.ioloop.IOLoop.current().start()
