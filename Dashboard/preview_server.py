#!/usr/bin/env python3
"""Serve the VORTEX rover dashboard locally without ESP32 hardware."""
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlsplit, unquote
import os

ROOT = Path(__file__).resolve().parent
PORT = int(os.environ.get("VORTEX_PREVIEW_PORT", "8765"))

class Handler(BaseHTTPRequestHandler):
    def _send_file(self, path: Path, content_type: str):
        if not path.is_file():
            self.send_error(404)
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = unquote(urlsplit(self.path).path)
        if route in ("/", "/index.html", "/dashboard_preview"):
            return self._send_file(ROOT / "dashboard_preview.html", "text/html; charset=utf-8")
        if route in ("/settings", "/camera_settings"):
            return self._send_file(ROOT / "camera_settings.html", "text/html; charset=utf-8")
        if route == "/dashboard_preview.html":
            return self._send_file(ROOT / "dashboard_preview.html", "text/html; charset=utf-8")
        if route == "/camera_settings.html":
            return self._send_file(ROOT / "camera_settings.html", "text/html; charset=utf-8")
        if route == "/dashboard_preview.png":
            return self._send_file(ROOT / "dashboard_preview.png", "image/png")
        if route.startswith("/previews/"):
            rel = route.removeprefix("/previews/")
            target = (ROOT / "previews" / rel).resolve()
            if ROOT.resolve() in target.parents and target.is_file():
                return self._send_file(target, "image/png")
        self.send_error(404)

    def log_message(self, fmt, *args):
        # Keep the launcher console quiet except for errors.
        if args and len(args) >= 1 and str(args[0]) not in ("200",):
            super().log_message(fmt, *args)

if __name__ == "__main__":
    print(f"VORTEX local dashboard: http://127.0.0.1:{PORT}/")
    print(f"Camera settings:       http://127.0.0.1:{PORT}/settings")
    print("Local preview mode: camera/video/UART actions are simulated.")
    print("Press Ctrl+C to stop.")
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
