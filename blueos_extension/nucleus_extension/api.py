"""
REST API and web page.

GET  /api/status                 connection state, counters, live raw/rotated values, settings
GET  /api/settings               saved settings
POST /api/settings               partial update, e.g. {"message_type": "SPEED_ESTIMATE"}
POST /api/mount                  {"mount_pitch_deg": 30, "mount_roll_deg": 0}
POST /api/connect | /api/disconnect | /api/start | /api/stop
"""

import os

from flask import Flask, jsonify, request

from nucleus_extension.settings import validate_update

STATIC_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


def create_app(link) -> Flask:
    app = Flask(__name__, static_folder=STATIC_FOLDER, static_url_path="/static")

    def apply(update):
        try:
            cleaned = validate_update(update)
        except ValueError as error:
            return jsonify({"error": str(error)}), 400
        link.update_settings(cleaned)
        return jsonify(link.settings)

    @app.route("/")
    def index():
        return app.send_static_file("index.html")

    @app.route("/register_service")
    def register_service():
        return app.send_static_file("service.json")

    @app.get("/api/status")
    def status():
        return jsonify(link.get_status())

    @app.get("/api/settings")
    def get_settings():
        return jsonify(link.settings)

    @app.post("/api/settings")
    def post_settings():
        return apply(request.get_json(silent=True))

    @app.post("/api/mount")
    def post_mount():
        body = request.get_json(silent=True)
        if not isinstance(body, dict) or not set(body) <= {"mount_pitch_deg", "mount_roll_deg"} or not body:
            return jsonify({"error": 'expected {"mount_pitch_deg": <deg>, "mount_roll_deg": <deg>}'}), 400
        return apply(body)

    def action(func):
        ok = func()
        return jsonify({"ok": ok, "status": link.status_text}), (200 if ok else 409)

    @app.post("/api/connect")
    def connect():
        return action(link.connect)

    @app.post("/api/disconnect")
    def disconnect():
        return action(link.disconnect)

    @app.post("/api/start")
    def start():
        return action(link.start)

    @app.post("/api/stop")
    def stop():
        return action(link.stop)

    return app
