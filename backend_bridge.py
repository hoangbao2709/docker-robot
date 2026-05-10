import json
import os
import threading
import urllib.error
import urllib.request


def _backend_base_url():
    raw = os.environ.get("DOGZILLA_BACKEND_URL") or os.environ.get("BACKEND_URL")
    return (raw or "http://127.0.0.1:8000").rstrip("/")


def _robot_id():
    return os.environ.get("DOGZILLA_ROBOT_ID") or os.environ.get("ROBOT_ID") or "robot-a"


def _timeout_sec():
    try:
        return float(os.environ.get("DOGZILLA_BACKEND_NOTIFY_TIMEOUT", "0.8"))
    except ValueError:
        return 0.8


def notify_backend_nav_action(action, x, y, yaw, u=None, v=None, source="docker-ui"):
    payload = {
        "action": action,
        "x": x,
        "y": y,
        "yaw": yaw,
        "u": u,
        "v": v,
        "source": source,
    }

    def _post():
        url = f"{_backend_base_url()}/control/api/robots/{_robot_id()}/patrol/ui-action/"
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=_timeout_sec()):
                pass
        except (OSError, urllib.error.URLError, urllib.error.HTTPError) as e:
            print(f"[backend_bridge] notify failed: {e}")

    threading.Thread(target=_post, daemon=True).start()
