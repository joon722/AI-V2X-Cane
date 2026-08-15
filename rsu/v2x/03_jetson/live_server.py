#!/usr/bin/env python3
"""화면이 최신 판정을 가져가는 창구. step8 안에서 데몬 스레드로 돈다.

    GET /            web/drive.html
    GET /api/state   live_state.LiveState 의 최신 스냅샷 (JSON)

현장에서 폰이 젯슨에 직접 붙는 용도다. 그래서 모든 인터페이스에 바인딩하고
인증을 두지 않는다 - 같은 핫스팟 안에서만 닿는다는 전제다. 인터넷에 노출되는
자리로 옮길 일이 생기면 그때는 이 전제부터 다시 봐야 한다.

내주는 경로를 파일 이름으로 고정했다(화이트리스트). 요청 경로를 파일 경로로
이어 붙이지 않으므로 `../`류가 성립하지 않는다.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from live_state import LiveState

DEFAULT_PORT = 8080
DEFAULT_WEB_DIR = Path(__file__).resolve().parent / "web"
PAGE_FILE = "drive.html"


def _silent(self, fmt, *args):
    """요청 로그를 버린다.

    화면이 4Hz로 폴링하므로 기본 로깅을 두면 journalctl에 초당 네 줄이 쌓여
    진짜 경보 로그가 묻힌다.
    """


def _make_handler(live_state, web_dir, clock):
    class LiveHandler(BaseHTTPRequestHandler):
        # 브라우저가 캐시 무효화용 쿼리를 붙이므로 경로만 떼어 본다.
        def _route(self):
            return self.path.split("?", 1)[0]

        def do_GET(self):
            route = self._route()
            if route == "/api/state":
                self._send_json(live_state.snapshot(clock()))
            elif route in ("/", "/" + PAGE_FILE):
                self._send_page()
            else:
                self.send_error(404)

        def _send_json(self, payload):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            # 캐시가 걸리면 멈춘 화면이 실시간인 척한다.
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_page(self):
            try:
                body = (web_dir / PAGE_FILE).read_bytes()
            except OSError:
                # 화면 파일을 안 올렸어도 /api/state 는 계속 살아 있어야 한다.
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        log_message = _silent

    return LiveHandler


def start_or_none(port, web_dir=None, on_message=None):
    """화면 서버를 띄우고 LiveState를 돌려준다. 못 띄우면 None.

    포트가 이미 쓰이고 있거나 바인딩이 막혀도 경보는 계속돼야 한다. 그래서
    실패를 알리기만 하고 삼킨다 - 호출자는 None을 받아 화면 없이 그냥 돈다.
    port가 0이면 화면을 끈다는 뜻이다.
    """
    say = on_message or (lambda text: None)
    if not port:
        return None
    state = LiveState()
    try:
        start(state, port=port, web_dir=web_dir)
    except OSError as exc:
        say(f"[WARN] live_view_disabled port={port} error={exc}")
        return None
    say(f"[INFO] live_view port={port}")
    return state


def start(live_state, port=DEFAULT_PORT, web_dir=None, clock=time.time):
    """서버를 데몬 스레드로 띄우고 httpd를 반환한다.

    데몬으로 두는 이유는 step8이 끝날 때 이 스레드가 프로세스를 붙잡으면
    안 되기 때문이다. 실제 포트는 httpd.server_address[1]에 있다 - port=0을
    주면 OS가 빈 포트를 고른다.
    """
    directory = Path(web_dir) if web_dir is not None else DEFAULT_WEB_DIR
    httpd = ThreadingHTTPServer(("", port), _make_handler(live_state, directory, clock))
    thread = threading.Thread(
        target=httpd.serve_forever, daemon=True, name="v2x-live-http"
    )
    thread.start()
    httpd.v2x_thread = thread
    return httpd
