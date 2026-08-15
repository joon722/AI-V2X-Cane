import json
import shutil
import tempfile
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest import mock

import live_server
from live_state import LiveState
from test_live_state import FakeStore, make_assessment, make_decision, make_kinematics


class ServerCase(unittest.TestCase):
    """포트 0으로 띄워 OS가 준 빈 포트를 쓴다. 테스트끼리 포트를 다투지 않는다."""

    def setUp(self):
        self.state = LiveState()
        self.now = 1000.0
        self.web_dir = Path(tempfile.mkdtemp())
        (self.web_dir / "drive.html").write_text("<h1>차량뷰</h1>", encoding="utf-8")
        self.httpd = live_server.start(
            self.state, port=0, web_dir=self.web_dir, clock=lambda: self.now
        )
        # addCleanup은 LIFO다. 소켓을 먼저 닫으면 serve_forever가 닫힌 소켓을
        # select해 스레드가 예외로 끝난다. shutdown이 먼저 돌도록 뒤에 건다.
        self.addCleanup(shutil.rmtree, self.web_dir, ignore_errors=True)
        self.addCleanup(self.httpd.server_close)
        self.addCleanup(self.httpd.shutdown)
        self.base = f"http://127.0.0.1:{self.httpd.server_address[1]}"

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as response:
            return response.status, response.read(), dict(response.headers)

    def get_state(self):
        _status, body, _headers = self.get("/api/state")
        return json.loads(body)

    def feed(self, **kwargs):
        self.state.update(
            now=kwargs.pop("t", self.now),
            store=FakeStore(),
            decision=kwargs.pop("decision", make_decision()),
            assessment=kwargs.pop("assessment", make_assessment()),
            kinematics=make_kinematics(),
        )


class TestStateEndpoint(ServerCase):
    def test_serves_json_before_any_data(self):
        """신호가 오기 전에도 화면이 뜰 수 있어야 한다."""
        status, body, headers = self.get("/api/state")
        self.assertEqual(status, 200)
        self.assertIn("application/json", headers["Content-Type"])
        self.assertIsNone(json.loads(body)["t"])

    def test_serves_the_latest_measurement(self):
        self.feed()
        payload = self.get_state()
        self.assertEqual(payload["distance_m"], 12.3)
        self.assertEqual(payload["level"], 2)

    def test_age_reflects_the_clock_at_request_time(self):
        """나이는 판정 시각이 아니라 요청 시각 기준이어야 한다."""
        self.feed(t=1000.0)
        self.now = 1004.0
        self.assertAlmostEqual(self.get_state()["age_s"], 4.0)

    def test_response_is_not_cached(self):
        """브라우저가 오래된 상태를 붙들면 정지 화면이 실시간인 척한다."""
        self.feed()
        _status, _body, headers = self.get("/api/state")
        self.assertIn("no-store", headers.get("Cache-Control", ""))

    def test_successive_requests_see_updates(self):
        self.feed(assessment=make_assessment(ttc=2.1))
        self.assertEqual(self.get_state()["ttc_s"], 2.1)
        self.feed(assessment=make_assessment(ttc=1.2))
        self.assertEqual(self.get_state()["ttc_s"], 1.2)


class TestPageEndpoint(ServerCase):
    def test_root_serves_the_drive_page(self):
        status, body, headers = self.get("/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", headers["Content-Type"])
        self.assertIn("차량뷰", body.decode("utf-8"))

    def test_unknown_path_is_404(self):
        with self.assertRaises(urllib.error.HTTPError) as caught:
            self.get("/secrets")
        self.assertEqual(caught.exception.code, 404)

    def test_path_traversal_is_refused(self):
        """폐쇄망이라도 상위 경로를 열어줄 이유는 없다."""
        for path in ("/../step7_risk.py", "/..%2fstep7_risk.py", "/web/../../etc/passwd"):
            with self.assertRaises(urllib.error.HTTPError) as caught:
                self.get(path)
            self.assertEqual(caught.exception.code, 404)

    def test_missing_page_file_is_404_not_a_crash(self):
        """drive.html을 안 올렸을 때 서버가 죽으면 안 된다. /api/state는 살아야 한다."""
        (self.web_dir / "drive.html").unlink()
        with self.assertRaises(urllib.error.HTTPError) as caught:
            self.get("/")
        self.assertEqual(caught.exception.code, 404)
        self.assertEqual(self.get("/api/state")[0], 200)


class TestIsolation(unittest.TestCase):
    def test_thread_is_a_daemon(self):
        """step8이 끝날 때 이 스레드가 프로세스를 붙잡고 있으면 안 된다."""
        httpd = live_server.start(LiveState(), port=0, web_dir=Path("."))
        self.addCleanup(httpd.server_close)  # LIFO: shutdown 다음에 돈다
        self.addCleanup(httpd.shutdown)
        self.assertTrue(httpd.v2x_thread.daemon)
        self.assertTrue(httpd.v2x_thread.is_alive())

    def test_port_zero_means_the_screen_is_off(self):
        self.assertIsNone(live_server.start_or_none(0))

    def test_a_bind_failure_disables_the_screen_instead_of_raising(self):
        """포트가 막혀 있다고 경보까지 멈추면 안 된다.

        진짜 포트를 두 번 잡아 재현하지 않는 이유: Windows는 SO_REUSEADDR가
        리눅스와 달라 이미 쓰이는 포트에도 바인딩이 성공한다. 개발 PC에서
        조용히 통과해버리므로, 확인하려는 것(실패를 삼키는가)만 직접 만든다.
        """
        said = []
        with mock.patch.object(live_server, "start", side_effect=OSError("in use")):
            state = live_server.start_or_none(8080, on_message=said.append)

        self.assertIsNone(state)
        self.assertTrue(any("live_view_disabled" in line for line in said))

    def test_request_logging_is_silenced(self):
        """4Hz 폴링의 요청 로그가 journalctl을 덮으면 진짜 경보 로그가 묻힌다."""
        handler = live_server._make_handler(LiveState(), Path("."), lambda: 0.0)
        self.assertEqual(handler.log_message, live_server._silent)


if __name__ == "__main__":
    unittest.main()
