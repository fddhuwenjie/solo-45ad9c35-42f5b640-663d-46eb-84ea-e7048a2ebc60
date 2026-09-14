#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回归验证：静态资源请求 + 出线方向 90°/90° 边界 + 盘架反力 48252.3N 越限。

仅依赖 Python 标准库（unittest / wsgiref / json / sqlite3 / tempfile / io）。
运行：python3 -m unittest discover -s tests -v
   或：python3 tests/test_regression.py
"""

import io
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import server  # noqa: E402


def call_wsgi(method, path, body=None, content_type="application/json"):
    """不经过网络，直接驱动 WSGI app，返回 (status_int, headers, bytes)。"""
    payload = b""
    if body is not None:
        payload = body if isinstance(body, bytes) else json.dumps(body).encode("utf-8")
    environ = {
        "REQUEST_METHOD": method,
        "PATH_INFO": path,
        "QUERY_STRING": "",
        "CONTENT_LENGTH": str(len(payload)),
        "CONTENT_TYPE": content_type if payload else "",
        "SERVER_NAME": "test",
        "SERVER_PORT": "0",
        "wsgi.input": io.BytesIO(payload),
        "wsgi.errors": io.StringIO(),
    }
    captured = {}

    def start_response(status, headers, exc_info=None):
        captured["status"] = status
        captured["headers"] = headers

    chunks = server.app(environ, start_response)
    data = b"".join(chunks)
    for chunk in chunks:
        if hasattr(chunk, "close"):
            chunk.close()
    code = int(captured["status"].split(" ", 1)[0])
    return code, dict(captured["headers"]), data


def vertical_scenario(payout_angle, **param_overrides):
    """纯竖向线路：路线切线 90°，盘架出线方向同为 90°。"""
    params = {
        "cableWeight": 60, "friction": 0.15, "shaftDepth": 0,
        "initialTension": 100, "standCapacity": 90000,
        "allowTension": 50000, "brakeCapacity": 90000,
        "pullerCapacity": 50000,
    }
    params.update(param_overrides)
    return {
        "nodes": [
            {"id": "n0", "type": "start", "x": 0, "y": 0, "z": 0},
            {"id": "n9", "type": "shaft", "x": 0, "y": 10, "z": 0},
        ],
        "walls": [],
        "equipment": [
            {"id": "reel", "type": "reel", "x": 0, "y": 3, "payoutAngle": payout_angle},
            {"id": "pu", "type": "puller", "x": 0, "y": 8},
        ],
        "params": params,
    }


def overload_scenario():
    """盘架反力在首个牵引直线段达到 48252.3N，限值 45000N。

    W=80000N、盘径 2.6m、支腿基距 2.34m，半静载 40000N；
    取张力 14854.14N 时：40000 + 14854.14*1.3/2.34 = 48252.3N。
    盘架位于 1m 处、牵引机位于 29.8m 处，首段长 28.8m。
    """
    target_tension = (48252.3 - 40000.0) * 2.34 / 1.3
    cable_weight = (target_tension - 600.0) / (0.5 * 28.8)
    return {
        "nodes": [
            {"id": "n0", "type": "start", "x": 0, "y": 0, "z": 0},
            {"id": "n9", "type": "shaft", "x": 0, "y": 30, "z": 0},
        ],
        "walls": [],
        "equipment": [
            {"id": "reel-1", "type": "reel", "x": 0, "y": 1, "payoutAngle": 90},
            {"id": "pu-1", "type": "puller", "x": 0, "y": 29.8, "capacity": 200000},
        ],
        "params": {
            "totalWeight": 80000, "reelDiameter": 2.6, "standCapacity": 45000,
            "anchorCapacity": 200000, "cableWeight": cable_weight, "friction": 0.5,
            "initialTension": 600, "allowTension": 200000, "allowSidePressure": 200000,
            "shaftDepth": 0, "brakeCapacity": 200000, "pullerCapacity": 200000,
            "pullerBackTension": 600, "shaftSheaveRadius": 2.0, "minBendRadius": 1.0,
        },
    }


class StaticResourceTests(unittest.TestCase):
    """页面引用的静态资源必须可访问，app.js 不得再返回 404。"""

    def test_index_html(self):
        for path in ("/", "/index.html"):
            code, headers, body = call_wsgi("GET", path)
            self.assertEqual(code, 200, path)
            self.assertIn("text/html", headers["Content-Type"])
            self.assertIn(b"app.js", body)

    def test_app_js_exists(self):
        code, headers, body = call_wsgi("GET", "/app.js")
        self.assertEqual(code, 200)
        self.assertIn("text/javascript", headers["Content-Type"])
        self.assertGreater(len(body), 1000)
        self.assertIn(b"/api/simulate", body)

    def test_style_css_exists(self):
        code, headers, _ = call_wsgi("GET", "/style.css")
        self.assertEqual(code, 200)
        self.assertIn("text/css", headers["Content-Type"])

    def test_unknown_static_is_404(self):
        code, _, body = call_wsgi("GET", "/does-not-exist.js")
        self.assertEqual(code, 404)
        self.assertIn(b"Not found", body)

    def test_path_traversal_blocked(self):
        code, _, _ = call_wsgi("GET", "/../server.py")
        # normpath 跳出 STATIC_DIR 后按 404 处理，绝不回送源码
        self.assertEqual(code, 404)


class PayoutAngleBoundaryTests(unittest.TestCase):
    """竖向线路切线 90°、盘架出线同为 90° 时，不得误报“偏差 88°”。"""

    def test_vertical_tangent_matches_payout(self):
        result = server.simulate(vertical_scenario(90))
        self.assertEqual(result["status"], "passed", result.get("error"))
        setup = next(c for c in result["checkpoints"] if c["id"] == "setup")
        self.assertEqual(setup["status"], "success")
        self.assertNotIn("88", setup["message"])
        self.assertFalse(any("偏差 88" in c["message"] for c in result["checkpoints"]))

    def test_vertical_tangent_via_http(self):
        code, _, body = call_wsgi("POST", "/api/simulate",
                                  {"scenario": vertical_scenario(90)})
        self.assertEqual(code, 200)
        result = json.loads(body)
        self.assertEqual(result["status"], "passed")

    def test_reverse_payout_still_blocked(self):
        # 反方向（-90° 对 90° 切线，夹角 180°）仍必须判为摆反
        result = server.simulate(vertical_scenario(-90))
        self.assertEqual(result["status"], "blocked")
        fc = result["checkpoints"][result["failedIndex"]]
        self.assertEqual(fc["id"], "setup")
        self.assertIn("摆反", fc["message"])
        self.assertIn("180", fc["message"])


class StandReactionOverloadTests(unittest.TestCase):
    """盘架反力 48252.3N 超过 45000N 限值时，在首次越限处停止并判未通过。"""

    def test_overload_stops_at_first_limit(self):
        result = server.simulate(overload_scenario())
        self.assertEqual(result["status"], "blocked")
        idx = result["failedIndex"]
        self.assertIsNotNone(idx)
        fc = result["checkpoints"][idx]
        self.assertEqual(fc["phase"], "pull")
        self.assertEqual(fc["status"], "failed")
        # 责任对象是盘架，而不是牵引机/弯段
        self.assertEqual(fc["responsibleId"], "reel-1")
        self.assertEqual(fc["responsibleType"], "equipment")
        # 失败检查点自身指标保留越限值，不得被后续分段覆盖
        self.assertAlmostEqual(fc["metrics"]["standReaction"], 48252.3, delta=0.1)
        self.assertEqual(fc["metrics"]["standLimit"], 45000.0)
        self.assertIn("盘架", fc["message"])
        self.assertIn("45000", fc["message"])
        # 停在首次受阻处：其后不再产生检查点
        self.assertEqual(idx, len(result["checkpoints"]) - 1)

    def test_overload_via_http(self):
        code, _, body = call_wsgi("POST", "/api/simulate",
                                  {"scenario": overload_scenario()})
        self.assertEqual(code, 200)
        result = json.loads(body.decode("utf-8"))
        self.assertEqual(result["status"], "blocked")
        fc = result["checkpoints"][result["failedIndex"]]
        self.assertAlmostEqual(fc["metrics"]["standReaction"], 48252.3, delta=0.1)

    def test_capacity_above_reaction_passes(self):
        # 限值抬高到 50000N 后同一受力不得再报盘架过载
        sc = vertical_scenario(90)  # 占位避免空文件引用
        sc = overload_scenario()
        sc["params"]["standCapacity"] = 50000.0
        result = server.simulate(sc)
        self.assertEqual(result["status"], "passed",
                         [c["message"] for c in result.get("checkpoints", [])])
        self.assertFalse(any("盘架" in c["message"] and "过载" in c["message"]
                             for c in result["checkpoints"]))


if __name__ == "__main__":
    # 每个用例使用独立临时 sqlite，避免污染工作目录下的归档库
    tmp = tempfile.NamedTemporaryFile(prefix="cable-test-", suffix=".db", delete=False)
    tmp.close()
    server.DB_PATH = tmp.name
    server.init_db()
    try:
        unittest.main(verbosity=2)
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
