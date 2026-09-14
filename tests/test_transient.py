#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""放盘—牵引联动瞬态工作区回归测试。

覆盖：
- 五类瞬态冲突（张力/储热/超速/余缆/松弛）在最早时刻停帧；
- 正常点动—换速—急停脚本算通，逐时量齐全；
- 关键帧/制动参数改动只重算波及时段，已确认时段 hash 锁定；
- 归档后逐时 CSV / 曲线 SVG / 复算 JSON 含输入曲线、计算值与人工说明；
- 静力 /api/simulate 与静态资源不回退。

仅依赖标准库。运行：python3 -m unittest discover -s tests -v
"""

import io
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import server  # noqa: E402


def call_wsgi(method, path, body=None):
    payload = b"" if body is None else json.dumps(body).encode("utf-8")
    env = {"REQUEST_METHOD": method, "PATH_INFO": path, "QUERY_STRING": "",
           "CONTENT_LENGTH": str(len(payload)),
           "CONTENT_TYPE": "application/json" if payload else "",
           "SERVER_NAME": "test", "SERVER_PORT": "0",
           "wsgi.input": io.BytesIO(payload), "wsgi.errors": io.StringIO()}
    captured = {}

    def start_response(status, headers, exc_info=None):
        captured["status"] = status
        captured["headers"] = headers

    chunks = server.app(env, start_response)
    data = b"".join(chunks)
    return int(captured["status"].split(" ", 1)[0]), dict(captured["headers"]), data


def base_scenario(**overrides):
    params = {
        "cableWeight": 60, "shaftDepth": 0, "initialTension": 600,
        "standCapacity": 90000, "allowTension": 18000, "brakeCapacity": 90000,
        "pullerCapacity": 50000,
    }
    params.update(overrides)
    return {
        "nodes": [
            {"id": "n0", "type": "start", "x": 0, "y": 0, "z": 0},
            {"id": "n9", "type": "shaft", "x": 0, "y": 30, "z": 0},
        ],
        "walls": [],
        "equipment": [
            {"id": "reel", "type": "reel", "x": 0, "y": 3, "payoutAngle": 90},
            {"id": "pu", "type": "puller", "x": 0, "y": 13},
        ],
        "params": params,
    }


NORMAL_KEYFRAMES = [
    {"id": "k1", "type": "speed", "time": 1, "speed": 1.2, "ramp": 2},
    {"id": "k2", "type": "jog", "time": 6, "speed": 1.8, "hold": 1.2, "ramp": 0.5},
    {"id": "k3", "type": "speed", "time": 11, "speed": 0.8, "ramp": 2},
    {"id": "k4", "type": "estop", "time": 17, "ramp": 0.4},
    {"id": "k5", "type": "speed", "time": 21, "speed": 0.6, "ramp": 1},
]


def transient(keyframes=None, **conf):
    out = {"duration": 26, "dt": 0.1,
           "keyframes": NORMAL_KEYFRAMES if keyframes is None else keyframes}
    out.update(conf)
    return out


class TransientPhysicsTests(unittest.TestCase):
    def test_normal_script_passes_with_all_metrics(self):
        r = server.simulate_transient(
            {"scenario": base_scenario(), "transient": transient()})
        self.assertEqual(r["status"], "passed", r.get("error"))
        self.assertIsNone(r["conflict"])
        self.assertEqual(len(r["rows"]), 261)  # 0..26s，步长 0.1
        required = {"t", "radius", "omega", "rpm", "vCmd", "vReel", "dv",
                    "tension", "inertiaTension", "brakeCmd", "brakeTorque",
                    "brakeEnergy", "brakeHeat", "slack", "reserve", "station"}
        for row in r["rows"]:
            self.assertTrue(required <= set(row), required - set(row))
        # 卷绕半径随余缆放出而单调减小
        self.assertLess(r["rows"][-1]["radius"], r["rows"][0]["radius"])
        # 区间全部算通，且覆盖整个时间轴
        self.assertTrue(all(iv["status"] == "success" for iv in r["intervals"]))
        self.assertAlmostEqual(r["intervals"][0]["start"], 0.0, places=2)
        self.assertAlmostEqual(r["intervals"][-1]["end"], 26.0, places=2)

    def test_tension_spike_conflict(self):
        r = server.simulate_transient({"scenario": base_scenario(), "transient": transient(
            keyframes=[{"id": "k", "type": "speed", "time": 1, "speed": 2.5, "ramp": 0.05}],
            duration=10, dt=0.05, axialStiffness=4e7)})
        self.assertEqual(r["status"], "blocked")
        c = r["conflict"]
        self.assertEqual(c["kind"], "tension")
        self.assertEqual(c["responsibleType"], "equipment")
        self.assertIn("reel", c["responsibleId"])
        self.assertIn("张力", c["message"])

    def test_overspeed_conflict(self):
        r = server.simulate_transient({"scenario": base_scenario(), "transient": transient(
            keyframes=[{"id": "k", "type": "speed", "time": 1, "speed": 1.2, "ramp": 2}],
            duration=10, dt=0.05, bearingTorque=10, brakeMaxSpeed=0.8)})
        self.assertEqual(r["status"], "blocked")
        self.assertEqual(r["conflict"]["kind"], "overspeed")

    def test_brake_heat_conflict(self):
        r = server.simulate_transient({"scenario": base_scenario(), "transient": transient(
            keyframes=[{"id": "k1", "type": "speed", "time": 1, "speed": 2.0, "ramp": 2},
                       {"id": "k2", "type": "estop", "time": 7, "ramp": 0.3}],
            duration=12, dt=0.05, brakeHeatCapacity=200)})
        self.assertEqual(r["status"], "blocked")
        self.assertEqual(r["conflict"]["kind"], "heat")
        self.assertIn("热容量", r["conflict"]["message"])

    def test_reserve_exhausted_conflict(self):
        r = server.simulate_transient({"scenario": base_scenario(), "transient": transient(
            keyframes=[{"id": "k", "type": "speed", "time": 1, "speed": 1.0, "ramp": 2}],
            duration=30, reserveCable=5)})
        self.assertEqual(r["status"], "blocked")
        self.assertEqual(r["conflict"]["kind"], "reserve")
        self.assertGreater(r["conflict"]["time"], 0)

    def test_slack_loop_clearance_conflict(self):
        r = server.simulate_transient({"scenario": base_scenario(), "transient": transient(
            keyframes=[{"id": "k1", "type": "speed", "time": 1, "speed": 1.5, "ramp": 2},
                       {"id": "k2", "type": "speed", "time": 8, "speed": 0, "ramp": 0.3}],
            duration=14, dt=0.05, emptyInertia=5000, bearingTorque=5,
            slackClearance=0.15)})
        self.assertEqual(r["status"], "blocked")
        self.assertEqual(r["conflict"]["kind"], "slack")
        self.assertIn("松弛", r["conflict"]["message"])

    def test_playback_stops_at_earliest_conflict_only(self):
        """冲突行之后不再产生行；冲突区间标 failed，其前为 success。"""
        r = server.simulate_transient({"scenario": base_scenario(), "transient": transient(
            keyframes=[{"id": "k", "type": "speed", "time": 1, "speed": 2.5, "ramp": 0.05}],
            duration=10, dt=0.05, axialStiffness=4e7)})
        self.assertEqual(r["rows"][-1]["t"], r["conflict"]["time"])
        failed = [i for i, iv in enumerate(r["intervals"]) if iv["status"] == "failed"]
        self.assertEqual(failed, [r["failedInterval"]])
        self.assertTrue(all(iv["status"] == "success"
                            for i, iv in enumerate(r["intervals"]) if i < r["failedInterval"]))


class IncrementalLockTests(unittest.TestCase):
    def _blocked(self):
        return server.simulate_transient({"scenario": base_scenario(), "transient": transient(
            keyframes=[{"id": "k1", "type": "speed", "time": 1, "speed": 2.5, "ramp": 0.05}],
            duration=10, dt=0.05, axialStiffness=4e7)})

    def test_resume_recomputes_only_affected_interval(self):
        full = self._blocked()
        lc = full["failedInterval"]
        self.assertGreater(lc, 0)
        boundary = full["intervals"][lc]["start"]
        b_row = next(x for x in full["rows"] if abs(x["t"] - boundary) < 0.03)
        start_state = {"t": b_row["t"], "omega": b_row["omega"], "slack": b_row["slack"],
                       "reserve": b_row["reserve"], "tension": b_row["tension"],
                       "heat": b_row["brakeHeat"], "energy": b_row["brakeEnergy"],
                       "paidOut": b_row["paidOut"]}
        r = server.simulate_transient({"scenario": base_scenario(),
            "transient": transient(
                keyframes=[{"id": "k1", "type": "speed", "time": 1, "speed": 2.5, "ramp": 0.05}],
                duration=10, dt=0.05, axialStiffness=4e7),
            "lockedCount": lc, "verifiedHashes": full["hashes"][:lc],
            "startState": start_state})
        self.assertEqual(r["status"], "blocked")
        self.assertEqual(r["fromInterval"], lc)
        self.assertAlmostEqual(r["fromTime"], boundary, places=2)
        # 前段区间保持 locked
        self.assertTrue(all(iv["status"] == "locked" for iv in r["intervals"][:lc]))
        # 增量复算应在同一冲突时刻、同一张力停帧
        self.assertAlmostEqual(r["conflict"]["time"], full["conflict"]["time"], places=2)
        self.assertAlmostEqual(r["conflict"]["metrics"]["tension"],
                               full["conflict"]["metrics"]["tension"], delta=5.0)

    def test_change_inside_locked_window_is_rejected(self):
        full = self._blocked()
        lc = full["failedInterval"]
        modified = transient(
            keyframes=[{"id": "k1", "type": "speed", "time": 1, "speed": 2.5, "ramp": 0.2}],
            duration=10, dt=0.05, axialStiffness=4e7)
        r = server.simulate_transient(
            {"scenario": base_scenario(), "transient": modified,
             "lockedCount": lc, "verifiedHashes": full["hashes"][:lc]})
        self.assertEqual(r["status"], "lock-changed")
        self.assertLessEqual(r["lockIntervalIndex"], lc)

    def test_change_after_lock_window_keeps_lock(self):
        """只改动已确认区间之后的关键帧，hash 不变，应继续增量复算。"""
        full = self._blocked()
        lc = full["failedInterval"]
        kfs = [{"id": "k1", "type": "speed", "time": 1, "speed": 2.5, "ramp": 0.05},
               {"id": "k2", "type": "estop", "time": 9, "ramp": 0.3}]
        kfs2 = [dict(kfs[0]), {"id": "k2", "type": "estop", "time": 9.5, "ramp": 0.3}]
        first = server.simulate_transient(
            {"scenario": base_scenario(), "transient": transient(keyframes=kfs, duration=10, dt=0.05, axialStiffness=4e7)})
        self.assertEqual(first["status"], "blocked")
        lc2 = first["failedInterval"]
        second = server.simulate_transient(
            {"scenario": base_scenario(), "transient": transient(keyframes=kfs2, duration=10, dt=0.05, axialStiffness=4e7),
             "lockedCount": lc2, "verifiedHashes": first["hashes"][:lc2]})
        self.assertNotEqual(second["status"], "lock-changed")

    def test_brake_curve_change_invalidates_lock(self):
        full = self._blocked()
        lc = full["failedInterval"]
        r = server.simulate_transient({
            "scenario": base_scenario(),
            "transient": transient(
                keyframes=[{"id": "k1", "type": "speed", "time": 1, "speed": 2.5, "ramp": 0.05}],
                duration=10, dt=0.05, axialStiffness=4e7,
                brakeCurve=[[0, 0], [0.5, 4000], [1, 9000]]),
            "lockedCount": lc, "verifiedHashes": full["hashes"][:lc]})
        self.assertEqual(r["status"], "lock-changed")


class TransientHttpAndArchiveTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.NamedTemporaryFile(prefix="cable-test-", suffix=".db", delete=False)
        tmp.close()
        self._db = tmp.name
        server.DB_PATH = tmp.name
        server.init_db()

    def tearDown(self):
        try:
            os.unlink(self._db)
        except OSError:
            pass

    def test_api_transient_simulate(self):
        code, _, body = call_wsgi("POST", "/api/transient/simulate",
                                  {"scenario": base_scenario(), "transient": transient()})
        self.assertEqual(code, 200)
        r = json.loads(body)
        self.assertEqual(r["status"], "passed")
        self.assertGreater(len(r["rows"]), 100)

    def test_archive_outputs_contain_inputs_values_and_note(self):
        note = "现场复测：制动曲线来自厂家试验台；交底人张工"
        code, _, body = call_wsgi("POST", "/api/archives", {
            "name": "联动归档", "note": note,
            "scenario": base_scenario(), "transient": transient()})
        self.assertEqual(code, 200)
        aid = json.loads(body)["id"]

        code, _, body = call_wsgi("GET", "/api/archives/%d/transient-csv" % aid)
        self.assertEqual(code, 200)
        text = body.decode("utf-8")
        self.assertIn("time_s,cmd_speed_mps,reel_speed_mps", text)
        self.assertIn(note, text)
        self.assertIn("指令—扭矩曲线", text)
        self.assertIn("关键帧", text)
        # 逐时数据行
        self.assertGreater(len([ln for ln in text.splitlines() if ln and ln[0].isdigit()]), 200)

        code, _, body = call_wsgi("GET", "/api/archives/%d/transient-svg" % aid)
        self.assertEqual(code, 200)
        svg = body.decode("utf-8")
        self.assertTrue(svg.lstrip().startswith("<svg"))
        self.assertIn("张力", svg)
        self.assertIn("制动", svg)
        self.assertIn("松弛", svg)
        self.assertIn(note[:20], svg)

        code, _, body = call_wsgi("GET", "/api/archives/%d/transient-json" % aid)
        self.assertEqual(code, 200)
        payload = json.loads(body)
        self.assertEqual(payload["manualNote"], note)
        self.assertGreaterEqual(len(payload["input"]["brakeCommandTorqueCurve"]), 2)
        self.assertEqual(len(payload["input"]["keyframes"]), 5)
        self.assertIn("emptyInertia", payload["input"])
        self.assertGreater(len(payload["computed"]["rows"]), 100)
        self.assertIn("reelRotation", payload["formulas"])

        code, _, body = call_wsgi("GET", "/api/archives/%d" % aid)
        self.assertEqual(code, 200)
        detail = json.loads(body)
        self.assertIsNotNone(detail["transient"])
        self.assertIsNotNone(detail["transientResult"])

        code, _, body = call_wsgi("GET", "/api/archives")
        items = json.loads(body)["items"]
        self.assertTrue(items[0]["hasTransient"])
        self.assertEqual(items[0]["transientStatus"], "passed")

    def test_transient_endpoints_404_without_archive(self):
        code, _, _ = call_wsgi("GET", "/api/archives/999999/transient-csv")
        self.assertEqual(code, 404)

    def test_csv_url_is_served_not_404(self):
        """用户复核的使用阻断：逐时 CSV 请求必须可达。"""
        code, _, body = call_wsgi("POST", "/api/archives", {
            "name": "csv可达", "note": "n", "scenario": base_scenario(),
            "transient": transient()})
        aid = json.loads(body)["id"]
        for kind in ("transient-csv", "transient-svg", "transient-json"):
            code, _, _ = call_wsgi("GET", "/api/archives/%d/%s" % (aid, kind))
            self.assertEqual(code, 200, kind)


class StaticRegressionGuardTests(unittest.TestCase):
    """瞬态改动不得影响静力推演与静态资源。"""

    def test_static_simulation_still_passes(self):
        sc = {
            "nodes": [{"id": "n0", "type": "start", "x": 0, "y": 0, "z": 0},
                      {"id": "n9", "type": "shaft", "x": 0, "y": 10, "z": 0}],
            "walls": [],
            "equipment": [{"id": "reel", "type": "reel", "x": 0, "y": 3, "payoutAngle": 90},
                          {"id": "pu", "type": "puller", "x": 0, "y": 8}],
            "params": {"cableWeight": 60, "friction": 0.15, "shaftDepth": 0,
                       "initialTension": 100, "standCapacity": 90000,
                       "allowTension": 50000, "brakeCapacity": 90000,
                       "pullerCapacity": 50000},
        }
        code, _, body = call_wsgi("POST", "/api/simulate", {"scenario": sc})
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(body)["status"], "passed")

    def test_index_exposes_transient_workspace(self):
        code, _, body = call_wsgi("GET", "/index.html")
        self.assertEqual(code, 200)
        html = body.decode("utf-8")
        for token in ("空盘转动惯量", "轴承阻力矩", "初始余缆", "电缆轴向刚度",
                      "制动器热容量", "制动器转速上限", "指令—扭矩曲线",
                      "点动", "换速", "急停", "transient.js", "逐时复算"):
            self.assertIn(token, html, token)

    def test_transient_js_served(self):
        code, headers, body = call_wsgi("GET", "/transient.js")
        self.assertEqual(code, 200)
        self.assertIn("text/javascript", headers["Content-Type"])
        self.assertIn(b"/api/transient/simulate", body)
        self.assertIn(b"transient-csv", body)


if __name__ == "__main__":
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
