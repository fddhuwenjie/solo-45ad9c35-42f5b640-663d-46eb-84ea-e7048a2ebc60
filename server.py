#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电缆盘进场与放缆推演 WSGI 服务。

后端仅依赖 Python 标准库：wsgiref / json / sqlite3（math/os 仅作数值与路径处理）。
"""

import json
import math
import os
import sqlite3
from wsgiref.simple_server import make_server

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
DB_PATH = os.path.join(BASE_DIR, "cable_rigging.db")

FLOAT_FIELDS = [
    "reelDiameter", "barrelDiameter", "reelWidth", "totalWeight",
    "cableDiameter", "cableWeight", "friction", "rollerFriction",
    "rollingResistance", "initialTension", "allowTension",
    "allowSidePressure", "standCapacity", "anchorCapacity",
    "brakeCapacity", "minBendRadius", "clearanceMargin",
    "guideRadius", "guideCount", "guideCapacity",
    "pullerCapacity", "pullerBackTension", "shaftDepth",
    "shaftSheaveRadius",
    # 放盘—牵引瞬态联动参数
    "emptyInertia", "bearingTorque", "reserveCable", "axialStiffness",
    "slackClearance", "brakeMaxSpeed", "brakeHeatCapacity",
    "brakeWindingPack", "transientStartSpeed", "transientDuration",
    "transientDt",
]

GRAVITY = 9.81
DEFAULT_BRAKE_CURVE = [[0.0, 0.0], [0.25, 1400.0], [0.5, 3000.0],
                       [0.75, 5000.0], [1.0, 7500.0]]


# ----------------------------- 通用工具 -----------------------------

def num(value, default=0.0):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def clamp(value, low, high):
    return max(low, min(high, value))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(a, b):
    d = dist(a, b)
    if d < 1e-9:
        return 0.0, 0.0
    return (b[0] - a[0]) / d, (b[1] - a[1]) / d


def angle_between(u, v):
    """有向夹角，弧度；屏幕坐标 y 向下也沿用同一叉积符号。"""
    cross = u[0] * v[1] - u[1] * v[0]
    dot = clamp(u[0] * v[0] + u[1] * v[1], -1.0, 1.0)
    return math.atan2(cross, dot)


def wrap_angle(a):
    while a <= -math.pi:
        a += 2 * math.pi
    while a > math.pi:
        a -= 2 * math.pi
    return a


def stable_hash(value):
    """64 位 FNV 散列，用于前端“已锁定区段”指纹。"""
    text = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    h = 1469598103934665603
    for ch in text.encode("utf-8"):
        h ^= ch
        h = (h * 1099511628211) % (2 ** 64)
    return "%016x" % h


def point_segment_distance(px, py, x1, y1, x2, y2):
    l2 = (x2 - x1) ** 2 + (y2 - y1) ** 2
    if l2 == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2
    t = clamp(t, 0.0, 1.0)
    qx, qy = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
    return math.hypot(px - qx, py - qy)


def segment_distance(x1, y1, x2, y2, x3, y3, x4, y4):
    """两线段最近距离；相交时返回 0。"""
    def seg_intersects():
        d1x, d1y = x2 - x1, y2 - y1
        d2x, d2y = x4 - x3, y4 - y3
        denom = d1x * d2y - d1y * d2x
        if abs(denom) < 1e-12:
            return False
        t = ((x3 - x1) * d2y - (y3 - y1) * d2x) / denom
        s = ((x3 - x1) * d1y - (y3 - y1) * d1x) / denom
        return -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= s <= 1 + 1e-9

    if seg_intersects():
        return 0.0

    def point_sq(px, py, ax, ay, bx, by):
        l2 = (bx - ax) ** 2 + (by - ay) ** 2
        if l2 == 0:
            return (px - ax) ** 2 + (py - ay) ** 2
        t = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / l2
        t = clamp(t, 0, 1)
        qx, qy = ax + t * (bx - ax), ay + t * (by - ay)
        return (px - qx) ** 2 + (py - qy) ** 2

    vals = [
        point_sq(x1, y1, x3, y3, x4, y4),
        point_sq(x2, y2, x3, y3, x4, y4),
        point_sq(x3, y3, x1, y1, x2, y2),
        point_sq(x4, y4, x1, y1, x2, y2),
    ]
    return math.sqrt(min(vals))


def rect_segment_distance(cx, cy, ex, ext, hx, hwt, wall):
    """OBB 矩形与墙线段的净距。

    ex：出线方向单位轴；ext：沿出线方向的半长；
    hx：盘轴方向单位轴；hwt：盘轴方向半宽。
    """
    x1, y1, x2, y2 = wall["x1"], wall["y1"], wall["x2"], wall["y2"]

    def to_local(x, y):
        dx, dy = x - cx, y - cy
        return dx * ex[0] + dy * ex[1], dx * hx[0] + dy * hx[1]

    l1 = to_local(x1, y1)
    l2 = to_local(x2, y2)
    corners = [(-ext, -hwt), (ext, -hwt), (ext, hwt), (-ext, hwt)]

    def inside(lx, ly):
        return -ext - 1e-9 <= lx <= ext + 1e-9 and -hwt - 1e-9 <= ly <= hwt + 1e-9

    if inside(l1[0], l1[1]) or inside(l2[0], l2[1]):
        return 0.0

    best = float("inf")
    for i in range(4):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % 4]
        best = min(best, segment_distance(l1[0], l1[1], l2[0], l2[1], ax, ay, bx, by))
    return best


# ----------------------------- 参数与线形 -----------------------------

def get_params(scenario):
    raw = scenario.get("params", {}) or {}
    p = {}
    defaults = {
        "reelDiameter": 2.6,
        "barrelDiameter": 1.6,
        "reelWidth": 1.45,
        "totalWeight": 80000.0,
        "cableDiameter": 0.085,
        "cableWeight": 118.0,
        "friction": 0.22,
        "rollerFriction": 0.08,
        "rollingResistance": 0.035,
        "initialTension": 600.0,
        "allowTension": 18000.0,
        "allowSidePressure": 3500.0,
        "standCapacity": 55000.0,
        "anchorCapacity": 30000.0,
        "brakeCapacity": 12000.0,
        "minBendRadius": 1.7,
        "clearanceMargin": 0.10,
        "guideRadius": 0.75,
        "guideCount": 4.0,
        "guideCapacity": 6000.0,
        "pullerCapacity": 18000.0,
        "pullerBackTension": 300.0,
        "shaftDepth": 12.0,
        "shaftSheaveRadius": 1.8,
        # 放盘—牵引瞬态联动默认值（空盘惯量按盘体重心估算给出）
        "emptyInertia": 520.0,
        "bearingTorque": 260.0,
        "reserveCable": 60.0,
        "axialStiffness": 4.0e6,
        "slackClearance": 1.5,
        "brakeMaxSpeed": 12.0,
        "brakeHeatCapacity": 260000.0,
        "brakeWindingPack": 0.85,
        "transientStartSpeed": 0.0,
        "transientDuration": 30.0,
        "transientDt": 0.1,
    }
    for key, val in defaults.items():
        p[key] = num(raw.get(key), val)
    p["guideCount"] = max(1.0, round(p["guideCount"]))
    return p


def edge_grade(nodes, i):
    a, b = nodes[i], nodes[i + 1]
    length = dist((a["x"], a["y"]), (b["x"], b["y"]))
    if length < 1e-9:
        return 0.0
    return (num(b.get("z")) - num(a.get("z"))) / length * 100.0


def build_alignment(nodes):
    """生成由直线和圆弧组成的平面线形。station 为水平里程。"""
    sections = []
    node_station = {nodes[0]["id"]: 0.0}
    station = 0.0
    current = [nodes[0]["x"], nodes[0]["y"]]
    warnings = []

    for i in range(len(nodes) - 1):
        a, b = nodes[i], nodes[i + 1]
        u = unit((a["x"], a["y"]), (b["x"], b["y"]))
        full_len = dist((a["x"], a["y"]), (b["x"], b["y"]))
        is_turn = b.get("type") == "turn" and i + 2 < len(nodes)
        c = nodes[i + 2] if is_turn else None

        if is_turn:
            v = unit((b["x"], b["y"]), (c["x"], c["y"]))
            next_len = dist((b["x"], b["y"]), (c["x"], c["y"]))
            theta = angle_between(u, v)
            if abs(theta) < math.radians(2.0):
                is_turn = False

        if is_turn:
            radius = num(b.get("radius"), 3.0)
            tan_half = math.tan(abs(theta) / 2)
            max_d = min(full_len, next_len) * 0.45
            d = radius * tan_half
            if d > max_d and tan_half > 1e-6:
                old_radius = radius
                radius = max_d / tan_half
                warnings.append("%s 受相邻直线长度限制，弯曲半径由 %.2fm 调整为 %.2fm"
                                % (b.get("id"), old_radius, radius))
                d = max_d

            p_enter = [b["x"] - u[0] * d, b["y"] - u[1] * d]
            p_exit = [b["x"] + v[0] * d, b["y"] + v[1] * d]

            if dist(current, p_enter) > 1e-6:
                sections.append({
                    "kind": "straight", "edge": i,
                    "start": current[:], "end": p_enter[:],
                    "length": dist(current, p_enter),
                    "startStation": station,
                    "endStation": station + dist(current, p_enter),
                    "grade": edge_grade(nodes, i),
                })
                station += dist(current, p_enter)

            sign = 1.0 if theta >= 0 else -1.0
            left = (-u[1], u[0])
            center = [p_enter[0] + left[0] * radius * sign,
                      p_enter[1] + left[1] * radius * sign]
            start_angle = math.atan2(p_enter[1] - center[1], p_enter[0] - center[0])
            arc_len = radius * abs(theta)
            sec = {
                "kind": "arc", "edge": i, "turn": b["id"],
                "start": p_enter, "end": p_exit, "length": arc_len,
                "startStation": station, "endStation": station + arc_len,
                "center": center, "radius": radius,
                "startAngle": start_angle, "sweep": theta,
                "grade": 0.0,
            }
            sections.append(sec)
            node_station[b["id"]] = station + arc_len / 2.0
            station += arc_len
            current = p_exit
        else:
            length = dist(current, (b["x"], b["y"]))
            sections.append({
                "kind": "straight", "edge": i,
                "start": current[:], "end": [b["x"], b["y"]],
                "length": length,
                "startStation": station,
                "endStation": station + length,
                "grade": edge_grade(nodes, i),
            })
            station += length
            node_station[b["id"]] = station
            current = [b["x"], b["y"]]

    return sections, node_station, station, warnings


def point_at(sections, station):
    if not sections:
        return None
    station = clamp(station, sections[0]["startStation"], sections[-1]["endStation"])
    for sec in sections:
        if station <= sec["endStation"] + 1e-9 or sec is sections[-1]:
            span = max(sec["endStation"] - sec["startStation"], 1e-9)
            t = clamp((station - sec["startStation"]) / span, 0.0, 1.0)
            if sec["kind"] == "straight":
                x = sec["start"][0] + (sec["end"][0] - sec["start"][0]) * t
                y = sec["start"][1] + (sec["end"][1] - sec["start"][1]) * t
                tangent = unit(sec["start"], sec["end"])
            else:
                a = sec["startAngle"] + sec["sweep"] * t
                x = sec["center"][0] + sec["radius"] * math.cos(a)
                y = sec["center"][1] + sec["radius"] * math.sin(a)
                radial = (math.cos(a), math.sin(a))
                sign = 1.0 if sec["sweep"] >= 0 else -1.0
                tangent = (-radial[1] * sign, radial[0] * sign)
            return {"x": x, "y": y, "tangent": tangent, "section": sec, "t": t}
    return None


def project_point(sections, x, y):
    best = None
    for sec in sections:
        if sec["kind"] == "straight":
            sx, sy = sec["start"]
            ex, ey = sec["end"]
            l2 = max((ex - sx) ** 2 + (ey - sy) ** 2, 1e-12)
            t = clamp(((x - sx) * (ex - sx) + (y - sy) * (ey - sy)) / l2, 0, 1)
            qx, qy = sx + t * (ex - sx), sy + t * (ey - sy)
            d = math.hypot(x - qx, y - qy)
            st = sec["startStation"] + t * (sec["endStation"] - sec["startStation"])
            tan = unit(sec["start"], sec["end"])
        else:
            ang = math.atan2(y - sec["center"][1], x - sec["center"][0])
            delta = ang - sec["startAngle"]
            if sec["sweep"] >= 0:
                delta %= 2 * math.pi
                frac = clamp(delta / abs(sec["sweep"]), 0, 1)
            else:
                delta = delta % (-2 * math.pi)
                frac = clamp(delta / sec["sweep"], 0, 1)
            a = sec["startAngle"] + sec["sweep"] * frac
            qx = sec["center"][0] + sec["radius"] * math.cos(a)
            qy = sec["center"][1] + sec["radius"] * math.sin(a)
            d = math.hypot(x - qx, y - qy)
            st = sec["startStation"] + frac * sec["length"]
            radial = (math.cos(a), math.sin(a))
            sign = 1.0 if sec["sweep"] >= 0 else -1.0
            tan = (-radial[1] * sign, radial[0] * sign)
        if best is None or d < best["offset"]:
            best = {"station": st, "x": qx, "y": qy, "offset": d,
                    "tangent": tan, "sectionIndex": sections.index(sec),
                    "sectionKind": sec["kind"]}
    return best


# ----------------------------- 受力计算 -----------------------------

def slope_alpha(grade_percent):
    return math.atan(grade_percent / 100.0)


def ramp_brake(p, grade):
    alpha = slope_alpha(grade)
    if grade >= 0:
        force = p["totalWeight"] * (math.sin(alpha) + p["rollingResistance"] * math.cos(alpha))
        return 0.0, force, alpha
    brake = p["totalWeight"] * (math.sin(abs(alpha)) - p["rollingResistance"] * math.cos(alpha))
    return max(0.0, brake), 0.0, alpha


def stand_reactions(p, grade, tension):
    alpha = slope_alpha(grade)
    w = p["totalWeight"]
    radius = p["reelDiameter"] / 2.0
    base = max(1.2, p["reelDiameter"] * 0.9)
    normal = w * math.cos(alpha)
    moment = (tension * math.cos(alpha) + w * math.sin(alpha)) * radius
    r_a = normal / 2.0 + moment / base
    r_b = normal / 2.0 - moment / base
    horizontal = tension * math.cos(alpha) + w * abs(math.sin(alpha))
    return {"a": r_a, "b": r_b, "max": max(r_a, r_b),
            "min": min(r_a, r_b), "horizontal": horizontal, "base": base}


def base_metrics(p):
    return {
        "tensionBefore": p["initialTension"],
        "tension": p["initialTension"],
        "tensionLimit": p["allowTension"],
        "bendRadius": None,
        "radiusLimit": p["minBendRadius"],
        "sidePressure": None,
        "sideLimit": p["allowSidePressure"],
        "standReaction": p["totalWeight"] / 2.0,
        "standLimit": p["standCapacity"],
        "brakeForce": 0.0,
        "brakeLimit": p["brakeCapacity"],
        "radialLoad": None,
        "wheelLoad": None,
        "slope": 0.0,
        "length": 0.0,
    }


def make_checkpoint(cid, phase, title, s0, s1, cid_hash, metrics,
                    responsible_id="", responsible_type="", message="",
                    status="success", severity="info"):
    return {
        "id": cid, "phase": phase, "title": title,
        "startStation": round(s0, 3), "endStation": round(s1, 3),
        "hash": cid_hash, "metrics": metrics,
        "responsibleId": responsible_id, "responsibleType": responsible_type,
        "message": message, "status": status, "severity": severity,
    }


# ----------------------------- 主推演 -----------------------------

def simulate(scenario, locked_count=0, verified_hashes=None):
    verified_hashes = verified_hashes or []
    p = get_params(scenario)
    nodes = scenario.get("nodes") or []
    walls = scenario.get("walls") or []
    equipment = scenario.get("equipment") or []
    if len(nodes) < 2:
        return {"ok": False, "error": "至少需要起点和井口两个线形节点。"}
    if nodes[0].get("type") != "start" or nodes[-1].get("type") != "shaft":
        return {"ok": False, "error": "线形必须以起点开始、以竖井井口结束。"}

    try:
        sections, node_station, total_station, warnings = build_alignment(nodes)
    except Exception as exc:  # 线形构造异常仍以 JSON 反馈
        return {"ok": False, "error": "线形构造失败：%s" % exc}

    reel = next((e for e in equipment if e.get("type") == "reel"), None)
    if not reel:
        return {"ok": False, "error": "请先拖放电缆盘架。"}
    reel_proj = project_point(sections, num(reel.get("x")), num(reel.get("y")))
    if not reel_proj or reel_proj["station"] <= 0.1 or reel_proj["station"] >= total_station - 0.1:
        return {"ok": False, "error": "电缆盘架必须位于起点与井口之间的路线上。"}

    pullers, guides = [], []
    for e in equipment:
        if e.get("type") == "puller":
            pr = project_point(sections, num(e.get("x")), num(e.get("y")))
            pullers.append({"obj": e, "proj": pr})
        elif e.get("type") == "guide":
            pr = project_point(sections, num(e.get("x")), num(e.get("y")))
            guides.append({"obj": e, "proj": pr})
    pullers.sort(key=lambda x: x["proj"]["station"])
    pullers_ahead = [x for x in pullers if x["proj"]["station"] > reel_proj["station"]]

    guide_by_turn = {}
    for sec in sections:
        if sec["kind"] != "arc":
            continue
        candidates = [g for g in guides if g["obj"]["id"] not in guide_by_turn.values()]
        candidates = [g for g in guides if g["obj"]["id"] not in [v["obj"]["id"] for v in guide_by_turn.values()]]
        if not candidates:
            continue
        g = min(candidates, key=lambda x: abs(x["proj"]["station"] - (sec["startStation"] + sec["endStation"]) / 2))
        guide_by_turn[sec["startStation"]] = {"guide": g, "section": sec}

    checkpoints = []
    metrics = base_metrics(p)
    tension = p["initialTension"]
    failed_index = None

    def finish(cp, stop=False):
        nonlocal failed_index
        idx = len(checkpoints)
        if idx < locked_count:
            if idx >= len(verified_hashes) or cp["hash"] != verified_hashes[idx]:
                return {
                    "ok": True, "status": "lock-changed",
                    "checkpoints": checkpoints,
                    "lockCheckpointId": cp["id"],
                    "error": "已核实区段的输入发生变化，请解锁后从头复算。",
                }
            cp["status"] = "locked"
            cp["message"] = cp["message"] or "已核实锁定"
        elif cp["status"] == "failed":
            failed_index = idx if stop else failed_index
        checkpoints.append(cp)
        if stop:
            return {"ok": True, "status": "blocked", "checkpoints": checkpoints,
                    "failedIndex": idx, "summary": summary(), "warnings": warnings,
                    "geometry": geometry_brief()}
        return None

    def summary():
        max_t = max([p["initialTension"]] + [c["metrics"].get("tension") or 0 for c in checkpoints])
        max_side = [c["metrics"].get("sidePressure") for c in checkpoints if c["metrics"].get("sidePressure") is not None]
        return {
            "totalStation": round(total_station, 3),
            "reelStation": round(reel_proj["station"], 3),
            "maxTension": round(max_t, 1),
            "maxSidePressure": round(max(max_side), 1) if max_side else None,
            "allowTension": p["allowTension"],
            "allowSidePressure": p["allowSidePressure"],
        }

    def geometry_brief():
        return {"totalStation": total_station,
                "nodeStations": node_station,
                "sections": sections,
                "reelStation": reel_proj["station"]}

    def first_wall_hit(s0, s1, radius):
        steps = max(2, int(math.ceil((s1 - s0) / 0.25)))
        for wall in walls:
            for i in range(steps + 1):
                st = s0 + (s1 - s0) * i / steps
                q = point_at(sections, st)
                d = point_segment_distance(q["x"], q["y"], wall["x1"], wall["y1"], wall["x2"], wall["y2"])
                if d < radius:
                    return st, wall
        return None, None

    # 1) 滚运进场：从起点到盘架
    swing = max(p["reelDiameter"] / 2.0, p["reelWidth"] / 2.0) + p["clearanceMargin"]
    wall_sig = [(w.get("id"), round(w["x1"], 2), round(w["y1"], 2),
                 round(w["x2"], 2), round(w["y2"], 2)) for w in walls]
    reel_s = reel_proj["station"]
    for sec in sections:
        if sec["startStation"] >= reel_s:
            break
        a = sec["startStation"]
        b = min(sec["endStation"], reel_s)
        if b <= a:
            continue
        end_station, msg, resp_id, resp_type, severity = b, "", "", "", "info"
        hit_st, wall = first_wall_hit(a, b, swing)

        # 门洞位于该直线区间时优先判定。
        gate_node = None
        gate_station = None
        for n in nodes:
            if n.get("type") != "gate":
                continue
            ns = node_station.get(n["id"])
            if ns is not None and a - 0.05 <= ns <= b + 0.05:
                gate_node, gate_station = n, ns
        if gate_node:
            req_w = p["reelWidth"] + p["clearanceMargin"]
            req_h = p["reelDiameter"] + p["clearanceMargin"]
            if req_w > num(gate_node.get("openingWidth"), 0) or req_h > num(gate_node.get("openingHeight"), 0):
                hit_st = None
                end_station = gate_station
                msg = "门洞净宽 %.2fm/净高 %.2fm 不足；盘体需 %.2fm×%.2fm。" % (
                    num(gate_node.get("openingWidth")), num(gate_node.get("openingHeight")),
                    req_w, req_h)
                resp_id, resp_type, severity = gate_node["id"], "node", "error"
        if hit_st is not None and (gate_station is None or hit_st <= gate_station):
            end_station = hit_st
            msg = "盘体滚运转场转动范围碰墙，净距小于 %.2fm。" % p["clearanceMargin"]
            resp_id, resp_type, severity = wall["id"], "wall", "error"

        m = json.loads(json.dumps(metrics))
        title = "滚运直线段"
        if abs(sec["grade"]) > 5.0 and sec["kind"] == "straight":
            brake, haul, alpha = ramp_brake(p, sec["grade"])
            m["slope"] = sec["grade"]
            m["brakeForce"] = brake
            if sec["grade"] < 0:
                title = "滚运下坡段"
                if brake > p["brakeCapacity"] and not resp_id:
                    end_station = b
                    msg = "坡道制动力 %.0fN 超过制动能力 %.0fN。" % (brake, p["brakeCapacity"])
                    resp_id, resp_type, severity = reel["id"], "equipment", "error"
            else:
                title = "滚运上坡段"
                m["tractionForce"] = haul
        if sec["kind"] == "arc":
            title = "滚运转向"
            m["bendRadius"] = sec["radius"]

        wall_sig = [(w.get("id"), round(w["x1"], 2), round(w["y1"], 2),
                     round(w["x2"], 2), round(w["y2"], 2)) for w in walls]
        h = stable_hash([
            "roll", sec["kind"], sec["edge"], round(a, 2), round(b, 2),
            sec.get("radius"), sec.get("sweep"), sec.get("grade"),
            p["reelDiameter"], p["reelWidth"], p["totalWeight"],
            p["rollingResistance"], p["clearanceMargin"], wall_sig,
            gate_node and {k: gate_node.get(k) for k in ("id", "openingWidth", "openingHeight")}
        ])
        cp = make_checkpoint("roll:%s:%s" % (sec["kind"], sec["edge"]), "roll", title,
                             a, end_station, h, m, resp_id, resp_type, msg,
                             "failed" if severity == "error" else "success", severity)
        if severity != "error":
            metrics = m
        stopped = finish(cp, bool(resp_id))
        if stopped:
            return stopped

    # 2) 架设与出线方向
    m = json.loads(json.dumps(metrics))
    setup_msg, setup_resp, severity = "", reel["id"], "info"
    if reel_proj["sectionKind"] == "arc":
        setup_msg = "盘架落在转弯弧段上，无法保持出线方向稳定。"
        severity = "error"
    else:
        tan = reel_proj["tangent"]
        tangent_deg = math.degrees(math.atan2(tan[1], tan[0]))
        payout_deg = num(reel.get("payoutAngle"))
        # 有向夹角：出线方向须与朝井口的路线切线同向；
        # 注意度数相减后只能包一次 radians，不能再嵌套 radians。
        delta = math.degrees(abs(wrap_angle(math.radians(payout_deg - tangent_deg))))
        if delta > 100:
            setup_msg = "盘架出线方向摆反（偏差 %.0f°），电缆将在竖井口形成扭转。" % delta
            severity = "error"
        elif delta > 12:
            setup_msg = "盘架出线方向偏差 %.0f°，需就地转向至路线切线。" % delta
            severity = "error"
        if severity != "error":
            grade = sections[reel_proj["sectionIndex"]]["grade"]
            react = stand_reactions(p, grade, tension)
            m["standReaction"] = react["max"]
            m["standLimit"] = p["standCapacity"]
            if react["min"] < 0 or react["max"] > p["standCapacity"]:
                setup_msg = "盘架支反力 %.0fN 越限（限值 %.0fN），存在翘腿或过载。" % (
                    react["max"], p["standCapacity"])
                severity = "error"
            elif react["horizontal"] > p["anchorCapacity"]:
                setup_msg = "盘架水平锚固力 %.0fN 越限（限值 %.0fN）。" % (
                    react["horizontal"], p["anchorCapacity"])
                severity = "error"

        # 架设状态下的固定盘体包络复核
        exv = (math.cos(math.radians(num(reel.get("payoutAngle")))),
               math.sin(math.radians(num(reel.get("payoutAngle")))))
        hxv = (-exv[1], exv[0])
        half_l = p["reelDiameter"] / 2
        half_w = p["reelWidth"] / 2
        for wall in walls:
            d = rect_segment_distance(reel_proj["x"], reel_proj["y"],
                                      exv, half_l, hxv, half_w, wall)
            if d < p["clearanceMargin"]:
                setup_msg = "架设后盘体转动范围碰墙，净距 %.2fm。" % d
                severity = "error"
                setup_resp = wall["id"]
                break

    if not pullers_ahead:
        setup_msg = "尚未在盘架下游放置牵引机。"
        severity = "error"

    h = stable_hash([
        "setup", reel.get("id"), round(reel_proj["station"], 2),
        reel.get("payoutAngle"), p["reelDiameter"], p["reelWidth"],
        p["totalWeight"], p["initialTension"], p["rollingResistance"],
        wall_sig if 'wall_sig' in locals() else [],
    ])
    cp = make_checkpoint("setup", "setup", "盘架架设与出线核对",
                         reel_s, reel_s, h, m, setup_resp,
                         "wall" if setup_resp in [w.get("id") for w in walls] else "equipment",
                         setup_msg, "failed" if severity == "error" else "success", severity)
    if severity != "error":
        metrics = m
    stopped = finish(cp, severity == "error")
    if stopped:
        return stopped

    # 3) 牵引放缆
    def update_stand(grade, t, target=None):
        react = stand_reactions(p, grade, t)
        mm = target if target is not None else metrics
        mm["standReaction"] = react["max"]
        mm["standLimit"] = p["standCapacity"]
        mm["standHorizontal"] = react["horizontal"]
        return react

    def stand_overload(react):
        """盘架反力首次越限判定；责任对象固定为盘架。"""
        if react["min"] < 0:
            return ("盘架支腿出现负反力 %.0fN（翘腿），盘架过载，限值 %.0fN。"
                    % (react["min"], p["standCapacity"]))
        if react["max"] > p["standCapacity"]:
            return ("盘架支反力 %.1fN 超过盘架允许值 %.0fN，盘架过载。"
                    % (react["max"], p["standCapacity"]))
        if react["horizontal"] > p["anchorCapacity"]:
            return ("盘架水平锚固力 %.0fN 超过限值 %.0fN。"
                    % (react["horizontal"], p["anchorCapacity"]))
        return ""

    def straight_span(sec, a, b):
        nonlocal tension, metrics
        if b <= a:
            return None
        grade = sec["grade"]
        alpha = slope_alpha(grade)
        horizontal_len = b - a
        slope_len = horizontal_len / max(math.cos(alpha), 0.01)
        t0 = tension
        tension = max(0.0, tension + p["cableWeight"] * slope_len * (
            p["friction"] * math.cos(alpha) + math.sin(alpha)))
        m = json.loads(json.dumps(metrics))
        m.update({"tensionBefore": t0, "tension": tension,
                  "slope": grade, "length": slope_len})
        react = update_stand(grade, tension, m)
        stand_msg = stand_overload(react)
        # 失败段不把越限指标写回共享状态，避免后续分段点覆盖本检查点数值。
        if not stand_msg and tension <= p["allowTension"]:
            metrics = m
        msg = ""
        resp = pullers_ahead[-1]["obj"]["id"] if pullers_ahead else reel["id"]
        rtype = "equipment"
        status, severity = "success", "info"
        if stand_msg:
            msg = stand_msg
            status, severity = "failed", "error"
            resp, rtype = reel["id"], "equipment"
        elif tension > p["allowTension"]:
            msg = "累计张力 %.0fN 超过允许张力 %.0fN。" % (tension, p["allowTension"])
            status, severity = "failed", "error"
        elif abs(grade) >= 5:
            msg = "坡道段长度 %.1fm，纵坡 %.1f%%，张力由 %.0fN 增至 %.0fN。" % (
                slope_len, grade, t0, tension)
        else:
            msg = "直线段长度 %.1fm，张力 %.0fN。" % (slope_len, tension)
        h = stable_hash([
            "pull-straight", sec["edge"], round(a, 2), round(b, 2), grade,
            p["cableWeight"], p["friction"], p["initialTension"],
        ])
        cp = make_checkpoint("pull:straight:%s:%.1f" % (sec["edge"], a),
                             "pull", "牵引直线/坡道段", a, b, h, m,
                             resp, rtype, msg, status, severity)
        return finish(cp, status == "failed")

    def arc_span(sec, a, b):
        nonlocal tension, metrics
        turn_node = next(n for n in nodes if n["id"] == sec["turn"])
        assigned = None
        for value in guide_by_turn.values():
            if value["section"] is sec:
                assigned = value["guide"]
                break
        theta = abs(sec["sweep"])
        t0 = tension
        m = json.loads(json.dumps(metrics))
        m["bendRadius"] = sec["radius"]
        m["radiusLimit"] = p["minBendRadius"]
        m["sidePressure"] = t0 / sec["radius"]
        msg, resp, rtype = "", turn_node["id"], "node"
        status, severity = "success", "info"

        if not assigned and theta >= math.radians(20):
            status, severity = "failed", "error"
            msg = "窄弯前缺少导向轮组，张力将沿弯段放大且侧压力无约束。"
        elif assigned:
            g = assigned["obj"]
            gr = num(g.get("radius"), p["guideRadius"])
            count = max(1, int(num(g.get("count"), p["guideCount"])))
            distance_to_mid = abs(assigned["proj"]["station"] -
                                  (sec["startStation"] + sec["endStation"]) / 2)
            tolerance = sec["length"] / 2 + 3.0
            if distance_to_mid > tolerance:
                status, severity = "failed", "error"
                msg = "导向轮组距弯段 %.1fm，未在窄弯前有效导向。" % distance_to_mid
                resp, rtype = g["id"], "equipment"
            else:
                radius = max(sec["radius"], gr)
                mu = num(g.get("friction"), p["rollerFriction"])
                tension = t0 * math.exp(mu * theta)
                side = tension / radius
                radial = 2 * tension * math.sin(theta / 2)
                wheel_load = radial / count
                m.update({"tensionBefore": t0, "tension": tension,
                          "bendRadius": radius, "sidePressure": side,
                          "radialLoad": radial, "wheelLoad": wheel_load})
                react = update_stand(0, tension, m)
                if radius < p["minBendRadius"]:
                    status, severity = "failed", "error"
                    msg = "弯曲半径 %.2fm 小于电缆允许半径 %.2fm。" % (radius, p["minBendRadius"])
                    resp, rtype = turn_node["id"], "node"
                elif side > p["allowSidePressure"]:
                    status, severity = "failed", "error"
                    msg = "弯段侧压力 %.0fN/m 超过限值 %.0fN/m。" % (side, p["allowSidePressure"])
                    resp, rtype = g["id"], "equipment"
                elif wheel_load > num(g.get("capacity"), p["guideCapacity"]):
                    status, severity = "failed", "error"
                    msg = "单只导向轮受力 %.0fN 超过额定 %.0fN。" % (
                        wheel_load, num(g.get("capacity"), p["guideCapacity"]))
                    resp, rtype = g["id"], "equipment"
                else:
                    stand_msg = stand_overload(react)
                    if stand_msg:
                        status, severity = "failed", "error"
                        msg = stand_msg
                        resp, rtype = reel["id"], "equipment"
                    elif tension > p["allowTension"]:
                        status, severity = "failed", "error"
                        msg = "弯后累计张力 %.0fN 超过允许张力 %.0fN。" % (tension, p["allowTension"])
                        resp, rtype = g["id"], "equipment"
                    else:
                        msg = "弯段 θ=%.0f° R=%.2fm，导向轮×%d，张力 %.0fN，侧压 %.0fN/m。" % (
                            math.degrees(theta), radius, count, tension, side)
        else:
            tension = t0 * math.exp(p["friction"] * theta)
            side = tension / sec["radius"]
            m.update({"tensionBefore": t0, "tension": tension,
                      "sidePressure": side})
            react = update_stand(0, tension, m)
            stand_msg = stand_overload(react)
            if stand_msg:
                status, severity = "failed", "error"
                msg = stand_msg
                resp, rtype = reel["id"], "equipment"
            elif tension > p["allowTension"]:
                status, severity = "failed", "error"
                msg = "弯后累计张力 %.0fN 超过允许张力 %.0fN。" % (tension, p["allowTension"])
                resp, rtype = turn_node["id"], "node"
            else:
                msg = "小微弯未设导向轮，按摩擦 %.2f 复算。" % p["friction"]

        if status != "failed":
            metrics = m
        h = stable_hash([
            "pull-arc", sec["turn"], round(sec["radius"], 3), round(sec["sweep"], 5),
            assigned and {
                "id": assigned["obj"].get("id"),
                "x": assigned["obj"].get("x"), "y": assigned["obj"].get("y"),
                "radius": assigned["obj"].get("radius"),
                "count": assigned["obj"].get("count"),
            },
            p["cableDiameter"], p["cableWeight"], p["rollerFriction"],
            p["friction"],
        ])
        cp = make_checkpoint("pull:arc:%s" % sec["turn"], "pull",
                             "牵引转弯：%s" % turn_node["id"], sec["startStation"],
                             sec["endStation"] if status == "success" else sec["startStation"],
                             h, m, resp, rtype, msg, status, severity)
        return finish(cp, status == "failed")

    puller_events = [x for x in pullers_ahead]
    pi = 0
    for sec in sections:
        if sec["endStation"] <= reel_s:
            continue
        a = max(sec["startStation"], reel_s)
        b = sec["endStation"]
        while pi < len(puller_events) and puller_events[pi]["proj"]["station"] < b:
            pu = puller_events[pi]
            ps = pu["proj"]["station"]
            if ps <= a:
                pi += 1
                continue
            if pu["proj"]["sectionKind"] == "arc":
                m = json.loads(json.dumps(metrics))
                h = stable_hash(["puller-on-arc", pu["obj"].get("id")])
                cp = make_checkpoint("puller:%s" % pu["obj"]["id"], "pull", "牵引分段",
                                     ps, ps, h, m, pu["obj"]["id"], "equipment",
                                     "牵引机不能压在圆弧弯段上。", "failed", "error")
                stopped = finish(cp, True)
                if stopped:
                    return stopped
            if sec["kind"] == "straight":
                stopped = straight_span(sec, a, ps)
                if stopped:
                    return stopped
            a = ps

            obj = pu["obj"]
            cap = num(obj.get("capacity"), p["pullerCapacity"])
            m = json.loads(json.dumps(metrics))
            status, severity, msg = "success", "info", ""
            resp = obj["id"]
            if tension > cap:
                status, severity = "failed", "error"
                msg = "牵引分段张力 %.0fN 超过牵引机额定 %.0fN。" % (tension, cap)
            else:
                msg = "牵引分段达标：入口张力 %.0fN ≤ 额定 %.0fN；下段起始张力重置为 %.0fN。" % (
                    tension, cap, num(obj.get("backTension"), p["pullerBackTension"]))
            h = stable_hash(["puller", obj.get("id"), round(ps, 2),
                             obj.get("capacity"), obj.get("backTension")])
            cp = make_checkpoint("puller:%s" % obj.get("id"), "pull",
                                 "牵引分段：%s" % obj.get("id"), ps, ps, h, m,
                                 resp, "equipment", msg, status, severity)
            if status != "failed":
                tension = num(obj.get("backTension"), p["pullerBackTension"])
                metrics["tension"] = tension
                metrics["tensionBefore"] = tension
            stopped = finish(cp, status == "failed")
            if stopped:
                return stopped
            pi += 1

        if sec["kind"] == "straight":
            stopped = straight_span(sec, a, b)
        else:
            stopped = arc_span(sec, a, b)
        if stopped:
            return stopped

    # 4) 井口转向与竖井口
    last_sec = sections[-1]
    alpha = slope_alpha(last_sec["grade"])
    theta = max(0.0, math.pi / 2.0 - abs(alpha)) if alpha >= 0 else math.pi / 2.0 + abs(alpha)
    radius = p["shaftSheaveRadius"]
    t0 = tension
    m = json.loads(json.dumps(metrics))
    status, severity, msg = "success", "info", ""
    if radius < p["minBendRadius"]:
        status, severity = "failed", "error"
        msg = "井口导向弯半径 %.2fm 小于允许值 %.2fm。" % (radius, p["minBendRadius"])
    else:
        tension = t0 * math.exp(p["rollerFriction"] * theta)
        side = tension / radius
        radial = 2 * tension * math.sin(theta / 2)
        if p["shaftDepth"] > 0:
            tension += p["cableWeight"] * p["shaftDepth"]
        m.update({"tensionBefore": t0, "tension": tension, "bendRadius": radius,
                  "sidePressure": side, "radialLoad": radial, "length": p["shaftDepth"]})
        react = update_stand(alpha, tension, m)
        stand_msg = stand_overload(react)
        if side > p["allowSidePressure"]:
            status, severity = "failed", "error"
            msg = "井口侧压力 %.0fN/m 超限。" % side
        elif stand_msg:
            status, severity = "failed", "error"
            msg = stand_msg
        elif tension > p["allowTension"]:
            status, severity = "failed", "error"
            msg = "竖井终点张力 %.0fN 超过允许张力 %.0fN。" % (tension, p["allowTension"])
        else:
            msg = "井口导向 θ=%.0f° R=%.2fm，竖井深 %.1fm，最终张力 %.0fN。" % (
                math.degrees(theta), radius, p["shaftDepth"], tension)
    metrics = m
    h = stable_hash(["shaft", p["shaftSheaveRadius"], p["shaftDepth"],
                     p["cableWeight"], p["rollerFriction"], p["minBendRadius"],
                     round(last_sec["grade"], 3)])
    cp = make_checkpoint("shaft", "shaft", "竖井井口转向与下放",
                         total_station, total_station, h, m,
                         nodes[-1]["id"], "node", msg, status, severity)
    stopped = finish(cp, status == "failed")
    if stopped:
        return stopped

    return {"ok": True, "status": "passed", "failedIndex": None,
            "checkpoints": checkpoints, "summary": summary(),
            "warnings": warnings, "geometry": geometry_brief()}


# ----------------------------- 放盘—牵引瞬态联动 -----------------------------

# 冲突判定固定顺序：同一时刻多项越限时，回放停在排序最前者。
TRANSIENT_CONFLICTS = [
    ("tension", "拉力超标"),
    ("heat", "制动储热不足"),
    ("overspeed", "盘轴超速"),
    ("reserve", "余缆耗尽"),
    ("slack", "松弛圈超出净空"),
]


def get_transient_conf(raw, p):
    """从 scenario.params + transient 覆盖项取瞬态配置。"""
    raw = raw or {}

    def pick(key):
        return num(raw.get(key), p[key])

    return {
        "emptyInertia": pick("emptyInertia"),
        "bearingTorque": pick("bearingTorque"),
        "reserveCable": pick("reserveCable"),
        "axialStiffness": pick("axialStiffness"),
        "slackClearance": pick("slackClearance"),
        "brakeMaxSpeed": pick("brakeMaxSpeed"),
        "brakeHeatCapacity": pick("brakeHeatCapacity"),
        "windingPack": num(raw.get("brakeWindingPack"), p["brakeWindingPack"]),
        "startSpeed": num(raw.get("startSpeed"), p["transientStartSpeed"]),
        "duration": max(0.1, num(raw.get("duration"), p["transientDuration"])),
        "dt": clamp(num(raw.get("dt"), p["transientDt"]), 0.01, 1.0),
        "brakeCurve": normalize_curve(raw.get("brakeCurve"), DEFAULT_BRAKE_CURVE),
    }


def normalize_curve(value, default):
    """制动器指令—扭矩曲线：[[指令0~1, 扭矩N·m], ...]，按指令排序。"""
    pts = []
    for item in value or []:
        try:
            x, y = float(item[0]), float(item[1])
        except (TypeError, ValueError, IndexError):
            continue
        pts.append([clamp(x, 0.0, 1.5), max(0.0, y)])
    pts.sort(key=lambda q: q[0])
    return pts if len(pts) >= 2 else [list(q) for q in default]


def brake_torque_at(curve, cmd):
    """分段线性插值，超界取端点。"""
    if cmd <= curve[0][0]:
        return curve[0][1]
    for i in range(1, len(curve)):
        if cmd <= curve[i][0] or i == len(curve) - 1:
            x0, y0 = curve[i - 1]
            x1, y1 = curve[i]
            if x1 == x0:
                return y1
            f = clamp((cmd - x0) / (x1 - x0), 0.0, 1.0)
            return y0 + (y1 - y0) * f
    return curve[-1][1]


def wound_radius(reserve, conf, p):
    """余缆在筒体上的卷绕半径：r² = r筒² + L·d²/(4·装填系数)。"""
    barrel_r = max(p["barrelDiameter"] / 2.0, 0.05)
    d2 = p["cableDiameter"] ** 2
    r2 = barrel_r ** 2 + max(0.0, reserve) * d2 / (4.0 * max(conf["windingPack"], 0.05))
    return math.sqrt(r2)


def snap_grid(t, dt):
    return round(round(t / dt) * dt, 6)


def normalize_keyframes(kfs, dt, duration):
    out = []
    for raw in kfs or []:
        k = dict(raw or {})
        if k.get("type") not in ("speed", "jog", "estop"):
            k["type"] = "speed"
        t = snap_grid(clamp(num(k.get("time"), 0.0), 0.0, duration), dt)
        k["time"] = t
        k["speed"] = num(k.get("speed"), 0.0)
        b = k.get("brake")
        k["brake"] = None if b is None or b == "" else clamp(num(b), 0.0, 1.0)
        k["ramp"] = max(0.0, num(k.get("ramp"), -1.0))
        k["hold"] = max(0.0, num(k.get("hold"), 1.0))
        k["id"] = str(k.get("id") or ("kf-%d" % (len(out) + 1)))
        out.append(k)
    out.sort(key=lambda q: q["time"])
    return out


def build_commands(keyframes, conf):
    """把关键帧展开为指令速度折线点与制动阶跃点；时间吸附到积分网格。"""
    dt, duration = conf["dt"], conf["duration"]
    speed_pts = [(0.0, conf["startSpeed"], None)]
    brake_pts = []  # (t, 指令, 关键帧id)，阶跃保持

    def speed_now(t):
        return piecewise_linear(speed_pts, t)

    for k in keyframes:
        t0 = k["time"]
        if k["type"] == "speed":
            ramp = k["ramp"] if k["ramp"] >= 0 else 0.8
            t1 = snap_grid(min(duration, t0 + ramp), dt)
            v0 = speed_now(t0)
            if t0 > speed_pts[-1][0] + 1e-9:
                speed_pts.append((t0, v0, None))
            speed_pts.append((max(t0, t1), k["speed"], k["id"]))
            if k["brake"] is not None:
                brake_pts.append((t0, k["brake"], k["id"]))
        elif k["type"] == "jog":
            ramp = k["ramp"] if k["ramp"] >= 0 else 0.3
            t1 = snap_grid(min(duration, t0 + ramp), dt)
            t2 = snap_grid(min(duration, t1 + max(k["hold"], 0.05)), dt)
            t3 = snap_grid(min(duration, t2 + ramp), dt)
            v0 = speed_now(t0)
            if t0 > speed_pts[-1][0] + 1e-9:
                speed_pts.append((t0, v0, None))
            speed_pts.append((t1, k["speed"], k["id"]))       # 点动升速（斜坡）
            speed_pts.append((t2, k["speed"], k["id"]))       # 保持
            speed_pts.append((t3, v0, k["id"]))               # 回落（斜坡）
            if k["brake"] is not None:
                brake_pts.append((t0, k["brake"], k["id"]))
        else:  # 急停：指令速度在 ramp 内归零，制动指令置 100%（直至下一次换速/点动解除）
            ramp = k["ramp"] if k["ramp"] >= 0 else 0.25
            t1 = snap_grid(min(duration, t0 + ramp), dt)
            v0 = speed_now(t0)
            if t0 > speed_pts[-1][0] + 1e-9:
                speed_pts.append((t0, v0, None))
            speed_pts.append((max(t0, t1), 0.0, k["id"]))
            brake_pts.append((t0, 1.0, k["id"]))
            # 之后任何换速/点动关键帧视为复位急停，制动指令归零
            for later in keyframes:
                if later["time"] > t0 and later["type"] in ("speed", "jog"):
                    brake_pts.append((later["time"], 0.0, later["id"]))
                    break
    speed_pts.sort(key=lambda q: q[0])
    brake_pts.sort(key=lambda q: q[0])
    return speed_pts, brake_pts


def piecewise_linear(pts, t):
    if t <= pts[0][0]:
        return pts[0][1]
    for i in range(1, len(pts)):
        if t <= pts[i][0] or i == len(pts) - 1:
            t0, v0 = pts[i - 1][0], pts[i - 1][1]
            t1, v1 = pts[i][0], pts[i][1]
            if t1 <= t0:
                return v1
            f = clamp((t - t0) / (t1 - t0), 0.0, 1.0)
            return v0 + (v1 - v0) * f
    return pts[-1][1]


def brake_cmd_at(brake_pts, t):
    cmd = 0.0
    for bt, bc, _ in brake_pts:
        if bt <= t + 1e-9:
            cmd = bc
        else:
            break
    return cmd


def transient_geometry(scenario):
    """复用静力线形，求盘架、首台下游牵引机里程与轴向有效长度。"""
    nodes = scenario.get("nodes") or []
    equipment = scenario.get("equipment") or []
    out = {"reelStation": None, "pullerStation": None, "pullerId": None,
           "reelId": None, "totalStation": None}
    if len(nodes) < 2:
        return out
    try:
        sections, _, total, _ = build_alignment(nodes)
    except Exception:
        return out
    reel = next((e for e in equipment if e.get("type") == "reel"), None)
    if not reel:
        return out
    rp = project_point(sections, num(reel.get("x")), num(reel.get("y")))
    out.update({"reelStation": rp["station"], "reelId": reel.get("id"),
                "totalStation": total})
    ahead = []
    for e in equipment:
        if e.get("type") != "puller":
            continue
        pr = project_point(sections, num(e.get("x")), num(e.get("y")))
        if pr["station"] > rp["station"]:
            ahead.append((pr["station"], e.get("id")))
    if ahead:
        ahead.sort()
        out["pullerStation"], out["pullerId"] = ahead[0]
    return out


def build_intervals(keyframes, conf):
    """按关键帧时刻（含点动脉冲结束时刻）切分时间轴区间。"""
    dt, duration = conf["dt"], conf["duration"]
    bounds = {0.0, snap_grid(duration, dt)}
    for k in keyframes:
        bounds.add(k["time"])
        if k["type"] == "jog":
            ramp = k["ramp"] if k["ramp"] >= 0 else 0.3
            end = snap_grid(min(duration, k["time"] + 2 * ramp + max(k["hold"], 0.05)), dt)
            bounds.add(end)
    bounds = sorted(b for b in bounds if 0.0 - 1e-9 <= b <= duration + 1e-9)
    intervals = []
    for i in range(len(bounds) - 1):
        a, b = bounds[i], bounds[i + 1]
        if b - a < dt / 2 - 1e-9:
            continue
        kf = next((k for k in keyframes if abs(k["time"] - a) <= dt / 2), None)
        if kf:
            key, kid = kf["id"], kf["id"]
        elif b >= duration - 1e-9:
            prev = keyframes[-1] if keyframes else None
            key, kid = "tail", prev["id"] if prev else None
        else:
            nxt = next((k for k in keyframes if k["time"] >= b - 1e-9), None)
            key, kid = "hold:" + (nxt["id"] if nxt else "end"), nxt["id"] if nxt else None
        intervals.append({"index": len(intervals), "start": round(a, 3),
                          "end": round(b, 3), "key": key, "keyframeId": kid})
    return intervals


def transient_global_digest(conf, p, geom, keyframes):
    return [
        "transient-v1",
        round(conf["emptyInertia"], 4), round(conf["bearingTorque"], 4),
        round(conf["reserveCable"], 4), round(conf["axialStiffness"], 4),
        round(conf["slackClearance"], 4), round(conf["brakeMaxSpeed"], 5),
        round(conf["brakeHeatCapacity"], 2), round(conf["windingPack"], 4),
        round(conf["startSpeed"], 4), round(conf["duration"], 3), round(conf["dt"], 4),
        conf["brakeCurve"],
        round(p["cableDiameter"], 5), round(p["cableWeight"], 4),
        round(p["barrelDiameter"], 4), round(p["initialTension"], 3),
        round(p["allowTension"], 3),
        geom.get("reelStation"), geom.get("pullerStation"),
        [{"id": k["id"], "type": k["type"]} for k in keyframes],
    ]


def interval_hash(interval, keyframes, conf, p, geom):
    kf = next((k for k in keyframes if k["id"] == interval["keyframeId"]), None)
    payload = {
        "g": transient_global_digest(conf, p, geom, keyframes),
        "key": interval["key"], "from": interval["start"], "to": interval["end"],
        "kf": ({"id": kf["id"], "type": kf["type"], "time": kf["time"],
                "speed": kf["speed"], "brake": kf["brake"],
                "ramp": kf["ramp"], "hold": kf["hold"]} if kf else None),
    }
    return stable_hash(payload)


def integrate_transient(conf, p, geom, keyframes, speed_pts, brake_pts,
                        start_time=0.0, start_state=None):
    """半显式逐时积分；返回逐时行、冲突、结束状态。"""
    dt = conf["dt"]
    duration = conf["duration"]
    base_t = p["initialTension"]
    linear_mass = max(p["cableWeight"] / GRAVITY, 0.01)

    reel_station = geom.get("reelStation") or 0.0
    puller_station = geom.get("pullerStation")
    if puller_station is not None and puller_station > reel_station:
        span = max(puller_station - reel_station, 1.0)
    else:
        span = 10.0
    k_eff = max(conf["axialStiffness"] / span, 1.0)

    if start_state:
        st = {k: num(start_state.get(k), 0.0) for k in
              ("omega", "slack", "reserve", "tension", "heat", "energy", "paidOut")}
        st["reserve"] = num(start_state.get("reserve"), conf["reserveCable"])
    else:
        r0 = wound_radius(conf["reserveCable"], conf, p)
        omega0 = conf["startSpeed"] / max(r0, 1e-6)
        st = {"omega": omega0, "slack": 0.0, "reserve": conf["reserveCable"],
              "tension": base_t, "heat": 0.0, "energy": 0.0, "paidOut": 0.0}

    rows = []
    n_steps = int(round((duration - start_time) / dt))
    conflict = None

    for i in range(n_steps + 1):
        t = round(start_time + i * dt, 6)
        reserve = max(0.0, st["reserve"])
        radius = wound_radius(reserve, conf, p)
        v = st["omega"] * radius
        vcmd = piecewise_linear(speed_pts, t)
        brake_cmd = brake_cmd_at(brake_pts, t)
        brake_m = brake_torque_at(conf["brakeCurve"], brake_cmd)
        i_rot = conf["emptyInertia"] + linear_mass * reserve * radius ** 2
        resist = brake_m + conf["bearingTorque"]
        sgn = 1.0 if st["omega"] >= 0 else -1.0

        # 隐式向后欧拉联立，避免显式弹簧—惯量自激振荡：
        # ω' = ω + dt·(T'·r − sgn·M阻)/I
        # T' = T + k·dt·(vcmd − ω'·r)（绷紧时）
        denom = 1.0 + k_eff * dt * dt * radius ** 2 / i_rot
        stuck = abs(st["omega"]) < 1e-7 and vcmd <= 1e-9 \
            and st["tension"] * radius <= brake_m + conf["bearingTorque"] + 1e-9

        if stuck:
            # 静阻力矩锁止：盘轴不转不放线，张力保持，无瞬态附加张力。
            tension, alpha = st["tension"], 0.0
        elif st["slack"] > 1e-9:
            slack_new = st["slack"] - (vcmd - v) * dt
            if slack_new > 0.0:
                # 电缆松弛：盘轴不受缆张力驱动，仅轴承阻力/制动滑行。
                st["slack"] = slack_new
                tension = base_t
                alpha = -resist / i_rot
                st["omega"] = max(0.0, st["omega"] + alpha * dt)
            else:
                # 本时步内松弛耗尽、电缆重新绷紧；以基线张力起步，
                # 对剩余时长按隐式联立求解，dv 取本时步末真实速度差。
                st["slack"] = 0.0
                t_star = base_t + k_eff * dt * (vcmd - v) \
                    + k_eff * dt * dt * (base_t * radius - resist) * radius / i_rot
                tension = max(base_t, t_star / denom)
                alpha = (tension * radius - resist) / i_rot
                st["omega"] = max(0.0, st["omega"] + alpha * dt)
        else:
            t_star = st["tension"] + k_eff * dt * (vcmd - v) \
                + k_eff * dt * dt * (st["tension"] * radius - resist) * radius / i_rot
            tension = t_star / denom
            if tension < base_t:
                st["slack"] += (base_t - tension) / k_eff
                tension = base_t
            alpha = (tension * radius - resist) / i_rot
            st["omega"] = max(0.0, st["omega"] + alpha * dt)
        inertia_force = i_rot * alpha / max(radius, 1e-6)
        if stuck or abs(st["omega"]) < 1e-7:
            inertia_force = 0.0  # 静止锁止时不展示惯性附加张力
        v_after = st["omega"] * radius

        rows.append({
            "t": round(t, 3),
            "radius": round(radius, 4),
            "omega": round(st["omega"], 4),
            "rpm": round(abs(st["omega"]) * 60.0 / (2 * math.pi), 1),
            "vCmd": round(vcmd, 4),
            "vReel": round(v_after, 4),
            "dv": round(vcmd - v_after, 4),
            "tension": round(tension, 1),
            "inertiaTension": round(max(0.0, inertia_force), 1),
            "brakeCmd": round(brake_cmd, 3),
            "brakeTorque": round(brake_m, 1),
            "brakeEnergy": round(st["energy"], 1),
            "brakeHeat": round(st["heat"], 1),
            "heatRatio": round(st["heat"] / max(conf["brakeHeatCapacity"], 1.0), 3),            "slack": round(st["slack"], 3),
            "reserve": round(reserve, 3),
            "paidOut": round(st["paidOut"], 3),
            "station": round(reel_station + max(0.0, st["paidOut"]), 3),
        })
        st["tension"] = tension

        if i < n_steps:
            st["energy"] += brake_m * abs(st["omega"]) * dt
            st["heat"] = st["energy"]  # 无冷却假设：制动耗能全部储热（偏安全）
            st["paidOut"] += st["omega"] * radius * dt
            st["reserve"] = conf["reserveCable"] - st["paidOut"]

        checks = [
            ("tension", tension > p["allowTension"], "reel",
             "瞬时张力 %.0fN 超过允许张力 %.0fN（放线跟不上，惯性附加张力 %.0fN）。" % (
                 tension, p["allowTension"], max(0.0, inertia_force))),
            ("heat", st["heat"] > conf["brakeHeatCapacity"], "reel",
             "制动器累计储热 %.0fJ 超过热容量 %.0fJ（%.0f%%）。" % (
                 st["heat"], conf["brakeHeatCapacity"],
                 100.0 * st["heat"] / max(conf["brakeHeatCapacity"], 1.0))),
            ("overspeed", abs(st["omega"]) > conf["brakeMaxSpeed"], "reel",
             "盘轴转速 %.2frad/s（%.0fr/min）超过制动器转速上限 %.2frad/s。" % (
                 st["omega"], abs(st["omega"]) * 60.0 / (2 * math.pi),
                 conf["brakeMaxSpeed"])),
            ("reserve", reserve <= 1e-6 and vcmd > 1e-6, "reel",
             "初始余缆 %.1fm 已在 t=%.1fs 耗尽，牵引机仍在收线。" % (
                 conf["reserveCable"], t)),
            ("slack", st["slack"] > conf["slackClearance"], "reel",
             "盘轴惯性甩出松弛圈 %.2fm，超出盘前净空允许 %.2fm。" % (
                 st["slack"], conf["slackClearance"])),
        ]
        hit = next((c for c in checks if c[1]), None)
        if hit:
            kind, _, who, message = hit
            conflict = {
                "kind": kind, "title": dict(TRANSIENT_CONFLICTS)[kind],
                "time": round(t, 3), "message": message,
                "responsibleId": geom.get("reelId") if who == "reel" else geom.get("pullerId"),
                "responsibleType": "equipment",
                "pullerId": geom.get("pullerId"),
                "reelId": geom.get("reelId"),
                "station": rows[-1]["station"],
                "metrics": rows[-1],
            }
            break

    end_state = {"t": rows[-1]["t"], "omega": st["omega"], "slack": st["slack"],
                 "reserve": max(0.0, st["reserve"]), "tension": st["tension"],
                 "heat": st["heat"], "energy": st["energy"], "paidOut": st["paidOut"]}
    return rows, conflict, end_state


def simulate_transient(data):
    scenario = data.get("scenario") or {}
    tr = data.get("transient") or {}
    p = get_params(scenario)
    conf = get_transient_conf(tr, p)
    keyframes = normalize_keyframes(tr.get("keyframes"), conf["dt"], conf["duration"])
    if conf["emptyInertia"] <= 0:
        return {"ok": False, "error": "空盘惯量须大于 0。"}
    if conf["axialStiffness"] <= 0:
        return {"ok": False, "error": "电缆轴向刚度 EA 须大于 0。"}
    geom = transient_geometry(scenario)

    speed_pts, brake_pts = build_commands(keyframes, conf)
    intervals = build_intervals(keyframes, conf)
    for iv in intervals:
        iv["hash"] = interval_hash(iv, keyframes, conf, p, geom)
    hashes = [iv["hash"] for iv in intervals]

    locked_count = int(clamp(num(data.get("lockedCount"), 0), 0, len(intervals)))
    verified = data.get("verifiedHashes") or []
    start_state = data.get("startState") or None

    for idx in range(locked_count):
        if idx >= len(verified) or verified[idx] != hashes[idx]:
            ivs = [dict(j, status=("locked" if k < idx else "pending"))
                   for k, j in enumerate(intervals)]
            return {"ok": True, "status": "lock-changed",
                    "intervals": ivs, "lockIntervalIndex": idx,
                    "error": "已确认时段 %d 的输入（关键帧或制动参数）发生变化，请解锁后重算。" % (idx + 1)}

    if locked_count > 0 and start_state and locked_count < len(intervals):
        start_time = intervals[locked_count]["start"]
        if abs(num(start_state.get("t"), -1) - start_time) > conf["dt"] * 1.5:
            start_time, start_state = 0.0, None
    else:
        start_time, start_state = 0.0, None

    rows, conflict, end_state = integrate_transient(
        conf, p, geom, keyframes, speed_pts, brake_pts, start_time, start_state)

    computed_from = 0 if start_state is None else locked_count
    failed_index = None
    for j, iv in enumerate(intervals):
        if j < locked_count:
            iv["status"] = "locked"
        elif conflict is not None and conflict["time"] >= iv["start"] - 1e-9 \
                and conflict["time"] <= iv["end"] + 1e-9:
            iv["status"], failed_index = "failed", j
        elif conflict is None or conflict["time"] > iv["end"] + 1e-9:
            iv["status"] = "success"
        else:
            iv["status"] = "pending"

    if conflict is not None:
        status, new_locked = "blocked", failed_index
    else:
        status, new_locked = "passed", len(intervals)

    summary = {
        "duration": conf["duration"], "dt": conf["dt"],
        "maxTension": round(max(r["tension"] for r in rows), 1),
        "allowTension": p["allowTension"],
        "maxOmega": round(max(abs(r["omega"]) for r in rows), 3),
        "omegaLimit": conf["brakeMaxSpeed"],
        "maxRpm": round(max(r["rpm"] for r in rows), 1),
        "rpmLimit": round(conf["brakeMaxSpeed"] * 60.0 / (2 * math.pi), 1),
        "maxHeatRatio": round(max(r["heatRatio"] for r in rows), 3),
        "maxSlack": round(max(r["slack"] for r in rows), 3),
        "slackClearance": conf["slackClearance"],
        "brakeEnergy": rows[-1]["brakeEnergy"],
        "brakeHeatCapacity": conf["brakeHeatCapacity"],
        "reserveLeft": rows[-1]["reserve"],
    }
    return {
        "ok": True, "status": status, "rows": rows, "intervals": intervals,
        "hashes": hashes, "lockedCount": new_locked,
        "verifiedHashes": hashes[:new_locked],
        "failedInterval": failed_index, "conflict": conflict,
        "startState": end_state if status == "blocked" else None,
        "endState": end_state, "fromInterval": computed_from, "fromTime": rows[0]["t"],
        "keyframes": keyframes, "brakeCurve": conf["brakeCurve"],
        "config": {
            "emptyInertia": conf["emptyInertia"], "bearingTorque": conf["bearingTorque"],
            "reserveCable": conf["reserveCable"], "axialStiffness": conf["axialStiffness"],
            "slackClearance": conf["slackClearance"], "brakeMaxSpeed": conf["brakeMaxSpeed"],
            "brakeHeatCapacity": conf["brakeHeatCapacity"],
            "windingPack": conf["windingPack"],
            "startSpeed": conf["startSpeed"], "duration": conf["duration"], "dt": conf["dt"],
        },
        "geometry": geom, "summary": summary,
    }


# ----------------------------- 归档输出 -----------------------------

def init_db():
    conn = sqlite3.connect(DB_PATH)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS archives (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            created_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
            name TEXT NOT NULL,
            note TEXT NOT NULL DEFAULT '',
            scenario_json TEXT NOT NULL,
            result_json TEXT NOT NULL
        )
    """)
    existing = {r[1] for r in conn.execute("PRAGMA table_info(archives)").fetchall()}
    for column, decl in (("transient_json", "TEXT"), ("transient_result_json", "TEXT")):
        if column not in existing:
            conn.execute("ALTER TABLE archives ADD COLUMN %s %s" % (column, decl))
    conn.commit()
    conn.close()


def checkpoint_at(result, station):
    for c in result.get("checkpoints", []):
        if c["startStation"] - 1e-6 <= station <= c["endStation"] + 1e-6:
            return c
    if result.get("checkpoints"):
        if station < result["checkpoints"][0]["startStation"]:
            return result["checkpoints"][0]
        return result["checkpoints"][-1]
    return None


def status_color(cp):
    if not cp:
        return "#94a3b8"
    if cp["status"] == "locked":
        return "#16a34a"
    if cp["status"] == "failed":
        return "#dc2626"
    if cp["status"] == "success":
        return "#2563eb"
    return "#94a3b8"


def svg_text(x, y, text, size=0.42, fill="#0f172a", anchor="start", weight="normal"):
    return ('<text x="%.2f" y="%.2f" font-size="%.2f" fill="%s" '
            'text-anchor="%s" font-weight="%s">%s</text>') % (
        x, y, size, fill, anchor, weight, escape_xml(text))


def escape_xml(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def build_svg(scenario, result, archive=None):
    nodes = scenario.get("nodes", [])
    walls = scenario.get("walls", [])
    equipment = scenario.get("equipment", [])
    sections, node_station, total_len, _ = build_alignment(nodes)
    xs = [n["x"] for n in nodes]
    ys = [n["y"] for n in nodes]
    minx, maxx = max(0, min(xs) - 2), max(xs) + 2
    miny, maxy = max(0, min(ys) - 2), max(ys) + 2
    width = maxx - minx
    height = (maxy - miny) + 10.0
    name = (archive or {}).get("name", scenario.get("name", "放缆推演"))
    note = (archive or {}).get("note", "")

    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="%.2f %.2f %.2f %.2f" '
             'font-family="Arial, &quot;Microsoft YaHei&quot;, sans-serif">'
             % (minx, miny, width, height)]
    parts.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="#f8fafc"/>'
                 % (minx, miny, width, height))

    # 底图中心线
    for sec in sections:
        if sec["kind"] == "straight":
            parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" '
                         'stroke="#cbd5e1" stroke-width="0.18" stroke-dasharray="0.4 0.35"/>'
                         % (sec["start"][0], sec["start"][1], sec["end"][0], sec["end"][1]))
        else:
            sweep = 1 if sec["sweep"] > 0 else 0
            parts.append('<path d="M %.2f %.2f A %.2f %.2f 0 0 %d %.2f %.2f" fill="none" '
                         'stroke="#cbd5e1" stroke-width="0.18" stroke-dasharray="0.4 0.35"/>'
                         % (sec["start"][0], sec["start"][1], sec["radius"], sec["radius"],
                            sweep, sec["end"][0], sec["end"][1]))

    # 按推演状态标色路径
    if total_len > 0:
        step = 0.20
        s = 0.0
        while s < total_len:
            q1, q2 = point_at(sections, s), point_at(sections, min(s + step, total_len))
            cp = checkpoint_at(result, s)
            color = status_color(cp)
            parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" '
                         'stroke="%s" stroke-width="0.34" stroke-linecap="round"/>'
                         % (q1["x"], q1["y"], q2["x"], q2["y"], color))
            s += step

    # 墙体
    for w in walls:
        resp = result.get("checkpoints", [{}])[-1]
        stroke = "#dc2626" if any(c.get("responsibleId") == w.get("id") and c.get("status") == "failed"
                                  for c in result.get("checkpoints", [])) else "#111827"
        parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" '
                     'stroke-width="0.28" stroke-linecap="round"/>'
                     % (w["x1"], w["y1"], w["x2"], w["y2"], stroke))

    # 节点与构造物
    for i, n in enumerate(nodes):
        x, y = n["x"], n["y"]
        ttype = n.get("type")
        if ttype == "start":
            parts.append('<circle cx="%.2f" cy="%.2f" r="0.42" fill="#22c55e"/>' % (x, y))
            parts.append(svg_text(x, y - 0.62, "起点", 0.42, "#166534", "middle", "bold"))
        elif ttype == "shaft":
            parts.append('<circle cx="%.2f" cy="%.2f" r="0.55" fill="none" stroke="#7c3aed" stroke-width="0.18"/>' % (x, y))
            parts.append('<circle cx="%.2f" cy="%.2f" r="0.22" fill="#7c3aed"/>' % (x, y))
            parts.append(svg_text(x, y - 0.75, "竖井井口", 0.42, "#5b21b6", "middle", "bold"))
        elif ttype == "turn":
            failed = any(c.get("responsibleId") == n.get("id") and c.get("status") == "failed"
                         for c in result.get("checkpoints", []))
            color = "#dc2626" if failed else "#0ea5e9"
            parts.append('<rect x="%.2f" y="%.2f" width="0.7" height="0.7" '
                         'transform="rotate(45 %.2f %.2f)" fill="%s"/>'
                         % (x - 0.35, y - 0.35, x, y, color))
            parts.append(svg_text(x + 0.55, y + 0.18, "R%.2f" % num(n.get("radius"), 0), 0.36, color))
        elif ttype == "gate":
            prev_n = nodes[i - 1] if i > 0 else n
            next_n = nodes[i + 1] if i + 1 < len(nodes) else n
            u = unit((prev_n["x"], prev_n["y"]), (next_n["x"], next_n["y"]))
            nv = (-u[1], u[0])
            ow = num(n.get("openingWidth"), 2.0) / 2
            for sign in (-1, 1):
                x1 = x + nv[0] * sign * ow
                y1 = y + nv[1] * sign * ow
                x2 = x + nv[0] * sign * (ow + 1.4)
                y2 = y + nv[1] * sign * (ow + 1.4)
                parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" '
                             'stroke="#475569" stroke-width="0.34" stroke-linecap="round"/>'
                             % (x1, y1, x2, y2))
            parts.append(svg_text(x, y - 0.62, "门洞 %.2f×%.2f" % (
                num(n.get("openingWidth")), num(n.get("openingHeight"))), 0.34, "#334155", "middle"))
        else:
            parts.append('<circle cx="%.2f" cy="%.2f" r="0.22" fill="#64748b"/>' % (x, y))

    # 设备
    for e in equipment:
        x, y = num(e.get("x")), num(e.get("y"))
        failed = any(c.get("responsibleId") == e.get("id") and c.get("status") == "failed"
                     for c in result.get("checkpoints", []))
        color = "#dc2626" if failed else "#f97316"
        if e.get("type") == "reel":
            ang = num(e.get("payoutAngle"))
            parts.append('<g transform="translate(%.2f %.2f) rotate(%.2f)">'
                         '<rect x="-1.05" y="-0.55" width="2.1" height="1.1" rx="0.12" '
                         'fill="#ffedd5" stroke="%s" stroke-width="0.18"/>'
                         '<line x1="-0.55" y1="-0.55" x2="-0.55" y2="0.55" stroke="%s" stroke-width="0.12"/>'
                         '<line x1="0.55" y1="-0.55" x2="0.55" y2="0.55" stroke="%s" stroke-width="0.12"/>'
                         '</g>' % (x, y, ang, color, color, color))
            parts.append(svg_text(x, y - 0.9, "盘架", 0.38, color, "middle", "bold"))
        elif e.get("type") == "puller":
            parts.append('<rect x="-0.55" y="-0.38" width="1.1" height="0.76" rx="0.08" '
                         'fill="#dbeafe" stroke="%s" stroke-width="0.18"/>' % color)
            parts.append(svg_text(x, y + 0.14, "牵", 0.42, color, "middle", "bold"))
        elif e.get("type") == "guide":
            parts.append('<circle cx="%.2f" cy="%.2f" r="0.38" fill="#dcfce7" stroke="%s" stroke-width="0.16"/>'
                         % (x, y, color))
            parts.append(svg_text(x, y + 0.14, "导", 0.32, color, "middle", "bold"))

    # 纵剖面小图
    prof_y = maxy + 3.0
    zs = [num(n.get("z")) for n in nodes] or [0]
    zmin, zmax = min(zs), max(zs)
    if zmax - zmin < 4:
        zmid = (zmin + zmax) / 2
        zmin, zmax = zmid - 2, zmid + 2
    scale_y = 3.2 / max(zmax - zmin, 0.1)
    base_y = prof_y + 2.0

    def py(z):
        return base_y - (z - zmin) * scale_y

    parts.append(svg_text(minx, prof_y - 0.55, "纵剖面（里程—高程）", 0.5, "#0f172a", "start", "bold"))
    parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#94a3b8" stroke-width="0.12"/>'
                 % (minx + 0.5, base_y, maxx - 0.5, base_y))
    prev = None
    for n in nodes:
        st = node_station.get(n["id"], 0)
        px = minx + 0.8 + (total_len and st / total_len) * (width - 1.6)
        q = (px, py(num(n.get("z"))))
        if prev:
            cp = checkpoint_at(result, st)
            parts.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="0.28"/>'
                         % (prev[0], prev[1], q[0], q[1], status_color(cp)))
        prev = q
        parts.append('<circle cx="%.2f" cy="%.2f" r="0.16" fill="#334155"/>' % q)

    title = "标色放缆路径 - %s" % escape_xml(name)
    parts.append(svg_text(minx, miny - 0.55, title, 0.62, "#0f172a", "start", "bold"))
    if note:
        parts.append(svg_text(minx, miny - 0.02, "人工说明：%s" % note[:80], 0.36, "#475569"))
    parts.append(svg_text(maxx, maxy + 1.2,
                          "■ 已核实(绿)  ■ 未播放/待算(灰)  ■ 当前算通(蓝)  ■ 首次受阻(红)",
                          0.36, "#475569", "end"))
    parts.append("</svg>")
    return "\n".join(parts)


def build_steps(scenario, result, archive):
    p = get_params(scenario)
    lines = []
    lines.append("# 放缆步骤单")
    lines.append("")
    lines.append("- 版本：%s" % archive.get("name", ""))
    lines.append("- 归档时间：%s" % archive.get("created_at", ""))
    lines.append("- 推演结论：%s" % {"passed": "通过", "blocked": "受阻停止",
                                    "lock-changed": "锁定输入变化"}.get(result.get("status"), result.get("status")))
    lines.append("- 人工说明：%s" % (archive.get("note") or "无"))
    lines.append("")
    lines.append("## 主要输入")
    lines.append("")
    lines.append("| 项目 | 数值 | 项目 | 数值 |")
    lines.append("|---|---:|---|---:|")
    lines.append("| 盘体直径 | %.2f m | 盘宽 | %.2f m |" % (p["reelDiameter"], p["reelWidth"]))
    lines.append("| 盘体总重 | %.0f N | 电缆单位重 | %.1f N/m |" % (p["totalWeight"], p["cableWeight"]))
    lines.append("| 电缆外径 | %.3f m | 摩擦系数 | %.2f |" % (p["cableDiameter"], p["friction"]))
    lines.append("| 允许张力 | %.0f N | 允许侧压力 | %.0f N/m |" % (p["allowTension"], p["allowSidePressure"]))
    lines.append("| 最小弯曲半径 | %.2f m | 竖井深度 | %.1f m |" % (p["minBendRadius"], p["shaftDepth"]))
    lines.append("")
    lines.append("## 逐段演算结果")
    lines.append("")
    lines.append("| # | 阶段 | 区段 | 里程(m) | 张力(N) | 半径(m) | 侧压(N/m) | 盘架反力(N) | 制动力(N) | 状态 | 说明 |")
    lines.append("|---:|---|---|---:|---:|---:|---:|---:|---:|---|---|")
    for i, c in enumerate(result.get("checkpoints", []), 1):
        m = c["metrics"]
        state = {"locked": "锁定", "success": "通过", "failed": "受阻"}.get(c["status"], c["status"])
        msg = c["message"].replace("|", "／")
        lines.append("| %d | %s | %s | %.2f–%.2f | %.0f | %s | %s | %.0f | %.0f | %s | %s |" % (
            i, {"roll": "滚运", "setup": "架设", "pull": "牵引", "shaft": "井口"}.get(c["phase"], c["phase"]),
            c["title"].replace("|", "／"), c["startStation"], c["endStation"],
            m.get("tension") or 0,
            "%.2f" % m["bendRadius"] if m.get("bendRadius") is not None else "-",
            "%.0f" % m["sidePressure"] if m.get("sidePressure") is not None else "-",
            m.get("standReaction") or 0, m.get("brakeForce") or 0, state, msg))
    lines.append("")
    if result.get("status") == "blocked":
        c = result["checkpoints"][result["failedIndex"]]
        lines.append("## 首次受阻处")
        lines.append("")
        lines.append("- 责任对象：`%s`（%s）" % (c.get("responsibleId"), c.get("responsibleType")))
        lines.append("- 停止里程：%.2f m" % c["endStation"])
        lines.append("- 原因：%s" % c["message"])
        lines.append("")
        lines.append("请就地调整盘架出线方向、导向轮位置/半径/数量或牵引分段；已核实区段保持锁定后继续演算。")
    else:
        lines.append("## 实施步骤提示")
        lines.append("")
        lines.append("1. 核对门洞净空、坡道制动和通道围挡，按滚运里程慢速放盘。")
        lines.append("2. 盘架就位后按推演角度锁定出线方向，复核支腿、锚固和盘体转动包络。")
        lines.append("3. 每个窄弯前安装导向轮组，先点动牵引再连续放缆。")
        lines.append("4. 牵引机分段处设联络和急停，到达井口后降低速度，核对井口导向与竖井张力。")
    lines.append("")
    lines.append("> 本步骤单由后端 Python 模型复算生成；现场实施前仍应由技术负责人核对。")
    return "\n".join(lines)


def build_recalc_json(scenario, result):
    p = get_params(scenario)
    payload = {
        "schemaVersion": 1,
        "generator": "wsgiref + json + sqlite3 cable rigging model",
        "units": {"length": "m", "force": "N", "sidePressure": "N/m", "massWeight": "N", "angle": "degree"},
        "formulas": {
            "straightTension": "T2 = T1 + w*L/slopeLengthFactor*(mu*cos(alpha)+sin(alpha))",
            "bendTension": "T2 = T1*exp(mu*abs(theta))",
            "sidePressure": "P = T/R",
            "radialLoad": "F = 2*T*sin(theta/2)",
            "rampBrake": "Fb=max(0,W*(sin(abs(alpha))-rollingResistance*cos(alpha)))",
            "standReaction": "N=W*cos(alpha); R=N/2 +/- (T*cos(alpha)+W*sin(alpha))*R_reel/base"
        },
        "params": p,
        "scenario": scenario,
        "result": result,
    }
    return json.dumps(payload, ensure_ascii=False, indent=2)


def build_transient_csv(transient, tr, archive=None):
    """逐时 CSV：输入指令曲线 + 计算值 + 人工说明。"""
    cfg = tr.get("config") or {}
    rows = tr.get("rows") or []
    headers = ["time_s", "cmd_speed_mps", "reel_speed_mps", "speed_gap_mps",
               "wind_radius_m", "shaft_omega_radps", "shaft_rpm",
               "cable_tension_N", "inertia_tension_N",
               "brake_cmd", "brake_torque_Nm", "brake_energy_J", "brake_heat_J",
               "slack_loop_m", "reserve_cable_m", "line_station_m"]
    lines = ["# 放盘—牵引联动逐时复算"]
    lines.append("# 版本：%s" % (archive or {}).get("name", ""))
    lines.append("# 人工说明：%s" % ((archive or {}).get("note") or "无"))
    lines.append("# 输入：空盘惯量=%.1f kg·m²；轴承阻力矩=%.1f N·m；初始余缆=%.1f m；"
                 "轴向刚度EA=%.0f N；制动热容量=%.0f J；制动转速上限=%.2f rad/s；净空=%.2f m"
                 % (cfg.get("emptyInertia", 0), cfg.get("bearingTorque", 0),
                    cfg.get("reserveCable", 0), cfg.get("axialStiffness", 0),
                    cfg.get("brakeHeatCapacity", 0), cfg.get("brakeMaxSpeed", 0),
                    cfg.get("slackClearance", 0)))
    lines.append("# 输入指令—扭矩曲线：" + ";".join(
        "%.2f→%.0fN·m" % (q[0], q[1]) for q in (tr.get("brakeCurve") or [])))
    lines.append("# 关键帧：" + ";".join(
        "%s@%.1fs v=%.2f" % (k.get("type"), k.get("time", 0), k.get("speed", 0))
        for k in (tr.get("keyframes") or [])))
    c = tr.get("conflict")
    if c:
        lines.append("# 最早冲突：t=%.2fs %s；%s" % (
            c["time"], c["title"], c["message"]))
    lines.append(",".join(headers))
    for r in rows:
        lines.append("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.3f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f" % (
            r["t"], r["vCmd"], r["vReel"], r["dv"], r["radius"], r["omega"], r["rpm"],
            r["tension"], r["inertiaTension"], r["brakeCmd"], r["brakeTorque"],
            r["brakeEnergy"], r["brakeHeat"], r["slack"], r["reserve"], r["station"]))
    return "\n".join(lines) + "\n"


def build_transient_svg(transient, tr, archive=None):
    """瞬态曲线 SVG：速度/转速/张力/制动/松弛五条曲线 + 关键帧与冲突标记。"""
    rows = tr.get("rows") or []
    width = 980
    panels = [
        ("速度 m/s", [("vCmd", "#475569", "指令速度"), ("vReel", "#2563eb", "盘轴线速度")], None),
        ("盘轴转速 r/min", [("rpm", "#7c3aed", "转速")],
         (tr.get("summary") or {}).get("omegaLimit", 0) * 60 / (2 * math.pi)),
        ("张力 N", [("tension", "#f97316", "电缆张力"),
                   ("inertiaTension", "#dc2626", "惯性附加张力")],
         (tr.get("summary") or {}).get("allowTension")),
        ("制动 J / 指令", [("brakeHeat", "#dc2626", "累计储热")],
         (tr.get("config") or {}).get("brakeHeatCapacity")),
        ("松弛圈 m", [("slack", "#16a34a", "松弛长度")],
         (tr.get("config") or {}).get("slackClearance")),
    ]
    ph = 150
    height = 60 + ph * len(panels) + 70
    x0, pw = 70, 760
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
             'font-family="Arial, &quot;Microsoft YaHei&quot;, sans-serif">'
             % (width, height, width, height)]
    parts.append('<rect width="%d" height="%d" fill="#f8fafc"/>' % (width, height))
    name = escape_xml((archive or {}).get("name") or "放盘—牵引联动瞬态复算")
    parts.append('<text x="20" y="28" font-size="18" font-weight="bold">%s</text>' % name)
    note = (archive or {}).get("note") or ""
    if note:
        parts.append('<text x="20" y="48" font-size="11" fill="#475569">人工说明：%s</text>'
                     % escape_xml(note[:90]))
    t_end = (tr.get("config") or {}).get("duration") or (rows[-1]["t"] if rows else 1)
    conflict = tr.get("conflict")

    for pi, (title, series, limit) in enumerate(panels):
        y0 = 70 + pi * ph
        parts.append('<rect x="%d" y="%d" width="%d" height="%d" fill="#fff" stroke="#e2e8f0"/>'
                     % (x0, y0, pw, ph - 18))
        parts.append('<text x="14" y="%d" font-size="12" font-weight="bold">%s</text>'
                     % (y0 + 16, title))
        for gi in range(5):
            gy = y0 + (ph - 18) * gi / 4
            parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#f1f5f9"/>'
                         % (x0, gy, x0 + pw, gy))
        allvals = [r[k] for r in rows for k, _, _ in series]
        if limit is not None:
            allvals.append(limit)
        vmin, vmax = min(allvals or [0]), max(allvals or [1])
        if abs(vmax - vmin) < 1e-9:
            vmax += 1.0
        pad = (vmax - vmin) * 0.08
        vmin, vmax = vmin - pad, vmax + pad

        def xy(r, key):
            x = x0 + pw * r["t"] / max(t_end, 1e-9)
            y = y0 + (ph - 18) * (1 - (r[key] - vmin) / (vmax - vmin))
            return x, y

        for key, color, _label in series:
            pts = " ".join("%.1f,%.1f" % xy(r, key) for r in rows)
            parts.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="1.6"/>'
                         % (pts, color))
        if limit is not None:
            ly = y0 + (ph - 18) * (1 - (limit - vmin) / (vmax - vmin))
            parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" '
                         'stroke="#dc2626" stroke-width="1.1" stroke-dasharray="5 4"/>'
                         % (x0, ly, x0 + pw, ly))
            parts.append('<text x="%d" y="%.1f" font-size="10" fill="#dc2626">限值 %.3g</text>'
                         % (x0 + pw - 64, ly - 3, limit))
        for kf in (tr.get("keyframes") or []):
            kx = x0 + pw * kf["time"] / max(t_end, 1e-9)
            col = {"jog": "#0ea5e9", "speed": "#2563eb", "estop": "#dc2626"}.get(kf["type"], "#64748b")
            parts.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s" '
                         'stroke-width="1" stroke-dasharray="2 3"/>'
                         % (kx, y0, kx, y0 + ph - 18, col))
        if conflict:
            cx = x0 + pw * conflict["time"] / max(t_end, 1e-9)
            parts.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#dc2626" stroke-width="1.6"/>'
                         % (cx, y0, cx, y0 + ph - 18))
        lx = x0 + 8
        for key, color, label in series:
            parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="2.5"/>'
                         % (lx, y0 + ph - 26, lx + 16, y0 + ph - 26, color))
            parts.append('<text x="%d" y="%d" font-size="11" fill="#334155">%s</text>'
                         % (lx + 20, y0 + ph - 22, label))
            lx += 24 + len(label) * 12

    axis_y = 70 + ph * len(panels) - 8
    for s in range(11):
        tt = t_end * s / 10
        xx = x0 + pw * s / 10
        parts.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#94a3b8"/>'
                     % (xx, axis_y - 6, xx, axis_y))
        parts.append('<text x="%.1f" y="%d" font-size="10" fill="#64748b" text-anchor="middle">%.0fs</text>'
                     % (xx, axis_y + 12, tt))
    legend_y = axis_y + 34
    for i, (ktype, c, label) in enumerate((("jog", "#0ea5e9", "点动"),
                                            ("speed", "#2563eb", "换速"),
                                            ("estop", "#dc2626", "急停"))):
        lx = 20 + i * 120
        parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="2"/>'
                     % (lx, legend_y, lx + 20, legend_y, c))
        parts.append('<text x="%d" y="%d" font-size="11">%s</text>' % (lx + 25, legend_y + 4, label))
    if conflict:
        parts.append('<text x="380" y="%d" font-size="12" fill="#dc2626" font-weight="bold">'
                     '最早冲突 t=%.2fs：%s</text>'
                     % (legend_y, conflict["time"], escape_xml(conflict["title"])))
    parts.append("</svg>")
    return "\n".join(parts)


def build_transient_recalc_json(scenario, transient, tr, archive=None):
    p = get_params(scenario)
    payload = {
        "schemaVersion": 1,
        "generator": "wsgiref + json + sqlite3 cable rigging transient model",
        "archive": {"name": (archive or {}).get("name"), "note": (archive or {}).get("note"),
                    "createdAt": (archive or {}).get("created_at")},
        "units": {"length": "m", "time": "s", "angle": "rad", "force": "N",
                  "torque": "N·m", "energy": "J", "stiffness": "N"},
        "formulas": {
            "windRadius": "r = sqrt(r_barrel^2 + L_reserve*d_cable^2/(4*pack))",
            "slackTension": "slack>0 → T=T0; else dT = EA/L_eff*(v_pull-v_reel)*dt; T<T0 → 转入松弛",
            "reelRotation": "(I0 + m/L*L_reserve*r^2)*dω = T*r - M_brake(cmd) - M_bearing",
            "inertiaTension": "dT_inertia = I*alpha/r",
            "brakeHeat": "E += M_brake*|ω|*dt（无冷却假设，全部储热）",
            "reserve": "L_reserve = L0 - ∫ω*r dt",
            "slackLoop": "s += (v_reel-v_pull)*dt（盘轴甩出松弛）",
        },
        "input": {
            "params": p,
            "emptyInertia": (tr.get("config") or {}).get("emptyInertia"),
            "bearingTorque": (tr.get("config") or {}).get("bearingTorque"),
            "reserveCable": (tr.get("config") or {}).get("reserveCable"),
            "axialStiffness": (tr.get("config") or {}).get("axialStiffness"),
            "slackClearance": (tr.get("config") or {}).get("slackClearance"),
            "brakeMaxSpeed": (tr.get("config") or {}).get("brakeMaxSpeed"),
            "brakeHeatCapacity": (tr.get("config") or {}).get("brakeHeatCapacity"),
            "brakeCommandTorqueCurve": tr.get("brakeCurve"),
            "keyframes": tr.get("keyframes"),
        },
        "computed": {
            "rows": tr.get("rows"),
            "intervals": tr.get("intervals"),
            "conflict": tr.get("conflict"),
            "summary": tr.get("summary"),
            "geometry": tr.get("geometry"),
        },
        "manualNote": (archive or {}).get("note") or "",
    }
    return json.dumps(payload, ensure_ascii=False, indent=2)


# ----------------------------- HTTP 路由 -----------------------------

def json_response(start_response, obj, status="200 OK"):
    body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    start_response(status, [
        ("Content-Type", "application/json; charset=utf-8"),
        ("Content-Length", str(len(body))),
    ])
    return [body]


def text_response(start_response, text, content_type="text/plain; charset=utf-8",
                  status="200 OK", filename=None):
    body = text.encode("utf-8")
    headers = [("Content-Type", content_type), ("Content-Length", str(len(body)))]
    if filename:
        headers.append(("Content-Disposition", "attachment; filename=\"%s\"" % filename))
    start_response(status, headers)
    return [body]


def read_json(environ):
    length = int(environ.get("CONTENT_LENGTH") or 0)
    raw = environ["wsgi.input"].read(length) if length else b"{}"
    return json.loads(raw.decode("utf-8") or "{}")


def serve_static(start_response, path):
    if path == "/":
        path = "/index.html"
    rel = path.lstrip("/")
    file_path = os.path.normpath(os.path.join(STATIC_DIR, rel))
    if not file_path.startswith(STATIC_DIR) or not os.path.isfile(file_path):
        return text_response(start_response, "Not found", status="404 Not Found")
    ext = os.path.splitext(file_path)[1]
    mime = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
            ".css": "text/css; charset=utf-8"}.get(ext, "application/octet-stream")
    with open(file_path, "rb") as fh:
        body = fh.read()
    start_response("200 OK", [("Content-Type", mime), ("Content-Length", str(len(body)))])
    return [body]


def fetch_archive(conn, archive_id):
    row = conn.execute("SELECT * FROM archives WHERE id=?", (archive_id,)).fetchone()
    if not row:
        return None
    cols = [d[0] for d in conn.execute("SELECT * FROM archives WHERE id=?", (archive_id,)).description]
    rec = dict(zip(cols, row))
    return {
        "id": rec["id"], "created_at": rec["created_at"], "name": rec["name"],
        "note": rec["note"],
        "scenario": json.loads(rec["scenario_json"]),
        "result": json.loads(rec["result_json"]),
        "transient": json.loads(rec.get("transient_json") or "null"),
        "transientResult": json.loads(rec.get("transient_result_json") or "null"),
    }


def app(environ, start_response):
    method = environ.get("REQUEST_METHOD", "GET")
    path = environ.get("PATH_INFO", "/") or "/"
    try:
        if path == "/api/simulate" and method == "POST":
            data = read_json(environ)
            result = simulate(data.get("scenario") or {},
                              int(num(data.get("lockedCount"), 0)),
                              data.get("verifiedHashes") or [])
            return json_response(start_response, result)

        if path == "/api/transient/simulate" and method == "POST":
            data = read_json(environ)
            result = simulate_transient(data)
            return json_response(start_response, result)

        if path == "/api/archives" and method == "GET":
            conn = sqlite3.connect(DB_PATH)
            rows = conn.execute(
                "SELECT id,created_at,name,note,result_json,transient_result_json "
                "FROM archives ORDER BY id DESC").fetchall()
            conn.close()
            items = []
            for r in rows:
                result = json.loads(r[4])
                tr = json.loads(r[5]) if len(r) > 5 and r[5] else None
                items.append({"id": r[0], "createdAt": r[1], "name": r[2], "note": r[3],
                              "status": result.get("status"),
                              "failedIndex": result.get("failedIndex"),
                              "hasTransient": bool(tr),
                              "transientStatus": (tr or {}).get("status"),
                              "summary": result.get("summary")})
            return json_response(start_response, {"items": items})

        if path == "/api/archives" and method == "POST":
            data = read_json(environ)
            scenario = data.get("scenario") or {}
            result = simulate(scenario)
            transient_in = data.get("transient")
            transient = transient_in if isinstance(transient_in, dict) and transient_in else None
            transient_result = None
            if transient is not None:
                transient_result = simulate_transient(
                    {"scenario": scenario, "transient": transient})
            name = (data.get("name") or "未命名推演版本").strip()
            note = data.get("note") or ""
            conn = sqlite3.connect(DB_PATH)
            cur = conn.execute(
                "INSERT INTO archives(name,note,scenario_json,result_json,"
                "transient_json,transient_result_json) VALUES(?,?,?,?,?,?)",
                (name, note, json.dumps(scenario, ensure_ascii=False),
                 json.dumps(result, ensure_ascii=False),
                 json.dumps(transient, ensure_ascii=False) if transient else None,
                 json.dumps(transient_result, ensure_ascii=False) if transient_result else None))
            conn.commit()
            archive_id = cur.lastrowid
            conn.close()
            return json_response(start_response,
                                 {"id": archive_id, "result": result,
                                  "transientResult": transient_result})

        if path.startswith("/api/archives/"):
            parts = [p for p in path.split("/") if p]
            try:
                archive_id = int(parts[2])
            except (IndexError, ValueError):
                return text_response(start_response, "Bad archive id", status="400 Bad Request")
            conn = sqlite3.connect(DB_PATH)
            archive = fetch_archive(conn, archive_id)
            if not archive:
                conn.close()
                return text_response(start_response, "Archive not found", status="404 Not Found")
            if len(parts) == 3 and method == "GET":
                conn.close()
                return json_response(start_response, {
                    "id": archive["id"], "createdAt": archive["created_at"],
                    "name": archive["name"], "note": archive["note"],
                    "scenario": archive["scenario"], "result": archive["result"],
                    "transient": archive["transient"],
                    "transientResult": archive["transientResult"],
                })
            if len(parts) == 4 and method == "GET":
                kind = parts[3]
                if kind == "svg":
                    conn.close()
                    svg = build_svg(archive["scenario"], archive["result"], archive)
                    return text_response(start_response, svg, "image/svg+xml; charset=utf-8",
                                         filename="cable-route-%d.svg" % archive_id)
                if kind == "steps":
                    conn.close()
                    return text_response(start_response,
                                         build_steps(archive["scenario"], archive["result"], archive),
                                         "text/markdown; charset=utf-8",
                                         filename="cable-steps-%d.md" % archive_id)
                if kind == "recalc":
                    conn.close()
                    return text_response(start_response,
                                         build_recalc_json(archive["scenario"], archive["result"]),
                                         "application/json; charset=utf-8",
                                         filename="cable-recalc-%d.json" % archive_id)
                if kind == "transient-csv" and archive["transientResult"]:
                    conn.close()
                    return text_response(start_response,
                                         build_transient_csv(archive["transient"],
                                                             archive["transientResult"], archive),
                                         "text/csv; charset=utf-8",
                                         filename="reel-transient-%d.csv" % archive_id)
                if kind == "transient-svg" and archive["transientResult"]:
                    conn.close()
                    svg = build_transient_svg(archive["transient"],
                                              archive["transientResult"], archive)
                    return text_response(start_response, svg, "image/svg+xml; charset=utf-8",
                                         filename="reel-transient-%d.svg" % archive_id)
                if kind == "transient-json" and archive["transientResult"]:
                    conn.close()
                    return text_response(start_response,
                                         build_transient_recalc_json(
                                             archive["scenario"], archive["transient"],
                                             archive["transientResult"], archive),
                                         "application/json; charset=utf-8",
                                         filename="reel-transient-%d.json" % archive_id)
            conn.close()

        if method == "GET":
            return serve_static(start_response, path)
        return text_response(start_response, "Not found", status="404 Not Found")
    except Exception as exc:  # 防止服务进程因单个请求退出
        return json_response(start_response, {"ok": False, "error": "服务异常：%s" % exc},
                             "500 Internal Server Error")


if __name__ == "__main__":
    init_db()
    host = os.environ.get("HOST", "127.0.0.1")
    port = int(os.environ.get("PORT", "8000"))
    print("电缆盘进场与放缆推演服务：http://%s:%d" % (host, port))
    make_server(host, port, app).serve_forever()
