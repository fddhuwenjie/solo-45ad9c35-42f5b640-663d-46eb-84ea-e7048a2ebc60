/* 电缆盘进场与放缆推演 —— 纯 HTML/SVG/Canvas 前端，无第三方依赖。 */
(function () {
  "use strict";

  var SVG_NS = "http://www.w3.org/2000/svg";
  var PLAN_W = 42, PLAN_H = 26;

  // --------------------------- 默认场景 ---------------------------
  function defaultScenario() {
    return {
      name: "示例：隧道左线—三号竖井",
      params: {
        reelDiameter: 2.6, barrelDiameter: 1.6, reelWidth: 1.45, totalWeight: 80000,
        cableDiameter: 0.085, cableWeight: 118, friction: 0.22, rollerFriction: 0.08,
        rollingResistance: 0.035, initialTension: 600, allowTension: 18000,
        allowSidePressure: 3500, standCapacity: 55000, anchorCapacity: 30000,
        brakeCapacity: 12000, minBendRadius: 1.7, clearanceMargin: 0.1,
        guideRadius: 0.75, guideCount: 4, guideCapacity: 6000,
        pullerCapacity: 18000, pullerBackTension: 300,
        shaftDepth: 12, shaftSheaveRadius: 1.8
      },
      nodes: [
        { id: "n0", type: "start", x: 3, y: 20, z: 0 },
        { id: "n1", type: "gate", x: 10, y: 20, z: 0, openingWidth: 2.2, openingHeight: 3.0 },
        { id: "n2", type: "route", x: 18, y: 20, z: -0.5 },
        { id: "n3", type: "turn", x: 24, y: 20, z: -0.5, radius: 4 },
        { id: "n4", type: "shaft", x: 24, y: 5, z: -0.5 }
      ],
      walls: [
        { id: "w1", x1: 0.5, y1: 17, x2: 9.2, y2: 17 },
        { id: "w2", x1: 0.5, y1: 23, x2: 9.2, y2: 23 },
        { id: "w3", x1: 10.8, y1: 17, x2: 19.5, y2: 17 },
        { id: "w4", x1: 10.8, y1: 23, x2: 19.5, y2: 23 }
      ],
      equipment: [
        { id: "eq-reel", type: "reel", x: 14, y: 20, payoutAngle: 0 },
        { id: "eq-guide", type: "guide", x: 24, y: 16, radius: 0.75, count: 4,
          capacity: 6000, friction: 0.08 },
        { id: "eq-puller", type: "puller", x: 24, y: 8, capacity: 18000, backTension: 300 }
      ]
    };
  }

  var state = {
    scenario: defaultScenario(),
    tool: "select",
    selected: null,              // {kind:'node'|'wall'|'equipment', id}
    result: null,
    play: { active: false, index: 0, station: 0, timer: null },
    lockedCount: 0,
    verifiedHashes: [],
    currentArchiveId: null,
    drawingWall: null
  };

  // --------------------------- 小工具 ---------------------------
  function $(id) { return document.getElementById(id); }
  function num(v, d) {
    var n = parseFloat(v);
    return isNaN(n) ? (d === undefined ? 0 : d) : n;
  }
  function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }
  function dist(ax, ay, bx, by) { return Math.hypot(bx - ax, by - ay); }
  function uid(prefix) {
    return prefix + "-" + Date.now().toString(36) + "-" + Math.floor(Math.random() * 1e4).toString(36);
  }
  function findNode(id) {
    return state.scenario.nodes.find(function (n) { return n.id === id; }) || null;
  }
  function findWall(id) {
    return state.scenario.walls.find(function (w) { return w.id === id; }) || null;
  }
  function findEquip(id) {
    return state.scenario.equipment.find(function (e) { return e.id === id; }) || null;
  }
  function findSelected() {
    if (!state.selected) return null;
    if (state.selected.kind === "node") return findNode(state.selected.id);
    if (state.selected.kind === "wall") return findWall(state.selected.id);
    return findEquip(state.selected.id);
  }
  function phaseName(ph) {
    return { roll: "滚运进场", setup: "盘架架设", pull: "牵引放缆", shaft: "井口下放" }[ph] || ph;
  }
  function statusName(st) {
    return { success: "通过", failed: "受阻", locked: "已锁定", info: "信息" }[st] || st;
  }

  // ---------------------- 本地平面线形（与后端同构，用于绘制/播放） ----------------------
  function angBetween(u, v) {
    var cross = u[0] * v[1] - u[1] * v[0];
    var dot = clamp(u[0] * v[0] + u[1] * v[1], -1, 1);
    return Math.atan2(cross, dot);
  }
  function buildAlignment(nodes) {
    var sections = [], station = 0;
    var nodeStation = {};
    nodeStation[nodes[0].id] = 0;
    var cur = [nodes[0].x, nodes[0].y];
    for (var i = 0; i < nodes.length - 1; i++) {
      var a = nodes[i], b = nodes[i + 1];
      var full = dist(a.x, a.y, b.x, b.y);
      var u = [(b.x - a.x) / full, (b.y - a.y) / full];
      var isTurn = b.type === "turn" && i + 2 < nodes.length;
      var theta = 0, v = null, nextLen = 0;
      if (isTurn) {
        var c = nodes[i + 2];
        nextLen = dist(b.x, b.y, c.x, c.y);
        v = [(c.x - b.x) / nextLen, (c.y - b.y) / nextLen];
        theta = angBetween(u, v);
        if (Math.abs(theta) < 0.035) isTurn = false;
      }
      if (isTurn) {
        var radius = num(b.radius, 3);
        var tanHalf = Math.tan(Math.abs(theta) / 2);
        var maxD = Math.min(full, nextLen) * 0.45;
        var d = radius * tanHalf;
        if (d > maxD && tanHalf > 1e-6) { radius = maxD / tanHalf; d = maxD; }
        var pEnter = [b.x - u[0] * d, b.y - u[1] * d];
        var pExit = [b.x + v[0] * d, b.y + v[1] * d];
        if (dist(cur[0], cur[1], pEnter[0], pEnter[1]) > 1e-6) {
          var L = dist(cur[0], cur[1], pEnter[0], pEnter[1]);
          sections.push({ kind: "straight", start: cur.slice(), end: pEnter.slice(),
            startStation: station, endStation: station + L });
          station += L;
        }
        var sign = theta >= 0 ? 1 : -1;
        var left = [-u[1], u[0]];
        var center = [pEnter[0] + left[0] * radius * sign, pEnter[1] + left[1] * radius * sign];
        var startAngle = Math.atan2(pEnter[1] - center[1], pEnter[0] - center[0]);
        var arcLen = radius * Math.abs(theta);
        sections.push({ kind: "arc", start: pEnter, end: pExit, startStation: station,
          endStation: station + arcLen, center: center, radius: radius,
          startAngle: startAngle, sweep: theta, turn: b.id });
        nodeStation[b.id] = station + arcLen / 2;
        station += arcLen;
        cur = pExit;
      } else {
        var len = dist(cur[0], cur[1], b.x, b.y);
        sections.push({ kind: "straight", start: cur.slice(), end: [b.x, b.y],
          startStation: station, endStation: station + len });
        station += len;
        nodeStation[b.id] = station;
        cur = [b.x, b.y];
      }
    }
    return { sections: sections, total: station, nodeStations: nodeStation };
  }

  function pointAt(sections, s) {
    if (!sections.length) return null;
    s = clamp(s, sections[0].startStation, sections[sections.length - 1].endStation);
    for (var i = 0; i < sections.length; i++) {
      var sec = sections[i];
      if (s <= sec.endStation + 1e-9 || i === sections.length - 1) {
        var span = Math.max(sec.endStation - sec.startStation, 1e-9);
        var t = clamp((s - sec.startStation) / span, 0, 1);
        if (sec.kind === "straight") {
          return {
            x: sec.start[0] + (sec.end[0] - sec.start[0]) * t,
            y: sec.start[1] + (sec.end[1] - sec.start[1]) * t,
            tangent: [(sec.end[0] - sec.start[0]) / span, (sec.end[1] - sec.start[1]) / span]
          };
        }
        var ang = sec.startAngle + sec.sweep * t;
        var radial = [Math.cos(ang), Math.sin(ang)];
        var sgn = sec.sweep >= 0 ? 1 : -1;
        return {
          x: sec.center[0] + sec.radius * radial[0],
          y: sec.center[1] + sec.radius * radial[1],
          tangent: [-radial[1] * sgn, radial[0] * sgn]
        };
      }
    }
    return null;
  }

  function activeSections() {
    // 始终用本地线形（与后端同构），编辑时立即更新；后端 result 仅提供状态/指标。
    return buildAlignment(state.scenario.nodes);
  }

  function cpAt(station) {
    if (!state.result) return null;
    var cps = state.result.checkpoints || [];
    for (var i = 0; i < cps.length; i++) {
      if (station >= cps[i].startStation - 1e-6 && station <= cps[i].endStation + 1e-6) return cps[i];
    }
    return cps.length ? cps[Math.min(state.play.index, cps.length - 1)] : null;
  }

  function cpColor(cp) {
    if (!cp) return "#94a3b8";
    if (cp.status === "locked") return "#16a34a";
    if (cp.status === "failed") return "#dc2626";
    if (cp.status === "success") return "#2563eb";
    return "#94a3b8";
  }

  // --------------------------- SVG 辅助 ---------------------------
  function el(tag, attrs, parent) {
    var node = document.createElementNS(SVG_NS, tag);
    if (attrs) Object.keys(attrs).forEach(function (k) {
      if (k === "text") node.textContent = attrs[k];
      else node.setAttribute(k, attrs[k]);
    });
    if (parent) parent.appendChild(node);
    return node;
  }

  function renderPlan() {
    var svg = $("planSvg");
    svg.innerHTML = "";
    svg.setAttribute("viewBox", "0 0 " + PLAN_W + " " + PLAN_H);

    // 背景网格
    var grid = el("g", {}, svg);
    for (var gx = 0; gx <= PLAN_W; gx += 2)
      el("line", { x1: gx, y1: 0, x2: gx, y2: PLAN_H, stroke: "#eef2f7", "stroke-width": 0.05 }, grid);
    for (var gy = 0; gy <= PLAN_H; gy += 2)
      el("line", { x1: 0, y1: gy, x2: PLAN_W, y2: gy, stroke: "#eef2f7", "stroke-width": 0.05 }, grid);

    var align = activeSections();
    var sections = align.sections;

    // 底图线形（含圆弧）
    var baseD = sections.map(function (s) {
      if (s.kind === "straight")
        return "M" + s.start[0].toFixed(2) + " " + s.start[1].toFixed(2) +
               "L" + s.end[0].toFixed(2) + " " + s.end[1].toFixed(2);
      var sweep = s.sweep > 0 ? 1 : 0;
      return "M" + s.start[0].toFixed(2) + " " + s.start[1].toFixed(2) +
             "A" + s.radius + " " + s.radius + " 0 0 " + sweep + " " +
             s.end[0].toFixed(2) + " " + s.end[1].toFixed(2);
    }).join(" ");
    el("path", { d: baseD, class: "route-base" }, svg);

    // 推演后的标色路径
    if (state.result && sections.length) {
      var colored = el("g", {}, svg);
      var step = 0.25, s = 0, total = align.total;
      while (s < total) {
        var q1 = pointAt(sections, s);
        var q2 = pointAt(sections, Math.min(s + step, total));
        var cp = cpAt(s);
        el("line", { x1: q1.x, y1: q1.y, x2: q2.x, y2: q2.y,
          stroke: cpColor(cp), "stroke-width": 0.5, "stroke-linecap": "round" }, colored);
        s += step;
      }
    }

    // 墙
    state.scenario.walls.forEach(function (w) {
      var failed = isResponsible(w.id);
      var g = el("g", { "data-id": w.id, class: "wall" +
        (state.selected && state.selected.kind === "wall" && state.selected.id === w.id ? " selected" : "") +
        (failed ? " failed-object" : "") }, svg);
      el("line", { x1: w.x1, y1: w.y1, x2: w.x2, y2: w.y2,
        stroke: failed ? "#dc2626" : "#111827", "stroke-width": 0.32, "stroke-linecap": "round" }, g);
      el("line", { x1: w.x1, y1: w.y1, x2: w.x2, y2: w.y2, class: "hit" }, g);
    });

    // 画墙预览
    if (state.drawingWall) {
      var dw = state.drawingWall;
      el("line", { x1: dw.x1, y1: dw.y1, x2: dw.x2, y2: dw.y2,
        stroke: "#f97316", "stroke-width": 0.3, "stroke-dasharray": "0.5 0.35" }, svg);
    }

    // 节点
    state.scenario.nodes.forEach(function (n, idx) {
      var failed = isResponsible(n.id);
      var locked = isLockedObject("node", n.id);
      var g = el("g", { class: "node" +
        (state.selected && state.selected.kind === "node" && state.selected.id === n.id ? " selected" : "") +
        (failed ? " failed-object" : locked ? " locked-object" : "") }, svg);
      if (n.type === "start") {
        el("circle", { cx: n.x, cy: n.y, r: 0.45, fill: "#22c55e", "data-id": n.id, "data-kind": "node" }, g);
        label(g, n.x, n.y - 0.7, "起点", "#166534");
      } else if (n.type === "shaft") {
        el("circle", { cx: n.x, cy: n.y, r: 0.6, fill: "none", stroke: "#7c3aed",
          "stroke-width": 0.18, "data-id": n.id, "data-kind": "node" }, g);
        el("circle", { cx: n.x, cy: n.y, r: 0.24, fill: "#7c3aed",
          "data-id": n.id, "data-kind": "node" }, g);
        label(g, n.x, n.y - 0.85, "竖井井口", "#5b21b6");
      } else if (n.type === "turn") {
        el("polygon", { points: diamond(n.x, n.y, 0.5), fill: failed ? "#dc2626" : "#0ea5e9",
          "data-id": n.id, "data-kind": "node" }, g);
        label(g, n.x + 0.7, n.y + 0.25, "R" + num(n.radius, 3).toFixed(1), failed ? "#dc2626" : "#0369a1");
      } else if (n.type === "gate") {
        drawGate(g, n, idx, failed);
      } else {
        el("circle", { cx: n.x, cy: n.y, r: 0.28, fill: "#64748b",
          "data-id": n.id, "data-kind": "node" }, g);
      }
    });

    // 设备
    state.scenario.equipment.forEach(function (eq) {
      var failed = isResponsible(eq.id);
      var locked = isLockedObject("equipment", eq.id);
      var g = el("g", { class: "equipment" +
        (state.selected && state.selected.kind === "equipment" && state.selected.id === eq.id ? " selected" : "") +
        (failed ? " failed-object" : locked ? " locked-object" : "") }, svg);
      if (eq.type === "reel") {
        var rg = el("g", { transform: "translate(" + eq.x + " " + eq.y + ") rotate(" +
          num(eq.payoutAngle) + ")" }, g);
        el("rect", { x: -1.05, y: -0.6, width: 2.1, height: 1.2, rx: 0.12,
          fill: "#ffedd5", stroke: failed ? "#dc2626" : "#f97316", "stroke-width": 0.18,
          "data-id": eq.id, "data-kind": "equipment" }, rg);
        el("line", { x1: -0.55, y1: -0.6, x2: -0.55, y2: 0.6,
          stroke: failed ? "#dc2626" : "#f97316", "stroke-width": 0.12 }, rg);
        el("line", { x1: 0.55, y1: -0.6, x2: 0.55, y2: 0.6,
          stroke: failed ? "#dc2626" : "#f97316", "stroke-width": 0.12 }, rg);
        // 出线方向箭头
        el("line", { x1: 1.05, y1: 0, x2: 1.7, y2: 0,
          stroke: "#dc2626", "stroke-width": 0.14, "marker-end": "" }, rg);
        el("polygon", { points: "1.7,0 1.45,-0.16 1.45,0.16", fill: "#dc2626" }, rg);
        label(g, eq.x, eq.y - 1.0, "盘架 " + Math.round(num(eq.payoutAngle)) + "°",
          failed ? "#dc2626" : "#c2410c");
      } else if (eq.type === "puller") {
        el("rect", { x: eq.x - 0.6, y: eq.y - 0.42, width: 1.2, height: 0.84, rx: 0.08,
          fill: "#dbeafe", stroke: failed ? "#dc2626" : "#2563eb", "stroke-width": 0.18,
          "data-id": eq.id, "data-kind": "equipment" }, g);
        label(g, eq.x, eq.y + 0.16, "牵", failed ? "#dc2626" : "#1d4ed8");
      } else {
        el("circle", { cx: eq.x, cy: eq.y, r: 0.42, fill: "#dcfce7",
          stroke: failed ? "#dc2626" : "#16a34a", "stroke-width": 0.18,
          "data-id": eq.id, "data-kind": "equipment" }, g);
        label(g, eq.x, eq.y + 0.16, "导", failed ? "#dc2626" : "#15803d", 0.34);
      }
    });

    // 播放头
    if (state.result && state.play.station > 0) {
      var p = pointAt(sections, state.play.station);
      if (p) {
        var pg = el("g", { class: "playhead" }, svg);
        el("circle", { cx: p.x, cy: p.y, r: 0.55, fill: "#dc2626",
          stroke: "#fff", "stroke-width": 0.12 }, pg);
        label(pg, p.x, p.y + 0.18, "盘", "#fff", 0.4);
      }
    }

    bindPlanPointer(svg);
  }

  function label(parent, x, y, text, fill, size) {
    var t = el("text", { x: x, y: y, "font-size": size || 0.42, fill: fill || "#0f172a",
      "text-anchor": "middle", "font-weight": "bold", text: text }, parent);
    return t;
  }

  function diamond(x, y, r) {
    return (x) + "," + (y - r) + " " + (x + r) + "," + y + " " + x + "," + (y + r) + " " + (x - r) + "," + y;
  }

  function drawGate(g, n, idx, failed) {
    var prev = state.scenario.nodes[idx - 1] || state.scenario.nodes[idx + 1] || n;
    var next = state.scenario.nodes[idx + 1] || prev;
    var d = dist(prev.x, prev.y, next.x, next.y) || 1;
    var u = [(next.x - prev.x) / d, (next.y - prev.y) / d];
    var nv = [-u[1], u[0]];
    var ow = num(n.openingWidth, 2.0) / 2;
    [-1, 1].forEach(function (sgn) {
      el("line", {
        x1: n.x + nv[0] * sgn * ow, y1: n.y + nv[1] * sgn * ow,
        x2: n.x + nv[0] * sgn * (ow + 1.5), y2: n.y + nv[1] * sgn * (ow + 1.5),
        stroke: failed ? "#dc2626" : "#475569", "stroke-width": 0.36, "stroke-linecap": "round"
      }, g);
    });
    el("circle", { cx: n.x, cy: n.y, r: 0.22, fill: "#475569",
      "data-id": n.id, "data-kind": "node" }, g);
    label(g, n.x, n.y - 0.75,
      "门洞 " + num(n.openingWidth, 2).toFixed(1) + "×" + num(n.openingHeight, 2.5).toFixed(1),
      failed ? "#dc2626" : "#334155", 0.34);
  }

  // --------------------------- 责任/锁定判定 ---------------------------
  function isResponsible(id) {
    if (!state.result || state.result.status !== "blocked") return false;
    var fi = state.result.failedIndex;
    if (fi == null) return false;
    return state.result.checkpoints[fi].responsibleId === id;
  }
  function lockedCheckpointIds() {
    var set = {};
    if (!state.result) return set;
    state.result.checkpoints.forEach(function (cp, i) {
      if (cp.status === "locked" || i < state.lockedCount) {
        if (cp.responsibleId) set[cp.responsibleId] = true;
      }
    });
    return set;
  }
  function isLockedObject(kind, id) {
    // 盘架与滚运段一旦锁定即视为锁定对象；其余按后端 hash 把关，前端仅提示。
    return false;
  }

  // --------------------------- 平面指针交互 ---------------------------
  function svgPoint(evt) {
    var svg = $("planSvg");
    var pt = svg.createSVGPoint();
    pt.x = evt.clientX; pt.y = evt.clientY;
    var ctm = svg.getScreenCTM();
    if (!ctm) return { x: 0, y: 0 };
    var p = pt.matrixTransform(ctm.inverse());
    return { x: p.x, y: p.y };
  }

  var drag = null;

  function bindPlanPointer(svg) {
    svg.onpointerdown = onPlanDown;
    svg.onpointermove = onPlanMove;
    window.onpointerup = onPlanUp;
  }

  function nearestSegment(x, y) {
    var best = null;
    for (var i = 0; i < state.scenario.nodes.length - 1; i++) {
      var a = state.scenario.nodes[i], b = state.scenario.nodes[i + 1];
      var l2 = dist(a.x, a.y, b.x, b.y) ** 2;
      var t = l2 ? clamp(((x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y)) / l2, 0, 1) : 0;
      var qx = a.x + t * (b.x - a.x), qy = a.y + t * (b.y - a.y);
      var dd = dist(x, y, qx, qy);
      if (!best || dd < best.d) best = { d: dd, index: i + 1, x: qx, y: qy };
    }
    return best;
  }

  function onPlanDown(evt) {
    var p = svgPoint(evt);
    var target = evt.target;
    var kind = target.getAttribute && target.getAttribute("data-kind");
    var id = target.getAttribute && target.getAttribute("data-id");

    if (state.tool === "select" || kind) {
      if (kind && id) {
        state.selected = { kind: kind, id: id };
        var obj = findSelected();
        drag = { kind: kind, id: id, start: p, obj: JSON.parse(JSON.stringify(obj)) };
        svgSetCapture(evt);
        renderAll();
        return;
      }
    }

    if (state.tool === "wall") {
      state.drawingWall = { x1: p.x, y1: p.y, x2: p.x, y2: p.y };
      svgSetCapture(evt);
      renderPlan();
      return;
    }

    if (state.tool === "node") {
      if (state.scenario.nodes[state.scenario.nodes.length - 1].type !== "shaft") {
        setMessage("线形末端不是井口，请先使用示例或调整节点。", "warning");
        return;
      }
      // 插入到井口之前
      var shaft = state.scenario.nodes.pop();
      state.scenario.nodes.push({ id: uid("n"), type: "route", x: round(p.x), y: round(p.y), z: shaft.z });
      state.scenario.nodes.push(shaft);
      invalidateResult();
      renderAll();
      return;
    }

    if (state.tool === "gate" || state.tool === "turn") {
      var seg = nearestSegment(p.x, p.y);
      if (!seg || seg.d > 4) { setMessage("请在路线附近点击以插入" + (state.tool === "gate" ? "门洞" : "转弯区") + "。", "warning"); return; }
      var node = { id: uid("n"), type: state.tool, x: round(seg.x), y: round(seg.y), z: 0 };
      if (state.tool === "gate") { node.openingWidth = 2.2; node.openingHeight = 3.0; }
      if (state.tool === "turn") { node.radius = 4; }
      state.scenario.nodes.splice(seg.index, 0, node);
      // 继承前段高程
      node.z = state.scenario.nodes[seg.index - 1].z;
      state.selected = { kind: "node", id: node.id };
      invalidateResult();
      renderAll();
      return;
    }

    if (state.tool === "reel" || state.tool === "puller" || state.tool === "guide") {
      var eq = { id: uid("eq"), type: state.tool, x: round(p.x), y: round(p.y) };
      if (state.tool === "reel") { eq.payoutAngle = 0; }
      if (state.tool === "puller") { eq.capacity = state.scenario.params.pullerCapacity; eq.backTension = 300; }
      if (state.tool === "guide") {
        eq.radius = state.scenario.params.guideRadius;
        eq.count = state.scenario.params.guideCount;
        eq.capacity = state.scenario.params.guideCapacity;
        eq.friction = state.scenario.params.rollerFriction;
        // 同类只保留一个盘架/牵引机？允许多个牵引分段，但盘架仅一个
      }
      if (state.tool === "reel") {
        state.scenario.equipment = state.scenario.equipment.filter(function (e) { return e.type !== "reel"; });
      }
      state.scenario.equipment.push(eq);
      state.selected = { kind: "equipment", id: eq.id };
      state.tool = "select";
      setToolButtons();
      invalidateResult();
      renderAll();
      return;
    }

    state.selected = null;
    renderAll();
  }

  function svgSetCapture(evt) {
    try { $("planSvg").setPointerCapture(evt.pointerId); } catch (e) {}
  }

  function round(v) { return Math.round(v * 100) / 100; }

  function onPlanMove(evt) {
    var p = svgPoint(evt);
    if (state.drawingWall) {
      state.drawingWall.x2 = p.x; state.drawingWall.y2 = p.y;
      renderPlan();
      return;
    }
    if (!drag) return;
    var obj = null;
    if (drag.kind === "node") obj = findNode(drag.id);
    else if (drag.kind === "wall") obj = findWall(drag.id);
    else obj = findEquip(drag.id);
    if (!obj) return;
    var dx = p.x - drag.start.x, dy = p.y - drag.start.y;
    if (drag.kind === "wall") {
      obj.x1 = round(drag.obj.x1 + dx); obj.y1 = round(drag.obj.y1 + dy);
      obj.x2 = round(drag.obj.x2 + dx); obj.y2 = round(drag.obj.y2 + dy);
    } else {
      obj.x = clamp(round(drag.obj.x + dx), 0, PLAN_W);
      obj.y = clamp(round(drag.obj.y + dy), 0, PLAN_H);
    }
    invalidateResult();
    renderPlan();
    renderProfile();
    renderObjectEditor();
  }

  function onPlanUp() {
    if (state.drawingWall) {
      var w = state.drawingWall;
      if (dist(w.x1, w.y1, w.x2, w.y2) > 0.5) {
        w.id = uid("w");
        state.scenario.walls.push(w);
      }
      state.drawingWall = null;
      invalidateResult();
      renderAll();
    }
    drag = null;
  }

  // --------------------------- 纵剖面 Canvas ---------------------------
  var profileDrag = null;

  function renderProfile() {
    var cv = $("profileCanvas"), ctx = cv.getContext("2d");
    var W = cv.width, H = cv.height, pad = 56;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#fbfdff";
    ctx.fillRect(0, 0, W, H);

    var nodes = state.scenario.nodes;
    var align = activeSections();
    var total = Math.max(align.total, 1);
    var zs = nodes.map(function (n) { return num(n.z); });
    var zmin = Math.min.apply(null, zs), zmax = Math.max.apply(null, zs);
    if (zmax - zmin < 3) { var zm = (zmax + zmin) / 2; zmin = zm - 1.5; zmax = zm + 1.5; }
    zmin -= 0.8; zmax += 0.8;

    function X(st) { return pad + (st / total) * (W - pad * 2); }
    function Y(z) { return H - pad - ((z - zmin) / (zmax - zmin)) * (H - pad * 2); }

    // 网格/坐标轴
    ctx.strokeStyle = "#e2e8f0"; ctx.lineWidth = 1; ctx.font = "12px sans-serif";
    ctx.fillStyle = "#64748b";
    for (var zi = Math.ceil(zmin); zi <= Math.floor(zmax); zi++) {
      ctx.beginPath(); ctx.moveTo(pad, Y(zi)); ctx.lineTo(W - pad, Y(zi)); ctx.stroke();
      ctx.fillText(zi + " m", 8, Y(zi) + 4);
    }
    ctx.strokeStyle = "#94a3b8";
    ctx.beginPath(); ctx.moveTo(pad, H - pad); ctx.lineTo(W - pad, H - pad); ctx.stroke();
    ctx.fillText("里程 →", W - 60, H - 18);
    ctx.fillText("高程", 12, 18);

    // 纵坡折线（按本地线形节点 station，与后端同构）
    var stationOf = align.nodeStations;

    ctx.lineWidth = 3;
    for (var j = 1; j < nodes.length; j++) {
      var a = nodes[j - 1], b = nodes[j];
      var cp = cpAt((stationOf[a.id] + stationOf[b.id]) / 2);
      ctx.strokeStyle = cpColor(cp);
      ctx.beginPath();
      ctx.moveTo(X(stationOf[a.id]), Y(num(a.z)));
      ctx.lineTo(X(stationOf[b.id]), Y(num(b.z)));
      ctx.stroke();
      var run = dist(a.x, a.y, b.x, b.y);
      var grade = run ? ((num(b.z) - num(a.z)) / run * 100) : 0;
      if (Math.abs(grade) >= 3) {
        ctx.fillStyle = "#92400e";
        ctx.fillText(grade.toFixed(1) + "%",
          (X(stationOf[a.id]) + X(stationOf[b.id])) / 2 - 14,
          (Y(num(a.z)) + Y(num(b.z))) / 2 - 8);
      }
    }

    // 节点圆点
    nodes.forEach(function (n) {
      var failed = isResponsible(n.id);
      ctx.beginPath();
      ctx.arc(X(stationOf[n.id]), Y(num(n.z)), n.type === "shaft" ? 8 : 6, 0, Math.PI * 2);
      ctx.fillStyle = failed ? "#dc2626" :
        (n.type === "start" ? "#22c55e" : n.type === "shaft" ? "#7c3aed" :
         n.type === "turn" ? "#0ea5e9" : n.type === "gate" ? "#475569" : "#64748b");
      ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle = "#334155"; ctx.font = "11px sans-serif";
      var tag = { start: "起点", shaft: "井口", turn: "转弯", gate: "门洞", route: "点" }[n.type];
      ctx.fillText(tag + " " + num(n.z).toFixed(1) + "m", X(stationOf[n.id]) - 16, Y(num(n.z)) - 12);
    });

    // 播放头
    if (state.result && state.play.station > 0) {
      var stt = state.play.station;
      var zz = profileZAt(nodes, stationOf, stt);
      ctx.beginPath();
      ctx.arc(X(stt), Y(zz), 7, 0, Math.PI * 2);
      ctx.fillStyle = "#dc2626"; ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
    }

    // 拖拽改高程
    cv.onpointerdown = function (evt) {
      var rect = cv.getBoundingClientRect();
      var mx = (evt.clientX - rect.left) * (W / rect.width);
      var my = (evt.clientY - rect.top) * (H / rect.height);
      for (var k = 0; k < nodes.length; k++) {
        var cx = X(stationOf[nodes[k].id]), cy = Y(num(nodes[k].z));
        if (Math.hypot(mx - cx, my - cy) < 12) {
          if (nodes[k].type === "start") { setMessage("起点高程固定为基准 0。", "warning"); return; }
          profileDrag = nodes[k].id;
          cv.setPointerCapture(evt.pointerId);
          return;
        }
      }
    };
    cv.onpointermove = function (evt) {
      if (!profileDrag) return;
      var rect = cv.getBoundingClientRect();
      var my = (evt.clientY - rect.top) * (H / rect.height);
      var z = zmin + (1 - (my - pad) / (H - pad * 2)) * (zmax - zmin);
      var n = findNode(profileDrag);
      if (n) { n.z = Math.round(z * 100) / 100; invalidateResult(); renderProfile(); renderPlan(); renderObjectEditor(); }
    };
    cv.onpointerup = function () { profileDrag = null; };
  }

  function profileZAt(nodes, stationOf, st) {
    for (var i = 1; i < nodes.length; i++) {
      var s0 = stationOf[nodes[i - 1].id], s1 = stationOf[nodes[i].id];
      if (st >= s0 && st <= s1) {
        var t = s1 === s0 ? 0 : (st - s0) / (s1 - s0);
        return num(nodes[i - 1].z) + t * (num(nodes[i].z) - num(nodes[i - 1].z));
      }
    }
    return num(nodes[nodes.length - 1].z);
  }

  // --------------------------- 参数/对象表单 ---------------------------
  var PARAM_LABELS = {
    reelDiameter: "盘体外径 D(m)", barrelDiameter: "盘芯直径(m)", reelWidth: "盘宽(m)",
    totalWeight: "总重(N)", cableDiameter: "电缆外径(m)", cableWeight: "电缆单位重(N/m)",
    friction: "地面摩擦", rollerFriction: "导轮摩擦", rollingResistance: "滚运阻力",
    initialTension: "初始张力(N)", allowTension: "允许张力(N)",
    allowSidePressure: "允许侧压(N/m)", standCapacity: "盘架限值(N)",
    anchorCapacity: "锚固限值(N)", brakeCapacity: "制动限值(N)",
    minBendRadius: "最小弯曲半径(m)", clearanceMargin: "安全净距(m)",
    guideRadius: "导轮半径(m)", guideCount: "导轮数量", guideCapacity: "导轮荷载(N)",
    pullerCapacity: "牵引机额定(N)", pullerBackTension: "分段起始张力(N)",
    shaftDepth: "竖井深(m)", shaftSheaveRadius: "井口导向半径(m)"
  };

  function renderParamForm() {
    var form = $("paramForm");
    form.innerHTML = "";
    Object.keys(PARAM_LABELS).forEach(function (key) {
      var lab = document.createElement("label");
      lab.textContent = PARAM_LABELS[key];
      var inp = document.createElement("input");
      inp.type = "number"; inp.step = "any"; inp.value = state.scenario.params[key];
      inp.addEventListener("input", function () {
        state.scenario.params[key] = num(inp.value, state.scenario.params[key]);
        invalidateResult();
      });
      lab.appendChild(inp);
      form.appendChild(lab);
    });
  }

  function field(labelText, value, oninput, step) {
    var lab = document.createElement("label");
    lab.textContent = labelText;
    var inp = document.createElement("input");
    inp.type = "number"; inp.step = step || "any"; inp.value = value;
    inp.addEventListener("input", function () {
      oninput(num(inp.value));
      invalidateResult(); renderPlan(); renderProfile();
    });
    lab.appendChild(inp);
    return lab;
  }

  function selectField(labelText, value, options, onchange) {
    var lab = document.createElement("label");
    lab.textContent = labelText;
    var sel = document.createElement("select");
    options.forEach(function (o) {
      var opt = document.createElement("option");
      opt.value = o.value; opt.textContent = o.text;
      if (o.value === value) opt.selected = true;
      sel.appendChild(opt);
    });
    sel.addEventListener("change", function () { onchange(sel.value); invalidateResult(); renderAll(); });
    lab.appendChild(sel);
    return lab;
  }

  function renderObjectEditor() {
    var box = $("objectEditor");
    box.innerHTML = "";
    var obj = findSelected();
    if (!obj) {
      box.innerHTML = '<p class="muted">在平面或纵剖面中选择节点、墙或设备。</p>';
      return;
    }
    var kind = state.selected.kind;
    var title = document.createElement("div");
    title.className = "obj-title";
    var typeText = { node: "路线节点", wall: "墙体/障碍", equipment: "设备" }[kind];
    var sub = obj.type === "reel" ? "电缆盘架" : obj.type === "puller" ? "牵引机" :
              obj.type === "guide" ? "导向轮组" : obj.type;
    title.innerHTML = "<span>" + typeText + " · " + (sub || "") + "</span>";
    var del = document.createElement("button");
    del.textContent = "删除"; del.className = "danger";
    del.addEventListener("click", function () { deleteObject(); });
    title.appendChild(del);
    box.appendChild(title);

    if (kind === "node") {
      var order = state.scenario.nodes.indexOf(obj);
      box.appendChild(field("x(m)", obj.x, function (v) { obj.x = clamp(v, 0, PLAN_W); }));
      box.appendChild(field("y(m)", obj.y, function (v) { obj.y = clamp(v, 0, PLAN_H); }));
      box.appendChild(field("高程 z(m)", num(obj.z), function (v) { obj.z = v; }, "0.1"));
      if (order > 0 && order < state.scenario.nodes.length - 1) {
        box.appendChild(selectField("构造类型", obj.type, [
          { value: "route", text: "普通路线点" },
          { value: "gate", text: "门洞" },
          { value: "turn", text: "转弯区" }
        ], function (v) {
          obj.type = v;
          if (v === "gate" && obj.openingWidth == null) { obj.openingWidth = 2.2; obj.openingHeight = 3.0; }
          if (v === "turn" && obj.radius == null) obj.radius = 4;
        }));
      }
      if (obj.type === "turn") box.appendChild(field("弯曲半径 R(m)", num(obj.radius, 4), function (v) { obj.radius = Math.max(0.2, v); }, "0.1"));
      if (obj.type === "gate") {
        box.appendChild(field("门洞净宽(m)", num(obj.openingWidth, 2.2), function (v) { obj.openingWidth = v; }, "0.05"));
        box.appendChild(field("门洞净高(m)", num(obj.openingHeight, 3.0), function (v) { obj.openingHeight = v; }, "0.05"));
      }
    } else if (kind === "wall") {
      box.appendChild(field("起点 x", obj.x1, function (v) { obj.x1 = v; }));
      box.appendChild(field("起点 y", obj.y1, function (v) { obj.y1 = v; }));
      box.appendChild(field("终点 x", obj.x2, function (v) { obj.x2 = v; }));
      box.appendChild(field("终点 y", obj.y2, function (v) { obj.y2 = v; }));
    } else {
      box.appendChild(field("x(m)", obj.x, function (v) { obj.x = clamp(v, 0, PLAN_W); }));
      box.appendChild(field("y(m)", obj.y, function (v) { obj.y = clamp(v, 0, PLAN_H); }));
      if (obj.type === "reel") {
        var angField = field("出线方向(°)", num(obj.payoutAngle), function (v) { obj.payoutAngle = v; }, "1");
        box.appendChild(angField);
        var btnRow = document.createElement("div");
        btnRow.style.display = "flex"; btnRow.style.gap = "6px";
        [-90, 0, 90, 180].forEach(function (a) {
          var b = document.createElement("button");
          b.textContent = a + "°";
          b.addEventListener("click", function () {
            obj.payoutAngle = a; invalidateResult(); renderAll();
          });
          btnRow.appendChild(b);
        });
        box.appendChild(btnRow);
      }
      if (obj.type === "puller") {
        box.appendChild(field("额定牵引力(N)", num(obj.capacity, state.scenario.params.pullerCapacity),
          function (v) { obj.capacity = v; }, "100"));
        box.appendChild(field("分段后起始张力(N)", num(obj.backTension, 300),
          function (v) { obj.backTension = v; }, "50"));
      }
      if (obj.type === "guide") {
        box.appendChild(field("导轮半径(m)", num(obj.radius, 0.75), function (v) { obj.radius = v; }, "0.05"));
        box.appendChild(field("导轮数量", num(obj.count, 4), function (v) { obj.count = Math.max(1, Math.round(v)); }, "1"));
        box.appendChild(field("单只额定(N)", num(obj.capacity, 6000), function (v) { obj.capacity = v; }, "100"));
        box.appendChild(field("导轮摩擦系数", num(obj.friction, 0.08), function (v) { obj.friction = v; }, "0.01"));
      }
    }
  }

  function deleteObject() {
    if (!state.selected) return;
    var k = state.selected.kind, id = state.selected.id;
    if (k === "node") {
      var idx = state.scenario.nodes.findIndex(function (n) { return n.id === id; });
      var n = state.scenario.nodes[idx];
      if (n.type === "start" || n.type === "shaft") { setMessage("起点和井口不能删除。", "warning"); return; }
      state.scenario.nodes.splice(idx, 1);
    } else if (k === "wall") {
      state.scenario.walls = state.scenario.walls.filter(function (w) { return w.id !== id; });
    } else {
      state.scenario.equipment = state.scenario.equipment.filter(function (e) { return e.id !== id; });
    }
    state.selected = null;
    invalidateResult();
    renderAll();
  }

  // --------------------------- API ---------------------------
  function api(path, opts) {
    return fetch(path, opts).then(function (r) {
      return r.json().then(function (data) { return { ok: r.ok, data: data }; });
    });
  }
  function post(path, body) {
    return api(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body)
    });
  }

  function runSimulation(isContinue) {
    stopPlayback();
    var payload = { scenario: state.scenario };
    if (isContinue && state.lockedCount > 0) {
      payload.lockedCount = state.lockedCount;
      payload.verifiedHashes = state.verifiedHashes;
    }
    setMessage("后端正在逐步复算滚运、转向、架设与牵引…", "info");
    return post("/api/simulate", payload).then(function (res) {
      var r = res.data;
      if (r.ok === false) { setMessage(r.error || "推演失败。", "error"); return; }
      state.result = r;
      if (r.status === "lock-changed") {
        state.lockedCount = 0; state.verifiedHashes = [];
        setMessage("已核实区段的输入发生变化，已解除锁定，请从头复算：" + (r.error || ""), "warning");
        renderAll();
        return;
      }
      if (r.warnings && r.warnings.length) {
        setMessage("线形提示：" + r.warnings.join("；"), "warning");
      }
      if (r.status === "blocked") {
        state.lockedCount = r.failedIndex;
        state.verifiedHashes = r.checkpoints.slice(0, r.failedIndex).map(function (c) { return c.hash; });
        setMessage("播放停在首次受阻处：" + r.checkpoints[r.failedIndex].message +
          "  可就地调整责任对象后“继续演算”。", "error");
      } else if (r.status === "passed") {
        state.lockedCount = 0; state.verifiedHashes = [];
        setMessage("推演通过：最大张力 " + (r.summary.maxTension || 0) +
          "N，可归档并输出步骤单、标色 SVG 与复算 JSON。", "success");
      }
      renderAll();
      startPlayback();
    }).catch(function (e) {
      setMessage("请求失败：" + e.message, "error");
    });
  }

  // --------------------------- 播放控制 ---------------------------
  function startPlayback() {
    if (!state.result) return;
    var cps = state.result.checkpoints;
    var firstUnlocked = cps.findIndex(function (c) { return c.status !== "locked"; });
    state.play.active = true;
    state.play.index = firstUnlocked < 0 ? cps.length - 1 : firstUnlocked;
    var cp = cps[state.play.index];
    state.play.station = cp ? cp.startStation : 0;
    $("pauseBtn").disabled = false;
    $("stepBtn").disabled = false;
    tickPlayback();
  }

  function stopPlayback() {
    state.play.active = false;
    if (state.play.timer) { clearTimeout(state.play.timer); state.play.timer = null; }
  }

  function tickPlayback() {
    if (!state.result) return;
    var cps = state.result.checkpoints;
    if (state.play.index >= cps.length) { finishPlayback(); return; }
    var cp = cps[state.play.index];
    var speed = num($("speedInput").value, 1);
    var duration = (cp.endStation <= cp.startStation + 1e-6 ? 650 : 1300) / speed;
    var t0 = performance.now();
    function frame(now) {
      if (!state.play.active) return;
      var t = clamp((now - t0) / duration, 0, 1);
      state.play.station = cp.startStation + (cp.endStation - cp.startStation) * t;
      updateTelemetry(cp);
      renderPlan();
      renderProfile();
      highlightCheckpointItem(state.play.index);
      if (t < 1) {
        state.play.timer = setTimeout(function () { requestAnimationFrame(frame); }, 30);
      } else {
        if (cp.status === "failed") {
          state.play.active = false;
          state.play.station = cp.endStation;
          updateTelemetry(cp);
          renderPlan(); renderProfile();
          setMessage("首次受阻 → " + cp.message + "（责任对象：" + cp.responsibleId + "）", "error");
          return;
        }
        state.play.index += 1;
        if (state.play.index >= cps.length) { finishPlayback(); return; }
        tickPlayback();
      }
    }
    requestAnimationFrame(frame);
  }

  function finishPlayback() {
    state.play.active = false;
    var cps = state.result.checkpoints;
    var last = cps[cps.length - 1];
    if (last) { state.play.station = last.endStation; updateTelemetry(last); }
    renderPlan(); renderProfile();
  }

  function stepOnce() {
    if (!state.result) return;
    stopPlayback();
    var cps = state.result.checkpoints;
    if (state.play.index < cps.length - 1 || state.play.station < cps[state.play.index].endStation) {
      var cp = cps[state.play.index];
      if (state.play.station < cp.endStation - 1e-6) {
        state.play.station = cp.endStation;
      } else {
        state.play.index += 1;
        cp = cps[state.play.index];
        state.play.station = cp ? cp.endStation : state.play.station;
      }
      if (cp) { updateTelemetry(cp); highlightCheckpointItem(state.play.index); }
      renderPlan(); renderProfile();
    }
  }

  function fmtN(v) { return v == null || isNaN(v) ? "-" : Math.round(v).toLocaleString() + " N"; }
  function fmtM(v) { return v == null || isNaN(v) ? "-" : Number(v).toFixed(2) + " m"; }

  function updateTelemetry(cp) {
    var m = cp.metrics || {};
    $("teleTitle").textContent = cp.title;
    $("teleStation").textContent = cp.startStation === cp.endStation
      ? Number(cp.endStation).toFixed(2) + " m"
      : Number(cp.startStation).toFixed(2) + "–" + Number(cp.endStation).toFixed(2) + " m";
    $("teleTension").textContent = fmtN(m.tension);
    $("teleRadius").textContent = m.bendRadius == null ? "-" : Number(m.bendRadius).toFixed(2) + " m";
    $("teleSide").textContent = m.sidePressure == null ? "-" : Math.round(m.sidePressure).toLocaleString() + " N/m";
    $("teleStand").textContent = fmtN(m.standReaction);
    $("teleBrake").textContent = m.brakeForce ? fmtN(m.brakeForce) : "-";
    var st = $("teleState");
    st.textContent = statusName(cp.status) + " · " + phaseName(cp.phase);
    st.style.color = cp.status === "failed" ? "#dc2626" : cp.status === "locked" ? "#16a34a" : "#2563eb";
  }

  // --------------------------- 逐段结果列表 ---------------------------
  function renderCheckpointList() {
    var ol = $("checkpointList");
    ol.innerHTML = "";
    if (!state.result) return;
    state.result.checkpoints.forEach(function (cp, i) {
      var li = document.createElement("li");
      li.className = cp.status + (state.play.index === i ? " active" : "");
      li.dataset.index = i;
      var m = cp.metrics || {};
      li.innerHTML =
        '<div class="cp-title"><span>' + i + 1 + ". " + phaseName(cp.phase) + " · " + cp.title + "</span>" +
        "<span>" + statusName(cp.status) + "</span></div>" +
        '<div class="cp-num">里程 ' + Number(cp.startStation).toFixed(2) + "–" + Number(cp.endStation).toFixed(2) +
        " m ｜ 张力 " + Math.round(m.tension || 0) + " N" +
        (m.bendRadius != null ? " ｜ R " + Number(m.bendRadius).toFixed(2) + " m" : "") +
        (m.sidePressure != null ? " ｜ 侧压 " + Math.round(m.sidePressure) + " N/m" : "") +
        (m.standReaction ? " ｜ 盘架 " + Math.round(m.standReaction) + " N" : "") + "</div>" +
        (cp.message ? '<div class="cp-msg">' + cp.message + "</div>" : "");
      li.addEventListener("click", function () {
        stopPlayback();
        state.play.index = i;
        state.play.station = cp.endStation;
        updateTelemetry(cp);
        renderPlan(); renderProfile(); highlightCheckpointItem(i);
      });
      ol.appendChild(li);
    });
  }

  function highlightCheckpointItem(i) {
    var lis = $("checkpointList").querySelectorAll("li");
    lis.forEach(function (li, k) { li.classList.toggle("active", k === i); });
  }

  // --------------------------- 消息 ---------------------------
  function setMessage(text, kind) {
    var bar = $("messageBar");
    bar.textContent = text;
    bar.className = "message " + (kind || "info");
  }

  function invalidateResult() {
    if (state.result && state.lockedCount === 0) state.result = null;
    state.currentArchiveId = null;
    ["stepsBtn", "svgBtn", "jsonBtn"].forEach(function (id) { $(id).disabled = true; });
  }

  // --------------------------- 归档 ---------------------------
  function archiveCurrent() {
    var name = $("archiveName").value.trim() || ("推演版本 " + new Date().toLocaleString());
    var note = $("archiveNote").value;
    post("/api/archives", { name: name, note: note, scenario: state.scenario })
      .then(function (res) {
        var r = res.data;
        if (r.id) {
          state.currentArchiveId = r.id;
          state.result = r.result;
          renderAll();
          setMessage("已归档 #" + r.id + "，结论：" + (r.result.status === "passed" ? "通过" : "受阻"), "success");
          loadArchives();
          enableExport(true);
        } else {
          setMessage(r.error || "归档失败", "error");
        }
      });
  }

  function enableExport(on) {
    ["stepsBtn", "svgBtn", "jsonBtn"].forEach(function (id) { $(id).disabled = !on; });
  }

  function loadArchives() {
    api("/api/archives").then(function (res) {
      var box = $("archiveList");
      box.innerHTML = "";
      (res.data.items || []).forEach(function (it) {
        var div = document.createElement("div");
        div.className = "archive-item";
        var stateText = it.status === "passed" ? "✅ 通过" : it.status === "blocked" ? "⛔ 受阻" : it.status;
        div.innerHTML = "<strong>#" + it.id + " " + it.name + "</strong>" +
          "<small>" + it.createdAt + " ｜ " + stateText + "</small>" +
          (it.note ? "<small>说明：" + String(it.note).slice(0, 60) + "</small>" : "");
        var row = document.createElement("div"); row.className = "row";
        [["载入", "load"], ["步骤单", "steps"], ["SVG", "svg"], ["复算JSON", "recalc"]].forEach(function (pair) {
          var b = document.createElement("button");
          b.textContent = pair[0];
          b.addEventListener("click", function () { archiveAction(it.id, pair[1]); });
          row.appendChild(b);
        });
        div.appendChild(row);
        box.appendChild(div);
      });
    });
  }

  function archiveAction(id, kind) {
    if (kind === "steps" || kind === "svg" || kind === "recalc") {
      window.open("/api/archives/" + id + "/" + kind, "_blank");
      return;
    }
    api("/api/archives/" + id).then(function (res) {
      var d = res.data;
      state.scenario = d.scenario;
      state.result = d.result;
      state.lockedCount = 0; state.verifiedHashes = [];
      state.currentArchiveId = id;
      $("archiveName").value = d.name;
      $("archiveNote").value = d.note;
      renderParamForm();
      renderAll();
      enableExport(true);
      setMessage("已载入归档 #" + id + "，结论：" + d.result.status, "info");
      if (d.result.status === "passed") startPlayback();
      else { state.play.station = 0; renderPlan(); renderProfile(); renderCheckpointList(); }
    });
  }

  // --------------------------- 总渲染/绑定 ---------------------------
  function renderAll() {
    renderPlan();
    renderProfile();
    renderObjectEditor();
    renderCheckpointList();
    var blocked = state.result && state.result.status === "blocked";
    $("continueBtn").disabled = !blocked;
    $("unlockBtn").disabled = !(state.lockedCount > 0 || blocked);
    enableExport(!!state.currentArchiveId || (state.result && state.result.status === "passed" && false));
    if (!state.result) {
      $("pauseBtn").disabled = true; $("stepBtn").disabled = true;
      ["teleTitle", "teleStation", "teleTension", "teleRadius", "teleSide", "teleStand", "teleBrake"].forEach(function (id) {
        $(id).textContent = "-";
      });
      $("teleState").textContent = "-";
    }
  }

  function setToolButtons() {
    document.querySelectorAll(".tool").forEach(function (b) {
      b.classList.toggle("active", b.dataset.tool === state.tool);
    });
  }

  function init() {
    renderParamForm();
    document.querySelectorAll(".tool").forEach(function (b) {
      b.addEventListener("click", function () {
        state.tool = b.dataset.tool;
        state.selected = null;
        setToolButtons();
        renderAll();
      });
    });
    $("runBtn").addEventListener("click", function () {
      state.lockedCount = 0; state.verifiedHashes = [];
      runSimulation(false);
    });
    $("continueBtn").addEventListener("click", function () { runSimulation(true); });
    $("unlockBtn").addEventListener("click", function () {
      state.lockedCount = 0; state.verifiedHashes = [];
      setMessage("已解除区段锁定，可重新从头推演。", "info");
      renderAll();
    });
    $("pauseBtn").addEventListener("click", function () {
      if (state.play.active) { stopPlayback(); $("pauseBtn").textContent = "▶ 继续播放"; }
      else { state.play.active = true; $("pauseBtn").textContent = "⏸ 暂停"; tickPlayback(); }
    });
    $("stepBtn").addEventListener("click", stepOnce);
    $("archiveBtn").addEventListener("click", archiveCurrent);
    $("stepsBtn").addEventListener("click", function () {
      if (state.currentArchiveId) window.open("/api/archives/" + state.currentArchiveId + "/steps", "_blank");
    });
    $("svgBtn").addEventListener("click", function () {
      if (state.currentArchiveId) window.open("/api/archives/" + state.currentArchiveId + "/svg", "_blank");
    });
    $("jsonBtn").addEventListener("click", function () {
      if (state.currentArchiveId) window.open("/api/archives/" + state.currentArchiveId + "/recalc", "_blank");
    });

    renderAll();
    loadArchives();
  }

  document.addEventListener("DOMContentLoaded", init);
})();
