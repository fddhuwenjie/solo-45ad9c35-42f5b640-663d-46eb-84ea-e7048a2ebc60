/* 放盘—牵引联动瞬态工作区：时间轴关键帧、逐时曲线、线路动画、增量锁定。
 * 依赖 app.js 维护的 scenario（基础参数/线形/设备）；本文件只负责瞬态部分。 */
(function () {
  "use strict";

  var SVG_NS = "http://www.w3.org/2000/svg";

  var DEFAULT_CURVE = [[0, 0], [0.25, 1400], [0.5, 3000], [0.75, 5000], [1, 7500]];

  function defaultTransient() {
    return {
      emptyInertia: 520, bearingTorque: 260, reserveCable: 60,
      axialStiffness: 4000000, slackClearance: 1.5, brakeMaxSpeed: 12,
      brakeHeatCapacity: 260000, windingPack: 0.85,
      startSpeed: 0, duration: 26, dt: 0.1,
      brakeCurve: DEFAULT_CURVE.map(function (p) { return p.slice(); }),
      keyframes: [
        { id: "kf-ramp", type: "speed", time: 1, speed: 1.2, ramp: 2, hold: 1, brake: null },
        { id: "kf-jog", type: "jog", time: 6, speed: 1.8, ramp: 0.5, hold: 1.2, brake: null },
        { id: "kf-slow", type: "speed", time: 11, speed: 0.8, ramp: 2, hold: 1, brake: null },
        { id: "kf-stop", type: "estop", time: 17, speed: 0, ramp: 0.4, hold: 1, brake: 1 },
        { id: "kf-resume", type: "speed", time: 21, speed: 0.6, ramp: 1, hold: 1, brake: 0 }
      ]
    };
  }

  var T = {
    conf: defaultTransient(),
    result: null,
    lockedCount: 0,
    verifiedHashes: [],
    startState: null,
    rows: [],                 // 已合并的完整逐时行（锁定段 + 最新重算段）
    selectedKf: null,
    draggingKf: null,
    archiveId: null,
    play: { active: false, index: 0, timer: null }
  };

  function $(id) { return document.getElementById(id); }
  function num(v, d) { var n = parseFloat(v); return isNaN(n) ? (d === undefined ? 0 : d) : n; }
  function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }
  function uid() {
    return "kf-" + Date.now().toString(36) + "-" + Math.floor(Math.random() * 1e3);
  }
  function el(tag, attrs, parent) {
    var n = document.createElementNS(SVG_NS, tag);
    if (attrs) Object.keys(attrs).forEach(function (k) {
      if (k === "text") n.textContent = attrs[k];
      else n.setAttribute(k, attrs[k]);
    });
    if (parent) parent.appendChild(n);
    return n;
  }
  function scenario() { return window.RiggingScenario && window.RiggingScenario.get(); }
  function setMsg(text, kind) {
    var bar = $("trMessageBar");
    bar.textContent = text;
    bar.className = "message " + (kind || "info");
  }
  function api(path, opts) {
    return fetch(path, opts).then(function (r) {
      return r.json().then(function (d) { return { ok: r.ok, data: d }; });
    });
  }
  function post(path, body) {
    return api(path, { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body) });
  }

  // --------------------------- 参数表单 ---------------------------
  var PARAM_KEYS = ["emptyInertia", "bearingTorque", "reserveCable", "axialStiffness",
    "slackClearance", "brakeMaxSpeed", "brakeHeatCapacity", "windingPack", "duration", "dt"];

  function renderParamForm() {
    var form = $("trParamForm");
    form.innerHTML = "";
    PARAM_KEYS.forEach(function (k) {
      var lab = document.createElement("label");
      var title = {
        emptyInertia: "空盘（含筒体）转动惯量", bearingTorque: "轴承等阻力矩",
        reserveCable: "盘上初始余缆长度", axialStiffness: "电缆轴向抗拉刚度 EA",
        slackClearance: "盘前允许松弛圈长度", brakeMaxSpeed: "制动器允许最高转速",
        brakeHeatCapacity: "制动器一次循环热容量", windingPack: "绕线横截面积装填系数",
        duration: "联动演算总时长", dt: "逐时积分步长"
      }[k];
      lab.textContent = title;
      var inp = document.createElement("input");
      inp.type = "number"; inp.step = "any"; inp.value = T.conf[k];
      inp.addEventListener("input", function () {
        T.conf[k] = num(inp.value, T.conf[k]);
        invalidate();
        renderTimeline(); drawCharts(); drawAnimation();
      });
      lab.appendChild(inp);
      form.appendChild(lab);
    });
  }

  // --------------------------- 制动曲线编辑 ---------------------------
  function drawBrakeCurve() {
    var cv = $("trBrakeCanvas"), ctx = cv.getContext("2d");
    var W = cv.width, H = cv.height, pad = 34;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#fbfdff"; ctx.fillRect(0, 0, W, H);
    function X(c) { return pad + c * (W - pad * 1.4); }
    var ymax = Math.max.apply(null, T.conf.brakeCurve.map(function (p) { return p[1]; })) * 1.15 || 8000;
    function Y(m) { return H - pad - (m / ymax) * (H - pad * 2); }
    ctx.strokeStyle = "#e2e8f0"; ctx.fillStyle = "#64748b"; ctx.font = "11px sans-serif";
    for (var g = 0; g <= 4; g++) {
      var gx = X(g / 4);
      ctx.beginPath(); ctx.moveTo(gx, pad); ctx.lineTo(gx, H - pad); ctx.stroke();
      ctx.fillText((g / 4).toFixed(2), gx - 12, H - 14);
    }
    ctx.beginPath(); ctx.moveTo(pad, H - pad); ctx.lineTo(W - pad * 0.4, H - pad);
    ctx.moveTo(pad, H - pad); ctx.lineTo(pad, pad); ctx.strokeStyle = "#94a3b8"; ctx.stroke();
    ctx.fillText("指令", W - 34, H - 14);
    ctx.save(); ctx.translate(12, pad + 8); ctx.rotate(-Math.PI / 2);
    ctx.fillText("N·m", 0, 0); ctx.restore();
    ctx.strokeStyle = "#dc2626"; ctx.lineWidth = 2; ctx.beginPath();
    T.conf.brakeCurve.forEach(function (p, i) {
      i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]));
    });
    ctx.stroke();
    T.conf.brakeCurve.forEach(function (p, i) {
      ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 5, 0, Math.PI * 2);
      ctx.fillStyle = i === curveDrag ? "#dc2626" : "#2563eb"; ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5; ctx.stroke();
    });
  }

  var curveDrag = null;
  function bindBrakeCanvas() {
    var cv = $("trBrakeCanvas");
    function locate(evt) {
      var r = cv.getBoundingClientRect();
      var mx = (evt.clientX - r.left) * (cv.width / r.width);
      var my = (evt.clientY - r.top) * (cv.height / r.height);
      var pad = 34, ymax = Math.max.apply(null, T.conf.brakeCurve.map(function (p) { return p[1]; })) * 1.15 || 8000;
      function X(c) { return pad + c * (cv.width - pad * 1.4); }
      function Y(m) { return cv.height - pad - (m / ymax) * (cv.height - pad * 2); }
      var best = -1, bd = 1e9;
      T.conf.brakeCurve.forEach(function (p, i) {
        var d = Math.hypot(mx - X(p[0]), my - Y(p[1]));
        if (d < bd) { bd = d; best = i; }
      });
      return { index: bd < 12 ? best : -1, mx: mx, my: my, X: X, Y: Y, ymax: ymax, pad: pad };
    }
    cv.addEventListener("pointerdown", function (evt) {
      var hit = locate(evt);
      if (hit.index >= 0) { curveDrag = hit.index; cv.setPointerCapture(evt.pointerId); }
    });
    cv.addEventListener("pointermove", function (evt) {
      if (curveDrag == null) return;
      var hit = locate(evt);
      var p = T.conf.brakeCurve[curveDrag];
      p[0] = clamp((hit.mx - hit.pad) / (cv.width - hit.pad * 1.4), 0, 1);
      p[1] = clamp(hit.ymax * (1 - (hit.my - hit.pad) / (cv.height - hit.pad * 2)), 0, 1e7);
      T.conf.brakeCurve.sort(function (a, b) { return a[0] - b[0]; });
      curveDrag = T.conf.brakeCurve.indexOf(p);
      invalidate(); drawBrakeCurve(); renderCurveRows();
    });
    window.addEventListener("pointerup", function () { curveDrag = null; drawBrakeCurve(); });
    cv.addEventListener("dblclick", function (evt) {
      var hit = locate(evt);
      if (hit.index < 0 || T.conf.brakeCurve.length <= 2) {
        var cmd = clamp((hit.mx - hit.pad) / (cv.width - hit.pad * 1.4), 0, 1);
        var torque = clamp(hit.ymax * (1 - (hit.my - hit.pad) / (cv.height - hit.pad * 2)), 0, 1e7);
        T.conf.brakeCurve.push([Math.round(cmd * 100) / 100, Math.round(torque)]);
        T.conf.brakeCurve.sort(function (a, b) { return a[0] - b[0]; });
      } else {
        T.conf.brakeCurve.splice(hit.index, 1);
      }
      invalidate(); drawBrakeCurve(); renderCurveRows();
    });
  }

  function renderCurveRows() {
    var box = $("trCurveRows");
    box.innerHTML = "";
    T.conf.brakeCurve.forEach(function (p, i) {
      var row = document.createElement("div");
      row.className = "curve-row";
      [["指令", p[0], 2, function (v) { p[0] = clamp(v, 0, 1); }],
       ["扭矩N·m", p[1], 0, function (v) { p[1] = Math.max(0, v); }]].forEach(function (f) {
        var lab = document.createElement("label");
        lab.textContent = f[0];
        var inp = document.createElement("input");
        inp.type = "number"; inp.step = "any"; inp.value = f[1];
        inp.addEventListener("input", function () {
          f[3](num(inp.value, f[1]));
          T.conf.brakeCurve.sort(function (a, b) { return a[0] - b[0]; });
          invalidate(); drawBrakeCurve();
        });
        lab.appendChild(inp); row.appendChild(lab);
      });
      if (T.conf.brakeCurve.length > 2) {
        var del = document.createElement("button");
        del.textContent = "×"; del.className = "danger";
        del.addEventListener("click", function () {
          T.conf.brakeCurve.splice(i, 1);
          invalidate(); drawBrakeCurve(); renderCurveRows();
        });
        row.appendChild(del);
      }
      box.appendChild(row);
    });
  }

  // --------------------------- 时间轴关键帧 ---------------------------
  var TL = { x0: 70, w: 800, tracks: { jog: 70, speed: 120, estop: 170 } };

  function kfColor(type) {
    return { jog: "#0ea5e9", speed: "#2563eb", estop: "#dc2626" }[type] || "#64748b";
  }
  function kfName(type) {
    return { jog: "点动", speed: "换速", estop: "急停" }[type] || type;
  }

  function renderTimeline() {
    var svg = $("trTimelineSvg");
    svg.innerHTML = "";
    var dur = Math.max(T.conf.duration, 1);
    function X(t) { return TL.x0 + (t / dur) * TL.w; }

    el("rect", { x: 0, y: 0, width: 900, height: 210, fill: "#f8fafc" }, svg);
    // 已确认/重算时段底色
    (T.result ? T.result.intervals : []).forEach(function (iv) {
      var col = iv.status === "locked" ? "#dcfce7"
        : iv.status === "failed" ? "#fecaca"
        : iv.status === "success" ? "#dbeafe" : "#f1f5f9";
      el("rect", { x: X(iv.start), y: 30, width: Math.max(2, X(iv.end) - X(iv.start)),
        height: 158, fill: col, opacity: 0.55 }, svg);
    });
    // 时间刻度
    for (var s = 0; s <= dur + 1e-9; s += Math.max(1, Math.round(dur / 20))) {
      el("line", { x1: X(s), y1: 34, x2: X(s), y2: 190, stroke: "#e2e8f0" }, svg);
      var t = el("text", { x: X(s), y: 202, "font-size": 11, fill: "#64748b",
        "text-anchor": "middle", text: s + "s" }, svg);
    }
    Object.keys(TL.tracks).forEach(function (type) {
      var y = TL.tracks[type];
      el("line", { x1: TL.x0 - 8, y1: y, x2: TL.x0 + TL.w + 8, y2: y,
        stroke: "#cbd5e1", "stroke-width": 1, "stroke-dasharray": "4 4" }, svg);
      el("text", { x: 10, y: y + 4, "font-size": 12, fill: kfColor(type),
        "font-weight": "bold", text: kfName(type) }, svg);
    });

    // 指令速度折线（输入曲线）
    var pts = commandProfile();
    if (pts.length) {
      var pl = pts.map(function (q) { return X(q[0]).toFixed(1) + "," + (44 - q[1] * 4).toFixed(1); }).join(" ");
      el("polyline", { points: pl, fill: "none", stroke: "#475569", "stroke-width": 1.6,
        "stroke-dasharray": "5 3" }, svg);
    }

    // 关键帧
    T.conf.keyframes.forEach(function (k) {
      var x = X(k.time), y = TL.tracks[k.type] || 120;
      var g = el("g", { class: "kf" + (T.selectedKf === k.id ? " selected" : "") }, svg);
      g.style.cursor = "grab";
      el("circle", { cx: x, cy: y, r: 9, fill: kfColor(k.type), stroke: "#fff",
        "stroke-width": 2, "data-id": k.id }, g);
      el("text", { x: x, y: y + 4, "font-size": 10, fill: "#fff", "text-anchor": "middle",
        "font-weight": "bold", text: { jog: "点", speed: "速", estop: "停" }[k.type] }, g);
      el("text", { x: x, y: y + 24, "font-size": 10, fill: "#334155", "text-anchor": "middle",
        text: k.time.toFixed(1) + "s" }, g);
      g.addEventListener("pointerdown", function (evt) {
        T.selectedKf = k.id;
        T.draggingKf = { id: k.id, x: evt.clientX };
        svg.setPointerCapture(evt.pointerId);
        renderTimeline(); renderKfList();
      });
    });

    // 冲突标记
    var conflict = T.result && T.result.conflict;
    if (conflict) {
      var cx = X(conflict.time);
      el("line", { x1: cx, y1: 30, x2: cx, y2: 190, stroke: "#dc2626", "stroke-width": 2 }, svg);
      el("polygon", { points: cx + ",30 " + (cx - 6) + ",18 " + (cx + 6) + ",18", fill: "#dc2626" }, svg);
    }
    // 回放头
    if (T.playHeadTime != null) {
      var hx = X(clamp(T.playHeadTime, 0, dur));
      el("line", { x1: hx, y1: 28, x2: hx, y2: 192, stroke: "#f97316", "stroke-width": 1.8 }, svg);
    }
  }

  function commandProfile() {
    // 与后端 build_commands 同构（仅用于时间轴预览）
    var dt = T.conf.dt, dur = T.conf.duration, pts = [[0, T.conf.startSpeed]], order = T.conf.keyframes.slice().sort(function (a, b) { return a.time - b.time; });
    function now(t) {
      var v = pts[0][1];
      for (var i = 0; i < pts.length; i++) if (pts[i][0] <= t + 1e-9) v = pts[i][1];
      return v;
    }
    function snap(t) { return Math.round(Math.round(t / dt) * dt * 1000) / 1000; }
    order.forEach(function (k) {
      var t0 = k.time;
      if (t0 > pts[pts.length - 1][0] + 1e-9) pts.push([t0, now(t0)]);
      if (k.type === "speed") {
        var ramp = k.ramp >= 0 ? k.ramp : 0.8;
        pts.push([snap(Math.min(dur, t0 + ramp)), k.speed]);
      } else if (k.type === "jog") {
        var rj = k.ramp >= 0 ? k.ramp : 0.3;
        var t1 = snap(Math.min(dur, t0 + rj));
        var t2 = snap(Math.min(dur, t1 + Math.max(k.hold, 0.05)));
        var t3 = snap(Math.min(dur, t2 + rj));
        pts.push([t1, k.speed], [t2, k.speed], [t3, now(t0)]);
      } else {
        var re = k.ramp >= 0 ? k.ramp : 0.25;
        pts.push([snap(Math.min(dur, t0 + re)), 0]);
      }
    });
    pts.sort(function (a, b) { return a[0] - b[0]; });
    return pts;
  }

  function bindTimelineDrag() {
    var svg = $("trTimelineSvg");
    svg.addEventListener("pointermove", function (evt) {
      if (!T.draggingKf) return;
      var k = T.conf.keyframes.find(function (q) { return q.id === T.draggingKf.id; });
      if (!k) return;
      var dur = Math.max(T.conf.duration, 1);
      var rect = svg.getBoundingClientRect();
      var mx = (evt.clientX - rect.left) / rect.width * 900;
      var t = clamp((mx - TL.x0) / TL.w * dur, 0, dur);
      k.time = Math.round(t / T.conf.dt) * T.conf.dt;
      invalidate(); renderTimeline(); renderKfList();
    });
    window.addEventListener("pointerup", function () {
      if (T.draggingKf) { T.draggingKf = null; renderTimeline(); }
    });
    // 轨道空白双击：按所在轨道快速加帧
    svg.addEventListener("dblclick", function (evt) {
      var rect = svg.getBoundingClientRect();
      var mx = (evt.clientX - rect.left) / rect.width * 900;
      var my = (evt.clientY - rect.top) / rect.height * 210;
      if (mx < TL.x0 || mx > TL.x0 + TL.w) return;
      var type = Math.abs(my - TL.tracks.jog) < 22 ? "jog"
        : Math.abs(my - TL.tracks.speed) < 22 ? "speed"
        : Math.abs(my - TL.tracks.estop) < 22 ? "estop" : null;
      if (!type) return;
      var dur = Math.max(T.conf.duration, 1);
      var t = clamp((mx - TL.x0) / TL.w * dur, 0, dur);
      addKeyframe(type, Math.round(t / T.conf.dt) * T.conf.dt);
    });
  }

  function addKeyframe(type, time) {
    var k = { id: uid(), type: type, time: time == null ? T.conf.duration / 2 : time,
      speed: type === "estop" ? 0 : 1, ramp: type === "jog" ? 0.5 : type === "estop" ? 0.4 : 1,
      hold: 1.2, brake: type === "estop" ? 1 : null };
    T.conf.keyframes.push(k);
    T.conf.keyframes.sort(function (a, b) { return a.time - b.time; });
    T.selectedKf = k.id;
    invalidate(); renderAll();
  }

  // --------------------------- 关键帧列表/编辑 ---------------------------
  function numField(labelText, value, step, fn) {
    var lab = document.createElement("label");
    lab.textContent = labelText;
    var inp = document.createElement("input");
    inp.type = "number"; inp.step = "any"; inp.value = value;
    inp.addEventListener("input", function () {
      fn(num(inp.value, value)); invalidate(); renderTimeline(); drawCharts();
    });
    lab.appendChild(inp);
    return lab;
  }

  function renderKfList() {
    var ol = $("trKfList");
    ol.innerHTML = "";
    T.conf.keyframes.slice().sort(function (a, b) { return a.time - b.time; }).forEach(function (k) {
      var li = document.createElement("li");
      li.className = "kf-item " + k.type + (T.selectedKf === k.id ? " active" : "");
      var head = document.createElement("div");
      head.className = "cp-title";
      head.innerHTML = "<span>" + kfName(k.type) + " @ " + k.time.toFixed(1) + "s</span>";
      var del = document.createElement("button");
      del.textContent = "删除"; del.className = "danger";
      del.addEventListener("click", function () {
        T.conf.keyframes = T.conf.keyframes.filter(function (q) { return q.id !== k.id; });
        if (T.selectedKf === k.id) T.selectedKf = null;
        invalidate(); renderAll();
      });
      head.appendChild(del);
      li.appendChild(head);
      li.appendChild(numField("时刻 t(s)", k.time, "0.1", function (v) {
        k.time = clamp(v, 0, T.conf.duration);
      }));
      if (k.type !== "estop")
        li.appendChild(numField("目标速度(m/s)", k.speed, "0.1", function (v) { k.speed = Math.max(0, v); }));
      if (k.type === "jog")
        li.appendChild(numField("保持时长(s)", k.hold, "0.1", function (v) { k.hold = Math.max(0.05, v); }));
      li.appendChild(numField(k.type === "estop" ? "制动斜坡(s)" : "升/降速斜坡(s)",
        k.ramp, "0.1", function (v) { k.ramp = Math.max(0, v); }));
      li.addEventListener("click", function () { T.selectedKf = k.id; renderKfList(); renderTimeline(); });
      ol.appendChild(li);
    });
  }

  // --------------------------- 逐时曲线 ---------------------------
  var CHART_PANELS = [
    { title: "速度 m/s", series: [["vCmd", "#475569", "指令速度"], ["vReel", "#2563eb", "盘轴线速度"]], limit: null },
    { title: "盘轴转速 r/min", series: [["rpm", "#7c3aed", "转速"]], limitKey: "rpmLimit" },
    { title: "张力 N", series: [["tension", "#f97316", "电缆张力"], ["inertiaTension", "#dc2626", "惯性附加"]], limitKey: "allowTension" },
    { title: "制动储热 J（柱：指令）", series: [["brakeHeat", "#dc2626", "累计储热"]], limitKey: "brakeHeatCapacity", cmdBars: true },
    { title: "松弛长度 m", series: [["slack", "#16a34a", "松弛圈"]], limitKey: "slackClearance" }
  ];

  function drawCharts() {
    var cv = $("trChartCanvas"), ctx = cv.getContext("2d");
    var W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#fff"; ctx.fillRect(0, 0, W, H);
    var rows = T.rows;
    var summary = T.result ? T.result.summary : null;
    var cfg = T.conf;
    var x0 = 76, pw = W - x0 - 90, ph = (H - 30) / CHART_PANELS.length;
    var dur = Math.max(cfg.duration, 1);
    function X(t) { return x0 + (t / dur) * pw; }

    CHART_PANELS.forEach(function (panel, pi) {
      var y0 = 8 + pi * ph, ih = ph - 16;
      ctx.strokeStyle = "#eef2f7"; ctx.lineWidth = 1;
      for (var g = 0; g <= 4; g++) {
        var gy = y0 + ih * g / 4;
        ctx.beginPath(); ctx.moveTo(x0, gy); ctx.lineTo(x0 + pw, gy); ctx.stroke();
      }
      ctx.strokeStyle = "#94a3b8";
      ctx.beginPath(); ctx.moveTo(x0, y0 + ih); ctx.lineTo(x0 + pw, y0 + ih);
      ctx.moveTo(x0, y0); ctx.lineTo(x0, y0 + ih); ctx.stroke();
      ctx.fillStyle = "#0f172a"; ctx.font = "bold 12px sans-serif";
      ctx.fillText(panel.title, 8, y0 + 14);

      var vals = [];
      rows.forEach(function (r) { panel.series.forEach(function (s) { vals.push(r[s[0]]); }); });
      var limit = null;
      if (panel.limitKey === "rpmLimit") limit = T.conf.brakeMaxSpeed * 60 / (2 * Math.PI);
      else if (panel.limitKey === "brakeHeatCapacity") limit = T.conf.brakeHeatCapacity;
      else if (panel.limitKey === "slackClearance") limit = T.conf.slackClearance;
      else if (panel.limitKey === "allowTension") limit = scenario().params.allowTension;
      if (limit != null) vals.push(limit);
      var vmin = Math.min.apply(null, vals.concat([0]));
      var vmax = Math.max.apply(null, vals.concat([1]));
      if (vmax - vmin < 1e-9) vmax += 1;
      var pad = (vmax - vmin) * 0.1; vmin -= pad; vmax += pad;
      function Y(v) { return y0 + ih * (1 - (v - vmin) / (vmax - vmin)); }

      ctx.font = "10px sans-serif"; ctx.fillStyle = "#94a3b8";
      ctx.fillText(vmax.toFixed(vmax > 1000 ? 0 : 2), x0 + pw + 4, y0 + 10);
      ctx.fillText(vmin.toFixed(vmin > 1000 ? 0 : 2), x0 + pw + 4, y0 + ih);

      if (panel.cmdBars) {
        ctx.fillStyle = "rgba(249,115,22,.25)";
        rows.forEach(function (r) {
          var hgt = r.brakeCmd * ih;
          ctx.fillRect(X(r.t) - 1, y0 + ih - hgt, 2, hgt);
        });
      }
      panel.series.forEach(function (s) {
        ctx.strokeStyle = s[1]; ctx.lineWidth = 1.8; ctx.beginPath();
        rows.forEach(function (r, i) {
          var xx = X(r.t), yy = Y(r[s[0]]);
          i ? ctx.lineTo(xx, yy) : ctx.moveTo(xx, yy);
        });
        ctx.stroke();
      });
      if (limit != null) {
        ctx.strokeStyle = "#dc2626"; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.1;
        ctx.beginPath(); ctx.moveTo(x0, Y(limit)); ctx.lineTo(x0 + pw, Y(limit)); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#dc2626"; ctx.font = "10px sans-serif";
        ctx.fillText("限值 " + (limit > 1000 ? Math.round(limit) : limit.toFixed(2)), x0 + pw - 70, Y(limit) - 3);
      }
      // 关键帧时刻标记
      T.conf.keyframes.forEach(function (k) {
        ctx.strokeStyle = kfColor(k.type); ctx.setLineDash([2, 3]); ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(X(k.time), y0); ctx.lineTo(X(k.time), y0 + ih); ctx.stroke();
        ctx.setLineDash([]);
      });
      var lx = x0 + 6;
      panel.series.forEach(function (s) {
        ctx.strokeStyle = s[1]; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(lx, y0 + ih - 6); ctx.lineTo(lx + 16, y0 + ih - 6); ctx.stroke();
        ctx.fillStyle = "#334155"; ctx.font = "10px sans-serif";
        ctx.fillText(s[2], lx + 20, y0 + ih - 2);
        lx += 26 + s[2].length * 11;
      });
    });

    // x 轴刻度
    ctx.fillStyle = "#64748b"; ctx.font = "10px sans-serif";
    for (var s2 = 0; s2 <= dur + 1e-9; s2 += Math.max(1, Math.round(dur / 20))) {
      ctx.fillText(s2 + "s", X(s2) - 8, H - 6);
    }
    // 冲突与回放头
    var conflict = T.result && T.result.conflict;
    if (conflict) {
      var cx = X(conflict.time);
      ctx.strokeStyle = "#dc2626"; ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(cx, 8); ctx.lineTo(cx, H - 20); ctx.stroke();
      ctx.fillStyle = "#dc2626"; ctx.font = "bold 11px sans-serif";
      ctx.fillText("最早冲突 " + conflict.time.toFixed(1) + "s " + conflict.title, cx + 5, 16);
    }
    if (T.playHeadTime != null) {
      var hx = X(clamp(T.playHeadTime, 0, dur));
      ctx.strokeStyle = "#f97316"; ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(hx, 8); ctx.lineTo(hx, H - 20); ctx.stroke();
    }
  }

  // --------------------------- 线路动画 ---------------------------
  function currentRow() {
    if (!T.rows.length) return null;
    if (T.playHeadTime == null) return T.rows[T.rows.length - 1];
    var best = T.rows[0], bd = 1e18;
    T.rows.forEach(function (r) {
      var d = Math.abs(r.t - T.playHeadTime);
      if (d < bd) { bd = d; best = r; }
    });
    return best;
  }

  function drawAnimation() {
    var svg = $("trAnimSvg");
    svg.innerHTML = "";
    var sc = scenario();
    if (!sc) return;
    var reel = sc.equipment.find(function (e) { return e.type === "reel"; });
    var puller = sc.equipment.find(function (e) { return e.type === "puller"; });
    if (!reel) {
      el("text", { x: 4, y: 8, "font-size": 1.4, fill: "#991b1b",
        text: "请先在静力页签放置盘架与牵引机。" }, svg);
      return;
    }
    var row = currentRow();
    var conflict = T.result && T.result.conflict;
    var rx = num(reel.x), ry = num(reel.y);
    var px = puller ? num(puller.x) : rx + 20;
    var py = puller ? num(puller.y) : ry;

    // 连接绳路
    var tensionRatio = row ? clamp(row.tension / Math.max(sc.params.allowTension || 18000, 1), 0, 1.5) : 0;
    var lineColor = tensionRatio > 1 ? "#dc2626" : tensionRatio > 0.6 ? "#f97316" : "#16a34a";
    el("line", { x1: rx, y1: ry, x2: px, y2: py, stroke: "#cbd5e1", "stroke-width": 0.5,
      "stroke-dasharray": "0.8 0.6" }, svg);
    el("line", { x1: rx, y1: ry, x2: px, y2: py, stroke: lineColor, "stroke-width": 0.28 + tensionRatio * 0.5 }, svg);

    // 松弛圈：盘前波浪
    if (row && row.slack > 0.01) {
      var n = clamp(Math.round(row.slack * 3), 1, 14), amp = clamp(0.3 + row.slack * 0.5, 0.3, 1.6);
      var d = "M" + rx.toFixed(2) + " " + ry.toFixed(2);
      for (var i = 1; i <= n; i++) {
        var fx = rx + (px - rx) * i / (n + 1);
        var fy = ry + (i % 2 ? -amp : amp) * 0.6;
        d += " Q" + (fx - 0.6).toFixed(2) + " " + (fy).toFixed(2) + " " + fx.toFixed(2) + " " + ry.toFixed(2);
      }
      var over = conflict && conflict.kind === "slack";
      el("path", { d: d, fill: "none", stroke: over ? "#dc2626" : "#16a34a",
        "stroke-width": 0.3, "stroke-linecap": "round" }, svg);
    }

    // 盘架：圆随卷绕半径缩放、按 omega 旋转
    var rWind = row ? row.radius : 1.1;
    var scale = clamp(rWind / 1.3, 0.6, 1.4);
    var ang = row ? (row.omega * 10) : 0;
    var rg = el("g", { transform: "translate(" + rx + " " + ry + ") rotate(" + ang + ") scale(" + scale + ")" }, svg);
    var reelBad = conflict && (conflict.kind === "tension" || conflict.kind === "overspeed"
      || conflict.kind === "heat" || conflict.kind === "reserve" || conflict.kind === "slack");
    el("circle", { cx: 0, cy: 0, r: 1.15, fill: "#ffedd5",
      stroke: reelBad ? "#dc2626" : "#f97316", "stroke-width": 0.2 }, rg);
    el("circle", { cx: 0, cy: 0, r: 0.55, fill: "none", stroke: reelBad ? "#dc2626" : "#c2410c",
      "stroke-width": 0.14 }, rg);
    el("line", { x1: -1.15, y1: 0, x2: 1.15, y2: 0, stroke: reelBad ? "#dc2626" : "#c2410c",
      "stroke-width": 0.12 }, rg);

    el("text", { x: rx, y: ry - 2, "font-size": 0.8, "text-anchor": "middle",
      fill: reelBad ? "#dc2626" : "#c2410c", "font-weight": "bold",
      text: row ? "r=" + row.radius.toFixed(2) + "m" : "盘架" }, svg);

    // 牵引机
    if (puller) {
      var puBad = conflict && conflict.kind === "tension";
      el("rect", { x: px - 0.9, y: py - 0.7, width: 1.8, height: 1.4, rx: 0.12,
        fill: "#dbeafe", stroke: puBad ? "#dc2626" : "#2563eb", "stroke-width": 0.2 }, svg);
      el("text", { x: px, y: py + 0.3, "font-size": 0.9, "text-anchor": "middle",
        fill: puBad ? "#dc2626" : "#1d4ed8", "font-weight": "bold", text: "牵" }, svg);
      if (row) {
        el("text", { x: px, y: py - 1.3, "font-size": 0.62, "text-anchor": "middle",
          fill: "#334155", text: "v=" + row.vCmd.toFixed(2) + "m/s" }, svg);
      }
    }
    if (conflict) {
      el("text", { x: 21, y: 2.2, "font-size": 1.1, fill: "#dc2626", "text-anchor": "middle",
        "font-weight": "bold", text: "⛔ " + conflict.title + " @ " + conflict.time.toFixed(1) + "s" }, svg);
    }
  }

  // --------------------------- 遥测/时段列表 ---------------------------
  function updateTelemetry(row) {
    if (!row) return;
    $("trTeleTime").textContent = row.t.toFixed(1) + " s";
    $("trTeleRadius").textContent = row.radius.toFixed(3) + " m";
    $("trTeleOmega").textContent = row.omega.toFixed(2) + " rad/s（" + row.rpm.toFixed(0) + " r/min）";
    $("trTeleDv").textContent = row.dv.toFixed(3) + " m/s";
    var t = $("trTeleTension");
    t.textContent = Math.round(row.tension).toLocaleString() + " N";
    t.style.color = row.tension > (scenario().params.allowTension || 18000) ? "#dc2626" : "";
    $("trTeleInertia").textContent = Math.round(row.inertiaTension).toLocaleString() + " N";
    var h = $("trTeleHeat");
    h.textContent = Math.round(row.brakeHeat).toLocaleString() + " J（" +
      Math.round(row.heatRatio * 100) + "%）";
    h.style.color = row.heatRatio > 1 ? "#dc2626" : "";
    var s = $("trTeleSlack");
    s.textContent = row.slack.toFixed(2) + " m";
    s.style.color = row.slack > T.conf.slackClearance ? "#dc2626" : "";
    $("trTeleReserve").textContent = row.reserve.toFixed(2) + " m";
    var st = $("trTeleState");
    var conflict = T.result && T.result.conflict;
    if (conflict && row.t >= conflict.time) {
      st.textContent = "⛔ " + conflict.title; st.style.color = "#dc2626";
    } else if (T.lockedCount && row.t < (T.result.intervals[T.lockedCount] || {}).start) {
      st.textContent = "✓ 已确认锁定"; st.style.color = "#16a34a";
    } else {
      st.textContent = "复算中"; st.style.color = "#2563eb";
    }
  }

  function renderIntervalList() {
    var ol = $("trIntervalList");
    ol.innerHTML = "";
    if (!T.result) {
      ol.innerHTML = '<li class="success">尚未复算。</li>';
      return;
    }
    T.result.intervals.forEach(function (iv, i) {
      var li = document.createElement("li");
      li.className = iv.status + (T.playHeadTime != null &&
        T.playHeadTime >= iv.start && T.playHeadTime <= iv.end ? " active" : "");
      var kf = T.conf.keyframes.find(function (k) { return k.id === iv.keyframeId; });
      var title = kf ? kfName(kf.type) + " @ " + kf.time.toFixed(1) + "s"
        : iv.key.indexOf("tail") === 0 ? "尾段保持" : "波及时段";
      li.innerHTML = '<div class="cp-title"><span>' + (i + 1) + ". " + title +
        "</span><span>" + { locked: "已确认", success: "算通", failed: "冲突", pending: "待算" }[iv.status] +
        "</span></div><div class='cp-num'>" + iv.start.toFixed(1) + "–" + iv.end.toFixed(1) + "s</div>";
      if (iv.status === "failed" && T.result.conflict) {
        var d = document.createElement("div");
        d.className = "cp-msg"; d.textContent = T.result.conflict.message;
        li.appendChild(d);
      }
      li.addEventListener("click", function () {
        stopPlay(); T.playHeadTime = iv.status === "locked" ? iv.start : iv.start;
        drawCharts(); drawAnimation(); updateTelemetry(currentRow()); renderIntervalList();
      });
      ol.appendChild(li);
    });
  }

  // --------------------------- 复算与增量锁定 ---------------------------
  function buildPayload(isContinue) {
    var p = {
      scenario: scenario(),
      transient: {
        emptyInertia: T.conf.emptyInertia, bearingTorque: T.conf.bearingTorque,
        reserveCable: T.conf.reserveCable, axialStiffness: T.conf.axialStiffness,
        slackClearance: T.conf.slackClearance, brakeMaxSpeed: T.conf.brakeMaxSpeed,
        brakeHeatCapacity: T.conf.brakeHeatCapacity, windingPack: T.conf.windingPack,
        startSpeed: T.conf.startSpeed, duration: T.conf.duration, dt: T.conf.dt,
        brakeCurve: T.conf.brakeCurve, keyframes: T.conf.keyframes
      }
    };
    if (isContinue && T.lockedCount > 0) {
      p.lockedCount = T.lockedCount;
      p.verifiedHashes = T.verifiedHashes;
      p.startState = T.startState;
    }
    return p;
  }

  function boundaryState(time) {
    var row = T.rows.find(function (r) { return Math.abs(r.t - time) < T.conf.dt * 0.5; });
    if (!row) return null;
    return { t: row.t, omega: row.omega, slack: row.slack, reserve: row.reserve,
      tension: row.tension, heat: row.brakeHeat, energy: row.brakeEnergy,
      paidOut: row.paidOut };
  }

  function mergeRows(res) {
    if (!res.fromTime || res.fromTime <= T.conf.dt * 1.5 || !T.rows.length) {
      T.rows = res.rows.slice();
      return;
    }
    var head = T.rows.filter(function (r) { return r.t < res.fromTime - T.conf.dt * 0.5; });
    T.rows = head.concat(res.rows);
  }

  function run(isContinue) {
    stopPlay();
    setMsg("后端逐时积分：卷绕半径、盘轴转速、速度差、惯性张力、制动储热与松弛…", "info");
    post("/api/transient/simulate", buildPayload(isContinue)).then(function (res) {
      var r = res.data;
      if (r.ok === false) { setMsg(r.error || "复算失败。", "error"); return; }
      if (r.status === "lock-changed") {
        T.lockedCount = 0; T.verifiedHashes = []; T.startState = null;
        setMsg("已确认时段的关键帧/制动参数已变化，已解除锁定，请整段重算：" + (r.error || ""), "warning");
        run(false);
        return;
      }
      T.result = r;
      mergeRows(r);
      if (r.status === "blocked") {
        T.lockedCount = r.failedInterval;
        T.verifiedHashes = r.hashes.slice(0, r.failedInterval);
        var nb = r.intervals[r.failedInterval];
        T.startState = boundaryState(nb.start);
        setMsg("回放停在最早冲突 t=" + r.conflict.time.toFixed(1) + "s：" +
          r.conflict.message + "  修改后可“仅重算波及时段”。", "error");
      } else {
        T.lockedCount = r.intervals.length;
        T.verifiedHashes = r.hashes.slice();
        T.startState = r.endState;
        setMsg("联动复算通过：最大张力 " + Math.round(r.summary.maxTension) +
          "N，最高转速 " + r.summary.maxOmega.toFixed(2) + "rad/s，最大松弛 " +
          r.summary.maxSlack.toFixed(2) + "m，可归档输出。", "success");
      }
      T.playHeadTime = 0;
      renderAll();
      startPlay();
    }).catch(function (e) { setMsg("请求失败：" + e.message, "error"); });
  }

  function invalidate() {
    // 关键帧/参数改动：不立即清空已锁定行，但标记结果失效；下次复算由后端 hash 把关。
    T.archiveId = null;
    ["trCsvBtn", "trSvgBtn", "trJsonBtn"].forEach(function (id) { $(id).disabled = true; });
    renderIntervalList();
    $("trContinueBtn").disabled = !(T.lockedCount > 0 && T.result && T.result.status === "blocked");
    $("trUnlockBtn").disabled = !(T.lockedCount > 0);
  }

  // --------------------------- 回放 ---------------------------
  function startPlay() {
    T.play.active = true;
    T.playHeadTime = 0;
    $("trPauseBtn").disabled = false;
    $("trPauseBtn").textContent = "⏸ 暂停";
    var last = performance.now();
    function frame(now) {
      if (!T.play.active) return;
      var speed = num($("trSpeedInput").value, 1);
      T.playHeadTime += (now - last) / 1000 * speed * 2;
      last = now;
      var conflict = T.result && T.result.conflict;
      if (conflict && T.playHeadTime >= conflict.time) {
        T.playHeadTime = conflict.time;
        stopPlay();
        renderFrame();
        return;
      }
      if (T.playHeadTime >= T.conf.duration) {
        T.playHeadTime = T.conf.duration;
        stopPlay();
        renderFrame();
        return;
      }
      renderFrame();
      requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
  }
  function stopPlay() {
    T.play.active = false;
    $("trPauseBtn").textContent = "▶ 继续回放";
  }
  function renderFrame() {
    var row = currentRow();
    drawCharts(); drawAnimation();
    if (row) updateTelemetry(row);
    renderTimeline(); renderIntervalList();
  }

  // --------------------------- 归档与输出 ---------------------------
  function archive() {
    var name = $("trArchiveName").value.trim() || ("联动版本 " + new Date().toLocaleString());
    var note = $("trArchiveNote").value;
    post("/api/archives", {
      name: name, note: note, scenario: scenario(),
      transient: buildPayload(false).transient
    }).then(function (res) {
      var r = res.data;
      if (!r.id) { setMsg(r.error || "归档失败", "error"); return; }
      T.archiveId = r.id;
      if (r.transientResult) {
        T.result = r.transientResult;
        T.rows = r.transientResult.rows.slice();
        T.lockedCount = r.transientResult.status === "blocked"
          ? r.transientResult.failedInterval : r.transientResult.intervals.length;
        T.verifiedHashes = r.transientResult.hashes.slice(0, T.lockedCount);
        T.startState = r.transientResult.endState;
      }
      ["trCsvBtn", "trSvgBtn", "trJsonBtn"].forEach(function (id) { $(id).disabled = false; });
      setMsg("已归档联动版本 #" + r.id + "，曲线 SVG、逐时 CSV、复算 JSON 均可下载。", "success");
      renderAll(); loadArchives();
    });
  }

  function loadArchives() {
    api("/api/archives").then(function (res) {
      var box = $("trArchiveList");
      box.innerHTML = "";
      (res.data.items || []).filter(function (it) { return it.hasTransient; }).forEach(function (it) {
        var div = document.createElement("div");
        div.className = "archive-item";
        div.innerHTML = "<strong>#" + it.id + " " + it.name + "</strong>" +
          "<small>" + it.createdAt + " ｜ 联动：" +
          (it.transientStatus === "passed" ? "✅ 通过" : it.transientStatus === "blocked" ? "⛔ 冲突" : it.transientStatus) +
          "</small>";
        var row = document.createElement("div"); row.className = "row";
        [["载入", "load"], ["CSV", "transient-csv"], ["SVG", "transient-svg"], ["JSON", "transient-json"]].forEach(function (pair) {
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
    if (kind !== "load") { window.open("/api/archives/" + id + "/" + kind, "_blank"); return; }
    api("/api/archives/" + id).then(function (res) {
      var d = res.data;
      if (!d.transient) { setMsg("该归档未含联动复算数据。", "warning"); return; }
      if (window.RiggingScenario) window.RiggingScenario.set(d.scenario);
      var t = d.transient;
      T.conf = {
        emptyInertia: num(t.emptyInertia), bearingTorque: num(t.bearingTorque),
        reserveCable: num(t.reserveCable), axialStiffness: num(t.axialStiffness),
        slackClearance: num(t.slackClearance), brakeMaxSpeed: num(t.brakeMaxSpeed),
        brakeHeatCapacity: num(t.brakeHeatCapacity), windingPack: num(t.windingPack, 0.85),
        startSpeed: num(t.startSpeed), duration: num(t.duration, 26), dt: num(t.dt, 0.1),
        brakeCurve: t.brakeCurve || DEFAULT_CURVE.map(function (p) { return p.slice(); }),
        keyframes: t.keyframes || []
      };
      T.result = d.transientResult;
      T.rows = (d.transientResult.rows || []).slice();
      T.lockedCount = d.transientResult.status === "blocked"
        ? d.transientResult.failedInterval : (d.transientResult.intervals || []).length;
      T.verifiedHashes = (d.transientResult.hashes || []).slice(0, T.lockedCount);
      T.startState = d.transientResult.endState;
      T.archiveId = id; T.playHeadTime = 0;
      $("trArchiveName").value = d.name;
      $("trArchiveNote").value = d.note;
      ["trCsvBtn", "trSvgBtn", "trJsonBtn"].forEach(function (bid) { $(bid).disabled = false; });
      setMsg("已载入联动归档 #" + id + "。", "info");
      renderAll();
    });
  }

  // --------------------------- 总渲染 ---------------------------
  function renderAll() {
    renderTimeline();
    renderKfList();
    renderCurveRows();
    drawBrakeCurve();
    drawCharts();
    drawAnimation();
    renderIntervalList();
    if (!T.rows.length) T.playHeadTime = null;
    else if (T.playHeadTime == null) T.playHeadTime = 0;
    var row = currentRow();
    if (row) updateTelemetry(row);
    $("trContinueBtn").disabled = !(T.lockedCount > 0 && T.result && T.result.status === "blocked");
    $("trUnlockBtn").disabled = !(T.lockedCount > 0);
  }

  function init() {
    renderParamForm();
    bindBrakeCanvas();
    bindTimelineDrag();
    document.querySelectorAll("[data-add]").forEach(function (b) {
      b.addEventListener("click", function () { addKeyframe(b.dataset.add); });
    });
    $("trCurveAdd").addEventListener("click", function () {
      var last = T.conf.brakeCurve[T.conf.brakeCurve.length - 1];
      T.conf.brakeCurve.push([Math.min(1, last[0] + 0.1), last[1]]);
      T.conf.brakeCurve.sort(function (a, b) { return a[0] - b[0]; });
      invalidate(); drawBrakeCurve(); renderCurveRows();
    });
    $("trCurveReset").addEventListener("click", function () {
      T.conf.brakeCurve = DEFAULT_CURVE.map(function (p) { return p.slice(); });
      invalidate(); drawBrakeCurve(); renderCurveRows();
    });
    $("trRunBtn").addEventListener("click", function () {
      T.lockedCount = 0; T.verifiedHashes = []; T.startState = null;
      run(false);
    });
    $("trContinueBtn").addEventListener("click", function () { run(true); });
    $("trUnlockBtn").addEventListener("click", function () {
      T.lockedCount = 0; T.verifiedHashes = []; T.startState = null;
      setMsg("已解除已确认时段锁定，将整段重算。", "info");
      renderAll();
    });
    $("trPauseBtn").addEventListener("click", function () {
      if (T.play.active) { stopPlay(); }
      else {
        T.play.active = true;
        var last = performance.now();
        (function loop(now) {
          if (!T.play.active) return;
          var speed = num($("trSpeedInput").value, 1);
          T.playHeadTime += (now - last) / 1000 * speed * 2; last = now;
          var cf = T.result && T.result.conflict;
          if ((cf && T.playHeadTime >= cf.time) || T.playHeadTime >= T.conf.duration) {
            T.playHeadTime = cf ? cf.time : T.conf.duration; stopPlay(); renderFrame(); return;
          }
          renderFrame(); requestAnimationFrame(loop);
        })(performance.now());
      }
    });
    $("trArchiveBtn").addEventListener("click", archive);
    ["trCsvBtn", "trSvgBtn", "trJsonBtn"].forEach(function (id) {
      $(id).addEventListener("click", function () {
        if (T.archiveId) {
          var kind = id === "trCsvBtn" ? "transient-csv"
            : id === "trSvgBtn" ? "transient-svg" : "transient-json";
          window.open("/api/archives/" + T.archiveId + "/" + kind, "_blank");
        }
      });
    });

    // 页签切换
    document.querySelectorAll("#viewTabs .tab").forEach(function (b) {
      b.addEventListener("click", function () {
        document.querySelectorAll("#viewTabs .tab").forEach(function (q) { q.classList.toggle("active", q === b); });
        var transientOn = b.dataset.view === "transient";
        $("staticView").hidden = transientOn;
        $("transientView").hidden = !transientOn;
        $("staticActions").style.visibility = transientOn ? "hidden" : "visible";
        if (transientOn) { renderAll(); drawCharts(); }
      });
    });

    // 静力侧编辑后，联动动画即时刷新
    ["RiggingChanged"].forEach(function () {});
    document.addEventListener("rigging:scenario-changed", function () {
      invalidate(); drawAnimation();
    });

    renderAll();
    loadArchives();
  }

  document.addEventListener("DOMContentLoaded", init);
})();
