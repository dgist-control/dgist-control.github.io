/**
 * pid_simulator.js
 * Interactive PID controller simulation for a first-order plant.
 *
 * Plant model (discrete-time, forward Euler):
 *   y[k+1] = y[k] + dt * (- a*y[k] + b*u[k])
 *
 * where a = 1 (time constant = 1 s), b = 1 (DC gain = 1).
 *
 * PID (discrete, anti-windup via clamping):
 *   e[k]   = r - y[k]
 *   P[k]   = Kp * e[k]
 *   I[k]   = I[k-1] + Ki * e[k] * dt   (clamped)
 *   D[k]   = Kd * (e[k] - e[k-1]) / dt
 *   u[k]   = clamp(P[k] + I[k] + D[k], -10, 10)
 */

(function () {
  "use strict";

  /* ── DOM references ── */
  const canvas   = document.getElementById("pid-canvas");
  const ctx      = canvas.getContext("2d");
  const btnRun   = document.getElementById("btn-run");
  const btnReset = document.getElementById("btn-reset");

  const sliders = {
    kp:       document.getElementById("kp"),
    ki:       document.getElementById("ki"),
    kd:       document.getElementById("kd"),
    setpoint: document.getElementById("setpoint"),
  };

  const displays = {
    kp:       document.getElementById("kp-val"),
    ki:       document.getElementById("ki-val"),
    kd:       document.getElementById("kd-val"),
    setpoint: document.getElementById("setpoint-val"),
  };

  /* ── Simulation parameters ── */
  const SIM_TIME = 20;      // seconds
  const DT       = 0.05;    // time step (s)
  const U_MAX    = 10;      // control signal clamp

  /* ── State ── */
  let running   = false;
  let animFrame = null;

  let t_data = [];
  let y_data = [];  // plant output
  let u_data = [];  // control signal
  let r_data = [];  // setpoint

  /* ── Helper: clamp ── */
  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  /* ── Run full simulation and store data ── */
  function runSimulation() {
    const Kp = parseFloat(sliders.kp.value);
    const Ki = parseFloat(sliders.ki.value);
    const Kd = parseFloat(sliders.kd.value);
    const r  = parseFloat(sliders.setpoint.value);

    /* Plant parameters */
    const a = 1.0;  // pole at s = -1
    const b = 1.0;

    const N = Math.round(SIM_TIME / DT) + 1;

    t_data = new Float64Array(N);
    y_data = new Float64Array(N);
    u_data = new Float64Array(N);
    r_data = new Float64Array(N);

    let y    = 0.0;   // plant state
    let I    = 0.0;   // integral accumulator
    let e_prev = r;   // previous error (for derivative)

    for (let k = 0; k < N; k++) {
      const t = k * DT;
      const e = r - y;

      /* PID terms */
      const P = Kp * e;
      I = clamp(I + Ki * e * DT, -U_MAX, U_MAX);   // anti-windup clamp
      const D = Kd * (e - e_prev) / DT;
      const u = clamp(P + I + D, -U_MAX, U_MAX);

      /* Store */
      t_data[k] = t;
      y_data[k] = y;
      u_data[k] = u;
      r_data[k] = r;

      /* Advance plant (forward Euler) */
      y = y + DT * (-a * y + b * u);
      e_prev = e;
    }
  }

  /* ── Canvas sizing ── */
  function resizeCanvas() {
    const w = canvas.parentElement.clientWidth - 4;  // 2px border × 2
    canvas.width  = w;
    canvas.height = 300;
  }

  /* ── Draw everything ── */
  function draw() {
    resizeCanvas();
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (t_data.length === 0) {
      drawPlaceholder();
      return;
    }

    drawGrid();
    drawSeries(y_data, "#4a7cdc", 2.5, false);   // output
    drawSeries(r_data, "#e63946", 1.5, true);     // setpoint (dashed)
    drawSeriesScaled(u_data, "#2a9d8f", 1.5);     // control signal (scaled)
    drawAxes();
    drawPerformanceStats();
  }

  function drawPlaceholder() {
    ctx.fillStyle = "#adb5bd";
    ctx.font = "14px 'Segoe UI', sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Press ▶ Run Simulation to start", canvas.width / 2, canvas.height / 2);
  }

  /* ── Grid ── */
  function drawGrid() {
    const { W, H, MARGIN } = layout();
    ctx.strokeStyle = "#e9ecef";
    ctx.lineWidth   = 1;

    /* Horizontal grid lines (y axis: 0 … 1.4 in data space) */
    for (let v = 0; v <= 1.4; v += 0.2) {
      const cy = dataToCanvasY(v, H, MARGIN);
      ctx.beginPath();
      ctx.moveTo(MARGIN.left, cy);
      ctx.lineTo(MARGIN.left + W, cy);
      ctx.stroke();
    }

    /* Vertical grid lines */
    for (let t = 0; t <= SIM_TIME; t += 5) {
      const cx = dataToCanvasX(t, W, MARGIN);
      ctx.beginPath();
      ctx.moveTo(cx, MARGIN.top);
      ctx.lineTo(cx, MARGIN.top + H);
      ctx.stroke();
    }
  }

  /* ── Axes ── */
  function drawAxes() {
    const { W, H, MARGIN } = layout();
    ctx.strokeStyle = "#495057";
    ctx.lineWidth   = 1.5;
    ctx.fillStyle   = "#495057";
    ctx.font        = "11px 'Segoe UI', sans-serif";

    /* X axis */
    ctx.beginPath();
    ctx.moveTo(MARGIN.left, MARGIN.top + H);
    ctx.lineTo(MARGIN.left + W, MARGIN.top + H);
    ctx.stroke();

    /* Y axis */
    ctx.beginPath();
    ctx.moveTo(MARGIN.left, MARGIN.top);
    ctx.lineTo(MARGIN.left, MARGIN.top + H);
    ctx.stroke();

    /* X labels */
    ctx.textAlign = "center";
    for (let t = 0; t <= SIM_TIME; t += 5) {
      const cx = dataToCanvasX(t, W, MARGIN);
      ctx.fillText(t + "s", cx, MARGIN.top + H + 14);
    }

    /* Y labels */
    ctx.textAlign = "right";
    for (let v = 0; v <= 1.4; v += 0.2) {
      const cy = dataToCanvasY(v, H, MARGIN);
      ctx.fillText(v.toFixed(1), MARGIN.left - 6, cy + 4);
    }

    /* Axis labels */
    ctx.textAlign   = "center";
    ctx.fillStyle   = "#6c757d";
    ctx.font        = "12px 'Segoe UI', sans-serif";
    ctx.fillText("Time (s)", MARGIN.left + W / 2, canvas.height - 2);
  }

  /* ── Draw a data series ── */
  function drawSeries(data, color, lineWidth, dashed) {
    const { W, H, MARGIN } = layout();
    ctx.strokeStyle = color;
    ctx.lineWidth   = lineWidth;
    ctx.setLineDash(dashed ? [6, 4] : []);
    ctx.beginPath();

    for (let k = 0; k < t_data.length; k++) {
      const cx = dataToCanvasX(t_data[k], W, MARGIN);
      const cy = dataToCanvasY(data[k], H, MARGIN);
      if (k === 0) ctx.moveTo(cx, cy);
      else         ctx.lineTo(cx, cy);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  /* Draw control signal, scaled to [0, 1.4] data range for visual comparison */
  function drawSeriesScaled(data, color, lineWidth) {
    /* Find max magnitude for scaling */
    let maxAbs = 0;
    for (let k = 0; k < data.length; k++) {
      if (Math.abs(data[k]) > maxAbs) maxAbs = Math.abs(data[k]);
    }
    if (maxAbs === 0) maxAbs = 1;

    const { W, H, MARGIN } = layout();
    ctx.strokeStyle = color;
    ctx.lineWidth   = lineWidth;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();

    for (let k = 0; k < t_data.length; k++) {
      const cx = dataToCanvasX(t_data[k], W, MARGIN);
      const scaled = (data[k] / maxAbs) * 1.2;   // scale to ~1.2 top
      const cy = dataToCanvasY(scaled, H, MARGIN);
      if (k === 0) ctx.moveTo(cx, cy);
      else         ctx.lineTo(cx, cy);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  /* ── Performance stats (rise time, overshoot, steady-state error) ── */
  function drawPerformanceStats() {
    const r  = r_data[0];
    const yf = y_data[y_data.length - 1];
    const ss_error = Math.abs(r - yf);

    /* Rise time: first time output crosses 90% of setpoint */
    let riseTime = null;
    for (let k = 0; k < y_data.length; k++) {
      if (y_data[k] >= 0.9 * r) {
        riseTime = t_data[k];
        break;
      }
    }

    /* Overshoot */
    let ymax = -Infinity;
    for (let k = 0; k < y_data.length; k++) {
      if (y_data[k] > ymax) ymax = y_data[k];
    }
    const overshoot = r > 0 ? Math.max(0, (ymax - r) / r * 100) : 0;

    /* Draw box */
    const x0 = canvas.width - 185;
    const y0 = 10;
    ctx.fillStyle = "rgba(255,255,255,0.88)";
    ctx.strokeStyle = "#dee2e6";
    ctx.lineWidth = 1;
    roundRect(ctx, x0, y0, 175, 72, 6);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle   = "#343a40";
    ctx.font        = "11px 'Segoe UI', sans-serif";
    ctx.textAlign   = "left";
    ctx.fillText("Rise time:  " + (riseTime !== null ? riseTime.toFixed(2) + " s" : "— "), x0 + 10, y0 + 18);
    ctx.fillText("Overshoot: " + overshoot.toFixed(1) + " %",              x0 + 10, y0 + 36);
    ctx.fillText("Steady-state error: " + ss_error.toFixed(4),             x0 + 10, y0 + 54);
  }

  /* ── Layout helper ── */
  function layout() {
    const MARGIN = { top: 15, right: 20, bottom: 30, left: 45 };
    const W = canvas.width  - MARGIN.left - MARGIN.right;
    const H = canvas.height - MARGIN.top  - MARGIN.bottom;
    return { W, H, MARGIN };
  }

  /* ── Coordinate transforms ── */
  function dataToCanvasX(t, W, MARGIN) {
    return MARGIN.left + (t / SIM_TIME) * W;
  }

  function dataToCanvasY(v, H, MARGIN) {
    const Y_MIN = -0.1, Y_MAX = 1.4;
    return MARGIN.top + H - ((v - Y_MIN) / (Y_MAX - Y_MIN)) * H;
  }

  /* ── Utility: rounded rectangle path ── */
  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r);
    ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    ctx.lineTo(x + r, y + h);
    ctx.quadraticCurveTo(x, y + h, x, y + h - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
  }

  /* ── Slider live-update ── */
  function bindSlider(id, decimals) {
    sliders[id].addEventListener("input", function () {
      displays[id].textContent = parseFloat(this.value).toFixed(decimals);
    });
  }

  bindSlider("kp",       1);
  bindSlider("ki",       2);
  bindSlider("kd",       2);
  bindSlider("setpoint", 2);

  /* ── Button handlers ── */
  btnRun.addEventListener("click", function () {
    runSimulation();
    draw();
  });

  btnReset.addEventListener("click", function () {
    t_data = [];
    y_data = [];
    u_data = [];
    r_data = [];

    /* Reset sliders to defaults */
    sliders.kp.value       = "2.0";
    sliders.ki.value       = "0.5";
    sliders.kd.value       = "0.2";
    sliders.setpoint.value = "1.0";
    displays.kp.textContent       = "2.0";
    displays.ki.textContent       = "0.50";
    displays.kd.textContent       = "0.20";
    displays.setpoint.textContent = "1.00";

    draw();
  });

  /* ── Resize handler ── */
  window.addEventListener("resize", function () {
    draw();
  });

  /* ── Initial draw (placeholder) ── */
  resizeCanvas();
  drawPlaceholder();
})();
