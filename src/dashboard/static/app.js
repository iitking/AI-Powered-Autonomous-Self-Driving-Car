// Autonomous Vehicle Cockpit Dashboard Frontend Script
const trackCanvas = document.getElementById("trackCanvas");
const trackCtx = trackCanvas.getContext("2d");
const camCanvas = document.getElementById("camCanvas");
const camCtx = camCanvas.getContext("2d");

// UI Elements
const modeBadge = document.getElementById("mode-badge");
const stateBadge = document.getElementById("state-badge");
const lapVal = document.getElementById("lap-val");
const timeVal = document.getElementById("time-val");
const estopBanner = document.getElementById("estop-banner");
const maneuverVal = document.getElementById("maneuver-val");
const signalVal = document.getElementById("signal-val");

const speedVal = document.getElementById("speed-val");
const speedBar = document.getElementById("speed-bar");
const steerVal = document.getElementById("steer-val");
const steerNeedle = document.getElementById("steer-needle");
const cteVal = document.getElementById("cte-val");
const cteMarker = document.getElementById("cte-marker");
const curvVal = document.getElementById("curv-val");

const pidP = document.getElementById("pid-p");
const pidI = document.getElementById("pid-i");
const pidD = document.getElementById("pid-d");
const pidOut = document.getElementById("pid-out");

const btnModeToggle = document.getElementById("btn-mode-toggle");
const btnToggleLight = document.getElementById("btn-toggle-light");
const btnDropObs = document.getElementById("btn-drop-obs");
const btnToggleTraffic = document.getElementById("btn-toggle-traffic");
const btnStallTraffic = document.getElementById("btn-stall-traffic");
const btnPedestrian = document.getElementById("btn-pedestrian");
const btnClearObs = document.getElementById("btn-clear-obs");
const btnReset = document.getElementById("btn-reset");
const btnApplyPid = document.getElementById("btn-apply-pid");
const manualHelp = document.getElementById("manual-help");

let showLidar = true;
let isManualMode = false;
let manualSteer = 0.0;
let manualThrottle = 0.0;
let keysPressed = {};

// Viewport transform for 2D track canvas
const SCALE = 70.0; // pixels per meter
const ORIGIN_X = trackCanvas.width / 2;
const ORIGIN_Y = trackCanvas.height / 2;

// Fetch simulation state
async function fetchState() {
  try {
    const res = await fetch("/api/state");
    if (!res.ok) return;
    const state = await res.json();
    renderDashboard(state);
  } catch (err) {
    // Network or server down
  }
}

function renderDashboard(state) {
  const veh = state.vehicle;
  const ctrl = state.control;
  const perc = state.perception;
  const world = state.world;

  // 1. Update Badges & Status
  isManualMode = (ctrl.mode === "MANUAL");
  modeBadge.textContent = ctrl.mode;
  modeBadge.style.color = isManualMode ? "#ffd600" : "#00e5ff";
  modeBadge.style.borderColor = isManualMode ? "#ffd600" : "#00e5ff";
  btnModeToggle.textContent = isManualMode ? "Switch to Autonomous" : "Switch to Manual";
  manualHelp.classList.toggle("hidden", !isManualMode);

  stateBadge.textContent = ctrl.state;
  if (ctrl.emergency_brake) {
    stateBadge.textContent = "EMERGENCY STOP";
    stateBadge.style.color = "#ff1744";
    stateBadge.style.borderColor = "#ff1744";
    estopBanner.classList.remove("hidden");
  } else {
    stateBadge.style.color = "#00e676";
    stateBadge.style.borderColor = "#00e676";
    estopBanner.classList.add("hidden");
  }

  lapVal.textContent = state.lap;
  timeVal.textContent = `${state.sim_time.toFixed(1)}s`;
  maneuverVal.textContent = ctrl.maneuver;

  // Signal
  const tlState = perc.traffic_light.state;
  signalVal.textContent = tlState;
  signalVal.className = `signal-${tlState.toLowerCase()}`;

  // Telemetry Gauges
  speedVal.innerHTML = `${veh.velocity_kmh.toFixed(1)} <small>km/h</small>`;
  const speedPct = Math.min(100, (veh.velocity_mps / 1.8) * 100);
  speedBar.style.width = `${speedPct}%`;

  steerVal.textContent = `${veh.steer_angle_deg >= 0 ? "+" : ""}${veh.steer_angle_deg.toFixed(1)}°`;
  const steerNorm = (veh.steer_angle_deg / 30.0); // [-1, 1]
  // Negative angle is right, positive angle is left
  steerNeedle.style.left = `${50 - steerNorm * 45}%`;

  cteVal.innerHTML = `${perc.cte_m >= 0 ? "+" : ""}${perc.cte_m.toFixed(2)} <small>m</small>`;
  const cteNorm = Math.max(-1, Math.min(1, perc.cte_m / 0.5));
  cteMarker.style.left = `${50 + cteNorm * 45}%`;

  curvVal.innerHTML = `${perc.curvature_radius_m > 500 ? "∞" : perc.curvature_radius_m.toFixed(1)} <small>m</small>`;

  // PID
  pidP.textContent = ctrl.pid_steer.p.toFixed(2);
  pidI.textContent = ctrl.pid_steer.i.toFixed(2);
  pidD.textContent = ctrl.pid_steer.d.toFixed(2);
  pidOut.textContent = `${ctrl.pid_steer.output.toFixed(1)}°`;

  // Render Visuals
  renderTrackCanvas(state);
  renderCameraView(state);
}

function worldToScreen(x, y) {
  return {
    sx: ORIGIN_X + x * SCALE,
    sy: ORIGIN_Y - y * SCALE, // Invert Y for screen coordinates
  };
}

function renderTrackCanvas(state) {
  const ctx = trackCtx;
  const w = trackCanvas.width;
  const h = trackCanvas.height;
  const world = state.world;
  const veh = state.vehicle;

  ctx.clearRect(0, 0, w, h);

  // Draw Ground Grid
  ctx.strokeStyle = "#141c2e";
  ctx.lineWidth = 1;
  const gridStep = 0.5 * SCALE;
  for (let x = 0; x < w; x += gridStep) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += gridStep) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  // Draw Oval Track As Asphalt Ribbon
  const r = world.track_radius;
  const straight = world.straight_length;
  const laneW = world.lane_width;

  // Road boundary (outer and inner)
  function drawTrackPath(offsetRadius, offsetStraight) {
    ctx.beginPath();
    // Top straight
    const p1 = worldToScreen(-offsetStraight / 2, offsetRadius);
    const p2 = worldToScreen(offsetStraight / 2, offsetRadius);
    ctx.moveTo(p1.sx, p1.sy);
    ctx.lineTo(p2.sx, p2.sy);
    // Right arc
    const centerRight = worldToScreen(offsetStraight / 2, 0);
    ctx.arc(centerRight.sx, centerRight.sy, offsetRadius * SCALE, -Math.PI / 2, Math.PI / 2, false);
    // Bottom straight
    const p3 = worldToScreen(offsetStraight / 2, -offsetRadius);
    const p4 = worldToScreen(-offsetStraight / 2, -offsetRadius);
    ctx.lineTo(p4.sx, p4.sy);
    // Left arc
    const centerLeft = worldToScreen(-offsetStraight / 2, 0);
    ctx.arc(centerLeft.sx, centerLeft.sy, offsetRadius * SCALE, Math.PI / 2, -Math.PI / 2, false);
    ctx.closePath();
  }

  // Draw road surface
  ctx.save();
  ctx.lineWidth = laneW * SCALE * 1.8;
  ctx.strokeStyle = "#1b2234";
  ctx.lineCap = "round";
  drawTrackPath(r, straight);
  ctx.stroke();

  // Draw white borders
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#475569";
  drawTrackPath(r + laneW * 0.9, straight);
  ctx.stroke();
  drawTrackPath(Math.max(0.2, r - laneW * 0.9), straight);
  ctx.stroke();

  // Draw dashed yellow centerline
  ctx.setLineDash([8, 8]);
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#eab308";
  drawTrackPath(r, straight);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.restore();

  // Draw Traffic Light
  world.traffic_lights.forEach((tl) => {
    const p = worldToScreen(tl.x, tl.y);
    // Pole
    ctx.fillStyle = "#334155";
    ctx.fillRect(p.sx - 6, p.sy - 24, 12, 48);

    const colors = ["#ef4444", "#eab308", "#22c55e"];
    const states = ["RED", "YELLOW", "GREEN"];
    for (let i = 0; i < 3; i++) {
      ctx.beginPath();
      ctx.arc(p.sx, p.sy - 14 + i * 14, 5, 0, Math.PI * 2);
      if (tl.state === states[i]) {
        ctx.fillStyle = colors[i];
        ctx.shadowColor = colors[i];
        ctx.shadowBlur = 10;
      } else {
        ctx.fillStyle = "#1e293b";
        ctx.shadowBlur = 0;
      }
      ctx.fill();
    }
    ctx.shadowBlur = 0;
  });

  // Draw Obstacles
  world.obstacles.forEach((obs) => {
    const p = worldToScreen(obs.x, obs.y);
    ctx.beginPath();
    ctx.arc(p.sx, p.sy, obs.radius * SCALE, 0, Math.PI * 2);
    ctx.fillStyle = "#f97316";
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#ea580c";
    ctx.stroke();

    // Danger halo
    ctx.beginPath();
    ctx.arc(p.sx, p.sy, (obs.radius + 0.15) * SCALE, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(249, 115, 22, 0.25)";
    ctx.stroke();
  });

  // Draw Crosswalk on bottom straight
  const cwP = worldToScreen(-0.5, -world.track_radius);
  ctx.save();
  ctx.fillStyle = "rgba(255, 255, 255, 0.35)";
  const cwWidth = world.lane_width * SCALE * 0.9;
  for (let s = -22; s <= 22; s += 8) {
    ctx.fillRect(cwP.sx + s, cwP.sy - cwWidth / 2, 4, cwWidth);
  }
  ctx.restore();

  // Draw Dynamic NPC Traffic Cars
  if (world.traffic_cars) {
    world.traffic_cars.forEach((tc) => {
      drawNpcCarOnTrack(tc);
    });
  }

  // Draw Dynamic Pedestrians
  if (world.pedestrians) {
    world.pedestrians.forEach((ped) => {
      if (ped.is_active) {
        drawPedestrianOnTrack(ped);
      }
    });
  }

  // Draw Lookahead Target Point & Path Preview
  if (world.target_point && (world.target_point.x !== 0 || world.target_point.y !== 0)) {
    const carP = worldToScreen(veh.x, veh.y);
    const targetP = worldToScreen(world.target_point.x, world.target_point.y);

    // Dotted trajectory line to lookahead point
    ctx.beginPath();
    ctx.moveTo(carP.sx, carP.sy);
    ctx.lineTo(targetP.sx, targetP.sy);
    ctx.strokeStyle = "rgba(0, 230, 118, 0.7)";
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    // Glowing target waypoint dot
    ctx.beginPath();
    ctx.arc(targetP.sx, targetP.sy, 5, 0, Math.PI * 2);
    ctx.fillStyle = "#00e676";
    ctx.shadowColor = "#00e676";
    ctx.shadowBlur = 8;
    ctx.fill();
    ctx.shadowBlur = 0;
  }

  // Draw LiDAR Beams
  if (showLidar && world.lidar_rays) {
    const carP = worldToScreen(veh.x, veh.y);
    world.lidar_rays.forEach((ray) => {
      const hitX = ray[2];
      const hitY = ray[3];
      const hitP = worldToScreen(hitX, hitY);

      ctx.beginPath();
      ctx.moveTo(carP.sx, carP.sy);
      ctx.lineTo(hitP.sx, hitP.sy);
      ctx.strokeStyle = "rgba(0, 229, 255, 0.35)";
      ctx.lineWidth = 1;
      ctx.stroke();

      // Hit point dot
      ctx.beginPath();
      ctx.arc(hitP.sx, hitP.sy, 2.5, 0, Math.PI * 2);
      ctx.fillStyle = "#00e5ff";
      ctx.fill();
    });
  }

  // Draw Autonomous Vehicle
  drawCarOnTrack(veh);
}

function drawCarOnTrack(veh) {
  const ctx = trackCtx;
  const p = worldToScreen(veh.x, veh.y);
  const yaw = -veh.yaw_rad; // Invert yaw for screen coords
  const steerRad = -((veh.steer_angle_deg * Math.PI) / 180.0);

  const carLength = 0.32 * SCALE;
  const carWidth = 0.20 * SCALE;

  ctx.save();
  ctx.translate(p.sx, p.sy);
  ctx.rotate(yaw);

  // Headlight beams
  const grad = ctx.createRadialGradient(carLength * 0.6, 0, 5, carLength * 1.5, 0, carLength * 1.5);
  grad.addColorStop(0, "rgba(255, 255, 200, 0.4)");
  grad.addColorStop(1, "rgba(255, 255, 200, 0.0)");
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.moveTo(carLength * 0.4, -carWidth * 0.4);
  ctx.lineTo(carLength * 1.8, -carWidth * 1.2);
  ctx.lineTo(carLength * 1.8, carWidth * 1.2);
  ctx.lineTo(carLength * 0.4, carWidth * 0.4);
  ctx.fill();

  // Wheels
  ctx.fillStyle = "#0f172a";
  const wheelL = carLength * 0.28;
  const wheelW = carWidth * 0.25;

  // Rear Wheels (fixed)
  ctx.fillRect(-carLength * 0.4, -carWidth * 0.5 - wheelW / 2, wheelL, wheelW);
  ctx.fillRect(-carLength * 0.4, carWidth * 0.5 - wheelW / 2, wheelL, wheelW);

  // Front Wheels (steered)
  ctx.save();
  ctx.translate(carLength * 0.35, -carWidth * 0.5);
  ctx.rotate(steerRad);
  ctx.fillRect(-wheelL / 2, -wheelW / 2, wheelL, wheelW);
  ctx.restore();

  ctx.save();
  ctx.translate(carLength * 0.35, carWidth * 0.5);
  ctx.rotate(steerRad);
  ctx.fillRect(-wheelL / 2, -wheelW / 2, wheelL, wheelW);
  ctx.restore();

  // Car Body Chassis
  ctx.fillStyle = "#0284c7";
  ctx.strokeStyle = "#38bdf8";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-carLength / 2, -carWidth / 2, carLength, carWidth, 4);
  ctx.fill();
  ctx.stroke();

  // Windshield
  ctx.fillStyle = "#0369a1";
  ctx.fillRect(carLength * 0.05, -carWidth * 0.35, carLength * 0.2, carWidth * 0.7);

  // Front heading indicator dot
  ctx.fillStyle = "#38bdf8";
  ctx.beginPath();
  ctx.arc(carLength * 0.45, 0, 3, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

function drawNpcCarOnTrack(tc) {
  const ctx = trackCtx;
  const p = worldToScreen(tc.x, tc.y);
  const yaw = -tc.yaw;

  const carLength = 0.32 * SCALE;
  const carWidth = 0.20 * SCALE;

  ctx.save();
  ctx.translate(p.sx, p.sy);
  ctx.rotate(yaw);

  // Wheels
  ctx.fillStyle = "#1e1b4b";
  const wheelL = carLength * 0.28;
  const wheelW = carWidth * 0.25;
  ctx.fillRect(-carLength * 0.4, -carWidth * 0.5 - wheelW / 2, wheelL, wheelW);
  ctx.fillRect(-carLength * 0.4, carWidth * 0.5 - wheelW / 2, wheelL, wheelW);
  ctx.fillRect(carLength * 0.35 - wheelL / 2, -carWidth * 0.5 - wheelW / 2, wheelL, wheelW);
  ctx.fillRect(carLength * 0.35 - wheelL / 2, carWidth * 0.5 - wheelW / 2, wheelL, wheelW);

  // Chassis (Purple for active NPC, Amber/Red for stalled)
  ctx.fillStyle = tc.is_stalled ? "#7c2d12" : "#7e22ce";
  ctx.strokeStyle = tc.is_stalled ? "#ea580c" : "#c084fc";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-carLength / 2, -carWidth / 2, carLength, carWidth, 4);
  ctx.fill();
  ctx.stroke();

  // Windshield
  ctx.fillStyle = "#3b0764";
  ctx.fillRect(carLength * 0.05, -carWidth * 0.35, carLength * 0.2, carWidth * 0.7);

  // Headlights
  ctx.fillStyle = tc.is_stalled ? "#f59e0b" : "#fef08a";
  ctx.beginPath();
  ctx.arc(carLength * 0.45, -carWidth * 0.3, 2.5, 0, Math.PI * 2);
  ctx.arc(carLength * 0.45, carWidth * 0.3, 2.5, 0, Math.PI * 2);
  ctx.fill();

  if (tc.is_stalled) {
    ctx.fillStyle = "#f87171";
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("STALLED", 0, -carWidth * 0.7);
  }

  ctx.restore();
}

function drawPedestrianOnTrack(ped) {
  const ctx = trackCtx;
  const p = worldToScreen(ped.x, ped.y);

  ctx.save();
  // Pedestrian aura / pulsing halo
  ctx.beginPath();
  ctx.arc(p.sx, p.sy, 11, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(236, 72, 153, 0.25)";
  ctx.fill();

  // Body
  ctx.beginPath();
  ctx.arc(p.sx, p.sy, 5.5, 0, Math.PI * 2);
  ctx.fillStyle = "#ec4899";
  ctx.shadowColor = "#ec4899";
  ctx.shadowBlur = 6;
  ctx.fill();
  ctx.shadowBlur = 0;

  // Head / label
  ctx.fillStyle = "#f472b6";
  ctx.font = "bold 9px sans-serif";
  ctx.textAlign = "center";
  ctx.fillText("🚶 PED", p.sx, p.sy - 9);
  ctx.restore();
}

function renderCameraView(state) {
  const ctx = camCtx;
  const w = camCanvas.width;
  const h = camCanvas.height;
  const perc = state.perception;
  const veh = state.vehicle;

  // Clear Sky & Road
  ctx.fillStyle = "#0f172a";
  ctx.fillRect(0, 0, w, h * 0.45); // Sky / horizon
  ctx.fillStyle = "#1e293b";
  ctx.fillRect(0, h * 0.45, w, h * 0.55); // Asphalt

  // Vanishing point
  const vpX = w / 2 - perc.cte_m * 150 - veh.steer_angle_deg * 2.0;
  const vpY = h * 0.45;

  // Draw Detected Lane Polygon (Green Overlay)
  ctx.fillStyle = "rgba(34, 197, 94, 0.2)";
  ctx.beginPath();
  ctx.moveTo(vpX - 10, vpY);
  ctx.lineTo(w * 0.15, h);
  ctx.lineTo(w * 0.85, h);
  ctx.lineTo(vpX + 10, vpY);
  ctx.closePath();
  ctx.fill();

  // Draw Left and Right Lane Lines
  ctx.lineWidth = 4;
  ctx.strokeStyle = "#ffffff";
  ctx.beginPath();
  ctx.moveTo(vpX - 15, vpY);
  ctx.lineTo(w * 0.15, h);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(vpX + 15, vpY);
  ctx.lineTo(w * 0.85, h);
  ctx.stroke();

  // Draw Center dashed line
  ctx.setLineDash([12, 12]);
  ctx.strokeStyle = "#eab308";
  ctx.beginPath();
  ctx.moveTo(vpX, vpY);
  ctx.lineTo(w / 2, h);
  ctx.stroke();
  ctx.setLineDash([]);

  // Draw Detected Bounding Boxes from YOLO
  perc.detected_objects.forEach((obj) => {
    // Project distance and lateral position to 2D camera viewport
    const scale = Math.max(0.15, 1.0 / Math.max(obj.distance_m, 0.3));
    const bx = vpX + obj.lateral_m * 180 * scale;
    const by = vpY + (h - vpY) * (1.0 - Math.min(1.0, obj.distance_m / 3.0));
    const boxW = 50 * scale;
    const boxH = 65 * scale;

    ctx.strokeStyle = obj.in_path ? "#ef4444" : "#eab308";
    ctx.lineWidth = 2;
    ctx.strokeRect(bx - boxW / 2, by - boxH, boxW, boxH);

    // Label tag
    ctx.fillStyle = obj.in_path ? "#ef4444" : "#eab308";
    ctx.font = "10px sans-serif";
    ctx.fillText(`${obj.class} ${obj.distance_m.toFixed(1)}m`, bx - boxW / 2, by - boxH - 4);
  });

  // HUD Crosshair
  ctx.strokeStyle = "rgba(0, 229, 255, 0.5)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(w / 2 - 10, h * 0.85);
  ctx.lineTo(w / 2 + 10, h * 0.85);
  ctx.moveTo(w / 2, h * 0.85 - 10);
  ctx.lineTo(w / 2, h * 0.85 + 10);
  ctx.stroke();
}

// Event Handlers & REST API calls
btnModeToggle.addEventListener("click", async () => {
  const newMode = isManualMode ? "AUTONOMOUS" : "MANUAL";
  await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "set_mode", mode: newMode }),
  });
});

btnToggleLight.addEventListener("click", async () => {
  await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "toggle_traffic_light" }),
  });
});

btnDropObs.addEventListener("click", async () => {
  await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "spawn_obstacle", class: "box" }),
  });
});

if (btnToggleTraffic) {
  btnToggleTraffic.addEventListener("click", async () => {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "toggle_traffic_car" }),
    });
  });
}

if (btnStallTraffic) {
  btnStallTraffic.addEventListener("click", async () => {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "stall_traffic_car" }),
    });
  });
}

if (btnPedestrian) {
  btnPedestrian.addEventListener("click", async () => {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "trigger_pedestrian" }),
    });
  });
}

btnClearObs.addEventListener("click", async () => {
  await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "clear_obstacles" }),
  });
});

btnReset.addEventListener("click", async () => {
  await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "reset" }),
  });
});

document.getElementById("btn-toggle-rays").addEventListener("click", () => {
  showLidar = !showLidar;
});

btnApplyPid.addEventListener("click", async () => {
  const kp = parseFloat(document.getElementById("input-kp").value);
  const ki = parseFloat(document.getElementById("input-ki").value);
  const kd = parseFloat(document.getElementById("input-kd").value);
  await fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "tune_pid", kp_steer: kp, ki_steer: ki, kd_steer: kd }),
  });
});

// Manual driving keyboard handling
window.addEventListener("keydown", (e) => {
  keysPressed[e.key.toLowerCase()] = true;
  updateManualDrive();
});

window.addEventListener("keyup", (e) => {
  keysPressed[e.key.toLowerCase()] = false;
  updateManualDrive();
});

function updateManualDrive() {
  if (!isManualMode) return;

  manualThrottle = 0.0;
  manualSteer = 0.0;

  if (keysPressed["w"] || keysPressed["arrowup"]) manualThrottle += 0.8;
  if (keysPressed["s"] || keysPressed["arrowdown"]) manualThrottle -= 0.8;
  if (keysPressed["a"] || keysPressed["arrowleft"]) manualSteer -= 25.0;
  if (keysPressed["d"] || keysPressed["arrowright"]) manualSteer += 25.0;

  fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "manual_drive", throttle: manualThrottle, steer_deg: manualSteer }),
  });
}

// Start continuous polling loop
setInterval(fetchState, 50); // 20 FPS
