"""
Project FINDER-Lite - Python Simulation Layer (Isometric Tactical HUD build)
------------------------------------------------------------------------------
Autonomous fusion logic (SCANNING -> DIVERTING -> STARING -> CONFIRMED/REJECTED,
driven by real velocity classification + synthetic breathing/acoustic readings),
rendered with a hand-rolled isometric projection.

HACKATHON-ROOM BUILD:
  - Square venue room with a raised STAGE zone at the front (marked no-scan)
  - On candidate lock, drone HOLDS POSITION for a full 10s dwell and fires an
    immediate "TARGET DETECTED" notification (separate from final confirm)
  - Live "vital sign" spike-graph panel (oscilloscope-style) showing the
    breathing-lock waveform while staring — clean rhythmic spikes = human,
    noisy flatline = false candidate
  - Independent scream/acoustic-distress listener (INMP441, ~5m omnidirectional)
    that fires a buzzer alert + broadcasts victim location any time it's in
    range, even outside the stare dwell
  - Location + grid-reference readout for rescuers

Run:
    python3 sar_sim.py
Requires: matplotlib, numpy
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon, FancyBboxPatch
from matplotlib.widgets import Button, RadioButtons
from collections import deque
import random

# ----------------------------- CONFIG ------------------------------------

# --- Square hackathon room (2x larger) ---
ARENA_W, ARENA_H = 32.0, 32.0
CELL_SIZE = 2.0
SENSOR_RANGE = 7.0
STARE_RANGE = 6.0
AUDIO_RANGE = 6.0                 # INMP441 omnidirectional scream/acoustic range
DT = 0.15
MOVE_SPEED_DRONE = 2.4
VEL_WINDOW = 14
MOVING_THRESHOLD = 0.35
STARE_DWELL = 10.0                # <-- drone holds position 10s on a candidate
CONFIRM_THRESHOLD = 60
DETECT_FLASH_DURATION = 2.2
BUZZER_DURATION = 3.0

# --- Stage zone (front of room), no-scan area ---
STAGE_X0, STAGE_Y0 = 8.0, 0.0
STAGE_W, STAGE_D = 16.0, 5.0
STAGE_HEIGHT = 0.6

OBST_H = 1.3
TARGET_H = 1.6
DRONE_BASE_Z = 2.8
DRONE_BOB = 0.22

# --- human stick-figure icon proportions (screen-space, drawn on top of iso base) ---
HUMAN_HEAD_R   = 0.16   # head circle radius
HUMAN_BODY_TOP = 1.00   # shoulder height fraction of TARGET_H
HUMAN_HIP_H    = 0.55   # hip height fraction of TARGET_H
HUMAN_ARM_W    = 0.30   # arm half-span
HUMAN_LEG_W    = 0.22   # leg foot half-span
HUMAN_HEAD_Y   = 1.18   # head center height fraction of TARGET_H

random.seed(7)
np.random.seed(7)

# --- Tactical color palette ---
BG        = "#050b14"
PANEL_BG  = "#0a1522"
FLOOR     = "#0d1c2b"
GRID_LINE = "#16283a"
RING      = "#1c3a52"
SWEEP     = "#22d3ee"
COVERED   = "#0e5a78"
TEXT_MAIN = "#7dd3fc"
TEXT_DIM  = "#3b5568"
MOVING_C    = "#38bdf8"
CANDIDATE_C = "#fbbf24"
CONFIRMED_C = "#ef4444"
REJECTED_C  = "#475569"
OBSTACLE_C  = "#2b3d52"
STAGE_C     = "#8b5e34"
BUZZER_C    = "#f97316"
SHADOW_C    = "#000000"
VITALS_C    = "#34d399"
VITALS_FLAT_C = "#64748b"


def shade(hexcol, factor):
    hexcol = hexcol.lstrip("#")
    r, g, b = (int(hexcol[i:i + 2], 16) for i in (0, 2, 4))
    r, g, b = (min(255, max(0, int(c * factor))) for c in (r, g, b))
    return f"#{r:02x}{g:02x}{b:02x}"


# ----------------------------- ISO PROJECTION ------------------------------

ISO_COS = np.cos(np.radians(30))
ISO_SIN = np.sin(np.radians(30))


def iso(x, y, z=0.0):
    sx = (x - y) * ISO_COS
    sy = (x + y) * ISO_SIN + z
    return sx, sy


def iso_circle(cx, cy, r, z=0.0, n=40):
    a = np.linspace(0, 2 * np.pi, n)
    xs, ys = cx + r * np.cos(a), cy + r * np.sin(a)
    pts = np.array([iso(x, y, z) for x, y in zip(xs, ys)])
    return pts


def human_icon(x, y, h=TARGET_H, pose="standing"):
    """Build a simple stick-figure (head/body/arms/legs) anchored at ground
    point (x, y), drawn in the iso 'vertical' (z) direction so it reads as a
    little human model rather than a plain marker.

    pose:
      "standing"       - upright, alert bystander (default, walks around)
      "standing_still" - upright but rigid/frozen (shock/trauma victim who
                          hasn't collapsed, just isn't moving)
      "kneeling"        - one knee down, hunched over an injured leg
      "lying"           - collapsed flat on the ground
    All poses return the SAME dict keys/shape so downstream drawing code
    doesn't need to branch on pose type.
    """
    ground = iso(x, y, 0.02)

    if pose == "lying":
        # body laid out flat along the ground, slightly offset so it reads
        # as a fallen figure rather than a standing one collapsed in place
        body_h = h * 0.28
        head_c = iso(x - h * 0.42, y, body_h * 0.9)
        shoulder = iso(x - h * 0.18, y, body_h)
        hip = iso(x + h * 0.30, y, body_h * 0.85)
        arm_l = (shoulder[0] - h * 0.10, shoulder[1] - h * 0.10)
        arm_r = (shoulder[0] + h * 0.10, shoulder[1] + h * 0.08)
        leg_l = (hip[0] + h * 0.34, hip[1] - h * 0.05)
        leg_r = (hip[0] + h * 0.40, hip[1] + h * 0.10)
        return {
            "head": head_c,
            "body": (shoulder, hip),
            "arms": (arm_l, arm_r, shoulder),
            "leg_l": (hip, leg_l),
            "leg_r": (hip, leg_r),
        }

    if pose == "kneeling":
        # crouched over one knee, hunched forward clutching the injured leg.
        # Proportions are scaled down together (not just squashed vertically)
        # so the silhouette stays compact and reads as "crouched", not spiky.
        kh = h * 0.60   # squashed overall height while kneeling
        head_c = iso(x, y, kh * HUMAN_HEAD_Y)
        shoulder = iso(x, y, kh * HUMAN_BODY_TOP)
        hip = iso(x, y, kh * HUMAN_HIP_H)
        # arms reach down toward the hurt knee, close to the body -- small
        # offsets relative to kh (not full h) keep them tucked in
        arm_l = (shoulder[0] - kh * 0.22, shoulder[1] - kh * 0.10)
        arm_r = (shoulder[0] + kh * 0.16, shoulder[1] - kh * 0.16)
        # down knee sits almost directly under the hip; other leg steps
        # slightly forward, both short since the figure is crouched low
        leg_l = (ground[0] - kh * 0.10, ground[1])
        leg_r = (ground[0] + kh * 0.30, ground[1] + kh * 0.03)
        return {
            "head": head_c,
            "body": (shoulder, hip),
            "arms": (arm_l, arm_r, shoulder),
            "leg_l": (hip, leg_l),
            "leg_r": (hip, leg_r),
        }

    # "standing" and "standing_still" share the same upright geometry;
    # any visual distinction (rigid posture, no idle sway, etc.) is applied
    # by the caller via color/animation, not the skeleton itself.
    head_c = iso(x, y, h * HUMAN_HEAD_Y)
    shoulder = iso(x, y, h * HUMAN_BODY_TOP)
    hip = iso(x, y, h * HUMAN_HIP_H)

    arm_l = (shoulder[0] - h * HUMAN_ARM_W, shoulder[1] - h * 0.05)
    arm_r = (shoulder[0] + h * HUMAN_ARM_W, shoulder[1] - h * 0.05)
    leg_l = (ground[0] - h * HUMAN_LEG_W, ground[1])
    leg_r = (ground[0] + h * HUMAN_LEG_W, ground[1])

    return {
        "head": head_c,
        "body": (shoulder, hip),
        "arms": (arm_l, arm_r, shoulder),
        "leg_l": (hip, leg_l),
        "leg_r": (hip, leg_r),
    }


# ----------------------------- ENTITIES -----------------------------------

class Target:
    def __init__(self, tid, kind, pos, pose="standing"):
        self.id = tid
        self.kind = kind
        self.pos = np.array(pos, dtype=float)
        self.pose = pose   # "standing" | "standing_still" | "kneeling" | "lying"
        self.history = deque(maxlen=VEL_WINDOW)
        self.waypoint = self._new_waypoint() if kind == "bystander" else None
        self.jitter_phase = random.uniform(0, 6.28)
        self.confidence = 0
        self.live_score = 0
        self.last_bpm = 0
        self.last_acoustic = False
        self.last_clean_lock = False
        self.last_scream = False
        self.scream_prev = False

    def _new_waypoint(self):
        return np.array([random.uniform(1.5, ARENA_W - 1.5),
                          random.uniform(1.5, ARENA_H - 1.5)])

    def step(self, t):
        self.history.append(self.pos.copy())
        if self.kind == "bystander":
            direction = self.waypoint - self.pos
            dist = np.linalg.norm(direction)
            if dist < 0.3:
                self.waypoint = self._new_waypoint()
            else:
                self.pos += (direction / dist) * 0.9 * DT
        else:
            self.jitter_phase += DT * 0.9
            jitter = 0.04 * np.array([np.sin(self.jitter_phase),
                                       np.cos(self.jitter_phase * 0.7)])
            self.pos += jitter * DT

    def velocity_estimate(self):
        if len(self.history) < 2:
            return 0.0
        disp = np.linalg.norm(self.history[-1] - self.history[0])
        time_span = DT * (len(self.history) - 1)
        return disp / max(time_span, 1e-6)


class Drone:
    def __init__(self):
        self.pos = np.array([2.0, 2.0])
        self.trail = deque(maxlen=50)
        nx = int(np.ceil(ARENA_W / CELL_SIZE))
        ny = int(np.ceil(ARENA_H / CELL_SIZE))
        self.grid_shape = (nx, ny)

        # mark stage-footprint cells as permanent no-scan zone
        self.no_scan = np.zeros((nx, ny), dtype=bool)
        for i in range(nx):
            for j in range(ny):
                cx0, cy0 = i * CELL_SIZE, j * CELL_SIZE
                ccx, ccy = cx0 + CELL_SIZE / 2, cy0 + CELL_SIZE / 2
                if (STAGE_X0 <= ccx <= STAGE_X0 + STAGE_W and
                        STAGE_Y0 <= ccy <= STAGE_Y0 + STAGE_D):
                    self.no_scan[i, j] = True

        self.visited = self.no_scan.copy()
        self.state = "COVERAGE"
        self.stare_target_id = None
        self.stare_timer = 0.0
        self.resolved = {}
        self.log = deque(maxlen=4)
        self.sweep_angle = 0.0

        # confirm/detect notification banner (center)
        self.flash_timer = 0.0
        self.flash_msg = ""
        self.flash_color = CONFIRMED_C

        # independent scream/buzzer alert (corner banner)
        self.buzzer_timer = 0.0
        self.buzzer_msg = ""
        self.buzzer_loc = None

        self.missions_done = 0

        # manual "HOLD POSITION" override (toggled by the HUD button) --
        # while true, the drone stops autonomously moving anywhere, but
        # sensors, staring/dwell timers, and detection logic still run.
        self.held = False

    def cell_of(self, pos):
        cx = int(np.clip(pos[0] // CELL_SIZE, 0, self.grid_shape[0] - 1))
        cy = int(np.clip(pos[1] // CELL_SIZE, 0, self.grid_shape[1] - 1))
        return cx, cy

    def mark_visited_near(self):
        cx, cy = self.cell_of(self.pos)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx_, ny_ = cx + dx, cy + dy
                if 0 <= nx_ < self.grid_shape[0] and 0 <= ny_ < self.grid_shape[1]:
                    center = np.array([nx_, ny_]) * CELL_SIZE + CELL_SIZE / 2
                    if np.linalg.norm(center - self.pos) <= SENSOR_RANGE:
                        self.visited[nx_, ny_] = True

    def next_unvisited_cell(self):
        unvisited = np.argwhere(~self.visited)
        if len(unvisited) == 0:
            self.visited[:, :] = False
            self.visited |= self.no_scan   # stage stays excluded after reset
            self.missions_done += 1
            return self.next_unvisited_cell()
        centers = unvisited * CELL_SIZE + CELL_SIZE / 2
        dists = np.linalg.norm(centers - self.pos, axis=1)
        nearest = centers[np.argmin(dists)]
        return nearest

    def move_towards(self, goal):
        if self.held:
            # HOLD POSITION engaged: drone stays put, no autonomous travel
            self.trail.append(self.pos.copy())
            return
        direction = goal - self.pos
        dist = np.linalg.norm(direction)
        if dist > 1e-6:
            step = min(MOVE_SPEED_DRONE * DT, dist)
            self.pos += (direction / dist) * step
        self.trail.append(self.pos.copy())


# ----------------------------- FUSION LOGIC --------------------------------

def classify_motion(target):
    v = target.velocity_estimate()
    return "MOVING" if v > MOVING_THRESHOLD else "QUASI_STATIC"


def synth_breathing_rate(target, t):
    if target.kind == "victim":
        bpm = 16 + 1.2 * np.sin(t * 0.6) + np.random.normal(0, 0.4)
        return max(bpm, 0), True
    bpm = np.random.choice([0, 0, 0, np.random.uniform(35, 70)])
    return bpm, False


def synth_acoustic_event(target, t):
    if target.kind == "victim":
        return (int(t * 2) % 9 == 0)
    return random.random() < 0.02


def synth_scream_event(target, t):
    """Distinct, louder distress event (independent of the tapping check
    above). Scripted to reliably occur so the buzzer path is demoable."""
    if target.kind != "victim":
        return False
    cycle = t % 18.0
    return 8.0 <= cycle <= 9.3


def vitals_sample(target, t):
    """Oscilloscope-style spike sample. Clean periodic spikes at the
    breathing rate = human signature. No lock = low noise flatline."""
    if target.last_clean_lock and target.last_bpm > 0:
        period = 60.0 / max(target.last_bpm, 1e-3)
        phase = (t % period) / period
        spike = np.exp(-((phase - 0.15) ** 2) / (2 * 0.015 ** 2))
        if target.last_acoustic:
            spike += 0.35
        return spike + np.random.normal(0, 0.03)
    return np.random.normal(0, 0.05)


# ----------------------------- FUSION STEP ----------------------------------

def fusion_step(drone, targets, t):
    drone.mark_visited_near()
    if drone.flash_timer > 0:
        drone.flash_timer -= DT
    if drone.buzzer_timer > 0:
        drone.buzzer_timer -= DT

    # --- independent scream/acoustic-distress listener (always on) ---
    for tg in targets:
        scream_now = synth_scream_event(tg, t)
        tg.last_scream = scream_now
        dist = np.linalg.norm(tg.pos - drone.pos)
        if scream_now and not tg.scream_prev and dist <= AUDIO_RANGE:
            drone.buzzer_timer = BUZZER_DURATION
            drone.buzzer_msg = f"BUZZER: DISTRESS SOUND — T{tg.id}"
            drone.buzzer_loc = tg.pos.copy()
            cx, cy = drone.cell_of(tg.pos)
            drone.log.append(
                f"[{t:5.1f}s] [BUZZER] Scream near T{tg.id} -> "
                f"loc ({tg.pos[0]:.1f},{tg.pos[1]:.1f}) R{cy+1}C{cx+1}"
            )
            try:
                sys.stdout.write("\a")
                sys.stdout.flush()
            except Exception:
                pass
        tg.scream_prev = scream_now

    candidates = [
        tg for tg in targets
        if tg.id not in drone.resolved
        and classify_motion(tg) == "QUASI_STATIC"
        and np.linalg.norm(tg.pos - drone.pos) <= STARE_RANGE
    ]

    if drone.state == "COVERAGE":
        if candidates:
            candidates.sort(key=lambda tg: np.linalg.norm(tg.pos - drone.pos))
            drone.stare_target_id = candidates[0].id
            drone.state = "DIVERTING"
            drone.log.append(f"[{t:5.1f}s] Candidate T{drone.stare_target_id} -> diverting")
        else:
            goal = drone.next_unvisited_cell()
            drone.move_towards(goal)

    elif drone.state == "DIVERTING":
        target = next(tg for tg in targets if tg.id == drone.stare_target_id)
        drone.move_towards(target.pos)
        if np.linalg.norm(target.pos - drone.pos) < 0.6:
            drone.state = "STARING"
            drone.stare_timer = 0.0
            drone.flash_timer = DETECT_FLASH_DURATION
            drone.flash_color = SWEEP
            drone.flash_msg = f"TARGET DETECTED — T{drone.stare_target_id} — HOLDING 10s SCAN"
            cx, cy = drone.cell_of(target.pos)
            drone.log.append(
                f"[{t:5.1f}s] Holding position on T{drone.stare_target_id} "
                f"@ ({target.pos[0]:.1f},{target.pos[1]:.1f}) R{cy+1}C{cx+1}"
            )

    elif drone.state == "STARING":
        drone.stare_timer += DT
        target = next(tg for tg in targets if tg.id == drone.stare_target_id)

        bpm, clean_lock = synth_breathing_rate(target, t)
        acoustic_hit = synth_acoustic_event(target, t)
        target.last_bpm = bpm
        target.last_acoustic = acoustic_hit
        target.last_clean_lock = clean_lock

        live = 0
        if clean_lock and 8 <= bpm <= 30:
            live += 40
        if classify_motion(target) == "QUASI_STATIC":
            live += 30
        if acoustic_hit:
            live += 20
        live += 10 * (drone.stare_timer / STARE_DWELL)
        target.live_score = live

        if drone.stare_timer >= STARE_DWELL:
            score = 0
            if clean_lock and 8 <= bpm <= 30:
                score += 40
            if classify_motion(target) == "QUASI_STATIC":
                score += 30
            if acoustic_hit:
                score += 20
            score += 10
            target.confidence = score
            target.live_score = score

            if score >= CONFIRM_THRESHOLD:
                drone.resolved[target.id] = "CONFIRMED"
                cx, cy = drone.cell_of(target.pos)
                drone.log.append(
                    f"[{t:5.1f}s] *** CONFIRMED T{target.id} (score {score}) "
                    f"@ ({target.pos[0]:.1f},{target.pos[1]:.1f}) R{cy+1}C{cx+1} ***"
                )
                drone.flash_timer = 2.5
                drone.flash_color = CONFIRMED_C
                drone.flash_msg = f"SURVIVOR CONFIRMED — TARGET T{target.id}"
            else:
                drone.resolved[target.id] = "REJECTED"
                drone.log.append(f"[{t:5.1f}s] Cleared T{target.id} (score {score})")

            drone.state = "COVERAGE"
            drone.stare_target_id = None


# ----------------------------- WORLD SETUP ---------------------------------

targets = [
    Target(0, "bystander", (6, 22)),
    Target(1, "bystander", (24, 12)),
    Target(2, "bystander", (16, 18)),
    Target(3, "victim", (26, 26), pose="lying"),          # collapsed, laying down
    Target(4, "bystander", (5, 6)),
    Target(5, "bystander", (20, 24)),
    Target(6, "victim", (9, 27), pose="kneeling"),         # down on one knee, injured knee
    Target(7, "bystander", (28, 6)),
    Target(8, "victim", (3, 17), pose="standing_still"),   # upright but frozen (shock/trauma)
    Target(9, "bystander", (14, 8)),
]
drone = Drone()
sim_t = 0.0

OBSTACLES = [
    (12.0, 12.0, 4.4, 2.4), (22.0, 8.0, 3.0, 4.0), (4.0, 20.0, 4.0, 2.0),
    (25.0, 19.0, 3.6, 4.0), (15.0, 24.0, 5.0, 2.0), (7.0, 12.0, 2.5, 2.5),
    (19.0, 18.0, 2.0, 3.0), (28.0, 14.0, 2.2, 2.2),
]

vitals_buffer = deque([0.0] * 200, maxlen=200)

# ----------------------------- VISUALIZATION -------------------------------

plt.rcParams["text.color"] = TEXT_MAIN
fig = plt.figure(figsize=(15, 7.4))
fig.patch.set_facecolor(BG)
fig.suptitle("PROJECT FINDER-LITE  //  AUTONOMOUS SAR TRIAGE — LIVE FEED",
             fontsize=13, color=SWEEP, family="monospace", fontweight="bold")

gs = fig.add_gridspec(4, 2, width_ratios=[2.15, 1], height_ratios=[1.0, 0.62, 0.62, 0.62],
                       hspace=0.5, wspace=0.06,
                       left=0.02, right=0.98, top=0.90, bottom=0.10)
ax_map = fig.add_subplot(gs[:, 0])
ax_panel = fig.add_subplot(gs[0, 1])
ax_minimap = fig.add_subplot(gs[1, 1])  # secondary map panel (toggle-able view)
ax_vitals = fig.add_subplot(gs[2, 1])   # graph #1: HUMAN vital-sign meter (only when staring)
ax_drone = fig.add_subplot(gs[3, 1])    # graph #2: DRONE position/travel meter (always live)

# --- HUD control strip along the bottom: map-view toggle + HOLD POSITION ---
# (reserved space via bottom=0.10 above)
ax_map_toggle = fig.add_axes([0.03, 0.015, 0.16, 0.05])   # radio: which secondary map to show
ax_hold_btn = fig.add_axes([0.24, 0.015, 0.14, 0.05])     # button: hold / resume drone

ax_map.set_facecolor(BG)
ax_map.set_xticks([]); ax_map.set_yticks([])
for spine in ax_map.spines.values():
    spine.set_visible(False)
ax_map.set_aspect("equal")

# --- compute iso bounding box for axis limits ---
corners = [(0, 0), (ARENA_W, 0), (ARENA_W, ARENA_H), (0, ARENA_H)]
all_pts = [iso(x, y, z) for x, y in corners for z in (0, DRONE_BASE_Z + DRONE_BOB + 1.0)]
all_pts = np.array(all_pts)
mx, Mx = all_pts[:, 0].min(), all_pts[:, 0].max()
my, My = all_pts[:, 1].min(), all_pts[:, 1].max()
pad = 1.5
ax_map.set_xlim(mx - pad, Mx + pad)
ax_map.set_ylim(my - pad, My + pad)

# --- floor plane (square room) ---
floor_poly = Polygon([iso(x, y, 0) for x, y in corners], closed=True,
                      facecolor=FLOOR, edgecolor=GRID_LINE, linewidth=1.2, zorder=0)
ax_map.add_patch(floor_poly)

for gx in np.arange(0, ARENA_W + 0.01, CELL_SIZE):
    p1, p2 = iso(gx, 0, 0), iso(gx, ARENA_H, 0)
    ax_map.plot([p1[0], p2[0]], [p1[1], p2[1]], color=GRID_LINE, linewidth=0.6, zorder=0.2)
for gy in np.arange(0, ARENA_H + 0.01, CELL_SIZE):
    p1, p2 = iso(0, gy, 0), iso(ARENA_W, gy, 0)
    ax_map.plot([p1[0], p2[0]], [p1[1], p2[1]], color=GRID_LINE, linewidth=0.6, zorder=0.2)

# --- STAGE platform (front of room, no-scan zone) ---
SX0, SY0, SW, SD = STAGE_X0, STAGE_Y0, STAGE_W, STAGE_D
sP1, sP2, sP3, sP4 = (SX0, SY0), (SX0 + SW, SY0), (SX0 + SW, SY0 + SD), (SX0, SY0 + SD)
s_top = [iso(*sP1, STAGE_HEIGHT), iso(*sP2, STAGE_HEIGHT), iso(*sP3, STAGE_HEIGHT), iso(*sP4, STAGE_HEIGHT)]
s_side_a = [iso(*sP1, 0), iso(*sP2, 0), iso(*sP2, STAGE_HEIGHT), iso(*sP1, STAGE_HEIGHT)]
s_side_b = [iso(*sP2, 0), iso(*sP3, 0), iso(*sP3, STAGE_HEIGHT), iso(*sP2, STAGE_HEIGHT)]
ax_map.add_patch(Polygon(s_side_b, closed=True, facecolor=shade(STAGE_C, 0.55),
                          edgecolor=BG, linewidth=0.5, zorder=1.5))
ax_map.add_patch(Polygon(s_side_a, closed=True, facecolor=shade(STAGE_C, 0.8),
                          edgecolor=BG, linewidth=0.5, zorder=1.51))
ax_map.add_patch(Polygon(s_top, closed=True, facecolor=shade(STAGE_C, 1.2),
                          edgecolor=BG, linewidth=0.5, zorder=1.52))
stage_label_pos = iso(SX0 + SW / 2, SY0 + SD / 2, STAGE_HEIGHT)
ax_map.text(stage_label_pos[0], stage_label_pos[1], "STAGE", ha="center", va="center",
            fontsize=9, color="#fde8c8", family="monospace", fontweight="bold", zorder=1.6)

# --- coverage cells (precomputed static polygons, alpha toggled per frame) ---
coverage_patches = {}
for i in range(drone.grid_shape[0]):
    for j in range(drone.grid_shape[1]):
        cx0, cy0 = i * CELL_SIZE, j * CELL_SIZE
        verts = [iso(cx0, cy0, 0.01), iso(cx0 + CELL_SIZE, cy0, 0.01),
                 iso(cx0 + CELL_SIZE, cy0 + CELL_SIZE, 0.01), iso(cx0, cy0 + CELL_SIZE, 0.01)]
        poly = Polygon(verts, closed=True, facecolor=COVERED, edgecolor="none",
                        alpha=0.0, zorder=0.5)
        ax_map.add_patch(poly)
        coverage_patches[(i, j)] = poly

# --- obstacles: extruded boxes (top + two shaded side faces), static ---
for (ox, oy, ow, oh) in OBSTACLES:
    P1, P2, P3, P4 = (ox, oy), (ox + ow, oy), (ox + ow, oy + oh), (ox, oy + oh)
    top = [iso(*P1, OBST_H), iso(*P2, OBST_H), iso(*P3, OBST_H), iso(*P4, OBST_H)]
    side_a = [iso(*P1, 0), iso(*P2, 0), iso(*P2, OBST_H), iso(*P1, OBST_H)]
    side_b = [iso(*P2, 0), iso(*P3, 0), iso(*P3, OBST_H), iso(*P2, OBST_H)]
    depth_key = (ox + oy + ow / 2 + oh / 2)
    z_base = 2 + depth_key * 0.01
    ax_map.add_patch(Polygon(side_b, closed=True, facecolor=shade(OBSTACLE_C, 0.55),
                              edgecolor=BG, linewidth=0.5, zorder=z_base))
    ax_map.add_patch(Polygon(side_a, closed=True, facecolor=shade(OBSTACLE_C, 0.8),
                              edgecolor=BG, linewidth=0.5, zorder=z_base + 0.01))
    ax_map.add_patch(Polygon(top, closed=True, facecolor=shade(OBSTACLE_C, 1.25),
                              edgecolor=BG, linewidth=0.5, zorder=z_base + 0.02))

# --- dynamic: range rings + sweep ---
range_ring_lines = []
for r in (2, 4, 6):
    line, = ax_map.plot([], [], color=RING, linewidth=1, linestyle="--", alpha=0.8, zorder=1)
    range_ring_lines.append((r, line))
sweep_line, = ax_map.plot([], [], color=SWEEP, linewidth=1.6, alpha=0.85, zorder=1.1)

# --- dynamic: drone (shadow, connector, body) ---
drone_shadow = Polygon(iso_circle(2.0, 2.0, 0.55, 0.02), closed=True,
                        facecolor=SHADOW_C, alpha=0.45, zorder=1.2)
ax_map.add_patch(drone_shadow)
drone_connector, = ax_map.plot([], [], color="#94a3b8", linewidth=1, linestyle=(0, (3, 2)), zorder=1.3)
drone_trail_line, = ax_map.plot([], [], color=SWEEP, alpha=0.3, linewidth=1.2, zorder=1.3)
drone_body, = ax_map.plot([], [], marker="D", markersize=13, color="white",
                           markeredgecolor=SWEEP, markeredgewidth=1.6, zorder=50)

# --- dynamic: targets (shadow, pole, head, glow, label, trail) ---
COLORS = {"MOVING": MOVING_C, "QUASI_STATIC": CANDIDATE_C,
          "CONFIRMED": CONFIRMED_C, "REJECTED": REJECTED_C}

t_shadow, t_glow, t_label, t_trail = {}, {}, {}, {}
t_head, t_body, t_arms, t_leg_l, t_leg_r = {}, {}, {}, {}, {}
for tg in targets:
    t_shadow[tg.id] = Polygon(iso_circle(*tg.pos, 0.32, 0.01), closed=True,
                               facecolor=SHADOW_C, alpha=0.4, zorder=2)
    ax_map.add_patch(t_shadow[tg.id])

    # --- little human stick-figure: head + body + arms + legs ---
    head, = ax_map.plot([], [], marker="o", markersize=11, zorder=2.4,
                        markeredgecolor="white", markeredgewidth=0.8)
    t_head[tg.id] = head
    body, = ax_map.plot([], [], color="white", linewidth=2.2, zorder=2.3, solid_capstyle="round")
    t_body[tg.id] = body
    arms, = ax_map.plot([], [], color="white", linewidth=1.8, zorder=2.2, solid_capstyle="round")
    t_arms[tg.id] = arms
    leg_l, = ax_map.plot([], [], color="white", linewidth=1.8, zorder=2.2, solid_capstyle="round")
    t_leg_l[tg.id] = leg_l
    leg_r, = ax_map.plot([], [], color="white", linewidth=1.8, zorder=2.2, solid_capstyle="round")
    t_leg_r[tg.id] = leg_r

    glow, = ax_map.plot([], [], color=CONFIRMED_C, linewidth=2, alpha=0.0, zorder=2.15)
    t_glow[tg.id] = glow
    trail, = ax_map.plot([], [], alpha=0.28, linewidth=1, zorder=1.9)
    t_trail[tg.id] = trail
    t_label[tg.id] = ax_map.text(0, 0, "", fontsize=8, color=TEXT_MAIN,
                                  family="monospace", zorder=2.5)

flash_text = ax_map.text((mx + Mx) / 2, My - 0.3, "", ha="center", va="center", fontsize=12.5,
                          color=CONFIRMED_C, family="monospace", fontweight="bold", zorder=60)

# buzzer_text is created here but positioned lower down, once the legend
# box geometry is known, so it always sits BELOW the legend with no overlap.
buzzer_text = ax_map.text(0, 0, "", ha="left", va="top", fontsize=10.5,
                           color=BUZZER_C, family="monospace", fontweight="bold", zorder=61)

# --- side info panel ---
ax_panel.set_facecolor(PANEL_BG)
ax_panel.set_xlim(0, 1); ax_panel.set_ylim(0, 1)
ax_panel.axis("off")

panel_text = ax_panel.text(0.04, 0.97, "", va="top", ha="left", family="monospace",
                            fontsize=8.0, color=TEXT_MAIN, transform=ax_panel.transAxes,
                            linespacing=1.32, zorder=1)

meter_bg = Polygon([(0.04, 0.07), (0.96, 0.07), (0.96, 0.12), (0.04, 0.12)],
                    closed=True, facecolor="#0f2233", edgecolor=RING, transform=ax_panel.transAxes, zorder=2)
meter_fill = Polygon([(0.04, 0.07), (0.04, 0.07), (0.04, 0.12), (0.04, 0.12)],
                      closed=True, facecolor=CANDIDATE_C, transform=ax_panel.transAxes, zorder=3)
ax_panel.add_patch(meter_bg)
ax_panel.add_patch(meter_fill)
ax_panel.text(0.04, 0.05, "CONFIDENCE", family="monospace", fontsize=7.6,
              color=TEXT_DIM, transform=ax_panel.transAxes)

# --- TARGET COLOR CODE legend, pinned to top-left corner of the tactical map ---
legend_items = [("MOVING BYSTANDER", MOVING_C),
                ("CANDIDATE (checking)", CANDIDATE_C),
                ("CONFIRMED SURVIVOR", CONFIRMED_C),
                ("REJECTED / false alarm", REJECTED_C)]
legend_w, legend_h = 3.6, 0.55 + 0.42 * len(legend_items)
legend_x0, legend_y0 = mx + 0.15, My - 0.15 - legend_h
legend_bg_map = Polygon(
    [(legend_x0, legend_y0), (legend_x0 + legend_w, legend_y0),
     (legend_x0 + legend_w, legend_y0 + legend_h), (legend_x0, legend_y0 + legend_h)],
    closed=True, facecolor=PANEL_BG, edgecolor=RING, linewidth=1.0, alpha=0.92, zorder=55)
ax_map.add_patch(legend_bg_map)
ax_map.text(legend_x0 + 0.12, legend_y0 + legend_h - 0.28, "TARGET COLOR CODE",
            fontsize=8.2, color=TEXT_DIM, family="monospace", fontweight="bold",
            va="top", zorder=56)
for k, (lbl, col) in enumerate(legend_items):
    y = legend_y0 + legend_h - 0.62 - k * 0.42
    ax_map.plot([legend_x0 + 0.22], [y], marker="o", markersize=8, color=col,
                markeredgecolor="white", markeredgewidth=0.6, clip_on=False, zorder=56)
    ax_map.text(legend_x0 + 0.42, y, lbl, fontsize=7.4, color="#a8c4dc",
                family="monospace", va="center", zorder=56)

# now that the legend box's bottom edge (legend_y0) is known, park the
# buzzer/scream banner just underneath it with a clear gap -- guarantees
# the two top-left HUD panels never overlap regardless of legend length
buzzer_text.set_position((legend_x0, legend_y0 - 0.25))

# --- vital-sign spike graph panel ---
ax_vitals.set_facecolor("#03080f")
ax_vitals.set_xticks([]); ax_vitals.set_yticks([])
for spine in ax_vitals.spines.values():
    spine.set_color(RING)
ax_vitals.set_xlim(0, vitals_buffer.maxlen)
ax_vitals.set_ylim(-0.3, 1.5)
ax_vitals.set_title("GRAPH 1 — HUMAN VITAL-SIGN METER  (breathing-lock waveform)",
                     fontsize=8.3, color=TEXT_DIM, family="monospace", loc="left")
vitals_line, = ax_vitals.plot([], [], color=VITALS_C, linewidth=1.3, zorder=2)
vitals_status = ax_vitals.text(0.02, 0.90, "", transform=ax_vitals.transAxes,
                                fontsize=8, color=VITALS_C, family="monospace",
                                fontweight="bold", va="top")
ax_vitals.text(0.02, 0.06, "green spikes = human breathing found | grey = no lock yet",
               transform=ax_vitals.transAxes, fontsize=6.6, color=TEXT_DIM, family="monospace")

# --- drone position/travel graph panel (separate graph, always live) ---
DRONE_BUF_LEN = 200
drone_x_buffer = deque([drone.pos[0] / ARENA_W] * DRONE_BUF_LEN, maxlen=DRONE_BUF_LEN)
drone_y_buffer = deque([drone.pos[1] / ARENA_H] * DRONE_BUF_LEN, maxlen=DRONE_BUF_LEN)

ax_drone.set_facecolor("#03080f")
ax_drone.set_xticks([]); ax_drone.set_yticks([])
for spine in ax_drone.spines.values():
    spine.set_color(RING)
ax_drone.set_xlim(0, DRONE_BUF_LEN)
ax_drone.set_ylim(-0.05, 1.05)
ax_drone.set_title("GRAPH 2 — DRONE POSITION TRACKER  (X/Y in room, over time)",
                    fontsize=8.3, color=TEXT_DIM, family="monospace", loc="left")
drone_pos_line_x, = ax_drone.plot([], [], color=SWEEP, linewidth=1.3, zorder=2, label="X pos")
drone_pos_line_y, = ax_drone.plot([], [], color=CANDIDATE_C, linewidth=1.3, zorder=2, label="Y pos")
ax_drone.legend(loc="upper right", fontsize=6.5, facecolor=PANEL_BG, edgecolor=RING,
                 labelcolor="#6b8caa", framealpha=0.7)
drone_graph_status = ax_drone.text(0.02, 0.90, "", transform=ax_drone.transAxes,
                                    fontsize=8, color=SWEEP, family="monospace",
                                    fontweight="bold", va="top")
ax_drone.text(0.02, 0.06, "cyan = drone X (left-right) | yellow = drone Y (front-back), 0-1 = room span",
              transform=ax_drone.transAxes, fontsize=6.6, color=TEXT_DIM, family="monospace")

# --- secondary map panel: TWO selectable top-down views sharing one axes ---
# View "Top-Down" : plain schematic showing target positions + drone
# View "Coverage" : the same schematic with the sweep-coverage heatmap on top
ax_minimap.set_facecolor("#03080f")
ax_minimap.set_xticks([]); ax_minimap.set_yticks([])
for spine in ax_minimap.spines.values():
    spine.set_color(RING)
ax_minimap.set_xlim(-0.5, ARENA_W + 0.5)
ax_minimap.set_ylim(-0.5, ARENA_H + 0.5)
ax_minimap.set_box_aspect(1)
ax_minimap.invert_yaxis()   # y=0 (front/stage) at top, matches iso view orientation
minimap_title = ax_minimap.set_title("MAP 1/2 — TOP-DOWN TARGET POSITIONS",
                                      fontsize=8.3, color=TEXT_DIM, family="monospace", loc="left")

# room outline (shared by both views)
ax_minimap.plot([0, ARENA_W, ARENA_W, 0, 0], [0, 0, ARENA_H, ARENA_H, 0],
                color=RING, linewidth=1.0, zorder=1)

# stage no-scan zone (shared by both views)
ax_minimap.add_patch(Polygon(
    [(STAGE_X0, STAGE_Y0), (STAGE_X0 + STAGE_W, STAGE_Y0),
     (STAGE_X0 + STAGE_W, STAGE_Y0 + STAGE_D), (STAGE_X0, STAGE_Y0 + STAGE_D)],
    closed=True, facecolor=STAGE_C, alpha=0.35, edgecolor=STAGE_C, linewidth=0.8, zorder=1.1))

# obstacles (static footprints, shared by both views)
for (ox, oy, ow, oh) in OBSTACLES:
    ax_minimap.add_patch(Polygon(
        [(ox, oy), (ox + ow, oy), (ox + ow, oy + oh), (ox, oy + oh)],
        closed=True, facecolor=OBSTACLE_C, alpha=0.7, edgecolor="none", zorder=1.2))

# --- "Coverage" view layer: per-cell sweep-coverage tiles (hidden by default) ---
minimap_coverage_patches = {}
for i in range(drone.grid_shape[0]):
    for j in range(drone.grid_shape[1]):
        cx0, cy0 = i * CELL_SIZE, j * CELL_SIZE
        poly = Polygon(
            [(cx0, cy0), (cx0 + CELL_SIZE, cy0), (cx0 + CELL_SIZE, cy0 + CELL_SIZE), (cx0, cy0 + CELL_SIZE)],
            closed=True, facecolor=COVERED, edgecolor="none", alpha=0.0, zorder=1.05)
        ax_minimap.add_patch(poly)
        minimap_coverage_patches[(i, j)] = poly

# drone marker (shared by both views)
minimap_drone, = ax_minimap.plot([], [], marker="D", markersize=7, color="white",
                                  markeredgecolor=SWEEP, markeredgewidth=1.3, zorder=5)

# one dot + label per target, color updated each frame to match main map color code
minimap_dots, minimap_labels = {}, {}
for tg in targets:
    dot, = ax_minimap.plot([tg.pos[0]], [tg.pos[1]], marker="o", markersize=8,
                            color=REJECTED_C, markeredgecolor="white",
                            markeredgewidth=0.7, zorder=4)
    minimap_dots[tg.id] = dot
    lbl = ax_minimap.text(tg.pos[0] + 0.6, tg.pos[1], f"T{tg.id}", fontsize=6.5,
                           color="#a8c4dc", family="monospace", va="center", zorder=4.5)
    minimap_labels[tg.id] = lbl

# current secondary-map selection ("topdown" or "coverage")
active_map_view = {"mode": "topdown"}

# --- map-view toggle (radio buttons) ---
ax_map_toggle.set_facecolor(PANEL_BG)
map_radio = RadioButtons(ax_map_toggle, ("Top-Down", "Coverage"), activecolor=SWEEP)
for rlbl in map_radio.labels:
    rlbl.set_fontsize(7.5)
    rlbl.set_color(TEXT_MAIN)
    rlbl.set_family("monospace")

def on_map_toggle(label):
    active_map_view["mode"] = "coverage" if label == "Coverage" else "topdown"
    if active_map_view["mode"] == "coverage":
        minimap_title.set_text("MAP 2/2 — SWEEP COVERAGE HEATMAP")
    else:
        minimap_title.set_text("MAP 1/2 — TOP-DOWN TARGET POSITIONS")
        for poly in minimap_coverage_patches.values():
            poly.set_alpha(0.0)

map_radio.on_clicked(on_map_toggle)

# --- HOLD POSITION button ---
ax_hold_btn.set_facecolor(PANEL_BG)
hold_button = Button(ax_hold_btn, "HOLD POSITION", color=PANEL_BG, hovercolor=shade(CANDIDATE_C, 0.5))
hold_button.label.set_fontsize(7.8)
hold_button.label.set_family("monospace")
hold_button.label.set_color(TEXT_MAIN)

def on_hold_clicked(event):
    drone.held = not drone.held
    if drone.held:
        hold_button.label.set_text("RESUME SWEEP")
        hold_button.ax.set_facecolor(shade(BUZZER_C, 0.5))
        drone.log.append(f"[{sim_t:5.1f}s] Manual HOLD POSITION engaged")
    else:
        hold_button.label.set_text("HOLD POSITION")
        hold_button.ax.set_facecolor(PANEL_BG)
        drone.log.append(f"[{sim_t:5.1f}s] HOLD released — resuming autonomous sweep")

hold_button.on_clicked(on_hold_clicked)


def update(frame):
    global sim_t
    sim_t += DT

    for tg in targets:
        tg.step(sim_t)
    fusion_step(drone, targets, sim_t)

    dz = DRONE_BASE_Z + DRONE_BOB * np.sin(sim_t * 2.2)
    body_xy = iso(drone.pos[0], drone.pos[1], dz)
    shadow_xy = iso(drone.pos[0], drone.pos[1], 0.02)
    drone_body.set_data([body_xy[0]], [body_xy[1]])
    drone_shadow.set_xy(iso_circle(drone.pos[0], drone.pos[1], 0.55, 0.02))
    drone_connector.set_data([body_xy[0], shadow_xy[0]], [body_xy[1], shadow_xy[1]])

    if drone.trail:
        tr = np.array([iso(p[0], p[1], 0.03) for p in drone.trail])
        drone_trail_line.set_data(tr[:, 0], tr[:, 1])

    for r, line in range_ring_lines:
        pts = iso_circle(drone.pos[0], drone.pos[1], r, 0.02)
        line.set_data(pts[:, 0], pts[:, 1])

    drone.sweep_angle += 0.12
    ex = drone.pos[0] + SENSOR_RANGE * np.cos(drone.sweep_angle)
    ey = drone.pos[1] + SENSOR_RANGE * np.sin(drone.sweep_angle)
    p1, p2 = iso(drone.pos[0], drone.pos[1], 0.02), iso(ex, ey, 0.02)
    sweep_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])

    for (i, j), poly in coverage_patches.items():
        poly.set_alpha(0.35 if drone.visited[i, j] else 0.0)

    for tg in targets:
        status = drone.resolved.get(tg.id)
        motion = classify_motion(tg)
        color = COLORS[status] if status else COLORS[motion]

        x, y = tg.pos
        pose = getattr(tg, "pose", "standing")
        # lying/kneeling figures sit lower, so the label/top marker anchors
        # to a shorter effective height instead of full standing height
        label_h = {"lying": TARGET_H * 0.32, "kneeling": TARGET_H * 0.65}.get(pose, TARGET_H)
        top = iso(x, y, label_h)
        # lying figures also get a longer shadow footprint (sprawled body)
        shadow_r = 0.42 if pose == "lying" else 0.32
        t_shadow[tg.id].set_xy(iso_circle(x, y, shadow_r, 0.01))

        fig_h = TARGET_H * (1.15 if tg.kind == "victim" else 1.0)
        icon = human_icon(x, y, h=fig_h, pose=pose)
        t_head[tg.id].set_data([icon["head"][0]], [icon["head"][1]])
        t_head[tg.id].set_color(color)
        t_head[tg.id].set_markersize(13 if tg.kind == "victim" else 11)

        sh, hp = icon["body"]
        t_body[tg.id].set_data([sh[0], hp[0]], [sh[1], hp[1]])
        t_body[tg.id].set_color(color)

        al, ar, sh2 = icon["arms"]
        t_arms[tg.id].set_data([al[0], ar[0]], [al[1], ar[1]])
        t_arms[tg.id].set_color(color)

        hp_l, ll = icon["leg_l"]
        t_leg_l[tg.id].set_data([hp_l[0], ll[0]], [hp_l[1], ll[1]])
        t_leg_l[tg.id].set_color(color)

        hp_r, lr = icon["leg_r"]
        t_leg_r[tg.id].set_data([hp_r[0], lr[0]], [hp_r[1], lr[1]])
        t_leg_r[tg.id].set_color(color)

        if tg.history:
            hist = np.array([iso(p[0], p[1], 0.02) for p in tg.history])
            t_trail[tg.id].set_data(hist[:, 0], hist[:, 1])
            t_trail[tg.id].set_color(color)

        if status == "CONFIRMED":
            pulse_r = 0.5 + 0.22 * (0.5 + 0.5 * np.sin(sim_t * 4))
            ring_pts = iso_circle(x, y, pulse_r, 0.02)
            t_glow[tg.id].set_data(ring_pts[:, 0], ring_pts[:, 1])
            t_glow[tg.id].set_alpha(0.7)
        else:
            t_glow[tg.id].set_alpha(0.0)

        label = f"T{tg.id}" + (" \u2605" if status == "CONFIRMED" else "")
        t_label[tg.id].set_position((top[0] + 0.25, top[1] + 0.25))
        t_label[tg.id].set_text(label)
        t_label[tg.id].set_color(color)

    # center notification banner (target detected / confirmed)
    if drone.flash_timer > 0:
        alpha = min(drone.flash_timer / 1.0, 1.0)
        flash_text.set_text(drone.flash_msg)
        flash_text.set_color(drone.flash_color)
        flash_text.set_alpha(alpha)
    else:
        flash_text.set_text("")

    # buzzer / scream alert banner (top-left, independent of stare cycle)
    if drone.buzzer_timer > 0:
        blink = 1.0 if int(sim_t * 4) % 2 == 0 else 0.35
        loc_str = ""
        if drone.buzzer_loc is not None:
            loc_str = f"  LOC:({drone.buzzer_loc[0]:.1f},{drone.buzzer_loc[1]:.1f})"
        buzzer_text.set_text("[!] " + drone.buzzer_msg + loc_str)
        buzzer_text.set_alpha(blink)
    else:
        buzzer_text.set_text("")

    # --- vitals spike graph ---
    if drone.state == "STARING":
        tg = next(t for t in targets if t.id == drone.stare_target_id)
        sample = vitals_sample(tg, sim_t)
        locked = tg.last_clean_lock and tg.last_bpm > 0
    else:
        sample = np.random.normal(0, 0.04)
        locked = False
    vitals_buffer.append(sample)
    vitals_line.set_data(range(len(vitals_buffer)), list(vitals_buffer))
    vitals_line.set_color(VITALS_C if locked else VITALS_FLAT_C)
    vitals_status.set_text("STATUS: HUMAN SIGNATURE LOCKED" if locked else "STATUS: NO LOCK / SCANNING")
    vitals_status.set_color(VITALS_C if locked else TEXT_DIM)

    # --- graph 2: drone position tracker (separate from the human graph above) ---
    drone_x_buffer.append(drone.pos[0] / ARENA_W)
    drone_y_buffer.append(drone.pos[1] / ARENA_H)
    drone_pos_line_x.set_data(range(len(drone_x_buffer)), list(drone_x_buffer))
    drone_pos_line_y.set_data(range(len(drone_y_buffer)), list(drone_y_buffer))
    drone_graph_status.set_text(f"STATE: {drone.state}  |  pos=({drone.pos[0]:.1f},{drone.pos[1]:.1f})m")

    # --- secondary map panel: drone position + target colors (shared by both views) ---
    minimap_drone.set_data([drone.pos[0]], [drone.pos[1]])
    for tg in targets:
        status = drone.resolved.get(tg.id)
        motion = classify_motion(tg)
        color = COLORS[status] if status else COLORS[motion]
        minimap_dots[tg.id].set_data([tg.pos[0]], [tg.pos[1]])
        minimap_dots[tg.id].set_color(color)
        label = f"T{tg.id}" + (" \u2605" if status == "CONFIRMED" else "")
        minimap_labels[tg.id].set_position((tg.pos[0] + 0.6, tg.pos[1]))
        minimap_labels[tg.id].set_text(label)
        minimap_labels[tg.id].set_color(color)

    # only redraw coverage-tile alpha values while that view is selected
    # (cheap to skip otherwise, and avoids stale tiles bleeding into the
    # top-down view when the user switches back)
    if active_map_view["mode"] == "coverage":
        for (i, j), poly in minimap_coverage_patches.items():
            poly.set_alpha(0.4 if drone.visited[i, j] else 0.0)

    lines = []
    lines.append(f" TIME {sim_t:6.1f}s   STATE: {drone.state}"
                 + ("  [HELD]" if drone.held else ""))
    lines.append(f" SWEEPS COMPLETE: {drone.missions_done}")
    lines.append("-" * 30)
    if drone.stare_target_id is not None:
        tg = next(t for t in targets if t.id == drone.stare_target_id)
        cx, cy = drone.cell_of(tg.pos)
        lines.append(f" ACTIVE TARGET   : T{tg.id} @ ({tg.pos[0]:4.1f},{tg.pos[1]:4.1f})m R{cy+1}C{cx+1}")
        lines.append(f"   {tg.last_bpm:5.1f} BPM | acoustic {'HIT' if tg.last_acoustic else '---'} "
                      f"| dwell {drone.stare_timer:4.1f}/{STARE_DWELL:.1f}s")
        live = tg.live_score
    else:
        lines.append(" ACTIVE TARGET   : none (scanning)")
        live = 0
    lines.append("-" * 30)
    lines.append(f" CONFIRMED: {sum(1 for v in drone.resolved.values() if v=='CONFIRMED')}  "
                 f"REJECTED: {sum(1 for v in drone.resolved.values() if v=='REJECTED')}")
    lines.append("-" * 30)
    lines.append(" EVENT LOG:")
    for entry in drone.log:
        lines.append(" " + entry)
    panel_text.set_text("\n".join(lines))

    frac = 0.92 * min(live, 100) / 100
    x0 = 0.04
    meter_fill.set_xy([(x0, 0.07), (x0 + frac, 0.07), (x0 + frac, 0.12), (x0, 0.12)])
    meter_fill.set_facecolor(CONFIRMED_C if live >= CONFIRM_THRESHOLD else CANDIDATE_C)

    return ([drone_body, drone_shadow, drone_connector, sweep_line, panel_text,
             meter_fill, flash_text, buzzer_text, vitals_line, vitals_status,
             drone_pos_line_x, drone_pos_line_y, drone_graph_status, minimap_drone]
            + list(t_head.values()) + list(t_body.values()) + list(t_arms.values())
            + list(t_leg_l.values()) + list(t_leg_r.values())
            + list(minimap_dots.values()) + list(minimap_labels.values())
            + list(minimap_coverage_patches.values()))


ani = animation.FuncAnimation(fig, update, interval=int(DT * 1000), blit=False,
                               cache_frame_data=False)
plt.show()
