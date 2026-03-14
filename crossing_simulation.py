"""
4-Lane Intersection Traffic Signal Simulation (Indian Left-Hand Traffic)
==============================================
A comprehensive, educational simulation of a standard 4-lane intersection
(North, South, East, West approaches) with realistic traffic signal phasing,
modelled for Indian left-hand traffic (vehicles drive on the left side).

Features:
  • Realistic multi-phase signal controller (NEMA-style phasing)
    – Straight (through) movement phases
    – Protected right-turn phases (right turns cross traffic in LHT)
    – Left-turn-on-red (permitted)
    – Opposing traffic simultaneous green
    – Pedestrian crossing phases with walk / don't-walk signals
    – All-red clearance intervals between every phase
  • Vehicle flow simulation: queue formation, discharge on green, turning
  • Modular OOP design: TrafficSignal, Vehicle, Lane, IntersectionController
  • Adjustable parameters via UI sliders (density, timing, speed)
  • Tkinter-based visual animation (no pygame)

Traffic Engineering Notes:
  – A "phase" grants right-of-way to a specific set of movements.
  – Opposing through movements share a phase (N+S through, E+W through).
  – Protected left turns get a dedicated phase before through green.
  – All-red clearance (typically 1-2 s) ensures the intersection clears
    before conflicting movements begin.
  – Pedestrian phases run concurrently with their parallel vehicle phase
    but add extra time if pedestrians are present.
  – Cycle length = sum of all phase durations (green+yellow+all-red).
"""

import tkinter as tk
from tkinter import ttk
import random
import math
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

# ══════════════════════════════════════════════════════════════════════════════
# LAYOUT CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════
W, H        = 500, 500          # canvas size
CX, CY      = W // 2, H // 2    # center of intersection
ROAD_W      = 200                # total road width (4 lanes)
LANE_W      = 40                 # width per lane
NUM_LANES   = 2                  # lanes per direction per approach
RW          = ROAD_W // 2       # half road width = 100

CAR_LEN     = 30                 # vehicle length (px)
CAR_WID     = 16                 # vehicle width (px)
STOP_GAP    = 5                  # gap between front bumper and stop line

# ── Colours ───────────────────────────────────────────────────────────────────
BG_COLOR     = "#1a1a2e"
ROAD_COLOR   = "#2d2d3a"
INTER_COLOR  = "#323242"
GRASS_COLOR  = "#1e3a2f"
SIDEWALK_CLR = "#252535"
LANE_LINE    = "#555566"
SUBLINE      = "#3d3d4e"
LIGHT_RED_C  = "#ff3b3b"
LIGHT_YEL_C  = "#ffd700"
LIGHT_GRN_C  = "#39ff14"
LIGHT_OFF_C  = "#2a2a3a"
PED_WALK_C   = "#39ff14"
PED_DONTWALK = "#ff3b3b"
DIR_COLOR    = {"N": "#00d4ff", "S": "#ff6b35", "E": "#c678dd", "W": "#98c379"}

# Movement types
class MoveType(Enum):
    STRAIGHT = auto()
    LEFT     = auto()
    RIGHT    = auto()


# ══════════════════════════════════════════════════════════════════════════════
# PHASE DEFINITION – realistic NEMA-style signal phasing
# ══════════════════════════════════════════════════════════════════════════════
# Each phase specifies which direction+movement combos get green.
# Phases execute in ring order. Yellow and all-red follow each green.

@dataclass
class PhaseSpec:
    """One signal phase: which movements get green, and the base durations."""
    name: str
    green_moves: List[Tuple[str, MoveType]]  # (direction, movement)
    green_time: int       # frames of green
    yellow_time: int      # frames of yellow
    allred_time: int      # frames of all-red clearance
    pedestrian: bool      # does a pedestrian walk run concurrently?
    ped_directions: List[str] = field(default_factory=list)  # which crosswalks

# Default phase plan (NEMA dual-ring simplified to sequential)
# Timing in frames at ~60 fps  (divide by 60 for real seconds)
DEFAULT_PHASES = [
    # Phase 1: N/S protected right turn (right turns cross traffic in LHT)
    PhaseSpec(
        name="N/S Protected Right",
        green_moves=[("N", MoveType.RIGHT), ("S", MoveType.RIGHT)],
        green_time=120, yellow_time=45, allred_time=20,
        pedestrian=False,
    ),
    # Phase 2: N/S through + left
    PhaseSpec(
        name="N/S Through + Left",
        green_moves=[
            ("N", MoveType.STRAIGHT), ("S", MoveType.STRAIGHT),
            ("N", MoveType.LEFT), ("S", MoveType.LEFT),
        ],
        green_time=240, yellow_time=50, allred_time=20,
        pedestrian=True, ped_directions=["E", "W"],  # E/W crosswalks parallel to N/S
    ),
    # Phase 3: E/W protected right turn
    PhaseSpec(
        name="E/W Protected Right",
        green_moves=[("E", MoveType.RIGHT), ("W", MoveType.RIGHT)],
        green_time=120, yellow_time=45, allred_time=20,
        pedestrian=False,
    ),
    # Phase 4: E/W through + left
    PhaseSpec(
        name="E/W Through + Left",
        green_moves=[
            ("E", MoveType.STRAIGHT), ("W", MoveType.STRAIGHT),
            ("E", MoveType.LEFT), ("W", MoveType.LEFT),
        ],
        green_time=240, yellow_time=50, allred_time=20,
        pedestrian=True, ped_directions=["N", "S"],
    ),
]


# ══════════════════════════════════════════════════════════════════════════════
# CLASS: TrafficSignal
# ══════════════════════════════════════════════════════════════════════════════
class TrafficSignal:
    """
    Manages the state (green / yellow / red) for ONE direction+movement.
    The IntersectionController sets the state externally based on the phase plan.
    """
    def __init__(self, direction: str, move_type: MoveType):
        self.direction = direction
        self.move_type = move_type
        self.state: str = "red"          # "green", "yellow", "red"
        self.arrow: bool = (move_type != MoveType.STRAIGHT)

    def set(self, state: str):
        self.state = state

    def can_go(self) -> bool:
        return self.state == "green"

    def is_yellow(self) -> bool:
        return self.state == "yellow"

    def __repr__(self):
        return f"Signal({self.direction},{self.move_type.name},{self.state})"


# ══════════════════════════════════════════════════════════════════════════════
# CLASS: IntersectionController  – phase sequencer
# ══════════════════════════════════════════════════════════════════════════════
class IntersectionController:
    """
    Cycles through the phase plan, setting all TrafficSignal objects each tick.

    Traffic Engineering Logic:
      1.  Phases run sequentially in a fixed ring.
      2.  Within a phase the sub-intervals are:  GREEN → YELLOW → ALL-RED.
      3.  During GREEN the permitted movements get green, all others red.
      4.  During YELLOW those movements switch to yellow (stop if able).
      5.  During ALL-RED every signal is red — clears the intersection.
      6.  Then the next phase starts.
      7.  Pedestrian walk signals run during green of their phase and switch
          to flashing-don't-walk during yellow.
    """

    def __init__(self, phases: List[PhaseSpec]):
        self.phases = phases
        # Create all signals: 4 directions × 3 movements
        self.signals: Dict[Tuple[str, MoveType], TrafficSignal] = {}
        for d in "NSEW":
            for m in MoveType:
                self.signals[(d, m)] = TrafficSignal(d, m)

        # Pedestrian walk state per crosswalk side
        self.ped_walk: Dict[str, str] = {d: "dont_walk" for d in "NSEW"}

        # Phase tracking
        self.phase_idx = 0
        self.sub = "green"        # "green", "yellow", "allred"
        self.timer = 0
        self._apply_phase()

    @property
    def current_phase(self) -> PhaseSpec:
        return self.phases[self.phase_idx]

    @property
    def cycle_length(self) -> int:
        """Total cycle length in frames."""
        return sum(p.green_time + p.yellow_time + p.allred_time for p in self.phases)

    def _apply_phase(self):
        """Set all signals according to current phase + sub-interval."""
        phase = self.current_phase
        # Reset everything to red
        for sig in self.signals.values():
            sig.set("red")
        for d in "NSEW":
            self.ped_walk[d] = "dont_walk"

        if self.sub == "allred":
            return  # all red already

        for (d, m) in phase.green_moves:
            self.signals[(d, m)].set(self.sub)  # "green" or "yellow"

        # Left-turn-on-red: in LHT, left turns (easy turns) may be
        # permitted when cross traffic is not going.
        # Simplified: only during the cross-direction's all-red or own green.

        # Pedestrian signals
        if phase.pedestrian and self.sub == "green":
            for pd in phase.ped_directions:
                self.ped_walk[pd] = "walk"
        elif phase.pedestrian and self.sub == "yellow":
            for pd in phase.ped_directions:
                self.ped_walk[pd] = "flashing"

    def step(self):
        """Advance one frame."""
        self.timer += 1
        phase = self.current_phase

        if self.sub == "green" and self.timer >= phase.green_time:
            self.sub = "yellow"
            self.timer = 0
            self._apply_phase()
        elif self.sub == "yellow" and self.timer >= phase.yellow_time:
            self.sub = "allred"
            self.timer = 0
            self._apply_phase()
        elif self.sub == "allred" and self.timer >= phase.allred_time:
            # Advance to next phase
            self.phase_idx = (self.phase_idx + 1) % len(self.phases)
            self.sub = "green"
            self.timer = 0
            self._apply_phase()

    def signal_for(self, direction: str, move: MoveType) -> TrafficSignal:
        return self.signals[(direction, move)]

    def phase_info_text(self) -> str:
        p = self.current_phase
        remaining = {
            "green": p.green_time,
            "yellow": p.yellow_time,
            "allred": p.allred_time,
        }[self.sub] - self.timer
        return f"{p.name}  [{self.sub.upper()} {remaining // 60 + 1}s]"


# ══════════════════════════════════════════════════════════════════════════════
# CLASS: Lane
# ══════════════════════════════════════════════════════════════════════════════
class Lane:
    """
    Represents a single lane on one approach.
    Tracks its queue of vehicles and the applicable signal.

    Lane indices (per approach, left-hand traffic):
      lane 0  = right-turn / through (inner lane)
      lane 1  = through / left-turn (outer lane)
    """

    def __init__(self, direction: str, lane_idx: int, controller: IntersectionController):
        self.direction = direction
        self.lane_idx  = lane_idx
        self.controller = controller
        self.vehicles: List['Vehicle'] = []

        # Determine primary movement for this lane
        if lane_idx == 0:
            self.primary_move = MoveType.RIGHT     # inner lane serves right turns
            self.alt_move     = MoveType.STRAIGHT   # also through traffic
        else:
            self.primary_move = MoveType.STRAIGHT   # outer lane serves through
            self.alt_move     = MoveType.LEFT        # also left turns

    @property
    def signal(self) -> TrafficSignal:
        """Return the most permissive signal for this lane's movements."""
        s1 = self.controller.signal_for(self.direction, self.primary_move)
        s2 = self.controller.signal_for(self.direction, self.alt_move)
        if s1.can_go() or s2.can_go():
            return s1 if s1.can_go() else s2
        if s1.is_yellow() or s2.is_yellow():
            return s1 if s1.is_yellow() else s2
        return s1  # both red, return either

    def queue_count(self) -> int:
        return sum(1 for v in self.vehicles if v.active and not v.cleared)


# ══════════════════════════════════════════════════════════════════════════════
# CLASS: Vehicle
# ══════════════════════════════════════════════════════════════════════════════
_uid = 0

# Conflict map: directions whose through traffic conflicts
CONFLICT = {
    "N": {"E", "W"}, "S": {"E", "W"},
    "E": {"N", "S"}, "W": {"N", "S"},
}

class Vehicle:
    """
    A vehicle that spawns, approaches the intersection, waits at a red signal,
    and proceeds through (straight, left, or right) when green.

    Movement model:
      - Straight: continues along entry axis.
      - Left turn: upon entering the intersection box, curves 90° left.
      - Right turn: upon entering the intersection box, curves 90° right.
    """

    def __init__(self, direction: str, lane: Lane, move_type: MoveType, priority: bool = False):
        global _uid
        _uid += 1
        self.uid        = _uid
        self.direction  = direction
        self.lane_obj   = lane
        self.move_type  = move_type
        self.priority   = priority  # Priority vehicle (e.g., ambulance)
        self.speed      = random.uniform(1.8, 2.8)
        self.base_color = DIR_COLOR[direction]
        if priority:
            self.color = "#ff0000"  # Red for priority vehicles
        else:
            self.color      = self._vary(self.base_color)
        self.active     = True
        self.cleared    = False
        self.turning    = False      # set once the car enters the box
        self.turn_progress = 0.0     # 0 → 1 along Bezier curve
        self._turn_a = (0.0, 0.0)   # Bezier start (entry point)
        self._turn_b = (0.0, 0.0)   # Bezier control point
        self._turn_c = (0.0, 0.0)   # Bezier end (exit point)
        self._turn_arc_len = 1.0    # approximate curve length for speed calc
        self._exit_dx = 0.0         # exit heading after turn
        self._exit_dy = 0.0
        self._turn_done = False     # set after turn completes

        lane_idx = lane.lane_idx
        offset = LANE_W // 2 + lane_idx * LANE_W

        # dx, dy: unit direction of travel before any turn
        # Left-hand traffic: vehicles drive on the LEFT side of the road
        if direction == "N":
            self.x, self.y = CX - offset, H + CAR_LEN
            self.dx, self.dy = 0, -1
        elif direction == "S":
            self.x, self.y = CX + offset, -CAR_LEN
            self.dx, self.dy = 0, 1
        elif direction == "E":
            self.x, self.y = -CAR_LEN, CY - offset
            self.dx, self.dy = 1, 0
        elif direction == "W":
            self.x, self.y = W + CAR_LEN, CY + offset
            self.dx, self.dy = -1, 0

        # Store entry coords for turn calculations
        self._entry_x = self.x
        self._entry_y = self.y
        self._heading = math.atan2(self.dy, self.dx)  # radians

    # ── helpers ───────────────────────────────────────────────────────────
    @staticmethod
    def _vary(col: str) -> str:
        r, g, b = int(col[1:3], 16), int(col[3:5], 16), int(col[5:7], 16)
        d = 30
        r = max(0, min(255, r + random.randint(-d, d)))
        g = max(0, min(255, g + random.randint(-d, d)))
        b = max(0, min(255, b + random.randint(-d, d)))
        return f"#{r:02x}{g:02x}{b:02x}"

    @property
    def in_intersection(self) -> bool:
        return (CX - RW < self.x < CX + RW and CY - RW < self.y < CY + RW)

    @property
    def signal(self) -> TrafficSignal:
        # Each vehicle obeys the signal for its OWN movement type.
        # This prevents left-turn cars from going during through phase.
        return self.lane_obj.controller.signal_for(self.direction, self.move_type)

    def _front_past_stop_line(self) -> bool:
        h = CAR_LEN // 2
        if   self.direction == "N": return self.y - h < CY + RW
        elif self.direction == "S": return self.y + h > CY - RW
        elif self.direction == "E": return self.x + h > CX - RW
        elif self.direction == "W": return self.x - h < CX + RW
        return False

    def _stop_coord(self) -> float:
        h = CAR_LEN // 2 + STOP_GAP
        if   self.direction == "N": return CY + RW + h
        elif self.direction == "S": return CY - RW - h
        elif self.direction == "E": return CX - RW - h
        elif self.direction == "W": return CX + RW + h
        return 0

    def _approaching_stop(self) -> bool:
        sc = self._stop_coord()
        sp = self.speed + 2
        if   self.direction == "N": return self.y >= sc - sp
        elif self.direction == "S": return self.y <= sc + sp
        elif self.direction == "E": return self.x <= sc + sp
        elif self.direction == "W": return self.x >= sc - sp
        return False

    def _is_blocked(self, others: list) -> bool:
        # Priority vehicles never wait for other traffic
        if self.priority:
            return False
            
        # While actively turning on a Bezier path, skip blocking
        if self.turning:
            return False
        GAP = CAR_LEN + 10
        if self._turn_done:
            # After turn: generic proximity check using dot-product with
            # current heading so we don't clip into exit-lane traffic.
            for c in others:
                if c is self or not c.active:
                    continue
                dx = c.x - self.x
                dy = c.y - self.y
                ahead = dx * self.dx + dy * self.dy   # positive = ahead
                if 0 < ahead < GAP:
                    lateral = abs(dx * self.dy - dy * self.dx)
                    if lateral < CAR_WID * 1.5:
                        return True
            return False
        # Normal same-lane check for approach traffic
        for c in others:
            if (c is self or not c.active or
                c.direction != self.direction or c.lane_obj is not self.lane_obj):
                continue
            if   self.direction == "N" and 0 < self.y - c.y < GAP: return True
            elif self.direction == "S" and 0 < c.y - self.y < GAP: return True
            elif self.direction == "E" and 0 < c.x - self.x < GAP: return True
            elif self.direction == "W" and 0 < self.x - c.x < GAP: return True
        return False

    # ── turn geometry setup ───────────────────────────────────────────────
    def _setup_turn(self):
        """Compute a quadratic Bezier path for turning through the intersection.

        Bezier: P(t) = (1-t)²·A + 2(1-t)t·B + t²·C   where t ∈ [0, 1]
          A = car's current position (entry edge of intersection)
          B = control point inside the intersection (shapes the curve)
          C = exit point on the opposite edge

        Exit lanes (left-hand traffic – India):
          Northbound exit  →  x < CX  (west half)
          Southbound exit  →  x > CX  (east half)
          Eastbound  exit  →  y < CY  (north half)
          Westbound  exit  →  y > CY  (south half)
        """
        self.turning = True
        self.turn_progress = 0.0

        # A = current position (at intersection edge)
        ax, ay = self.x, self.y

        # Inner-lane exit offset = 20, Outer-lane exit offset = 60
        IN_OFF = LANE_W // 2                 # 20
        OUT_OFF = LANE_W // 2 + LANE_W       # 60

        if self.move_type == MoveType.LEFT:
            # Left turns are tight/easy in LHT (don't cross oncoming traffic)
            # Exit into the outer lane of the cross road
            if self.direction == "N":      # N → West (tight turn near SW corner)
                bx, by = CX - OUT_OFF, CY + OUT_OFF
                cx, cy = CX - RW, CY + OUT_OFF
                self._exit_dx, self._exit_dy = -1, 0
            elif self.direction == "S":    # S → East (tight turn near NE corner)
                bx, by = CX + OUT_OFF, CY - OUT_OFF
                cx, cy = CX + RW, CY - OUT_OFF
                self._exit_dx, self._exit_dy = 1, 0
            elif self.direction == "E":    # E → North (tight turn near NW corner)
                bx, by = CX - OUT_OFF, CY - OUT_OFF
                cx, cy = CX - OUT_OFF, CY - RW
                self._exit_dx, self._exit_dy = 0, -1
            elif self.direction == "W":    # W → South (tight turn near SE corner)
                bx, by = CX + OUT_OFF, CY + OUT_OFF
                cx, cy = CX + OUT_OFF, CY + RW
                self._exit_dx, self._exit_dy = 0, 1
        else:  # RIGHT
            # Right turns are wide/crossing in LHT (cross oncoming traffic)
            # Exit into the inner lane of the cross road
            if self.direction == "N":      # N → East (wide crossing turn)
                bx, by = CX - IN_OFF, CY - 30
                cx, cy = CX + RW, CY - IN_OFF
                self._exit_dx, self._exit_dy = 1, 0
            elif self.direction == "S":    # S → West (wide crossing turn)
                bx, by = CX + IN_OFF, CY + 30
                cx, cy = CX - RW, CY + IN_OFF
                self._exit_dx, self._exit_dy = -1, 0
            elif self.direction == "E":    # E → South (wide crossing turn)
                bx, by = CX + 30, CY - IN_OFF
                cx, cy = CX + IN_OFF, CY + RW
                self._exit_dx, self._exit_dy = 0, 1
            elif self.direction == "W":    # W → North (wide crossing turn)
                bx, by = CX - 30, CY + IN_OFF
                cx, cy = CX - IN_OFF, CY - RW
                self._exit_dx, self._exit_dy = 0, -1

        self._turn_a = (ax, ay)
        self._turn_b = (bx, by)
        self._turn_c = (cx, cy)

        # Approximate arc length from chord distances for speed calculation
        d_ab = math.hypot(bx - ax, by - ay)
        d_bc = math.hypot(cx - bx, cy - by)
        self._turn_arc_len = d_ab + d_bc

    # ── step ──────────────────────────────────────────────────────────────
    def step(self, others: list):
        if not self.active:
            return

        inside = self.in_intersection

        # ── Stop logic (only before entering intersection) ────────────────
        if not inside and not self.turning:
            should_stop = False
            sig = self.signal

            if not self.priority:  # Priority vehicles completely ignore signals
                if not sig.can_go() and not sig.is_yellow():
                    should_stop = True
                elif sig.is_yellow() and not self._front_past_stop_line():
                    should_stop = True

            if should_stop and self._approaching_stop():
                return

            # Yield to conflicting traffic inside box (ambulances never yield)
            conflict_inside = any(
                c.direction in CONFLICT[self.direction] and c.in_intersection
                for c in others if c is not self and c.active
            )
            if conflict_inside and self._approaching_stop() and not self.priority:
                return

        # ── Follow-the-leader ─────────────────────────────────────────────
        if self._is_blocked(others):
            return

        # ── Turning logic in intersection ─────────────────────────────────
        if (self.move_type != MoveType.STRAIGHT and inside
                and not self.turning and not self._turn_done):
            self._setup_turn()

        if self.turning:
            # Follow quadratic Bezier: P(t) = (1-t)²A + 2(1-t)tB + t²C
            step_frac = self.speed / max(self._turn_arc_len, 1)
            self.turn_progress = min(1.0, self.turn_progress + step_frac)
            t = self.turn_progress
            u = 1 - t

            ax, ay = self._turn_a
            bx, by = self._turn_b
            cx, cy = self._turn_c
            self.x = u * u * ax + 2 * u * t * bx + t * t * cx
            self.y = u * u * ay + 2 * u * t * by + t * t * cy

            # Heading from derivative: P'(t) = 2(1-t)(B-A) + 2t(C-B)
            dx_dt = 2 * u * (bx - ax) + 2 * t * (cx - bx)
            dy_dt = 2 * u * (by - ay) + 2 * t * (cy - by)
            if abs(dx_dt) + abs(dy_dt) > 0.001:
                self._heading = math.atan2(dy_dt, dx_dt)

            if self.turn_progress >= 1.0:
                # Turn complete – switch to straight exit
                self.turning = False
                self._turn_done = True
                self.dx = self._exit_dx
                self.dy = self._exit_dy
                self._heading = math.atan2(self.dy, self.dx)
        else:
            # ── Straight movement ─────────────────────────────────────────
            self.x += self.dx * self.speed
            self.y += self.dy * self.speed

        # ── Mark cleared once rear exits far side ─────────────────────────
        if not self.cleared and not self.in_intersection:
            if self._front_past_stop_line():
                self.cleared = True

        # ── Remove when off-screen ────────────────────────────────────────
        margin = 120
        if self.x < -margin or self.x > W + margin or self.y < -margin or self.y > H + margin:
            self.active = False

    # ── draw ──────────────────────────────────────────────────────────────
    def draw(self, cv: tk.Canvas, frame: int = 0):
        if not self.active:
            return
        x, y = self.x, self.y
        heading = self._heading

        # Compute rotated rectangle corners
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        hl = CAR_LEN / 2
        hw = CAR_WID / 2

        corners = []
        for sx, sy in [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]:
            rx = x + sx * cos_h - sy * sin_h
            ry = y + sx * sin_h + sy * cos_h
            corners.append(rx)
            corners.append(ry)

        cv.create_polygon(corners, fill=self.color, outline="#111", width=1, tags="car")

        # Windshield (front)
        f = 0.35  # fraction from front
        ww = 0.6  # width fraction
        for (sf, fc) in [(f, "#aaddff"), (-f, "#ff3333")]:  # front=windshield, rear=tail
            ws = []
            for ssx, ssy in [(-ww * hw, sf * hl), (ww * hw, sf * hl),
                              (ww * hw, sf * hl * 0.6), (-ww * hw, sf * hl * 0.6)]:
                rx = x + ssy * cos_h - ssx * sin_h
                ry = y + ssy * sin_h + ssx * cos_h
                ws.extend([rx, ry])
            cv.create_polygon(ws, fill=fc, outline="", tags="car")

        # Turn indicator for turning vehicles
        if self.move_type == MoveType.LEFT:
            ix = x - hw * sin_h * 0.8 - hl * cos_h * 0.3
            iy = y + hw * cos_h * 0.8 - hl * sin_h * 0.3
            cv.create_oval(ix - 2, iy - 2, ix + 2, iy + 2,
                          fill="#ffaa00", outline="", tags="car")
        elif self.move_type == MoveType.RIGHT:
            ix = x + hw * sin_h * 0.8 - hl * cos_h * 0.3
            iy = y - hw * cos_h * 0.8 - hl * sin_h * 0.3
            cv.create_oval(ix - 2, iy - 2, ix + 2, iy + 2,
                          fill="#ffaa00", outline="", tags="car")

        # Priority vehicle siren (flashing red/blue)
        if self.priority:
            # Flash every few frames
            flash = (self.uid + frame) // 10 % 2
            siren_color = "#ff0000" if flash else "#0000ff"
            # Siren on roof
            sx = x - hw * sin_h * 0.5
            sy = y + hw * cos_h * 0.5
            cv.create_oval(sx - 3, sy - 3, sx + 3, sy + 3,
                          fill=siren_color, outline="", tags="car")


# ══════════════════════════════════════════════════════════════════════════════
# CLASS: Pedestrian  – simple crossing entity
# ══════════════════════════════════════════════════════════════════════════════
class Pedestrian:
    """A pedestrian that walks across the crosswalk during WALK phase."""

    def __init__(self, crosswalk: str, side: int):
        self.crosswalk = crosswalk  # "N","S","E","W" – which crosswalk
        self.side = side            # 0 or 1 – direction of walk
        self.active = True
        self.progress = 0.0
        self.speed = random.uniform(0.006, 0.012)
        self.offset = random.uniform(-8, 8)

        # Pre-compute path endpoints
        sw = 12  # sidewalk offset
        if crosswalk == "N":  # crosses top of intersection (horizontal)
            self.x0, self.y0 = CX - RW - 12, CY - RW - 10 + self.offset
            self.x1, self.y1 = CX + RW + 12, CY - RW - 10 + self.offset
        elif crosswalk == "S":
            self.x0, self.y0 = CX - RW - 12, CY + RW + 10 + self.offset
            self.x1, self.y1 = CX + RW + 12, CY + RW + 10 + self.offset
        elif crosswalk == "E":
            self.x0, self.y0 = CX + RW + 10 + self.offset, CY - RW - 12
            self.x1, self.y1 = CX + RW + 10 + self.offset, CY + RW + 12
        elif crosswalk == "W":
            self.x0, self.y0 = CX - RW - 10 + self.offset, CY - RW - 12
            self.x1, self.y1 = CX - RW - 10 + self.offset, CY + RW + 12

        if side == 1:
            self.x0, self.y0, self.x1, self.y1 = self.x1, self.y1, self.x0, self.y0

    def step(self):
        self.progress += self.speed
        if self.progress >= 1.0:
            self.active = False

    @property
    def x(self):
        return self.x0 + (self.x1 - self.x0) * self.progress

    @property
    def y(self):
        return self.y0 + (self.y1 - self.y0) * self.progress

    def draw(self, cv: tk.Canvas):
        if not self.active:
            return
        r = 4
        cv.create_oval(self.x - r, self.y - r, self.x + r, self.y + r,
                       fill="#eeeecc", outline="#888", tags="ped")
        # Head
        cv.create_oval(self.x - 2, self.y - r - 4, self.x + 2, self.y - r,
                       fill="#eeeecc", outline="", tags="ped")


# ══════════════════════════════════════════════════════════════════════════════
# CLASS: Simulation  – main app
# ══════════════════════════════════════════════════════════════════════════════
class Simulation:
    """
    Main simulation window. Creates the intersection, controller, vehicles,
    pedestrians, and a control panel with adjustable parameters.
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("4-Lane Intersection Traffic Signal Simulation")
        root.resizable(False, False)
        root.configure(bg="#0f0f1a")

        # ── Adjustable parameters ─────────────────────────────────────────
        self.sim_speed    = tk.IntVar(value=1)       # 1–5 multiplier
        self.traffic_density = tk.IntVar(value=50)   # spawn rate (lower = more)
        self.green_scale  = tk.DoubleVar(value=1.0)  # scale factor for green time
        self.paused       = tk.BooleanVar(value=False)

        # ── Header ────────────────────────────────────────────────────────
        hdr = tk.Frame(root, bg="#0f0f1a", pady=6)
        hdr.pack(fill="x")
        tk.Label(hdr, text="4-LANE INTERSECTION TRAFFIC SIGNAL SIMULATION",
                 font=("Consolas", 13, "bold"), fg="#00d4ff",
                 bg="#0f0f1a").pack(side="left", padx=12)
        self.phase_lbl = tk.Label(hdr, text="", font=("Consolas", 10),
                                  fg="#ffd700", bg="#0f0f1a")
        self.phase_lbl.pack(side="right", padx=12)

        # ── Canvas ────────────────────────────────────────────────────────
        self.cv = tk.Canvas(root, width=W, height=H, bg=BG_COLOR,
                            highlightthickness=0)
        self.cv.pack()

        # ── Stats bar ────────────────────────────────────────────────────
        self.stats_lbl = tk.Label(root, text="", font=("Consolas", 9),
                                  fg="#888899", bg="#0f0f1a", anchor="w")
        self.stats_lbl.pack(fill="x", padx=12)

        # ── Control panel ─────────────────────────────────────────────────
        ctrl = tk.LabelFrame(root, text=" Controls ", font=("Consolas", 10, "bold"),
                             fg="#aaaacc", bg="#0f0f1a", bd=1, relief="groove",
                             padx=8, pady=6)
        ctrl.pack(fill="x", padx=10, pady=(4, 8))

        self._add_slider(ctrl, "Sim Speed", self.sim_speed, 1, 5, 0)
        self._add_slider(ctrl, "Traffic Density", self.traffic_density, 20, 120, 1)
        self._add_slider(ctrl, "Green Time ×", self.green_scale, 0.5, 2.0, 2,
                         resolution=0.1)

        btn_frame = tk.Frame(ctrl, bg="#0f0f1a")
        btn_frame.grid(row=0, column=6, rowspan=2, padx=(20, 0))
        self.pause_btn = tk.Button(
            btn_frame, text="⏸ Pause", font=("Consolas", 9, "bold"),
            fg="#ffffff", bg="#444466", activebackground="#555577",
            command=self._toggle_pause, width=10)
        self.pause_btn.pack(pady=2)
        tk.Button(
            btn_frame, text="↻ Reset", font=("Consolas", 9, "bold"),
            fg="#ffffff", bg="#664444", activebackground="#775555",
            command=self._reset, width=10).pack(pady=2)

        # ── Legend ────────────────────────────────────────────────────────
        leg = tk.Frame(root, bg="#0f0f1a", pady=4)
        leg.pack(fill="x")
        for d, label in [("N", "North ↑"), ("S", "South ↓"),
                         ("E", "East →"), ("W", "West ←")]:
            f = tk.Frame(leg, bg="#0f0f1a")
            f.pack(side="left", padx=10)
            tk.Canvas(f, width=12, height=12, bg=DIR_COLOR[d],
                      highlightthickness=0).pack(side="left")
            tk.Label(f, text=f" {label}", font=("Consolas", 9),
                     fg="#aaaaaa", bg="#0f0f1a").pack(side="left")
        # Movement legend
        tk.Label(leg, text="  ◀ Left  ▲ Straight  ▶ Right",
                 font=("Consolas", 9), fg="#777", bg="#0f0f1a").pack(side="right", padx=12)

        # ── Build simulation objects ──────────────────────────────────────
        self._build_phases()
        self.controller = IntersectionController(self.phases)
        self._build_lanes()
        self.vehicles: List[Vehicle] = []
        self.pedestrians: List[Pedestrian] = []
        self.spawn_timers = {d: random.randint(0, 30) for d in "NSEW"}
        self.frame = 0
        self.total_passed = 0
        self.total_peds = 0

        self._draw_static()
        self._loop()

    # ── UI helpers ────────────────────────────────────────────────────────
    def _add_slider(self, parent, label, var, from_, to, col, resolution=1):
        tk.Label(parent, text=label, font=("Consolas", 9), fg="#aaaacc",
                 bg="#0f0f1a").grid(row=0, column=col * 2, sticky="w", padx=(0, 4))
        s = tk.Scale(parent, variable=var, from_=from_, to=to,
                     orient="horizontal", length=120,
                     bg="#0f0f1a", fg="#ccccdd", troughcolor="#333355",
                     highlightthickness=0, resolution=resolution,
                     font=("Consolas", 8))
        s.grid(row=1, column=col * 2, padx=(0, 12))

    def _toggle_pause(self):
        self.paused.set(not self.paused.get())
        self.pause_btn.config(text="▶ Resume" if self.paused.get() else "⏸ Pause")

    def _reset(self):
        self.vehicles.clear()
        self.pedestrians.clear()
        self.frame = 0
        self.total_passed = 0
        self.total_peds = 0
        self._build_phases()
        self.controller = IntersectionController(self.phases)
        self._build_lanes()
        self.spawn_timers = {d: random.randint(0, 30) for d in "NSEW"}

    def _build_phases(self):
        """Build phase list, applying green time scaling."""
        scale = self.green_scale.get()
        self.phases = []
        for p in DEFAULT_PHASES:
            self.phases.append(PhaseSpec(
                name=p.name,
                green_moves=list(p.green_moves),
                green_time=max(30, int(p.green_time * scale)),
                yellow_time=p.yellow_time,
                allred_time=p.allred_time,
                pedestrian=p.pedestrian,
                ped_directions=list(p.ped_directions),
            ))

    def _build_lanes(self):
        """Create Lane objects for every approach."""
        self.lanes: Dict[Tuple[str, int], Lane] = {}
        for d in "NSEW":
            for li in range(NUM_LANES):
                self.lanes[(d, li)] = Lane(d, li, self.controller)

    # ══════════════════════════════════════════════════════════════════════
    # DRAWING – static background (called once)
    # ══════════════════════════════════════════════════════════════════════
    def _draw_static(self):
        cv = self.cv
        sw = 10  # sidewalk width

        # Grass quadrants
        for x0, y0, x1, y1 in [
            (0, 0, CX - RW, CY - RW), (CX + RW, 0, W, CY - RW),
            (0, CY + RW, CX - RW, H), (CX + RW, CY + RW, W, H)
        ]:
            cv.create_rectangle(x0, y0, x1, y1, fill=GRASS_COLOR, outline="")

        # Trees
        for tx, ty in [(80, 80), (160, 95), (95, 160),
                       (W - 80, 80), (W - 160, 95), (W - 95, 160),
                       (80, H - 80), (160, H - 95), (95, H - 160),
                       (W - 80, H - 80), (W - 160, H - 95), (W - 95, H - 160)]:
            cv.create_oval(tx - 14, ty - 14, tx + 14, ty + 14,
                          fill="#2a5a3a", outline="")
            cv.create_oval(tx - 9, ty - 20, tx + 9, ty + 4,
                          fill="#3a7a4a", outline="")

        # Roads
        cv.create_rectangle(CX - RW, 0, CX + RW, H, fill=ROAD_COLOR, outline="")
        cv.create_rectangle(0, CY - RW, W, CY + RW, fill=ROAD_COLOR, outline="")
        cv.create_rectangle(CX - RW, CY - RW, CX + RW, CY + RW,
                           fill=INTER_COLOR, outline="")

        # Sidewalks
        for x0, y0, x1, y1 in [
            (CX - RW, 0, CX - RW + sw, H), (CX + RW - sw, 0, CX + RW, H),
            (0, CY - RW, W, CY - RW + sw), (0, CY + RW - sw, W, CY + RW)
        ]:
            cv.create_rectangle(x0, y0, x1, y1, fill=SIDEWALK_CLR, outline="")

        # Lane centre lines (dashed yellow)
        dash = (18, 14)
        cv.create_line(CX, 0, CX, CY - RW, fill=LANE_LINE, width=2, dash=dash)
        cv.create_line(CX, CY + RW, CX, H, fill=LANE_LINE, width=2, dash=dash)
        cv.create_line(0, CY, CX - RW, CY, fill=LANE_LINE, width=2, dash=dash)
        cv.create_line(CX + RW, CY, W, CY, fill=LANE_LINE, width=2, dash=dash)

        # Sub-lane lines
        for off in (-LANE_W, LANE_W):
            for args in [
                (CX + off, 0, CX + off, CY - RW),
                (CX + off, CY + RW, CX + off, H),
                (0, CY + off, CX - RW, CY + off),
                (CX + RW, CY + off, W, CY + off),
            ]:
                cv.create_line(*args, fill=SUBLINE, width=1, dash=(10, 20))

        # Stop lines
        lw = 3
        for (xa, ya, xb, yb) in [
            (CX - RW + sw, CY - RW, CX + RW - sw, CY - RW),
            (CX - RW + sw, CY + RW, CX + RW - sw, CY + RW),
            (CX - RW, CY - RW + sw, CX - RW, CY + RW - sw),
            (CX + RW, CY - RW + sw, CX + RW, CY + RW - sw),
        ]:
            cv.create_line(xa, ya, xb, yb, fill="white", width=lw)

        # Crosswalks (zebra stripes)
        s, gap = 8, 18
        for i in range(10):
            ox = CX - RW + sw + 6 + i * gap
            if ox + s > CX + RW - sw:
                break
            cv.create_rectangle(ox, CY - RW - 20, ox + s, CY - RW,
                               fill="#77778a", outline="")
            cv.create_rectangle(ox, CY + RW, ox + s, CY + RW + 20,
                               fill="#77778a", outline="")
        for i in range(10):
            oy = CY - RW + sw + 6 + i * gap
            if oy + s > CY + RW - sw:
                break
            cv.create_rectangle(CX - RW - 20, oy, CX - RW, oy + s,
                               fill="#77778a", outline="")
            cv.create_rectangle(CX + RW, oy, CX + RW + 20, oy + s,
                               fill="#77778a", outline="")

        # Direction labels in grass areas
        for d, tx, ty in [("N", CX, 30), ("S", CX, H - 30),
                          ("E", W - 30, CY), ("W", 30, CY)]:
            cv.create_text(tx, ty, text=d, font=("Consolas", 16, "bold"),
                          fill=DIR_COLOR[d])

        # Lane arrow markings on road surface
        self._draw_lane_arrows(cv)

    def _draw_lane_arrows(self, cv: tk.Canvas):
        """Draw arrow markings on the road indicating permitted movements."""
        arrow_color = "#555570"
        # For each approach, draw arrows on each lane
        off0 = LANE_W // 2          # inner lane offset
        off1 = LANE_W // 2 + LANE_W  # outer lane offset

        # N approach (coming from bottom going up, on WEST side in LHT):
        # Inner lane (lane 0) = straight + right, Outer lane (lane 1) = straight + left
        y_arr = CY + RW + 60
        # Inner lane (right turn + through)
        cv.create_text(CX - off0, y_arr, text="↑→", font=("Consolas", 10, "bold"),
                      fill=arrow_color, tags="static")
        cv.create_text(CX - off1, y_arr, text="↑←", font=("Consolas", 10, "bold"),
                      fill=arrow_color, tags="static")

        # S approach (on EAST side in LHT)
        y_arr = CY - RW - 60
        cv.create_text(CX + off0, y_arr, text="↓←", font=("Consolas", 10, "bold"),
                      fill=arrow_color, tags="static")
        cv.create_text(CX + off1, y_arr, text="↓→", font=("Consolas", 10, "bold"),
                      fill=arrow_color, tags="static")

        # E approach (on NORTH side in LHT)
        x_arr = CX - RW - 60
        cv.create_text(x_arr, CY - off0, text="→\n↓", font=("Consolas", 9, "bold"),
                      fill=arrow_color, tags="static")
        cv.create_text(x_arr, CY - off1, text="→\n↑", font=("Consolas", 9, "bold"),
                      fill=arrow_color, tags="static")

        # W approach (on SOUTH side in LHT)
        x_arr = CX + RW + 60
        cv.create_text(x_arr, CY + off0, text="←\n↑", font=("Consolas", 9, "bold"),
                      fill=arrow_color, tags="static")
        cv.create_text(x_arr, CY + off1, text="←\n↓", font=("Consolas", 9, "bold"),
                      fill=arrow_color, tags="static")

    # ══════════════════════════════════════════════════════════════════════
    # DRAWING – dynamic elements (each frame)
    # ══════════════════════════════════════════════════════════════════════
    def _draw_signals(self):
        """Draw signal heads for each approach showing current state."""
        self.cv.delete("light")
        cv = self.cv
        ctrl = self.controller

        # Signal head positions (for each direction, show a signal box)
        configs = {
            "N": (CX + RW + 14, CY + RW + 14),
            "S": (CX - RW - 56, CY - RW - 56),
            "E": (CX + RW + 14, CY - RW - 56),
            "W": (CX - RW - 56, CY + RW + 14),
        }
        bw, bh = 42, 54

        for d, (lx, ly) in configs.items():
            # Background box
            cv.create_rectangle(lx, ly, lx + bw, ly + bh,
                               fill="#111122", outline="#333344", width=2, tags="light")

            # Three lights: through signal
            through_sig = ctrl.signal_for(d, MoveType.STRAIGHT)
            right_sig   = ctrl.signal_for(d, MoveType.RIGHT)

            for i, phase in enumerate(["red", "yellow", "green"]):
                by = ly + 5 + i * 16
                # Through signal (right half)
                active_t = through_sig.state == phase
                col = {"red": LIGHT_RED_C, "yellow": LIGHT_YEL_C,
                       "green": LIGHT_GRN_C}[phase]
                fill_t = col if active_t else LIGHT_OFF_C
                cx_l = lx + bw // 2 + 8
                if active_t:
                    cv.create_oval(cx_l - 7, by - 1, cx_l + 7, by + 11,
                                  fill=fill_t, outline="", tags="light")
                cv.create_oval(cx_l - 5, by + 1, cx_l + 5, by + 9,
                              fill=fill_t, outline="#000", tags="light")

            # Right arrow indicator (left half) – just green/off
            right_col = LIGHT_GRN_C if right_sig.can_go() else (
                LIGHT_YEL_C if right_sig.is_yellow() else LIGHT_OFF_C)
            arr_cx = lx + bw // 2 - 8
            arr_cy = ly + bh // 2
            cv.create_oval(arr_cx - 5, arr_cy - 5, arr_cx + 5, arr_cy + 5,
                          fill=right_col, outline="#000", tags="light")
            if right_sig.can_go() or right_sig.is_yellow():
                cv.create_text(arr_cx, arr_cy, text="→", font=("Consolas", 7, "bold"),
                              fill="#000", tags="light")

            # Direction label
            cv.create_text(lx + bw // 2, ly + bh + 10, text=d,
                          font=("Consolas", 9, "bold"),
                          fill=DIR_COLOR[d], tags="light")

        # ── Pedestrian signal indicators ──────────────────────────────────
        ped_positions = {
            "N": (CX, CY - RW - 30),
            "S": (CX, CY + RW + 30),
            "E": (CX + RW + 30, CY),
            "W": (CX - RW - 30, CY),
        }
        for d, (px, py) in ped_positions.items():
            state = ctrl.ped_walk[d]
            if state == "walk":
                col = PED_WALK_C
                txt = "🚶"
            elif state == "flashing":
                # Flash: alternate visibility
                col = PED_DONTWALK if (self.frame // 8) % 2 == 0 else "#000000"
                txt = "✋"
            else:
                col = PED_DONTWALK
                txt = "✋"
            cv.create_rectangle(px - 10, py - 10, px + 10, py + 10,
                               fill="#111", outline="#333", tags="light")
            cv.create_text(px, py, text=txt, font=("Consolas", 10),
                          fill=col, tags="light")

    # ══════════════════════════════════════════════════════════════════════
    # SPAWNING
    # ══════════════════════════════════════════════════════════════════════
    def _spawn(self):
        density = self.traffic_density.get()
        for d in "NSEW":
            self.spawn_timers[d] += 1
            threshold = density + random.randint(-10, 10)
            if self.spawn_timers[d] < threshold:
                continue
            self.spawn_timers[d] = 0

            lane_idx = random.randint(0, NUM_LANES - 1)
            lane = self.lanes[(d, lane_idx)]

            # Decide movement type based on lane
            r = random.random()
            if lane_idx == 0:       # inner lane
                move = MoveType.RIGHT if r < 0.20 else MoveType.STRAIGHT
            else:                   # outer lane
                move = MoveType.LEFT if r < 0.20 else MoveType.STRAIGHT

            # Don't spawn on top of existing vehicles
            too_close = any(
                v.direction == d and v.lane_obj is lane and v.active and
                math.hypot(v.x - CX, v.y - CY) > (H // 2 - 80)
                for v in self.vehicles
            )
            if not too_close:
                # Occasionally spawn priority vehicles (ambulances)
                is_priority = random.random() < 0.15  # 15% chance for more frequent ambulances
                v = Vehicle(d, lane, move, priority=is_priority)
                self.vehicles.append(v)
                lane.vehicles.append(v)

    def _spawn_pedestrians(self):
        """Spawn pedestrians when walk signal activates."""
        ctrl = self.controller
        for d in "NSEW":
            if ctrl.ped_walk[d] == "walk" and random.random() < 0.008:
                side = random.randint(0, 1)
                self.pedestrians.append(Pedestrian(d, side))
                self.total_peds += 1

    # ══════════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ══════════════════════════════════════════════════════════════════════
    def _loop(self):
        if not self.paused.get():
            speed = self.sim_speed.get()
            for _ in range(speed):
                self._tick()

        # Redraw dynamic elements
        self.cv.delete("car")
        self.cv.delete("ped")
        self.cv.delete("light")
        self.cv.delete("queue")

        for v in self.vehicles:
            v.draw(self.cv, self.frame)
        for p in self.pedestrians:
            p.draw(self.cv)
        self._draw_signals()
        self._draw_queue_counts()

        # Update phase info display
        self.phase_lbl.config(text=self.controller.phase_info_text())

        # Stats
        queue_n = sum(self.lanes[(d, l)].queue_count()
                      for d in "NSEW" for l in range(NUM_LANES))
        in_box = sum(1 for v in self.vehicles if v.in_intersection)
        cycle_s = self.controller.cycle_length / 60
        self.stats_lbl.config(
            text=f"  Vehicles: {len(self.vehicles):3d}  │  "
                 f"Queued: {queue_n}  │  In intersection: {in_box}  │  "
                 f"Passed: {self.total_passed}  │  Pedestrians: {self.total_peds}  │  "
                 f"Cycle: {cycle_s:.1f}s  │  Frame: {self.frame}"
        )

        self.root.after(16, self._loop)

    def _tick(self):
        """One simulation frame."""
        self.frame += 1
        self.controller.step()
        self._spawn()
        self._spawn_pedestrians()

        before = len(self.vehicles)
        for v in self.vehicles:
            v.step(self.vehicles)

        # Clean up inactive
        for lane in self.lanes.values():
            lane.vehicles = [v for v in lane.vehicles if v.active]
        self.vehicles = [v for v in self.vehicles if v.active]
        self.total_passed += before - len(self.vehicles)

        for p in self.pedestrians:
            p.step()
        self.pedestrians = [p for p in self.pedestrians if p.active]

    def _draw_queue_counts(self):
        """Show vehicle queue counts near each approach."""
        cv = self.cv
        positions = {
            "N": (CX + RW + 65, CY + RW + 90),
            "S": (CX - RW - 65, CY - RW - 90),
            "E": (CX - RW - 90, CY + RW + 65),
            "W": (CX + RW + 90, CY - RW - 65),
        }
        for d, (qx, qy) in positions.items():
            total_q = sum(self.lanes[(d, l)].queue_count() for l in range(NUM_LANES))
            if total_q > 0:
                cv.create_text(qx, qy, text=f"Q:{total_q}",
                              font=("Consolas", 10, "bold"),
                              fill="#ffaa44", tags="queue")


# ══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app = Simulation(root)
    root.mainloop()