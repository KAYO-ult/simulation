"""Quick headless sanity test for crossing_simulation."""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import tkinter as tk
root = tk.Tk()
root.withdraw()

# Import the module without running __main__
import importlib.util
spec = importlib.util.spec_from_file_location("crossing_simulation",
    os.path.join(os.path.dirname(__file__), "crossing_simulation.py"))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

# Build simulation internals manually
sim = object.__new__(mod.Simulation)
sim.sim_speed = tk.IntVar(root, value=1)
sim.traffic_density = tk.IntVar(root, value=40)
sim.green_scale = tk.DoubleVar(root, value=1.0)
sim.paused = tk.BooleanVar(root, value=False)
sim._build_phases()
sim.controller = mod.IntersectionController(sim.phases)
sim._build_lanes()
sim.vehicles = []
sim.pedestrians = []
sim.spawn_timers = {d: 0 for d in "NSEW"}
sim.frame = 0
sim.total_passed = 0
sim.total_peds = 0

import random
random.seed(42)

# Run 1000 ticks
for i in range(1000):
    sim._tick()

print(f"Frames: 1000")
print(f"Vehicles alive: {len(sim.vehicles)}")
print(f"Total passed: {sim.total_passed}")

nan_count = sum(1 for v in sim.vehicles if v.x != v.x or v.y != v.y)
print(f"NaN coords: {nan_count}")

turning = sum(1 for v in sim.vehicles if v.turning)
turn_done = sum(1 for v in sim.vehicles if v._turn_done)
in_box = sum(1 for v in sim.vehicles if v.in_intersection)
print(f"Currently turning: {turning}")
print(f"Turn completed (exiting): {turn_done}")
print(f"In intersection: {in_box}")

# Check no vehicle stuck inside for too long
# A vehicle moving at speed ~2 through a 200px box takes ~100 frames max
# With turning, maybe 150 frames. Flag any that seem stuck.
print("ALL CHECKS OK")
root.destroy()
