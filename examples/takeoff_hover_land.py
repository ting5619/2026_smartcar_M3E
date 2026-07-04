#!/usr/bin/env python3
"""
Test: Take off 20cm, hover 2s, land.

Prerequisites:
  export ROS_MASTER_URI=http://172.20.10.6:11311
  Box must have PSDK + roscore + fcProxy running
"""

import time
import sys
import rospy
sys.path.insert(0, "D:/ME3-deploy/me3_groundstation")
from flight_control import ME3FlightController

rospy.init_node("test_takeoff", anonymous=True)

ctrl = ME3FlightController()

ALTITUDE = 0.2      # 20 cm
HOVER_TIME = 2.0    # 2 seconds
ASCEND_SPEED = 0.3  # m/s, gentle
ASCEND_TIME = ALTITUDE / ASCEND_SPEED  # ~0.67s

print("=" * 50)
print(f"  Takeoff Test: {ALTITUDE*100}cm, hover {HOVER_TIME}s")
print("=" * 50)
print()

# 1. Takeoff
print("[1/3] Requesting takeoff...")
if not ctrl.takeoff():
    print("ERROR: Takeoff failed!")
    sys.exit(1)
print("       Takeoff OK, waiting 3s for motors to spin up...")
time.sleep(3)

# 2. Ascend to target altitude
print(f"[2/3] Ascending to {ALTITUDE}m...")
ascend_msg_count = int(ASCEND_TIME / 0.1)
for i in range(ascend_msg_count):
    ctrl.set_velocity_body(0, 0, -ASCEND_SPEED, duration=0.1)
    time.sleep(0.1)

# Hover at altitude
print(f"       Hovering {HOVER_TIME}s at {ALTITUDE}m...")
hover_count = int(HOVER_TIME / 0.1)
for i in range(hover_count):
    ctrl.hover(0.1)
    time.sleep(0.1)
    if i % 10 == 0:
        sys.stdout.write(f"\r       {i*0.1:.1f}s / {HOVER_TIME:.1f}s")
        sys.stdout.flush()
print(f"\r       {HOVER_TIME:.1f}s done")

# 3. Land
print("[3/3] Landing...")
if ctrl.land():
    print("       Land command sent, waiting 3s...")
    time.sleep(3)

print()
print("=" * 50)
print("  TEST COMPLETE")
print("=" * 50)
