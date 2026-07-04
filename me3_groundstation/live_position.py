#!/usr/bin/env python3
"""
Live VO position viewer.

Connects to onboard ROS master, subscribes to /uavdata,
runs VO fusion, prints position at 1Hz.

Usage:
    export ROS_MASTER_URI=http://172.20.10.6:11311
    export ROS_IP=<your_ip>
    python live_position.py
"""

import time
import sys
import signal
import rospy
from telemetry import Telemetry, TelemetryFrame
from vo_fusion import VOFusion, FusionConfig


def main():
    rospy.init_node("me3_position_viewer", anonymous=True)

    config = FusionConfig(
        min_health_mask=0x03,   # xHealth AND yHealth
        init_samples=30,         # 3s init
        max_vo_jump=2.0,
        drift_warning=1.0,
        correction_timeout=30.0,
    )

    fusion = VOFusion(config)
    fusion.start()

    tel = Telemetry(queue_size=50)
    frames_processed = 0
    last_print = 0.0

    def on_frame(f: TelemetryFrame):
        nonlocal frames_processed, last_print

        fusion.feed_vo(f.vo_x, f.vo_y, f.vo_z, f.vo_health, f.timestamp)
        fusion.feed_quaternion(*f.quat, f.timestamp)
        fusion.feed_gyro(f.gyro_x, f.gyro_y, f.gyro_z, f.timestamp)

        frames_processed += 1

        # Print status every second
        if f.timestamp - last_print >= 1.0:
            last_print = f.timestamp
            x, y, z = fusion.get_position()
            heading = fusion.get_heading()
            healthy = fusion.is_healthy()
            needs_corr = fusion.needs_correction()

            status = "✓" if healthy else "✗"
            corr_warn = " ⚠ NEEDS CORRECTION" if needs_corr else ""

            sys.stdout.write(
                f"\r[{status}] Pos: ({x:+7.3f}, {y:+7.3f}, {z:+7.3f})m "
                f"| Hdg: {heading*57.3:+6.1f}° "
                f"| VO: ({f.vo_x:+5.2f}, {f.vo_y:+5.2f}) "
                f"| h={f.vo_health}{corr_warn}   "
            )
            sys.stdout.flush()

    tel.on_frame(on_frame)
    tel.start()

    print("\n" + "=" * 70)
    print("  ME3 Indoor Position Viewer")
    print("  Waiting for /uavdata... (needs PSDK + fcProxy running)")
    print("=" * 70)
    print()
    print("  Move the drone to see VO tracking.")
    print("  First ~3s: VO initialization (keep still)")
    print("  Then: position updates in real-time")
    print()

    def shutdown(sig, frame):
        print("\nStopping...")
        tel.stop()
        s = fusion.get_state_dict()
        print(f"  Frames: {frames_processed}")
        for k, v in s.items():
            print(f"  {k}: {v}")
        rospy.signal_shutdown("user")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
