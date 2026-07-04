"""
ME3 Ground Station — indoor drone positioning without magnetometer.

Modules:
    telemetry  — Subscribe /uavdata, parse telemetry frames
    vo_fusion  — VO + gyro + AprilTag fusion localization
    utils      — Coordinate transforms, filters, math utilities
    live_position — CLI live position viewer
"""
