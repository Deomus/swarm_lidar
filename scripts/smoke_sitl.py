#!/usr/bin/env python3
"""Проверка четырёх работающих PX4; запуск симулятора выполняется отдельно."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry, VehicleStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=40.)
    parser.add_argument('--report', default='log/sitl_smoke.json')
    args = parser.parse_args()
    rclpy.init()
    node = Node('sitl_smoke')
    positions, statuses, counts, received = {}, {}, {}, {}
    def odometry(i, msg):
        if all(math.isfinite(float(x)) for x in msg.position):
            positions[i] = [float(x) for x in msg.position]
            counts[i] = counts.get(i, 0) + 1
            received[i] = time.monotonic()
    def status(i, msg):
        statuses[i] = int(msg.system_id)
    subs = []
    for i in range(4):
        subs.append(node.create_subscription(VehicleOdometry, f'/uav_{i}/fmu/out/vehicle_odometry',
                    lambda msg, i=i: odometry(i, msg), qos_profile_sensor_data))
        subs.append(node.create_subscription(VehicleStatus, f'/uav_{i}/fmu/out/vehicle_status_v4',
                    lambda msg, i=i: status(i, msg), qos_profile_sensor_data))
    deadline = time.monotonic() + args.timeout
    ok = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            ok = all(counts.get(i, 0) >= 5 and statuses.get(i) == i + 1 and
                     time.monotonic() - received.get(i, 0.) < 1. for i in range(4))
            if ok:
                break
        report = dict(passed=ok, positions_ned=positions, system_ids=statuses, samples=counts,
                      checked_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
        target = Path(args.report)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
