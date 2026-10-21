#!/usr/bin/env python3
"""Полётная проверка прямого участка и поворота с посадкой в конце."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.signals import SignalHandlerOptions
from swarm_lidar.scenario import Scenario


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    scenario = Scenario('operator', args.output)
    result = dict(passed=False, landed=False, routes=[])
    try:
        scenario.prepare()
        # Карта штатной площадки должна поступить из runtime до любого S3.
        for x, y in [(12., 0.), (12., 10.)]:
            scenario.call('/leader/hold')
            scenario.planner = ''
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = scenario.node.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, 3.
            goal.pose.orientation.w = 1.
            scenario.goal_pub.publish(goal)
            scenario.wait(lambda: scenario.planner.startswith(('PLANNED', 'REJECTED')), 10., 'Нет результата планирования')
            if not scenario.planner.startswith('PLANNED'):
                raise RuntimeError(scenario.planner)
            scenario.call('/leader/mission', retry=3.)
            start, wall_deadline = scenario.now(), time.monotonic()+300.
            samples = []
            saw_mission = False
            while time.monotonic() < wall_deadline:
                rclpy.spin_once(scenario.node, timeout_sec=.02)
                status = scenario.execution.get('status')
                saw_mission |= status == 'MISSION'
                if scenario.fresh():
                    points = [[s.pose.position.x, s.pose.position.y, s.pose.position.z]
                              for s in [scenario.states[i] for i in range(4)]]
                    velocities = [[s.velocity.linear.x, s.velocity.linear.y, s.velocity.linear.z]
                                  for s in [scenario.states[i] for i in range(4)]]
                    if not samples or scenario.now()-samples[-1]['time'] >= .1:
                        samples.append(dict(time=scenario.now(), positions=points, velocities=velocities,
                                            errors=scenario.status.get('errors'), status=status))
                if saw_mission and status == 'COMPLETE':
                    break
                if status in ('WAYPOINT_TIMEOUT', 'FORMATION_UNAVAILABLE', 'STATE_STALE'):
                    raise RuntimeError(status)
                if scenario.now()-start > 180.:
                    raise RuntimeError('Маршрут не завершён за 180 с симуляции')
            else:
                raise RuntimeError('Истёк предел реального ожидания')
            p = scenario.states[0].pose.position
            duration = scenario.now()-start
            error = math.dist([p.x, p.y, p.z], [x, y, 3.])
            result['routes'].append(dict(goal=[x,y,3.], duration=duration, error=error, samples=samples))
            print(json.dumps(dict(goal=[x,y,3.],duration=duration,error=error),ensure_ascii=False), flush=True)
            rest = scenario.now()
            scenario.wait(lambda: scenario.now()-rest >= 8., 35., 'Нет времени удержания')
        result['passed'] = True
    finally:
        try:
            scenario.land()
            result['landed'] = True
        finally:
            (args.output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
            scenario.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
