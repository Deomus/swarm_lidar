#!/usr/bin/env python3
"""Проверка states/TF и опциональный последовательный полёт четырёх SITL."""
import argparse
import time

import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from swarm_interfaces.msg import SwarmCommand, SwarmState
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--flight', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('bridge_smoke', parameter_overrides=[Parameter('use_sim_time', value=True)])
    states, statuses = {}, {}
    subs = []
    for i in range(4):
        subs.append(node.create_subscription(SwarmState, f'/uav_{i}/autopilot/state',
                    lambda msg, i=i: states.update({i: msg}), 10))
        subs.append(node.create_subscription(VehicleStatus, f'/uav_{i}/fmu/out/vehicle_status_v4',
                    lambda msg, i=i: statuses.update({i: msg}), qos_profile_sensor_data))
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(SwarmCommand, '/swarm/commands', 10)
    clients = []

    def wait(predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(reason)

    def request(i, action):
        wait(lambda: i in states and states[i].valid, 5., 'Нет свежего состояния для команды')
        client = node.create_client(Trigger, f'/uav_{i}/{action}')
        clients.append(client)
        assert client.wait_for_service(timeout_sec=3.), f'Нет сервиса {i}/{action}'
        future = client.call_async(Trigger.Request())
        wait(future.done, 3., 'Нет ответа сервиса')
        result = future.result()
        assert result.success, result.message

    try:
        wait(lambda: all(i in states and states[i].valid for i in range(4)), 50.,
             'Не получены свежие SwarmState четырёх PX4')
        wait(lambda: all(buffer.can_transform('map', f'uav_{i}/lidar_link', rclpy.time.Time())
                         for i in range(4)), 5., 'Не собрано дерево TF')
        print('OK: четыре свежих SwarmState, TF map → base_link → lidar_link', flush=True)
        if args.flight:
            wait(lambda: all(i in statuses and statuses[i].pre_flight_checks_pass for i in range(4)),
                 40., 'Preflight не пройден')
            for i in range(4):
                request(i, 'arm')
                wait(lambda: states[i].armed, 5., 'Нет подтверждения arm')
                request(i, 'takeoff')
                wait(lambda: states[i].pose.position.z > 2., 25., 'Не подтверждён takeoff')
                print(f'OK: uav_{i} взлетел, z={states[i].pose.position.z:.2f}', flush=True)
            for i in range(4):
                request(i, 'enable_stream')
            wall = time.monotonic()
            wait(lambda: time.monotonic() - wall > 2., 3., 'Heartbeat')
            for i in range(4):
                request(i, 'offboard')
            wait(lambda: all(statuses[i].nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                             for i in range(4)), 5., 'Не подтверждён Offboard')
            print('OK: четыре PX4 подтвердили Offboard', flush=True)
            initial = [states[i].pose.position.x for i in range(4)]
            end = time.monotonic() + 3.
            while time.monotonic() < end:
                for i in range(4):
                    cmd = SwarmCommand(uav_id=i, frame_id='map', valid=True)
                    cmd.source_stamp = node.get_clock().now().to_msg()
                    cmd.target_velocity.linear.x = .3
                    publisher.publish(cmd)
                cycle = time.monotonic() + .05
                while time.monotonic() < cycle:
                    rclpy.spin_once(node, timeout_sec=.005)
            assert all(states[i].pose.position.x > initial[i] + .2 for i in range(4)), 'Нет движения на восток'
            print('OK: четыре аппарата выполнили ENU-команду на восток', flush=True)
            wait(lambda: all(abs(states[i].velocity.linear.x) < .15 for i in range(4)),
                 10., 'Нет торможения после прекращения команд')
            print('OK: устаревшая команда вызывает торможение', flush=True)
            for i in reversed(range(4)):
                request(i, 'land')
                wait(lambda: not states[i].armed, 35., 'Не подтверждена посадка')
                print(f'OK: uav_{i} сел и разоружён', flush=True)
    finally:
        if args.flight:
            for i in range(4):
                if i in states and states[i].armed:
                    try:
                        request(i, 'land')
                    except Exception:
                        pass
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
