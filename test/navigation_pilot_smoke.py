#!/usr/bin/env python3
"""No hardware: real manager/plan, simulated planning, executor and HRI.
Run after building and sourcing install/setup.bash. Uses an isolated ROS domain.
"""
import argparse
import os
import subprocess
import time
from pathlib import Path
import tempfile

os.environ.setdefault('ROS_DOMAIN_ID', '173')
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='navigation-pilot-ros-'))
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from clips_interfaces.srv import String
from std_srvs.srv import Trigger
from std_msgs.msg import String as Text, Empty, Bool
from supervisor_interfaces.msg import SystemStatus, TaskExecutionReport
from listener_interfaces.srv import Listen
from talker_interfaces.srv import Talk


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exhausted', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('navigation_pilot_fixture')
    raw = node.create_publisher(Text, '/planning/clips/results', 10)
    status = node.create_publisher(SystemStatus, '/system/status', 10)
    reports = node.create_publisher(TaskExecutionReport, '/supervisor/supervisor_execution/result', 10)
    instructions, phrases, requests = [], [], []
    pending = []
    active_pending = []
    active = node.create_publisher(Bool, '/supervisor/supervisor_plan/active', 10)
    # Deliberately deliver plan status AFTER the executor result.
    active_subscription = node.create_subscription(
        Bool, '/fixture/plan_active',
        lambda msg: active_pending.append((time.monotonic() + 0.8, msg)), 10)
    listen_answers = iter(['unknown place', 'still unknown'] if args.exhausted else
                          ['unknown place', 'kitchen table'])

    def plan(req, res):
        raw.publish(Text(data='Paso 1: Justina go_to missing\nPaso 2: Justina say done'))
        res.success = True
        return res

    def talk(req, res):
        phrases.append(req.text)
        res.success = True
        return res

    def listen(req, res):
        res.success = True
        res.recognized_speech = next(listen_answers)
        return res

    services = [node.create_service(Trigger, '/planning/clips/start_planning', plan),
                node.create_service(Talk, '/hri/talker/talk', talk),
                node.create_service(Listen, '/hri/listener/listen', listen)]
    client = node.create_client(Trigger, '/request_instruction')
    subscription = None
    recreate_at = None

    def trigger(msg):
        future = client.call_async(Trigger.Request())
        requests.append(future)

    logs = tempfile.mkdtemp(prefix='navigation-pilot-smoke-')
    handles = [open(Path(logs) / name, 'w') for name in ['plan.log', 'manager.log']]
    processes = [subprocess.Popen([executable('supervisor_plan', 'supervisor_plan_node'), '--ros-args',
                     '-r', '/supervisor/supervisor_plan/active:=/fixture/plan_active'], stdout=handles[0], stderr=subprocess.STDOUT),
                 subprocess.Popen([executable('supervisor_manager', 'supervisor_manager_node')], stdout=handles[1], stderr=subprocess.STDOUT)]
    started = time.monotonic()
    try:
        while time.monotonic() - started < 35:
            now = time.monotonic()
            # Start the executor late, then disconnect it during recovery.
            if subscription is None and (now - started > 2) and (recreate_at is None or now >= recreate_at):
                subscription = node.create_subscription(Empty, '/supervisor/supervisor_manager/allow_next_instruction', trigger, 10)
                recreate_at = None
            status.publish(SystemStatus(hardware=True, hri=True, navigation=True))
            rclpy.spin_once(node, timeout_sec=0.05)
            for future in list(requests):
                if not future.done():
                    continue
                requests.remove(future)
                response = future.result()
                assert response.success, response.message
                instructions.append(response.message)
                parts = response.message.split(maxsplit=2)
                action, argument = parts[1], parts[2]
                report = TaskExecutionReport(action=action, argument=argument, approved=True,
                    status=TaskExecutionReport.SUCCESS, failure_reason='none', worker_result='success')
                if argument == 'missing':
                    report.approved = False
                    report.status = TaskExecutionReport.FAILURE
                    report.failure_reason = 'location_not_found'
                    report.worker_result = 'navigation_failed: missing: location not found'
                    node.destroy_subscription(subscription)
                    subscription = None
                    recreate_at = now + 2.5
                elif action == 'go_to' and instructions.count('Justina go_to kitchen_table') == 1:
                    report.approved = False
                    report.status = TaskExecutionReport.FAILURE
                    report.failure_reason = 'navigation_failed'
                    report.worker_result = 'navigation_failed: blocked path'
                pending.append((now + 0.15, report))
            for due, message in list(active_pending):
                if now >= due:
                    active.publish(message)
                    active_pending.remove((due, message))
            for due, report in list(pending):
                if now >= due:
                    reports.publish(report)
                    pending.remove((due, report))
            if processes[1].poll() is not None:
                break
        assert processes[1].poll() == 0, f'manager did not finish; logs: {logs}'
        if args.exhausted:
            assert instructions == ['Justina go_to missing'], instructions
            assert any('stop this task' in phrase for phrase in phrases), phrases
            print('PASS: exhausted location captures abort without advancing the original plan')
            print(f'Logs: {logs}')
            return
        assert instructions == ['Justina go_to missing', 'Justina go_to kitchen_table',
                                'Justina go_to kitchen_table', 'Justina say done'], instructions
        assert any('another location' in phrase for phrase in phrases), phrases
        assert any('not a known location' in phrase for phrase in phrases), phrases
        assert any('try again' in phrase for phrase in phrases), phrases
        print('PASS: corrected destination persists across retry, invalid input reprompted, late executor recovered, delayed plan status retained, next original step preserved')
        print(f'Logs: {logs}')
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
