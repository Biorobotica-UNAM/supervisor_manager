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
from known_locations_interfaces.srv import LocationsGet
from listener_interfaces.srv import Listen
from talker_interfaces.srv import Talk


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manipulation', action='store_true')
    parser.add_argument('--fail-recovery', choices=['approach', 'take'])
    parser.add_argument('--real-locations', action='store_true')
    parser.add_argument('--exhausted', action='store_true')
    parser.add_argument('--continue-on-failure', action='store_true')
    parser.add_argument('--arrival-announcement', action='store_true')
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
                          ['Please go to the kitchen.', 'Could you go to the cooking table?'])

    def plan(req, res):
        raw.publish(Text(data=('Paso 1: Justina take demo_object_1\nPaso 2: Justina say done'
                               if args.manipulation else
                               'Paso 1: Justina go_to missing\n' +
                               ('Paso 2: Justina say I have arrived at missing\n'
                                'Paso 3: Justina say kitchen must remain unchanged\n'
                                'Paso 4: Justina say done' if args.arrival_announcement else
                                'Paso 2: Justina say done'))))
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

    def validate_location(req, res):
        res.success = req.location_name == 'cooking_table'
        res.message = 'OK' if res.success else "Location '%s' not found." % req.location_name
        return res

    services = [node.create_service(Trigger, '/planning/clips/start_planning', plan),
                node.create_service(Talk, '/hri/talker/talk', talk),
                node.create_service(Listen, '/hri/listener/listen', listen)]
    if not args.real_locations:
        services.append(node.create_service(LocationsGet, '/locations_manager/get', validate_location))
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
                 subprocess.Popen([executable('supervisor_manager', 'supervisor_manager_node'), '--ros-args',
                     '-p', 'continue_on_recovery_failure:=' + str(args.continue_on_failure).lower()], stdout=handles[1], stderr=subprocess.STDOUT)]
    handles.append(open(Path(logs) / 'entities.log', 'w'))
    processes.append(subprocess.Popen(
        [executable('semantic_parser', 'entity_extractor_node'), '--ros-args',
         '-r', '__ns:=/planning/semantic_parser'],
        stdout=handles[-1], stderr=subprocess.STDOUT))
    if args.real_locations:
        handles.append(open(Path(logs) / 'locations.log', 'w'))
        processes.append(subprocess.Popen(
            [executable('locations_manager', 'locations_manager_node')],
            stdout=handles[-1], stderr=subprocess.STDOUT))
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
                if args.manipulation and action == 'take' and (
                        instructions.count('Justina take demo_object_1') == 1 or
                        args.fail_recovery == 'take'):
                    report.approved = False
                    report.status = TaskExecutionReport.FAILURE
                    report.failure_reason = 'argument_mismatch'
                    report.worker_result = 'pose_out_of_range'
                elif args.manipulation and action == 'approach' and args.fail_recovery == 'approach':
                    report.approved = False
                    report.status = TaskExecutionReport.FAILURE
                    report.failure_reason = 'navigation_failed'
                    report.worker_result = 'navigation_failed: blocked path'
                elif argument == 'missing':
                    report.approved = False
                    report.status = TaskExecutionReport.FAILURE
                    report.failure_reason = 'location_not_found'
                    report.worker_result = 'navigation_failed: missing: location not found'
                    node.destroy_subscription(subscription)
                    subscription = None
                    recreate_at = now + 2.5
                elif action == 'go_to' and instructions.count('Justina go_to cooking_table') == 1:
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
        manager_log = (Path(logs) / 'manager.log').read_text()
        if args.manipulation:
            expected = ['Justina take demo_object_1', 'Justina approach demo_object_1']
            if args.fail_recovery != 'approach':
                expected.append('Justina take demo_object_1')
            if not args.fail_recovery or args.continue_on_failure:
                expected.append('Justina say done')
            assert instructions == expected, instructions
            assert '[ManipulationRecovery] state=IDLE -> WAIT_APPROACH' in manager_log
            assert "inserted instruction='Justina approach demo_object_1'" in manager_log
            if args.fail_recovery:
                outcome = 'CONTINUE' if args.continue_on_failure else 'STOP'
                assert '[RecoveryOutcome] ' + outcome in manager_log
            print('PASS: manipulation sequence, recovery ownership, continuation and limits')
            print(f'Logs: {logs}')
            return
        assert '[NavigationRecovery] BEGIN' in manager_log, manager_log
        assert 'state=START_SPEAK -> WAIT_SPEAK' in manager_log, manager_log
        assert 'auxiliary LISTEN known_location' in manager_log, manager_log
        if args.exhausted and args.continue_on_failure:
            assert instructions == ['Justina go_to missing', 'Justina say done'], instructions
            assert '[RecoveryOutcome] CONTINUE' in manager_log, manager_log
            assert any('continue with the plan' in phrase for phrase in phrases), phrases
            print('PASS: debug continuation skips failed navigation after exhausted recovery')
            print(f'Logs: {logs}')
            return
        if args.exhausted:
            assert '[RecoveryOutcome] STOP' in manager_log, manager_log
            assert instructions == ['Justina go_to missing'], instructions
            assert any('stop this task' in phrase for phrase in phrases), phrases
            print('PASS: exhausted location captures abort without advancing the original plan')
            print(f'Logs: {logs}')
            return
        expected = ['Justina go_to missing', 'Justina go_to cooking_table',
                    'Justina go_to cooking_table']
        if args.arrival_announcement:
            expected += ['Justina say I have arrived at cooking_table',
                         'Justina say kitchen must remain unchanged']
        expected.append('Justina say done')
        assert instructions == expected, instructions
        assert any('another location' in phrase for phrase in phrases), phrases
        assert any('not a known location' in phrase for phrase in phrases), phrases
        assert any('try again' in phrase for phrase in phrases), phrases
        assert "[PlanMutation] inserted instruction='Justina go_to cooking_table'" in manager_log
        assert '[PlanMutation] queued command=1' in manager_log
        if args.arrival_announcement:
            assert 'Justina say I have arrived at cooking_table' in instructions, instructions
            assert 'Justina say I have arrived at missing' not in instructions, instructions
            assert 'Justina say kitchen must remain unchanged' in instructions, instructions
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
