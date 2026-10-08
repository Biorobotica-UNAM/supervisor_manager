#!/usr/bin/env python3
"""Integration scenarios for real manager/plan with simulated services; no hardware."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '177')
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='recovery-modules-ros-'))
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo, GoalStatus
from executor_interfaces.action import ExecuteTask
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from supervisor_interfaces.msg import SystemStatus, TaskExecutionReport
from talker_interfaces.srv import Talk
from listener_interfaces.srv import Listen
from head_interfaces.srv import MoveHead

SCENARIOS = ['door_unrelated', 'door_closed', 'door_opens', 'door_service_failure', 'door_timeout',
             'follow', 'follow_enable_failure', 'follow_no_stop', 'follow_no_auth', 'follow_destination_timeout', 'follow_enable_failure',
             'perception', 'perception_exhausted', 'person_search', 'person_refinement',
             'interaction', 'interaction_exhausted', 'interaction_confirm_yes',
             'interaction_confirm_no', 'interaction_confirm_ambiguous',
             'drop', 'verify_take', 'cancel_unavailable', 'cancel_terminal', 'cancel_completed', 'cancel_incomplete']


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=SCENARIOS)
    parser.add_argument('--continue-on-failure', action='store_true')
    args = parser.parse_args()
    scenario, debug = args.scenario, args.continue_on_failure
    root = Path(tempfile.mkdtemp(prefix='recovery-modules-smoke-'))
    config = yaml.safe_load((Path(get_package_share_directory('supervisor_manager')) /
                             'config/recovery_modules.yaml').read_text())
    config['door']['retry_delay_sec'] = 0.05
    config['door']['execution_timeout_sec'] = 1.4
    config['coordinator']['execution_timeout_sec'] = 3.0
    config['coordinator']['cancel_timeout_sec'] = 0.5
    config['follow']['destination_timeout_sec'] = 0.2
    config['follow']['poll_interval_sec'] = 0.05
    config_path = root / 'recovery_modules.yaml'
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    if scenario.startswith('door') or scenario.startswith('cancel_'):
        nominal = ['Justina check door', 'Justina say done']
    elif scenario.startswith('follow'):
        trigger = 'instruct_follow_to_kitchen_table' if scenario == 'follow_destination_timeout' else 'instruct_follow_only'
        nominal = ['Justina say ' + trigger, 'Justina say done']
    elif scenario.startswith('perception'):
        nominal = ['Justina find_object demo_object_1', 'Justina say done']
    elif scenario == 'person_search':
        nominal = ['Justina find_person anybody', 'Justina say done']
    elif scenario == 'person_refinement':
        nominal = ['Justina find_pose waving_1', 'Justina approach customer_1', 'Justina say done']
    elif scenario.startswith('interaction'):
        nominal = ['Justina say ask_name', 'Justina listen guest_name_1', 'Justina say done']
    elif scenario == 'drop':
        nominal = ['Justina drop demo_object_1', 'Justina say done']
    else:
        nominal = ['Justina verify_take demo_object_1', 'Justina say done']
    rclpy.init()
    node = rclpy.create_node('recovery_modules_fixture')
    raw = node.create_publisher(String, '/planning/clips/results', 10)
    status = node.create_publisher(SystemStatus, '/system/status', 10)
    reports = node.create_publisher(TaskExecutionReport, '/supervisor/supervisor_execution/result', 10)
    active = node.create_publisher(Bool, '/supervisor/supervisor_plan/active', 10)
    active_pending, report_pending, requests = [], [], []
    instructions, phrases = [], []
    counts = {}
    cancellations = 0
    confirmation_answers = iter(['No, that is wrong.', 'Yes, that is correct.'])

    def plan(req, res):
        raw.publish(String(data='\n'.join('Paso %d: %s' % (i + 1, step) for i, step in enumerate(nominal))))
        res.success = True
        return res

    def talk(req, res):
        phrases.append(req.text)
        res.success = True
        return res

    def listen(req, res):
        res.success = True
        res.recognized_speech = (
            next(confirmation_answers) if scenario == 'interaction_confirm_no' else
            'Yes, no.' if scenario == 'interaction_confirm_ambiguous' else
            'Yes, that is correct.')
        return res

    def head(req, res):
        res.success = True
        return res

    def cancel(req, res):
        nonlocal cancellations
        cancellations += 1
        res.return_code = CancelGoal.Response.ERROR_NONE
        if scenario in ['cancel_terminal', 'cancel_completed', 'cancel_incomplete']:
            goal = GoalInfo(); goal.goal_id.uuid = [1] * 16
            res.goals_canceling = [goal]
        return res

    def get_result(req, res):
        res.status = (GoalStatus.STATUS_CANCELED if scenario == 'cancel_terminal' else
                      GoalStatus.STATUS_SUCCEEDED if scenario == 'cancel_completed' else GoalStatus.STATUS_EXECUTING)
        return res

    services = [node.create_service(Trigger, '/planning/clips/start_planning', plan),
                node.create_service(Talk, '/hri/talker/talk', talk),
                node.create_service(Listen, '/hri/listener/listen', listen),
                node.create_service(MoveHead, '/hardware/head/move_head', head)]
    if scenario != 'cancel_unavailable':
        services.append(node.create_service(CancelGoal, '/executor/execute_task/_action/cancel_goal', cancel))
    if scenario in ['cancel_terminal', 'cancel_completed', 'cancel_incomplete']:
        services.append(node.create_service(ExecuteTask.Impl.GetResultService, '/executor/execute_task/_action/get_result', get_result))
    client = node.create_client(Trigger, '/request_instruction')
    trigger_sub = node.create_subscription(Empty, '/supervisor/supervisor_manager/allow_next_instruction',
        lambda msg: requests.append(client.call_async(Trigger.Request())), 10)
    active_sub = node.create_subscription(Bool, '/fixture/plan_active',
        lambda msg: active_pending.append((time.monotonic() + 0.15, msg)), 10)
    handles = [open(root / name, 'w') for name in ['plan.log', 'manager.log']]
    processes = [subprocess.Popen([executable('supervisor_plan', 'supervisor_plan_node'), '--ros-args',
                 '-r', '/supervisor/supervisor_plan/active:=/fixture/plan_active'], stdout=handles[0], stderr=subprocess.STDOUT),
                 subprocess.Popen([executable('supervisor_manager', 'supervisor_manager_node'), '--ros-args',
                 '-p', 'continue_on_recovery_failure:=' + str(debug).lower(),
                 '-p', 'recovery_modules_config:=' + str(config_path),
                 '-p', 'ask_data_confirmation_enabled:=' + str('confirm_' in scenario).lower()], stdout=handles[1], stderr=subprocess.STDOUT)]
    if 'confirm_' in scenario:
        handles.append(open(root / 'entities.log', 'w'))
        processes.append(subprocess.Popen(
            [executable('semantic_parser', 'entity_extractor_node'), '--ros-args',
             '-r', '__ns:=/planning/semantic_parser'],
            stdout=handles[-1], stderr=subprocess.STDOUT))
    started = time.monotonic()
    try:
        while time.monotonic() - started < 25:
            now = time.monotonic()
            status.publish(SystemStatus(hardware=True, hri=True, navigation=True))
            rclpy.spin_once(node, timeout_sec=0.02)
            for future in list(requests):
                if not future.done():
                    continue
                requests.remove(future)
                response = future.result()
                assert response.success, response.message
                instructions.append(response.message)
                _, action, argument = response.message.split(maxsplit=2)
                key = action + ' ' + argument
                counts[key] = counts.get(key, 0) + 1
                attempt = counts[key]
                report = TaskExecutionReport(action=action, argument=argument, approved=True,
                    status=TaskExecutionReport.SUCCESS, failure_reason='none', worker_result='success')
                failure, reason = None, 'unspecified'
                if action == 'check':
                    if scenario in ['door_timeout', 'cancel_unavailable', 'cancel_terminal', 'cancel_completed', 'cancel_incomplete']:
                        continue  # Deliberately absent result: coordinator must cancel before retrying.
                    if scenario in ['door_opens', 'door_unrelated'] and attempt == 3:
                        report.worker_result = 'door_open:door'
                    elif scenario == 'door_service_failure' or (scenario in ['door_opens', 'door_unrelated'] and attempt == 1):
                        failure, reason = 'check_service_failed:door', 'check_service_failed'
                    else:
                        failure, reason = 'door_closed:door', 'door_closed'
                elif action == 'listen' and argument == 'follow':
                    if scenario == 'follow_no_auth' or (scenario == 'follow_no_stop' and attempt > 1):
                        failure, reason = 'no_input', 'no_input'
                    else:
                        report.worker_result = 'continue' if attempt == 1 else 'stop'
                elif scenario == 'follow_enable_failure' and action == 'follow' and argument == 'enable':
                    failure, reason = 'tracking_toggle_failed', 'navigation_failed'
                elif action == 'find_object' and (scenario == 'perception_exhausted' or attempt < 3):
                    failure, reason = 'demo_object_not_found', 'object_not_found'
                elif action == 'find_person' and attempt < 3:
                    failure = 'person_not_found:anybody'
                elif action == 'listen' and argument == 'guest_name_1':
                    if scenario == 'interaction_exhausted' or (attempt == 1 and 'confirm_' not in scenario):
                        failure, reason = 'listen_failed', 'no_input'
                    else:
                        report.worker_result = 'people[1].name=john'
                elif action == 'drop' and attempt == 1:
                    failure = 'pose_out_of_range'
                elif action == 'verify_take' and attempt == 1:
                    failure = 'take_uncertain:demo_object_1'
                if failure:
                    report.approved = False
                    report.status = TaskExecutionReport.FAILURE
                    report.failure_reason, report.worker_result = reason, failure
                if scenario == 'door_unrelated' and action == 'check':
                    alien = TaskExecutionReport(action='say', argument='unrelated', approved=False,
                        status=TaskExecutionReport.FAILURE, failure_reason='no_input', worker_result='unrelated')
                    report_pending.append((now + 0.2, alien))
                    report_pending.append((now + 0.4, report))
                else:
                    report_pending.append((now + 0.03, report))
            for due, msg in list(active_pending):
                if now >= due:
                    active.publish(msg)
                    active_pending.remove((due, msg))
            for due, report in list(report_pending):
                if now >= due:
                    reports.publish(report)
                    report_pending.remove((due, report))
            if processes[1].poll() is not None:
                break
        assert processes[1].poll() == 0, f'manager did not complete: {root}'
        failed = scenario in ['door_closed', 'door_service_failure', 'door_timeout',
                              'follow_no_stop', 'follow_no_auth', 'follow_destination_timeout', 'follow_enable_failure',
                              'perception_exhausted', 'interaction_exhausted', 'interaction_confirm_ambiguous', 'cancel_unavailable', 'cancel_terminal', 'cancel_completed', 'cancel_incomplete']
        should_continue = not failed or (debug and scenario not in ['cancel_unavailable', 'cancel_incomplete'])
        assert ('Justina say done' in instructions) == should_continue, instructions
        if scenario == 'door_unrelated':
            assert 'ignoring unexpected result' in (root / 'manager.log').read_text()
        if scenario.startswith('door'):
            assert counts.get('check door') == 3, instructions
        if scenario in ['door_timeout', 'cancel_terminal', 'cancel_completed']:
            assert cancellations == 3, cancellations
        if scenario in ['cancel_unavailable', 'cancel_incomplete']:
            assert counts.get('check door') == 1, instructions
        if scenario == 'follow_no_auth':
            assert counts.get('listen follow') == 3 and 'Justina follow enable' not in instructions, instructions
        if scenario in ['follow', 'follow_no_stop', 'follow_destination_timeout', 'follow_enable_failure']:
            assert 'Justina follow disable' in instructions, instructions
            if should_continue:
                assert instructions.index('Justina follow disable') < instructions.index('Justina say done')
        if scenario.startswith('perception'):
            assert counts.get('find_object demo_object_1') == 3, instructions
        if scenario == 'person_refinement':
            assert counts.get('find_pose waving_1') == 2, instructions
            assert counts.get('approach_partial customer_1') == 1, instructions
        if scenario.startswith('interaction'):
            attempts = 1 if scenario == 'interaction_confirm_yes' else 2
            assert counts.get('say ask_name') == attempts and counts.get('listen guest_name_1') == attempts, instructions
        if scenario == 'drop':
            assert 'Justina approach safe_drop_1' in instructions and counts.get('drop demo_object_1') == 2, instructions
        if scenario == 'verify_take':
            assert counts.get('verify_take demo_object_1') == 2, instructions
        print(f'PASS {scenario} debug={debug}: {instructions}')
        print(f'Logs: {root}')
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
