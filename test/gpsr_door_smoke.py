#!/usr/bin/env python3
"""Verify the GPSR startup door gate with simulated hardware, without a plan."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from known_locations_interfaces.srv import LocationsGet
from std_srvs.srv import Trigger
from supervisor_interfaces.msg import SystemStatus
from talker_interfaces.srv import Talk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('case', choices=['opens', 'closed', 'unavailable'])
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    root = Path(tempfile.mkdtemp(prefix='gpsr-door-'))
    os.environ['ROS_LOG_DIR'] = str(root)
    config = yaml.safe_load((Path(get_package_share_directory('supervisor_manager')) /
                             'config/recovery_modules.yaml').read_text())
    config['door']['retry_delay_sec'] = 0.05
    config['door']['execution_timeout_sec'] = 0.3
    path = root / 'recovery.yaml'
    path.write_text(yaml.safe_dump(config))
    rclpy.init()
    node = rclpy.create_node('gpsr_door_fixture')
    status = node.create_publisher(SystemStatus, '/system/status', 10)
    events = []

    def check(req, res):
        events.append('check')
        res.success = args.case == 'opens' and events.count('check') == 2
        res.message = 'Door is open' if res.success else 'Door is closed'
        return res

    def navigate(req, res):
        events.append('navigate:' + req.location_name)
        res.success = False  # Stop the fixture before acquiring an actual task.
        return res

    def talk(req, res):
        res.success = True
        return res

    services = [node.create_service(Talk, '/hri/talker/talk', talk),
                node.create_service(LocationsGet, '/nav_controller/go_to_location', navigate)]
    if args.case != 'unavailable':
        services.append(node.create_service(Trigger, '/hardware/door_detector/check', check))
    executable = Path(get_package_prefix('supervisor_manager')) / 'lib/supervisor_manager/supervisor_manager_node'
    with open(root / 'manager.log', 'w') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-p', 'natural_language_planning:=true',
            '-p', 'continue_on_recovery_failure:=' + str(args.debug).lower(),
            '-p', 'recovery_modules_config:=' + str(path)], stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and process.poll() is None:
                status.publish(SystemStatus(hardware=True, hri=True, navigation=True))
                rclpy.spin_once(node, timeout_sec=0.02)
                if any(e.startswith('navigate:') for e in events):
                    break
            navigated = 'navigate:instruction_point' in events
            assert navigated == (args.case == 'opens' or args.debug), (events, root)
            if args.case != 'unavailable':
                checks = 2 if args.case == 'opens' else 3
                assert events[:checks] == ['check'] * checks, (events, root)
            else:
                assert '[DoorSequence] check=3/3' in (root / 'manager.log').read_text()
            if not navigated:
                assert process.poll() == 0, root
            print(f'PASS case={args.case} debug={args.debug}: {events}; logs={root}')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
