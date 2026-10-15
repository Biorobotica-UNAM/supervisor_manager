#!/usr/bin/env python3
"""Real supervisor startup with a simulated disconnected GPSR response."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from known_locations_interfaces.srv import LocationsGet
from listener_interfaces.srv import Listen
from semantic_parser_interfaces.srv import ExtractEntity, Parser
from std_srvs.srv import Trigger
from supervisor_interfaces.msg import SystemStatus
from talker_interfaces.srv import Talk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--structured', action='store_true')
    parser.add_argument('--tts-failure', action='store_true')
    args = parser.parse_args()
    root = Path(tempfile.mkdtemp(prefix='gpsr-unavailable-'))
    os.environ['ROS_LOG_DIR'] = str(root)
    rclpy.init()
    node = rclpy.create_node('gpsr_unavailable_fixture')
    status = node.create_publisher(SystemStatus, '/system/status', 10)
    phrases, heard, parsed = [], [], []
    answers = iter(['Continue.', 'Go to the exit.', 'Yes.'])

    def talk(req, res):
        phrases.append(req.text)
        res.success = not (args.tts_failure and 'GPSR server' in req.text)
        return res

    def listen(req, res):
        res.recognized_speech = next(answers, '')
        heard.append(res.recognized_speech)
        res.success = True
        return res

    def extract(req, res):
        assert req.category.lower() == 'confirm'
        res.value = 'yes'
        return res

    def semantic(req, res):
        parsed.append(req.command)
        payload = {'status': 'aborted', 'input': req.command,
                   'reason': '[Errno 111] Connection refused'}
        if args.structured:
            payload.update(reason='timed out', error_code='gpsr_unavailable')
        res.response = json.dumps(payload)
        return res

    def success(req, res):
        res.success = True
        return res

    services = [node.create_service(Talk, '/hri/talker/talk', talk),
                node.create_service(Listen, '/hri/listener/listen', listen),
                node.create_service(ExtractEntity, '/planning/semantic_parser/extract_entity', extract),
                node.create_service(Parser, '/planning/semantic_parser/semantic_parser', semantic),
                node.create_service(Trigger, '/hardware/door_detector/check', success),
                node.create_service(LocationsGet, '/nav_controller/go_to_location', success)]
    executable = Path(get_package_prefix('supervisor_manager')) / 'lib/supervisor_manager/supervisor_manager_node'
    with open(root / 'manager.log', 'w') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-p', 'natural_language_planning:=true',
            '-p', 'continue_on_recovery_failure:=' + str(args.debug).lower()], stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and process.poll() is None:
                status.publish(SystemStatus(hardware=True, hri=True, navigation=True))
                rclpy.spin_once(node, timeout_sec=0.02)
            assert process.poll() == 0, root
            assert len(parsed) == 1 and len(heard) == 3, (parsed, heard, root)
            assert any('GPSR server' in phrase for phrase in phrases), (phrases, root)
            assert not any('repeat' in phrase.lower() for phrase in phrases), (phrases, root)
            text = (root / 'manager.log').read_text()
            assert '[Pipeline][FATAL]' in text and 'SM_FINISH' in text, root
            assert '[PlanDispatch]' not in text, root
            print(f'PASS debug={args.debug} structured={args.structured} tts_failure={args.tts_failure}; logs={root}')
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
