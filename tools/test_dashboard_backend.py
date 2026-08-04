"""Exercise dashboard_backend with NO robot, NO CAN and NO motors.

What it proves, in order:
  1. the services exist and answer
  2. the SAFETY GATE holds — build and restart refuse while the mode is unknown
     or teleop, and unlock only on disabled
  3. `git status` actually runs a subprocess and streams its output back
  4. save_params reads LIVE parameters off a running node and reports a real
     diff against real.yaml — in preview mode, writing nothing

Run it from the workspace root with the workspace sourced:

    source install/setup.bash
    python3 tools/test_dashboard_backend.py

It starts and stops the nodes it needs. Nothing it does can move a motor: the
only node it launches besides the backend is balance_controller, which it never
takes out of DISABLED, and there is no odrive_bridge for it to command.
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_interfaces.srv import SaveParams

PASS, FAIL = '\033[32mPASS\033[0m', '\033[31mFAIL\033[0m'
results = []


def check(name, ok, detail=''):
    results.append(ok)
    print(f'  [{PASS if ok else FAIL}] {name}' + (f'  — {detail}' if detail else ''))


class Harness(Node):
    def __init__(self):
        super().__init__('dashboard_backend_test')
        # TRANSIENT_LOCAL, matching mode_manager. A VOLATILE publisher would not
        # be received by the backend's latched subscription at all, and the test
        # would "fail" for a reason that has nothing to do with the gate.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/mode', latched)
        self.status = None
        self.log_lines = []
        self.create_subscription(String, '/dashboard/status', self._status, 10)
        self.create_subscription(String, '/dashboard/job_log', self._log, 50)

    def _status(self, msg):
        self.status = json.loads(msg.data)

    def _log(self, msg):
        self.log_lines.append(msg.data)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, name, srv_type=Trigger, request=None, timeout=25.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=8.0):
            return None
        future = client.call_async(request or srv_type.Request())
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def set_mode(self, mode):
        self.mode_pub.publish(String(data=mode))
        self.spin(1.0)


def spawn(*argv):
    """Launch a node in its OWN PROCESS GROUP so it can be killed for real.

    `ros2 run` is a WRAPPER: it execs nothing, it forks the node as a child. A
    plain terminate() on the wrapper therefore reaps the wrapper and orphans
    the node, which keeps running, keeps its name on the graph, and collides
    with the next test run. Two nodes called /balance_controller do not error —
    `ros2 param set` reaches one of them and the reader reaches the other, so
    values appear to be set and then read back unchanged. This is the same
    class of fault as the stale-Gazebo-server trap: everything looks healthy
    and nothing works. start_new_session + killpg is the fix.
    """
    return subprocess.Popen(argv, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)


def reap(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def main():
    if 'ROS_DOMAIN_ID' not in os.environ:
        print('NOTE: set ROS_DOMAIN_ID to something unused (e.g. 42) if you have\n'
              '      other BipedV1 nodes running — duplicate node names make this\n'
              '      test read one node while writing to another.\n')

    procs = []
    print('starting dashboard_backend and balance_controller…')
    procs.append(spawn('ros2', 'run', 'robot_dashboard', 'dashboard_backend'))
    # A node with real parameters for save_params to snapshot. It publishes
    # torque commands, but with no odrive_bridge listening and the mode left at
    # DISABLED, nothing anywhere can act on them.
    procs.append(spawn('ros2', 'run', 'robot_base', 'balance_controller',
                       '--ros-args', '--params-file',
                       'src/robot_bringup/config/real.yaml'))

    rclpy.init()
    node = Harness()
    try:
        node.spin(5.0)

        print('\n1. the backend is up and publishing status')
        check('/dashboard/status arrives', node.status is not None)
        if node.status:
            check('workspace was auto-discovered',
                  bool(node.status.get('workspace')), node.status.get('workspace'))

        print('\n2. the safety gate')
        # No /mode has been published yet, so the backend must treat the robot
        # state as unknown and refuse.
        res = node.call('/dashboard/build')
        check('build REFUSED while the mode is unknown',
              res is not None and not res.success,
              (res.message[:70] + '…') if res else 'no response')

        node.set_mode('teleop')
        res = node.call('/dashboard/restart_stack')
        check('restart REFUSED while the mode is teleop',
              res is not None and not res.success,
              (res.message[:70] + '…') if res else 'no response')

        node.set_mode('disabled')
        node.spin(1.0)
        check('gate reports disabled', node.status and node.status.get('mode') == 'disabled')

        print('\n3. a real subprocess runs and streams its output')
        before = len(node.log_lines)
        res = node.call('/dashboard/git_status')
        check('git_status accepted', res is not None and res.success,
              res.message if res else '')
        node.spin(12.0)
        check('output streamed on /dashboard/job_log',
              len(node.log_lines) > before, f'{len(node.log_lines) - before} lines')
        check('the command line itself was logged',
              any(line.startswith('$ git') for line in node.log_lines))
        res = node.call('/dashboard/get_log')
        check('get_log returns the retained tail',
              res is not None and 'git' in (res.message or ''))

        print('\n4. save_params reads LIVE values and diffs them (preview only)')
        req = SaveParams.Request()
        req.nodes = ['balance_controller']
        req.preview = True
        res = node.call('/dashboard/save_params', SaveParams, req, timeout=30.0)
        check('save_params answered', res is not None and res.success,
              res.message if res else 'no response')
        if res:
            check('preview wrote nothing', 'PREVIEW' in res.message)
            # Freshly launched from real.yaml, so nothing should have drifted.
            check('a clean node shows no differences', len(res.changed) == 0,
                  '; '.join(res.changed[:3]))

    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in procs:
            reap(p)

    print(f'\n{sum(results)}/{len(results)} checks passed')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
