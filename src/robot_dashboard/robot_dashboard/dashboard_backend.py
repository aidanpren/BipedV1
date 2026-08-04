"""The half of the dashboard that a web page cannot do by itself.

A browser can already talk to ROS: rosbridge lets the page subscribe to topics,
call services, and set parameters. Everything the old dashboard did needed
nothing else. Two new things do:

  1. RUNNING COMMANDS ON THE PI — `git pull`, `colcon build`, restarting the
     stack. A page has no shell. This node has one.
  2. WRITING FILES — persisting a live tuning session back into real.yaml so it
     survives a reboot. A page has no filesystem.

Both are privileged in the ordinary sense, so both are gated. Read the SAFETY
section below before changing anything here.


SAFETY: WHY `build` AND `restart` REFUSE UNLESS THE MODE IS DISABLED
--------------------------------------------------------------------
Restarting biped-stack.service kills balance_controller AND odrive_bridge.
odrive_bridge's shutdown path calls disarm(), which puts both wheel axes into
IDLE. On a robot that is balancing, that is not a restart — it is dropping the
robot on the floor from standing height, initiated by a button on a phone.

`colcon build` is the same hazard wearing a lab coat. It rewrites install/ under
a running process, and with --symlink-install a Python node that lazily imports
a module mid-build can import a half-written file.

So both check the mode first, and an UNKNOWN mode counts as unsafe. If no /mode
message has ever arrived, mode_manager is not running, which means nothing is
supervising the robot's state and this is exactly the wrong moment to guess.
`require_disabled: false` exists as a documented escape hatch for bench work
with the motors unpowered — it is not for use on a live robot.

`git pull` is NOT gated. It only rewrites files on disk; the running nodes
already hold their code in memory and are unaffected until they restart. Being
able to pull while the robot is up, then disable, then build, is a genuinely
useful sequence.


HOW THE JOB OUTPUT REACHES THE PAGE
-----------------------------------
The services START a job and return immediately. They do not block until the
build finishes — a colcon build takes minutes, and a service call that hangs for
minutes is a service call that dies to any timeout between here and the phone,
taking the output with it.

Instead:
  * /dashboard/job_log   one String per output line, published as it appears
  * /dashboard/status    small JSON at 2 Hz: mode, current job, exit code
  * /dashboard/get_log   returns the whole retained tail in one go

That combination survives a page reload mid-build, which the obvious design
does not: the reloaded page calls get_log once to prime itself and then follows
job_log for the rest. It also survives the restart_stack case, where the node
publishing the log is about to be killed by the very command it is running.
"""
import json
import os
import shutil
import subprocess
import threading
import time
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_interfaces.srv import SaveParams
from robot_dashboard import yaml_patch

# Retained output lines. A full colcon build of this workspace is a few hundred
# lines; 2000 holds one comfortably plus the failure that made you look.
LOG_LINES = 2000


def find_workspace(start):
    """Walk up from `start` looking for the colcon workspace root.

    Identified by having BOTH a src/ directory and a .git — BipedV1 is the repo
    and the workspace at the same time, and either marker alone would match
    something else on the way up. Returns None rather than guessing.

    This exists so the node works unchanged on the dev laptop
    (/home/roshub/Documents/ros2_ws/BipedV1) and on the Pi
    (/home/aidanpren/BipedV1) with no per-machine parameter. realpath() first,
    because under --symlink-install the installed module is a symlink pointing
    back into src/ and the un-resolved path leads into install/ instead.
    """
    path = os.path.realpath(start)
    for _ in range(10):
        if (os.path.isdir(os.path.join(path, 'src'))
                and os.path.exists(os.path.join(path, '.git'))):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


class Job:
    """One running subprocess and its output. At most one exists at a time."""

    def __init__(self, name, argv, cwd, timeout):
        self.name = name
        self.argv = argv
        self.cwd = cwd
        self.timeout = timeout
        self.started = time.time()
        self.finished = None
        self.exit_code = None
        self.proc = None


class DashboardBackend:
    def __init__(self):
        self.node = rclpy.create_node('dashboard_backend')

        auto_ws = find_workspace(__file__) or ''
        # BIPED_WS is what /etc/default/biped sets on the Pi, so honour it
        # first: if the operator has told systemd where the workspace is, that
        # answer beats anything inferred from a file path.
        self.node.declare_parameter('workspace', os.environ.get('BIPED_WS', auto_ws))

        # Files save_params is allowed to write. Paths are relative to the
        # workspace unless absolute. This list is also the ALLOWLIST: a node
        # whose parameters do not live in one of these files cannot be saved,
        # which keeps a stray service call from rewriting something unrelated.
        self.node.declare_parameter('config_files', [
            'src/robot_bringup/config/real.yaml',
            'src/robot_teleop/config/teleop_twist_joy.yaml',
        ])

        self.node.declare_parameter('stack_service', 'biped-stack.service')
        self.node.declare_parameter('git_remote', 'origin')
        self.node.declare_parameter('git_branch', '')     # '' = current branch
        self.node.declare_parameter('build_args', ['--symlink-install'])

        # See the SAFETY block at the top before setting this false.
        self.node.declare_parameter('require_disabled', True)

        self.node.declare_parameter('pull_timeout', 180.0)    # s
        self.node.declare_parameter('build_timeout', 1800.0)  # s

        self.ws = self.node.get_parameter('workspace').value
        self.require_disabled = self.node.get_parameter('require_disabled').value

        if not self.ws or not os.path.isdir(self.ws):
            self.node.get_logger().error(
                f'workspace {self.ws!r} does not exist. git/build/save are all '
                'disabled until the `workspace` parameter points at the colcon '
                'workspace root (the directory holding src/ and .git).')

        # ── state ───────────────────────────────────────────────────────────
        self.mode = None                 # None means "never heard from /mode"
        self.job = None
        self.job_lock = threading.Lock()
        self.log = deque(maxlen=LOG_LINES)
        self.log_rev = 0

        # ReentrantCallbackGroup + MultiThreadedExecutor is what lets
        # save_params call ANOTHER node's parameter services from inside its own
        # service callback. On the default single-threaded executor that
        # deadlocks instantly: the callback holds the only spin thread, so the
        # response it is waiting for can never be processed.
        self.cb = ReentrantCallbackGroup()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, 'mode', self._on_mode, latched)

        self.log_pub = self.node.create_publisher(String, 'dashboard/job_log', 50)
        self.status_pub = self.node.create_publisher(String, 'dashboard/status', 10)

        srv = self.node.create_service
        srv(Trigger, 'dashboard/git_status', self._svc_git_status, callback_group=self.cb)
        srv(Trigger, 'dashboard/git_pull', self._svc_git_pull, callback_group=self.cb)
        srv(Trigger, 'dashboard/build', self._svc_build, callback_group=self.cb)
        srv(Trigger, 'dashboard/restart_stack', self._svc_restart, callback_group=self.cb)
        srv(Trigger, 'dashboard/get_log', self._svc_get_log, callback_group=self.cb)
        srv(SaveParams, 'dashboard/save_params', self._svc_save_params,
            callback_group=self.cb)

        # 2 Hz unconditionally, not on-change. A page that connects or reloads
        # halfway through a build must not have to wait for the next state
        # transition to find out what is happening, and rosbridge does not
        # reliably replay a latched sample to a new WebSocket subscriber.
        self.node.create_timer(0.5, self._publish_status)

        self.node.get_logger().info(f'dashboard backend up; workspace {self.ws}')

    # ── mode gate ───────────────────────────────────────────────────────────
    def _on_mode(self, msg):
        self.mode = msg.data

    def _gate(self, action):
        """Return None if `action` may proceed, or the refusal message."""
        if not self.require_disabled:
            return None
        if self.mode is None:
            return (f'refusing to {action}: no /mode message has been received, '
                    'so mode_manager is not running and the robot state is '
                    'unknown. Start the stack, or set require_disabled:=false '
                    'for bench work with the motors unpowered.')
        if self.mode != 'disabled':
            return (f'refusing to {action}: mode is {self.mode!r}. This kills '
                    'balance_controller and idles the wheel axes, which drops a '
                    'balancing robot. Switch to DISABLED first.')
        return None

    # ── job runner ──────────────────────────────────────────────────────────
    def _append(self, line):
        self.log.append(line)
        self.log_rev += 1
        self.log_pub.publish(String(data=line))

    def _start(self, name, argv, timeout, cwd=None, env=None):
        """Launch a job in a background thread. Returns (ok, message)."""
        with self.job_lock:
            if self.job is not None and self.job.finished is None:
                return False, f'{self.job.name} is still running'
            if not self.ws or not os.path.isdir(self.ws):
                return False, f'workspace {self.ws!r} is not a directory'
            job = Job(name, argv, cwd or self.ws, timeout)
            self.job = job

        self._append(f'$ {" ".join(argv)}')
        threading.Thread(target=self._run, args=(job, env), daemon=True).start()
        return True, f'{name} started'

    def _run(self, job, env):
        merged = dict(os.environ)
        # NEVER let git stop and wait for a human. There is no terminal to type
        # a password into: a prompt would hang the job until the timeout with
        # no output explaining why. Failing immediately with git's own
        # "could not read Username" is a diagnosable error; a silent 3-minute
        # stall is not.
        merged.update({'GIT_TERMINAL_PROMPT': '0', 'GIT_ASKPASS': '/bin/true',
                       'SSH_ASKPASS': '/bin/true', 'DISPLAY': ''})
        if env:
            merged.update(env)

        try:
            job.proc = subprocess.Popen(
                job.argv, cwd=job.cwd, env=merged,
                stdout=subprocess.PIPE,
                # stderr FOLDED INTO stdout on purpose. Build errors and git
                # errors go to stderr, and they are the entire reason anyone
                # opens this panel. Two streams would also interleave
                # unpredictably in the log.
                stderr=subprocess.STDOUT,
                text=True, bufsize=1)
        except FileNotFoundError as exc:
            self._append(f'!! {exc}')
            job.exit_code, job.finished = 127, time.time()
            return
        except Exception as exc:                          # noqa: BLE001
            self._append(f'!! {exc}')
            job.exit_code, job.finished = 1, time.time()
            return

        deadline = time.time() + job.timeout
        try:
            for line in job.proc.stdout:
                self._append(line.rstrip('\n'))
                if time.time() > deadline:
                    self._append(f'!! timeout after {job.timeout:.0f}s — killing')
                    job.proc.kill()
                    break
            job.proc.wait(timeout=10)
        except Exception as exc:                          # noqa: BLE001
            self._append(f'!! {exc}')

        job.exit_code = job.proc.returncode
        job.finished = time.time()
        self._append(f'-- {job.name} exited {job.exit_code} '
                     f'after {job.finished - job.started:.1f}s')

    # ── services: git / build / restart ─────────────────────────────────────
    def _svc_git_status(self, request, response):
        # One shell pipeline would be shorter and would also be the one place a
        # future parameter could smuggle in a command. Three explicit argv
        # lists, no shell=True anywhere in this file.
        remote = self.node.get_parameter('git_remote').value
        ok, msg = self._start('git-status',
                              ['git', 'fetch', '--quiet', remote], 60.0)
        if ok:
            threading.Thread(target=self._git_status_tail, daemon=True).start()
        response.success, response.message = ok, msg
        return response

    def _git_status_tail(self):
        """After the fetch lands, report where HEAD sits relative to upstream."""
        for _ in range(600):
            with self.job_lock:
                if self.job is None or self.job.finished is not None:
                    break
            time.sleep(0.1)
        for argv in (['git', 'status', '--short', '--branch'],
                     ['git', 'log', '--oneline', '-5', 'HEAD..@{upstream}']):
            try:
                out = subprocess.run(argv, cwd=self.ws, capture_output=True,
                                     text=True, timeout=30)
            except Exception as exc:                      # noqa: BLE001
                self._append(f'!! {exc}')
                continue
            for line in (out.stdout + out.stderr).splitlines():
                self._append(line)
        self._append('-- incoming commits listed above (empty = up to date)')

    def _svc_git_pull(self, request, response):
        remote = self.node.get_parameter('git_remote').value
        branch = self.node.get_parameter('git_branch').value
        argv = ['git', 'pull', '--ff-only', remote]
        if branch:
            argv.append(branch)
        # --ff-only, NOT a plain pull. A plain pull on a Pi with a local edit
        # starts a MERGE, and a merge that conflicts leaves the working tree
        # half-resolved with no editor available to finish it — on the machine
        # that is supposed to be holding the robot up. --ff-only refuses
        # instead, which is recoverable over ssh.
        ok, msg = self._start('git-pull', argv,
                              self.node.get_parameter('pull_timeout').value)
        response.success, response.message = ok, msg
        return response

    def _svc_build(self, request, response):
        refusal = self._gate('build')
        if refusal:
            self._append(f'!! {refusal}')
            response.success, response.message = False, refusal
            return response
        argv = ['colcon', 'build'] + list(self.node.get_parameter('build_args').value)
        if shutil.which('colcon') is None:
            response.success = False
            response.message = ('colcon is not on PATH for this node. It is '
                                'launched from an environment that sourced the '
                                'ROS setup, so this usually means the stack was '
                                'started some other way.')
            return response
        ok, msg = self._start('colcon-build', argv,
                              self.node.get_parameter('build_timeout').value)
        response.success, response.message = ok, msg
        return response

    def _svc_restart(self, request, response):
        refusal = self._gate('restart the stack')
        if refusal:
            self._append(f'!! {refusal}')
            response.success, response.message = False, refusal
            return response

        unit = self.node.get_parameter('stack_service').value
        # --no-block IS LOAD-BEARING, not a nicety. This node is part of the
        # unit it is restarting, so systemd is about to SIGTERM the process
        # running this very command. Without --no-block, systemctl waits for the
        # restart to complete, gets killed partway through, and the outcome
        # depends on a race. With it, systemctl queues the job and exits
        # cleanly, and this node dies a moment later as intended. The page sees
        # the WebSocket drop and reconnects on its own.
        ok, msg = self._start('restart-stack',
                              ['sudo', '-n', 'systemctl', '--no-block', 'restart', unit],
                              30.0)
        if ok:
            msg = (f'restarting {unit}. The dashboard will disconnect and '
                   'reconnect in a few seconds.')
        response.success, response.message = ok, msg
        return response

    def _svc_get_log(self, request, response):
        response.success = True
        response.message = '\n'.join(self.log)
        return response

    # ── service: save live parameters back to YAML ──────────────────────────
    def _config_paths(self):
        out = []
        for entry in self.node.get_parameter('config_files').value:
            out.append(entry if os.path.isabs(entry) else os.path.join(self.ws, entry))
        return out

    def _svc_save_params(self, request, response):
        try:
            result = self._save_params(list(request.nodes), request.preview)
        except Exception as exc:                          # noqa: BLE001
            self.node.get_logger().exception('save_params failed')
            response.success, response.message, response.changed = False, str(exc), []
            return response
        response.success, response.message, response.changed = result
        return response

    def _save_params(self, node_names, preview):
        paths = [p for p in self._config_paths() if os.path.isfile(p)]
        if not paths:
            return False, f'no config files found under {self.ws}', []

        # Which top-level blocks each file owns. Derived from the files
        # themselves rather than from another parameter, so adding a node to
        # real.yaml is enough to make it saveable — there is no second list to
        # forget to update.
        owners = {}
        texts = {}
        for path in paths:
            with open(path, 'r') as handle:
                texts[path] = handle.read()
            for line in texts[path].splitlines():
                if line and not line[0].isspace() and line.rstrip().endswith(':'):
                    owners.setdefault(line.rstrip()[:-1].strip(), path)

        targets = node_names or sorted(owners)
        unknown = [n for n in targets if n not in owners]
        if unknown:
            return False, (f'no config file defines {", ".join(unknown)}. '
                           'save_params only rewrites values that already '
                           'exist in a file.'), []

        # Snapshot the LIVE values, one node at a time.
        by_file = {}
        skipped = []
        for name in targets:
            live, err = self._read_live_params(name)
            if err:
                skipped.append(f'{name}: {err}')
                continue
            by_file.setdefault(owners[name], {})[name] = live

        changed_all, missing_all = [], []
        for path, wanted in by_file.items():
            new_text, changed, missing = yaml_patch.patch(texts[path], wanted)
            changed_all += [f'{os.path.basename(path)}  {c}' for c in changed]
            # A live parameter with no line in the file is normal and not an
            # error: every node declares defaults the YAML never mentions.
            # Collected only so an unexpected one can be spotted.
            missing_all += missing
            if changed and not preview:
                # Backup then atomic replace. os.replace is atomic within a
                # filesystem, so a power cut during a save leaves either the old
                # file or the new one, never a truncated hybrid — which on this
                # robot would be a config that crashes every node at boot.
                shutil.copy2(path, path + '.bak')
                tmp = path + '.tmp'
                with open(tmp, 'w') as handle:
                    handle.write(new_text)
                os.replace(tmp, path)

        note = []
        if preview:
            note.append('PREVIEW — nothing written')
        if not changed_all:
            note.append('all values already match the files')
        else:
            note.append(f'{len(changed_all)} value(s) '
                        + ('would be written' if preview else 'written'))
            if not preview:
                note.append('a .bak was kept beside each file')
        if skipped:
            note.append('skipped ' + '; '.join(skipped))
        return True, '. '.join(note), changed_all

    def _read_live_params(self, node_name):
        """Ask a running node what it is actually using. -> (dict, error)."""
        client = AsyncParameterClient(self.node, node_name)
        if not client.wait_for_services(timeout_sec=3.0):
            return None, 'node is not running'
        try:
            listed = self._await(client.list_parameters([], 10), 5.0)
            if listed is None:
                return None, 'list_parameters timed out'
            names = [n for n in listed.result.names if n != 'use_sim_time']
            got = self._await(client.get_parameters(names), 5.0)
            if got is None:
                return None, 'get_parameters timed out'
        finally:
            # AsyncParameterClient creates four service clients. Left behind,
            # every save leaks a set into the graph and `ros2 node info` slowly
            # fills with them.
            for attr in ('list_parameters_client', 'get_parameters_client',
                         'set_parameters_client', 'describe_parameters_client'):
                handle = getattr(client, attr, None)
                if handle is not None:
                    self.node.destroy_client(handle)

        out = {}
        for name, value in zip(names, got.values):
            unpacked = self._unpack(value)
            if unpacked is not None:
                out[name] = unpacked
        return out, None

    def _await(self, future, timeout):
        """Block on a future while the executor keeps spinning in other threads.

        rclpy futures have no blocking get-with-timeout, and the usual
        spin_until_future_complete cannot be used from inside a callback — the
        executor is already spinning. Polling is the honest option here.
        """
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.01)
        return future.result()

    @staticmethod
    def _unpack(value):
        """rcl_interfaces/ParameterValue -> a plain Python value."""
        # 1 bool 2 int 3 double 4 string 5 byte[] 6 bool[] 7 int[] 8 double[] 9 string[]
        return {
            1: lambda v: v.bool_value,
            2: lambda v: v.integer_value,
            3: lambda v: v.double_value,
            4: lambda v: v.string_value,
            6: lambda v: list(v.bool_array_value),
            7: lambda v: list(v.integer_array_value),
            8: lambda v: [float(x) for x in v.double_array_value],
            9: lambda v: list(v.string_array_value),
        }.get(value.type, lambda v: None)(value)

    # ── status ──────────────────────────────────────────────────────────────
    def _publish_status(self):
        with self.job_lock:
            job = self.job
        payload = {
            'mode': self.mode,
            'workspace': self.ws,
            'require_disabled': self.require_disabled,
            'log_rev': self.log_rev,
            'job': None,
        }
        if job is not None:
            payload['job'] = {
                'name': job.name,
                'running': job.finished is None,
                'exit_code': job.exit_code,
                'elapsed': round((job.finished or time.time()) - job.started, 1),
            }
        self.status_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    backend = DashboardBackend()
    # MultiThreadedExecutor is REQUIRED, not an optimisation — see the
    # ReentrantCallbackGroup note in __init__.
    executor = MultiThreadedExecutor()
    executor.add_node(backend.node)
    try:
        executor.spin()
    # SIGTERM (how systemd stops us) arrives as ExternalShutdownException.
    # Catching it keeps a clean stop from looking like a crash in the journal.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
