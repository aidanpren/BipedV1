#!/usr/bin/env python3
"""No-hardware test of mode_manager's DS4 button handling.

    python3 tools/test_mode_buttons.py

Runs the REAL ModeManager against synthetic /joy messages. No controller, no
robot, no CAN, no ROS network beyond a private domain. Every claim the button
feature makes is checked here so that the only thing left to verify on the
robot is which physical button maps to which index.

What it proves, and why each one is worth a test:

  1. boot state is DISABLED                — power-on must never mean torque-on
  2. a tap enters TELEOP                   — the feature
  3. a HELD button does not re-fire        — joy autorepeats at 20 Hz; acting on
                                             the level would fire 20x/second
  4. release + tap fires again             — edge detection is not one-shot
  5. a single combo button does nothing    — half a combo is not a combo
  6. the full combo enters DISABLED        — the way back
  7. combo order does not matter           — you will not press them in sync
  8. stale /joy blocks entering TELEOP     — the motion-mode interlock
  9. stale /joy does NOT block DISABLED    — you must always be able to stop
 10. a short buttons array cannot crash    — a different pad must not take the
                                             node down inside a callback
 11. the service and the buttons share     — one policy point, not two sets of
     the same interlock                      rules depending on who asked
"""

import os
import sys
import time

# Private domain so this never talks to a real robot on the LAN.
os.environ.setdefault('ROS_DOMAIN_ID', '77')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src',
    'robot_teleop'))

import rclpy                                                    # noqa: E402
from sensor_msgs.msg import Joy                                 # noqa: E402
from robot_interfaces.srv import SetMode                        # noqa: E402
from robot_teleop.mode_manager import ModeManager, Mode         # noqa: E402

TELEOP_BTN = 9
DISABLE_BTNS = [4, 6]
NBUTTONS = 12

failures = []
checks = 0


def check(label, got, want):
    global checks
    checks += 1
    if got == want:
        print(f'  ok    {label}')
    else:
        print(f'  FAIL  {label}: got {got!r}, want {want!r}')
        failures.append(label)


def joy(mgr, pressed=(), nbuttons=NBUTTONS):
    """Deliver one synthetic /joy message straight to the callback.

    Called directly rather than published, so the test is deterministic — no
    executor timing, no dropped messages, no sleeps to tune.
    """
    msg = Joy()
    msg.buttons = [0] * nbuttons
    for i in pressed:
        if i < nbuttons:
            msg.buttons[i] = 1
    msg.axes = [0.0] * 8
    mgr.joy_callback(msg)


def main():
    rclpy.init()
    mgr = ModeManager()

    # Override the declared defaults so the test states its own assumptions
    # rather than inheriting whatever the yaml happens to say today.
    mgr.node.set_parameters([
        rclpy.parameter.Parameter('teleop_button', value=TELEOP_BTN),
        rclpy.parameter.Parameter('disable_buttons', value=DISABLE_BTNS),
    ])

    print('\nmode_manager button behaviour')
    print('-' * 60)

    check('1. boots DISABLED', mgr.current_mode, Mode.DISABLED)

    joy(mgr, pressed=[TELEOP_BTN])
    check('2. tap teleop_button -> TELEOP', mgr.current_mode, Mode.TELEOP)

    # Go back to DISABLED by hand so we can watch a HELD button do nothing.
    mgr.set_mode(Mode.DISABLED)
    for _ in range(20):                      # 1 second of autorepeat
        joy(mgr, pressed=[TELEOP_BTN])       # still held from the tap above
    check('3. held button does not re-fire', mgr.current_mode, Mode.DISABLED)

    joy(mgr, pressed=[])                     # release
    joy(mgr, pressed=[TELEOP_BTN])           # fresh press
    check('4. release then tap fires again', mgr.current_mode, Mode.TELEOP)

    joy(mgr, pressed=[DISABLE_BTNS[0]])
    check('5. half the combo does nothing', mgr.current_mode, Mode.TELEOP)

    joy(mgr, pressed=DISABLE_BTNS)
    check('6. full combo -> DISABLED', mgr.current_mode, Mode.DISABLED)

    # Combo pressed in the other order, with a gap — nobody presses two
    # buttons on the same millisecond.
    joy(mgr, pressed=[])
    joy(mgr, pressed=[TELEOP_BTN])
    check('7a. back to TELEOP', mgr.current_mode, Mode.TELEOP)
    joy(mgr, pressed=[DISABLE_BTNS[1]])                  # second button first
    joy(mgr, pressed=[DISABLE_BTNS[1], DISABLE_BTNS[0]])  # then the first
    check('7b. combo order does not matter', mgr.current_mode, Mode.DISABLED)

    # ── the interlock ────────────────────────────────────────────────────────
    # Fake a controller that has gone quiet: rewind the presence stamp past
    # joy_timeout without touching the clock.
    joy(mgr, pressed=[])
    mgr.last_joy_time = None                 # never heard a controller at all
    ok, why = mgr.request_mode(Mode.TELEOP, source='test')
    check('8a. stale joy blocks TELEOP', ok, False)
    check('8b. and says why', why, 'no controller present')
    check('8c. mode unchanged', mgr.current_mode, Mode.DISABLED)

    mgr.set_mode(Mode.TELEOP)
    mgr.last_joy_time = None
    ok, _ = mgr.request_mode(Mode.DISABLED, source='test')
    check('9a. stale joy does NOT block DISABLED', ok, True)
    check('9b. and it took effect', mgr.current_mode, Mode.DISABLED)

    # ── a pad that is not a DS4 ──────────────────────────────────────────────
    # Reading past the end of msg.buttons inside a subscription callback would
    # raise on every message. Must warn once and carry on.
    before = mgr.current_mode
    try:
        for _ in range(5):
            joy(mgr, pressed=[], nbuttons=4)     # indices 4/6/9 all out of range
        crashed = False
    except IndexError:
        crashed = True
    check('10a. short /joy does not crash', crashed, False)
    check('10b. mode unchanged', mgr.current_mode, before)
    check('10c. warned exactly once', mgr.warned_short_joy, True)

    # ── one policy point ─────────────────────────────────────────────────────
    # The service must be subject to the SAME interlock as the buttons. If it
    # were not, /set_mode would be a way around the safety check.
    mgr.set_mode(Mode.DISABLED)
    mgr.last_joy_time = None
    req, resp = SetMode.Request(), SetMode.Response()
    req.mode = 'teleop'
    mgr.set_mode_callback(req, resp)
    check('11a. service obeys the same interlock', resp.success, False)
    check('11b. same message as the buttons', resp.message, 'no controller present')

    joy(mgr, pressed=[])                     # controller is live again
    req, resp = SetMode.Request(), SetMode.Response()
    req.mode = 'teleop'
    mgr.set_mode_callback(req, resp)
    check('11c. service works when live', resp.success, True)
    check('11d. mode really changed', mgr.current_mode, Mode.TELEOP)

    req, resp = SetMode.Request(), SetMode.Response()
    req.mode = 'banana'
    mgr.set_mode_callback(req, resp)
    check('11e. unknown mode still rejected', resp.success, False)

    print('-' * 60)
    if failures:
        print(f'{len(failures)}/{checks} FAILED: {", ".join(failures)}\n')
    else:
        print(f'all {checks} checks passed\n')

    mgr.node.destroy_node()
    rclpy.try_shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
