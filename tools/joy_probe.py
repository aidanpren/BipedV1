#!/usr/bin/env python3
"""Find out which /joy index a controller button or axis actually is.

    ros2 run joy joy_node            # terminal A
    python3 tools/joy_probe.py       # terminal B, then press things

Why this exists: DS4 button numbering is NOT a fixed fact you can look up. It
differs between the SDL and joydev backends and between kernel versions, and a
wrong index in mode_manager does not raise — the button simply never fires,
which is indistinguishable from the feature being broken. This turns a
guess-and-check loop into one measurement.

It prints only CHANGES, so a pad sitting still prints nothing and a press
prints exactly one line. Analog triggers (L2/R2 on a DS4) often show up as an
axis as well as, or instead of, a button — this reports both so you can see
which you actually have.

Read-only: it subscribes to /joy and publishes nothing. Safe to run at any
time, including while the robot is balancing.
"""

import sys

import rclpy
from sensor_msgs.msg import Joy

# An analog axis is noisy at rest. Only report a move bigger than this, so a
# resting thumbstick does not scroll the screen.
AXIS_EPS = 0.30


class JoyProbe:
    def __init__(self):
        self.node = rclpy.create_node('joy_probe')
        self.prev_buttons = None
        self.prev_axes = None
        self.seen_any = False
        self.node.create_subscription(Joy, 'joy', self.callback, 10)

        self.node.get_logger().info(
            'listening on /joy — press buttons and move sticks. Ctrl-C to stop.')
        # A pad that is off, unpaired, or grabbed by another process produces a
        # completely silent /joy, which looks identical to "this tool is
        # broken". Say so rather than sitting there.
        self.timer = self.node.create_timer(3.0, self.nag)

    def nag(self):
        if not self.seen_any:
            self.node.get_logger().warn(
                'no /joy messages yet. Is joy_node running, is the pad paired '
                'and on, and does `ros2 topic hz /joy` show anything?')

    def callback(self, msg):
        if not self.seen_any:
            self.seen_any = True
            print(f'\nconnected: {len(msg.buttons)} buttons, {len(msg.axes)} axes\n')

        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)
            self.prev_axes = list(msg.axes)
            return

        for i, (now, before) in enumerate(zip(msg.buttons, self.prev_buttons)):
            if now != before:
                edge = 'PRESSED ' if now else 'released'
                print(f'  button {i:>2}  {edge}')

        for i, (now, before) in enumerate(zip(msg.axes, self.prev_axes)):
            if abs(now - before) > AXIS_EPS:
                print(f'  axis   {i:>2}  {now:+.2f}')

        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)


def main():
    rclpy.init(args=sys.argv)
    probe = JoyProbe()
    try:
        rclpy.spin(probe.node)
    except KeyboardInterrupt:
        pass
    finally:
        print('\nPut the numbers you measured into mode_manager\'s '
              'teleop_button / disable_buttons in real.yaml (and sim.yaml).')
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
