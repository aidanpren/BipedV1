"""Right-stick -> leg height teleop (RATE control).

teleop_twist_joy only produces a Twist on cmd_vel; it cannot drive the legs.
This node maps one joystick axis to /leg_position_cmd (Float64, in motor turns) —
the SAME topic the dashboard slider uses and that both sim_leg_bridge and the
real leg_controller consume. So the stick drives sim and hardware identically.

RATE, not absolute: a spring-return stick snaps back to centre when released, so
it cannot hold an absolute height. Instead the stick deflection is a SPEED —
hold up to raise, release (centre) to HOLD where you are. The node integrates
the axis each tick into a running command.

    subscribes  joy               sensor_msgs/Joy
    publishes   leg_position_cmd  Float64  (turns; 0 = retracted .. ~0.19 = extended)

NOTE it shares /leg_position_cmd with the dashboard slider — last writer wins, so
don't drive both at once (the cmd_vel twist_mux equivalent for legs is future
work).
"""
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class LegJoy:
    def __init__(self):
        self.node = rclpy.create_node('leg_joy')

        self.node.declare_parameter('axis', 4)           # right-stick vertical
        self.node.declare_parameter('invert', False)     # flip if up LOWERS the leg
        self.node.declare_parameter('rate', 0.2)         # turns/s at full deflection
        self.node.declare_parameter('deadzone', 0.1)     # ignore small stick noise
        self.node.declare_parameter('min_cmd', 0.0)      # retracted hard stop
        self.node.declare_parameter('max_cmd', 0.19)     # ~extended stop; map folds past ~0.194
        self.node.declare_parameter('publish_rate', 50.0)
        # deadman: -1 = always active; set e.g. 7 to require the drive enable button
        self.node.declare_parameter('enable_button', -1)

        def p(name):
            return self.node.get_parameter(name).value

        self.axis = p('axis')
        self.invert = p('invert')
        self.rate = p('rate')
        self.deadzone = p('deadzone')
        self.min_cmd, self.max_cmd = p('min_cmd'), p('max_cmd')
        self.enable_button = p('enable_button')
        self.dt = 1.0 / p('publish_rate')

        self.leg_cmd = self.min_cmd     # start retracted
        self.joy = None

        self.pub = self.node.create_publisher(Float64, 'leg_position_cmd', 10)
        self.sub = self.node.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.timer = self.node.create_timer(self.dt, self.update)

    def joy_cb(self, msg):
        self.joy = msg

    def update(self):
        if self.joy is None:
            return

        # deadman gate: only INTEGRATE while the enable button is held (or always
        # if enable_button is -1). We still publish the held value either way.
        active = True
        if self.enable_button >= 0:
            active = (self.enable_button < len(self.joy.buttons)
                      and bool(self.joy.buttons[self.enable_button]))

        if active and self.axis < len(self.joy.axes):
            stick = self.joy.axes[self.axis]
            if self.invert:
                stick = -stick
            if abs(stick) < self.deadzone: stick = 0.0
            self.leg_cmd += stick * self.rate * self.dt
            self.leg_cmd = max(self.min_cmd, min(self.max_cmd, self.leg_cmd))

        self.pub.publish(Float64(data=float(self.leg_cmd)))


def main(args=None):
    rclpy.init(args=args)
    node = LegJoy()
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
