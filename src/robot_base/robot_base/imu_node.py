"""BNO085 (BNO08x) -> sensor_msgs/Imu on /imu.

balance_controller consumes exactly two things from this topic:
    msg.orientation           -> pitch = asin(2*(w*y - z*x))
    msg.angular_velocity.y    -> pitch rate
so the mounting orientation of the sensor MUST be corrected here, or pitch is
measured about the wrong axis and the robot falls over confidently.

Set `mount_rpy` to the rotation that takes SENSOR axes into ROBOT axes
(ROS REP-103: x forward, y left, z up). Default is identity — verify it on the
bench by tilting the robot nose-down and checking pitch goes the expected way
BEFORE ever enabling torque.

driver:=fake generates synthetic motion so the whole pipeline can be tested
with no sensor attached:

    ros2 run robot_base imu_node --ros-args -p driver:=fake

Hardware note: the BNO08x uses I2C clock stretching, which the Raspberry Pi's
hardware I2C handles poorly. Prefer driver:=uart or driver:=spi on the Pi.
"""
import math
import time

import rclpy
from sensor_msgs.msg import Imu


# ── small quaternion helpers (w, x, y, z) — avoids a scipy dependency ────────
def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class FakeDriver:
    """Synthetic IMU: a slow pitch oscillation with a consistent gyro rate."""

    AMPLITUDE = 0.15   # rad
    RATE = 0.5         # Hz

    def __init__(self):
        self.t0 = time.time()

    def read(self):
        t = time.time() - self.t0
        w = 2 * math.pi * self.RATE
        pitch = self.AMPLITUDE * math.sin(w * t)
        pitch_rate = self.AMPLITUDE * w * math.cos(w * t)
        # pure pitch rotation about y
        quat = quat_from_rpy(0.0, pitch, 0.0)
        gyro = (0.0, pitch_rate, 0.0)
        accel = (-9.81 * math.sin(pitch), 0.0, 9.81 * math.cos(pitch))
        return quat, gyro, accel


class BNO08xDriver:
    """Real sensor. Imported lazily so the node runs with no hardware/libs."""

    def __init__(self, node, interface, port, baud, i2c_address):
        import adafruit_bno08x
        from adafruit_bno08x import (
            BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE,
            BNO_REPORT_ROTATION_VECTOR,
        )
        self._reports = (BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE,
                         BNO_REPORT_ROTATION_VECTOR)

        if interface == 'uart':
            import serial
            from adafruit_bno08x.uart import BNO08X_UART
            self.bno = BNO08X_UART(serial.Serial(port, baud, timeout=0.5))
        elif interface == 'spi':
            import board, busio, digitalio
            from adafruit_bno08x.spi import BNO08X_SPI
            spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
            cs = digitalio.DigitalInOut(board.D5)
            intr = digitalio.DigitalInOut(board.D6)
            reset = digitalio.DigitalInOut(board.D9)
            self.bno = BNO08X_SPI(spi, cs, intr, reset)
        else:  # i2c — works, but clock stretching makes it flaky on a Pi
            import board, busio
            from adafruit_bno08x.i2c import BNO08X_I2C
            self.bno = BNO08X_I2C(busio.I2C(board.SCL, board.SDA),
                                  address=i2c_address)
            node.get_logger().warn(
                'BNO08x over I2C on a Raspberry Pi is unreliable (clock '
                'stretching). Prefer driver:=uart or driver:=spi.')

        for r in self._reports:
            self.bno.enable_feature(r)

    def read(self):
        # library returns quaternion as (x, y, z, w); we use (w, x, y, z)
        qx, qy, qz, qw = self.bno.quaternion
        return (qw, qx, qy, qz), self.bno.gyro, self.bno.acceleration


class ImuNode:
    def __init__(self):
        self.node = rclpy.create_node('imu_node')

        self.node.declare_parameter('driver', 'fake')     # fake | uart | spi | i2c
        self.node.declare_parameter('port', '/dev/serial0')
        self.node.declare_parameter('baud', 3000000)
        self.node.declare_parameter('i2c_address', 0x4A)
        self.node.declare_parameter('frame_id', 'imu_link')
        self.node.declare_parameter('publish_rate', 100.0)
        # rotation taking SENSOR axes -> ROBOT axes (radians). VERIFY ON BENCH.
        self.node.declare_parameter('mount_rpy', [0.0, 0.0, 0.0])

        def p(name):
            return self.node.get_parameter(name).value

        self.frame_id = p('frame_id')
        rpy = list(p('mount_rpy'))
        self.q_mount = quat_from_rpy(*rpy)
        self.identity_mount = all(abs(v) < 1e-12 for v in rpy)

        kind = p('driver')
        if kind == 'fake':
            self.driver = FakeDriver()
            self.node.get_logger().warn('using FAKE IMU data — synthetic motion')
        else:
            self.driver = BNO08xDriver(self.node, kind, p('port'), p('baud'),
                                       p('i2c_address'))
            self.node.get_logger().info(f'BNO085 up on {kind}')

        if self.identity_mount:
            self.node.get_logger().warn(
                'mount_rpy is identity — pitch is being read straight off the '
                'sensor axes. Verify by tilting the robot before enabling torque.')

        self.pub = self.node.create_publisher(Imu, 'imu', 10)
        self.timer = self.node.create_timer(1.0 / p('publish_rate'), self.update)

    def update(self):
        try:
            quat, gyro, accel = self.driver.read()
        except Exception as exc:                      # a dropped sample is not fatal
            self.node.get_logger().warn(f'IMU read failed: {exc}',
                                        throttle_duration_sec=2.0)
            return

        # Correct for how the sensor is bolted to the chassis.
        #
        # SIDE MATTERS. q_mount takes SENSOR axes -> ROBOT axes, a BODY-frame
        # relationship, so the attitude correction is a RIGHT multiply by its
        # conjugate:  q_world<-robot = q_world<-sensor  (x)  conj(q_robot<-sensor)
        # Left-multiplying instead rotates in the WORLD frame. That can still
        # be tuned to make a level robot read zero, so it survives calibration
        # — but it does not correct the SIGN of pitch as the robot tilts, which
        # is the one thing this correction exists to get right. (Found on
        # hardware 2026-07-30: a backwards-mounted board read nose-down as
        # negative pitch and no mount_rpy value could fix it.)
        #
        # Vectors are different: gyro/accel are plain rotations by q_mount.
        if not self.identity_mount:
            quat = quat_mul(quat, quat_conj(self.q_mount))
            gyro = quat_rotate(self.q_mount, gyro)
            accel = quat_rotate(self.q_mount, accel)

        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = quat
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = accel
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
