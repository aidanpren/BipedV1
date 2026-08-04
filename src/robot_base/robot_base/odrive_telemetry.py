"""Per-axis electrical telemetry from the ODrives: amps, volts, watts, faults.

WHY THIS IS A SEPARATE NODE AND NOT A FEW LINES IN odrive_bridge
----------------------------------------------------------------
odrive_bridge is in the balance loop. Every line added to it is a line that can
stall the thing holding the robot up, and every parameter added to it is one
more knob that can be got wrong at 3 am. This node is outside that loop
entirely: if it dies, hangs, or is never started, the robot balances exactly as
it did before. That is worth one extra process.

It also covers axes odrive_bridge cannot see. The hips (node 1 and 3) are only
touched by leg_controller, which is NOT LAUNCHED by default — so on a normal
run nothing at all is watching two of the four motors. This node watches all
four regardless of who is driving them.

THIS NODE CANNOT MOVE THE ROBOT
-------------------------------
It transmits exactly one kind of frame: RTR (remote transmission request),
which is CAN's "please tell me X". It never sends Set_Input_Torque,
Set_Axis_State, Set_Limits or anything else with a payload. Read the code and
check: every `self.bus.send` in this file goes through `client.request()`.
That is a structural guarantee, not a promise — there is no code path here that
could energise a motor even if every parameter were set to nonsense.

WHAT "BATTERY VOLTAGE" ACTUALLY IS
----------------------------------
There is no separate battery sensor on this robot and there does not need to
be. Each ODrive measures the DC bus it is powered from, and that bus IS the
battery. Get_Bus_Voltage_Current (0x017) returns both, so pack voltage and pack
current come free over a bus that is already wired. The old dashboard's
"needs Pico" was never true — the data was three CAN frames away the whole time.

Test it with no hardware at all:
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    for id in 0 1 2 3; do python3 tools/fake_odrive.py --channel vcan0 --node-id $id & done
    ros2 run robot_base odrive_telemetry --ros-args -p can_channel:=vcan0
    ros2 topic echo /motor_telemetry
"""
import threading

import can
import rclpy
from rclpy.executors import ExternalShutdownException

from robot_interfaces.msg import MotorTelemetry
from robot_base.odrive_can import (
    CMD_GET_BUS_VI, CMD_GET_IQ, ODriveClient,
)


class Axis:
    """Everything known about one motor. Plain data, written by the CAN reader
    thread and read by the publish timer, both under the node's lock."""

    def __init__(self, name, node_id):
        self.name = name
        self.node_id = node_id
        self.iq_setpoint = 0.0
        self.iq_measured = 0.0
        self.bus_voltage = 0.0
        self.bus_current = 0.0
        self.axis_state = 0
        self.axis_error = 0
        self.last_heartbeat = None   # monotonic seconds, or None if never heard
        self.last_electrical = None  # monotonic seconds of the last iq/bus reply


class ODriveTelemetry:
    def __init__(self):
        self.node = rclpy.create_node('odrive_telemetry')

        self.node.declare_parameter('can_interface', 'socketcan')
        self.node.declare_parameter('can_channel', 'vcan0')
        self.node.declare_parameter('bitrate', 500000)

        # NAMES AND IDS ARE TWO PARALLEL ARRAYS rather than one list of pairs,
        # because a ROS parameter cannot be a list of dictionaries. Their
        # lengths must match; the node refuses to start if they do not, since
        # a silent off-by-one here would label the left wheel's current as the
        # right hip's and every reading after that would be a lie.
        self.node.declare_parameter(
            'axis_names', ['right_wheel', 'right_hip', 'left_wheel', 'left_hip'])
        self.node.declare_parameter('axis_node_ids', [0, 1, 2, 3])

        # MEASURED on hardware 2026-07-31 by reading
        # odrv0.axis0.motor.config.torque_constant. It is the ONE number that
        # converts the amps this node reads into the newton-metres the rest of
        # the stack speaks. Wrong here means every torque readout is wrong by a
        # constant factor while looking entirely plausible.
        self.node.declare_parameter('torque_constant', 0.43)   # Nm per motor A
        self.node.declare_parameter('gear_ratio', 8.0)

        # DEFAULT 5 Hz, deliberately low. Each cycle puts 2 requests + 2 replies
        # on the bus per axis = 16 frames, against odrive_bridge's 200 frames/s
        # of encoder traffic. Human eyes cannot read a number changing faster
        # than this anyway, and the bus belongs to the balance loop first.
        # Raise it for current-spike hunting, then put it back.
        self.node.declare_parameter('publish_rate', 5.0)       # Hz

        # How long without a heartbeat before an axis is called offline. The
        # ODrive heartbeats at ~10 Hz, so 1 s is ten missed frames — long
        # enough that ordinary bus contention never trips it, short enough to
        # notice a motor that lost power while you are looking at the screen.
        self.node.declare_parameter('offline_timeout', 1.0)    # s

        def p(name):
            return self.node.get_parameter(name).value

        names = list(p('axis_names'))
        ids = list(p('axis_node_ids'))
        if len(names) != len(ids):
            raise ValueError(
                f'axis_names has {len(names)} entries but axis_node_ids has '
                f'{len(ids)}. They are parallel arrays and must match.')

        self.kt = p('torque_constant')
        self.gear = p('gear_ratio')
        self.offline_timeout = p('offline_timeout')

        self.bus = can.interface.Bus(interface=p('can_interface'),
                                     channel=p('can_channel'),
                                     bitrate=p('bitrate'))

        # A SECOND SOCKET on the same interface as odrive_bridge, which is
        # fine and worth understanding: SocketCAN is not a serial port handed
        # to one owner. The kernel delivers a copy of every received frame to
        # every open socket, so this node and the bridge both see all traffic
        # without either one stealing frames from the other.
        self.axes = [Axis(n, int(i)) for n, i in zip(names, ids)]
        self.clients = {a.node_id: ODriveClient(self.bus, node_id=a.node_id,
                                                gear_ratio=self.gear)
                        for a in self.axes}
        self.by_id = {a.node_id: a for a in self.axes}

        self._lock = threading.Lock()
        self.pub = self.node.create_publisher(MotorTelemetry, 'motor_telemetry', 10)

        self.running = True
        self.reader = threading.Thread(target=self.can_reader, daemon=True)
        self.reader.start()
        self.timer = self.node.create_timer(1.0 / p('publish_rate'), self.update)

        self.node.get_logger().info(
            'telemetry watching ' +
            ', '.join(f'{a.name}=node{a.node_id}' for a in self.axes))

    # ── CAN receive ─────────────────────────────────────────────────────────
    def can_reader(self):
        """Stash whatever turns up. Replies are separate events from requests,
        so there is nothing to correlate — just decode and file."""
        import time
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
            except Exception:
                break
            if msg is None:
                continue

            node_id = msg.arbitration_id >> 5
            client = self.clients.get(node_id)
            if client is None:
                continue        # some other device on the bus; not ours
            decoded = client.decode(msg)
            if decoded is None:
                continue

            kind, values = decoded
            now = time.monotonic()
            axis = self.by_id[node_id]
            with self._lock:
                if kind == 'heartbeat':
                    axis.axis_error, axis.axis_state = values
                    axis.last_heartbeat = now
                elif kind == 'iq':
                    axis.iq_setpoint, axis.iq_measured = values
                    axis.last_electrical = now
                elif kind == 'bus':
                    axis.bus_voltage, axis.bus_current = values
                    axis.last_electrical = now

    # ── timer: ask, then publish last cycle's answers ───────────────────────
    def update(self):
        import time

        # Requests go out now; the replies land in the reader thread a
        # millisecond or two later and are published on the NEXT tick. That one
        # tick of latency (200 ms at the default rate) is invisible on a
        # readout and buys us a publish path that never blocks on the bus.
        for client in self.clients.values():
            try:
                client.request(CMD_GET_IQ)
                client.request(CMD_GET_BUS_VI)
            except can.CanError:
                # A full TX queue or a bus-off adapter. Not this node's problem
                # to fix, and definitely not worth raising out of a timer and
                # killing the process over — the offline flags below will show
                # it, which is exactly what a telemetry node is for.
                self.node.get_logger().warn(
                    'CAN transmit failed; is the bus up?',
                    throttle_duration_sec=10.0)
                break

        now = time.monotonic()
        msg = MotorTelemetry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'can'

        v_sum, i_sum, reporting = 0.0, 0.0, 0

        with self._lock:
            for a in self.axes:
                online = (a.last_heartbeat is not None
                          and now - a.last_heartbeat < self.offline_timeout)
                fresh = (a.last_electrical is not None
                         and now - a.last_electrical < self.offline_timeout)

                msg.name.append(a.name)
                msg.node_id.append(a.node_id)
                msg.online.append(online)
                msg.axis_state.append(a.axis_state if online else 0)
                msg.axis_error.append(a.axis_error if online else 0)

                # STALE DATA IS PUBLISHED AS ZERO, not as the last value seen.
                # A frozen-but-plausible number is the worst possible readout:
                # it looks like a motor sitting at a steady 3 A when in fact the
                # motor is gone. Zero next to online=false is unambiguous.
                if fresh:
                    msg.iq_setpoint.append(a.iq_setpoint)
                    msg.iq_measured.append(a.iq_measured)
                    msg.torque_est.append(a.iq_measured * self.kt * self.gear)
                    msg.bus_voltage.append(a.bus_voltage)
                    msg.bus_current.append(a.bus_current)
                    v_sum += a.bus_voltage
                    i_sum += a.bus_current
                    reporting += 1
                else:
                    msg.iq_setpoint.append(0.0)
                    msg.iq_measured.append(0.0)
                    msg.torque_est.append(0.0)
                    msg.bus_voltage.append(0.0)
                    msg.bus_current.append(0.0)

        # MEAN voltage, SUM current. One physical bus measured N times gives an
        # average; N independent draws off that bus add up.
        msg.axes_reporting = reporting
        msg.pack_voltage = (v_sum / reporting) if reporting else 0.0
        msg.pack_current = i_sum
        msg.pack_power = msg.pack_voltage * msg.pack_current
        self.pub.publish(msg)

    def shutdown(self):
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    telemetry = ODriveTelemetry()
    try:
        rclpy.spin(telemetry.node)
    # ExternalShutdownException is what a SIGTERM becomes, which is how systemd
    # stops every node on this robot. Without it here, each `systemctl stop`
    # writes a five-frame traceback into the journal and the next person to
    # read the log has to work out that a clean shutdown looks like a crash.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        telemetry.shutdown()
        telemetry.bus.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
