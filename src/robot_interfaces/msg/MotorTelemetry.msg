# Electrical telemetry for every ODrive axis on the CAN bus.
#
# Published by robot_base/odrive_telemetry, which is a READ-ONLY node: it sends
# nothing but RTR (remote-transmission-request) frames, which ask an ODrive to
# report. It cannot command torque, state, or limits. That property is what
# makes it safe to run inside the balance stack.
#
# PARALLEL ARRAYS, the same pattern sensor_msgs/JointState uses: index i of
# every array below describes the same axis. Find your axis by searching
# `name`, then index into the rest — never assume a fixed ordering, because
# the ordering comes from a parameter and can be reconfigured.

std_msgs/Header header

# ── per axis ────────────────────────────────────────────────────────────────
string[] name             # 'right_wheel' | 'right_hip' | 'left_wheel' | 'left_hip'
uint8[] node_id           # CAN node id; arbitration_id = (node_id << 5) | cmd

# `online` is driven by the ODrive's own cyclic HEARTBEAT frame, not by whether
# we got an answer to a request. A powered-but-faulted axis still heartbeats,
# so online=true with a non-zero axis_error is a meaningful, common state.
bool[] online
uint8[] axis_state        # ODrive AxisState: 1 IDLE, 8 CLOSED_LOOP_CONTROL
uint32[] axis_error       # 0 == healthy; anything else is an ODrive error code

# Iq is the torque-producing phase current, in MOTOR-shaft amps. This is the
# "amps per motor" figure — it is what the winding actually carries, and what
# heats it up.
float32[] iq_setpoint     # A, commanded
float32[] iq_measured     # A, measured

# iq_measured * torque_constant * gear_ratio. Reported at the OUTPUT shaft so
# it is directly comparable to balance_controller's max_torque, which is also
# an output-shaft number.
float32[] torque_est      # Nm at the output shaft

# Every axis measures the SAME physical DC bus, so the voltages should agree to
# within their own noise. They disagreeing is itself a finding — a bad crimp or
# an undersized wire shows up as one axis reading low under load.
float32[] bus_voltage     # V
float32[] bus_current     # A drawn by THIS axis

# ── pack rollup, computed from the axes that reported ───────────────────────
# voltage is the MEAN (one bus, so this is an average of repeated measurements)
# and current is the SUM (each axis draws its own share of the total).
float32 pack_voltage      # V
float32 pack_current      # A
float32 pack_power        # W
uint8 axes_reporting      # how many axes fed the rollup; 0 means it is meaningless
