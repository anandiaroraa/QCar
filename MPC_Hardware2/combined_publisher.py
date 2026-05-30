import pdb
import time
import math
#from ..environments.hardware_environment import PushingAmongObstaclesEnv
#from ..utils.load_config import multi_agent_config as config
#from ..utils.rate_for_simulation import Rate
#from ..utils.angle_utils import quat2euler_single as quat2euler
from .utils import quat2euler
#from ..controllers.state_machine_controller_hardware import ControlStateMachine
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
#from scipy.spatial.transform import Rotation as R
#added goal_dist
from .mpcspeed_steercontrol import (
    State,
    calc_ref_trajectory,
    iterative_linear_mpc_control,
    calc_speed_profile,
    smooth_yaw,
    GOAL_DIS,
)

from .qcar_params import MAX_SPEED, MAX_TIME, MIN_SPEED, MAX_STEER, MAX_DSTEER, MAX_ACCEL, DT, WB, RADIUS, TARGET_SPEED, DS, LENGTH

from geometry_msgs.msg import(
    PoseStamped,
)
####added for live plotting
from .live_plotter import LivePlotter
####
#from nav_msgs.msg import Path
#import rospy

#REACHED_GOAL = 8

# class _Pose():
#     def __init__(self):
#         self.reset()
from .trajectory import get_trajectory

# from geometry_msgs.msg import(
#     PoseStamped,
# )
#from nav_msgs.msg import Path
#import rospy

#REACHED_GOAL = 8

#added null live plotter for now. If live plotting is disabled, this will be used as a no-op placeholder to avoid needing conditionals around plotter calls.
class _NullLivePlotter:
    def update(self, *args, **kwargs):
        pass

    def save(self, *args, **kwargs):
        pass

    def close(self):
        pass

class _Pose():
    def __init__(self):
        self.reset()
    def reset(self): 
        self.x = 0
        self.y = 0
        self.theta = 0.0
        self.quat = [0, 0, 0, 1]
        self.worldx = 0
        self.worldy = 0
        #added pose received flag + speed estimate 
        self.received = False
        self.v = 0.0
        self._last_x = None
        self._last_y = None
        self._last_t = None

def data2mpc(x, y, theta, v):
    pass
    return [x, y, v, theta]

def mpc2data(x, y, theta, v):
    pass
    return [x, y, theta, v]

def _pose_received(data, require_block_pose=True):
    block_ready = (not require_block_pose) or data.block.received
    return data.car1.received and data.car2.received and block_ready

def _block_history_row(block):
    return [block.x, block.y, block.theta, time.time()]

def _publish_drive(pub, steer, speed, accel):
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.drive = AckermannDrive(
        steering_angle=float(steer),
        speed=float(speed),
        acceleration=float(np.clip(accel, -MAX_ACCEL, MAX_ACCEL)),
    )
    pub.publish(drive_msg)

def _wait_for_command_subscribers(pub1, pub2, topic1, topic2, timeout_sec=50.0):
    print("Waiting for both cars to subscribe to command topics...")
    rate = rospy.Rate(20)
    timeout = rospy.Time.now() + rospy.Duration(float(timeout_sec))

    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        car1_ready = pub1.get_num_connections() > 0
        car2_ready = pub2.get_num_connections() > 0
        if car1_ready and car2_ready:
            print("Both cars are subscribed to command topics.")
            return
        rate.sleep()

    missing = []
    if pub1.get_num_connections() == 0:
        missing.append(f"car1 command subscriber ({topic1})")
    if pub2.get_num_connections() == 0:
        missing.append(f"car2 command subscriber ({topic2})")
    raise RuntimeError("Missing required " + ", ".join(missing))

class Data():
    def __init__(self):
        self.car1 = _Pose()
        self.car2 = _Pose()
        self.block = _Pose()

    def update_block_pose(self, msg):
        pose = msg.pose.position
        orientation = msg.pose.orientation
        self.block.quat = [orientation.w, orientation.x, orientation.y, orientation.z]
        # quat to euler uses x y z w
        self.block.theta = float(quat2euler([
            self.block.quat[1],
            self.block.quat[2],
            self.block.quat[3],
            self.block.quat[0],
        ])[2])
        self.block.x = pose.x
        self.block.y = pose.y
        self.block.received = True
    
    def update_car1_pose(self, msg):
        pose = msg.pose.position
        orientation = msg.pose.orientation
        self.car1.quat = [orientation.w, orientation.x,orientation.y,orientation.z]
        # quat to euler uses x y z w
        self.car1.theta = float(quat2euler([self.car1.quat[1], self.car1.quat[2], self.car1.quat[3], self.car1.quat[0]])[2])
        self.car1.x = pose.x
        self.car1.y = pose.y
        #added mark received + estimate speed
        self.car1.received = True
        now = time.time()
        if self.car1._last_t is not None:
            dt = max(now - self.car1._last_t, 1e-3)
            self.car1.v = float(math.hypot(self.car1.x - self.car1._last_x,
                                           self.car1.y - self.car1._last_y) / dt)
            self.car1.v = float(np.clip(self.car1.v, 0.0, MAX_SPEED))
        self.car1._last_x = self.car1.x
        self.car1._last_y = self.car1.y
        self.car1._last_t = now
        
    def update_car2_pose(self, msg):
        pose = msg.pose.position
        orientation = msg.pose.orientation
        self.car2.quat = [orientation.w, orientation.x,orientation.y,orientation.z]
        # quat to euler uses x y z w
        self.car2.theta = float(quat2euler([self.car2.quat[1], self.car2.quat[2], self.car2.quat[3], self.car2.quat[0]])[2])
        self.car2.x = pose.x
        self.car2.y = pose.y
        #added mark received + estimate speed
        self.car2.received = True
        now = time.time()
        if self.car2._last_t is not None:
            dt = max(now - self.car2._last_t, 1e-3)
            self.car2.v = float(math.hypot(self.car2.x - self.car2._last_x,
                                           self.car2.y - self.car2._last_y) / dt)
            self.car2.v = float(np.clip(self.car2.v, 0.0, MAX_SPEED))
        self.car2._last_x = self.car2.x
        self.car2._last_y = self.car2.y
        self.car2._last_t = now
#added
def _make_reference_path(pose, cfg, label):
    trajectory_type = cfg.get("trajectory_type", "straight")  # Switch between circle, straight, or lemniscate
    target_speed = float(np.clip(float(cfg.get("target_speed", TARGET_SPEED)), 0.0, MAX_SPEED))
    radius = float(cfg.get("radius", RADIUS))
    dl = float(cfg.get("ds", DS))
    direction_cfg = cfg.get("circle_direction")
    if direction_cfg is None:
        clockwise = bool(cfg.get("clockwise", True))
    else:
        clockwise = str(direction_cfg).lower() == "cw"
    direction_sign = -1 if clockwise else 1

    if clockwise:
        center_x = pose.x + radius * math.sin(pose.theta)
        center_y = pose.y - radius * math.cos(pose.theta)
    else:
        center_x = pose.x - radius * math.sin(pose.theta)
        center_y = pose.y + radius * math.cos(pose.theta)

    print(f"{label} start: ({pose.x:.3f}, {pose.y:.3f}), theta: {pose.theta:.3f}")
    print(f"{label} target speed: {target_speed:.3f} m/s")

    if trajectory_type == "circle":
        cx, cy, cyaw, ck, _ = get_trajectory(
            "circle",
            radius=radius,
            ds=dl,
            center_x=center_x,
            center_y=center_y,
            direction_sign=direction_sign,
        )
        min_dist = float("inf")
        start_idx = 0
        for i in range(len(cx)):
            d = math.hypot(cx[i] - pose.x, cy[i] - pose.y)
            if d < min_dist:
                min_dist = d
                start_idx = i

        cx = np.roll(cx, -start_idx).tolist()
        cy = np.roll(cy, -start_idx).tolist()
        cyaw = np.roll(cyaw, -start_idx).tolist()
        ck = np.roll(ck, -start_idx).tolist()
        cyaw = smooth_yaw(cyaw)
        cx.append(cx[0])
        cy.append(cy[0])
        cyaw.append(cyaw[0] + direction_sign * 2.0 * math.pi)
        ck.append(ck[0])
        cyaw = smooth_yaw(cyaw)
    elif trajectory_type == "lemniscate":
        lemniscate_scale = float(cfg.get("scale", RADIUS))
        lemniscate_laps = float(cfg.get("laps", 1.0))
        start_angle = cfg.get("start_angle", pose.theta + 3.0 * math.pi / 4.0)
        print(
            f"{label} lemniscate: scale={lemniscate_scale:.1f}m, "
            f"laps={lemniscate_laps:.1f}, start_angle={start_angle:.3f}rad"
        )
        center_x = float(cfg.get("center_x", 0.0))
        center_y = float(cfg.get("center_y", 0.0))
        cx, cy, cyaw, ck, _ = get_trajectory(
            "lemniscate",
            scale=lemniscate_scale,
            ds=dl,
            center_x=center_x,
            center_y=center_y,
            start_angle=start_angle,
            laps=lemniscate_laps,
        )
        cx.append(cx[0])
        cy.append(cy[0])
        cyaw.append(cyaw[0] + 2.0 * math.pi)
        ck.append(ck[0])
        cyaw = smooth_yaw(cyaw)
    else:
        straight_length = float(cfg.get("length", LENGTH))
        start_angle = float(cfg.get("straight_angle", pose.theta))
        print(f"{label} straight line: length={straight_length:.1f}m, angle={start_angle:.3f}rad")
        cx, cy, cyaw, ck, _ = get_trajectory(
            "straight",
            length=straight_length,
            ds=dl,
            start_x=pose.x,
            start_y=pose.y,
            angle=start_angle,
        )

    sp = calc_speed_profile(cx, cy, cyaw, target_speed=target_speed)
    path_s = [0.0]
    for i in range(1, len(cx)):
        path_s.append(path_s[-1] + math.hypot(cx[i] - cx[i - 1], cy[i] - cy[i - 1]))

    return {
        "cx": cx,
        "cy": cy,
        "cyaw": cyaw,
        "ck": ck,
        "sp": sp,
        "path_s": path_s,
        "path_length": path_s[-1],
        "center_x": center_x,
        "center_y": center_y,
        "radius": radius,
        "target_speed": target_speed,
        "ds": dl,
        "circle_direction": "cw" if clockwise else "ccw",
    }
#added
def _publish_reference(ref_pose_pub, xref):
    ref_pose_msg = PoseStamped()
    ref_pose_msg.header.stamp = rospy.Time.now()
    ref_pose_msg.header.frame_id = "map"
    ref_pose_msg.pose.position.x = float(xref[0, 0])
    ref_pose_msg.pose.position.y = float(xref[1, 0])
    ref_pose_msg.pose.position.z = 0.0
    ref_pose_msg.pose.orientation.x = 0.0
    ref_pose_msg.pose.orientation.y = 0.0
    ref_pose_msg.pose.orientation.z = math.sin(float(xref[3, 0]) * 0.5)
    ref_pose_msg.pose.orientation.w = math.cos(float(xref[3, 0]) * 0.5)
    ref_pose_pub.publish(ref_pose_msg)
#added
def _mpc_action_for_pose(pose, ref, target_ind, oa, odelta, label, end_progress_margin):
    state = State(
        x=pose.x,
        y=pose.y,
        yaw=pose.theta,
        v=float(np.clip(pose.v, MIN_SPEED, MAX_SPEED)),
    )
    xref, target_ind, dref = calc_ref_trajectory(
        state,
        ref["cx"],
        ref["cy"],
        ref["cyaw"],
        ref["ck"],
        ref["sp"],
        ref["ds"],
        target_ind,
    )
    path_progress = ref["path_s"][min(target_ind, len(ref["path_s"]) - 1)]
    path_remaining = ref["path_length"] - path_progress
    print(
        f"{label}: target_ind={target_ind}, progress={path_progress:.3f}m, "
        f"remaining={path_remaining:.3f}m"
    )

    if target_ind >= len(ref["cx"]) - 1 or path_remaining <= end_progress_margin:
        return np.array([0.0, 0.0, 0.0, 0.0]), target_ind, oa, odelta, state, xref, None, None, True

    oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
        xref,
        [state.x, state.y, state.v, state.yaw],
        dref,
        oa,
        odelta,
    )
    if oa is None or odelta is None:
        ox, oy = None, None
        steer, speed, mpc_accel = 0.0, 0.0, 0.0
    else:
        steer = float(odelta[0])
        mpc_accel = float(np.clip(float(oa[0]), -MAX_ACCEL, MAX_ACCEL))
        speed = float(np.clip(state.v + mpc_accel * DT, MIN_SPEED, MAX_SPEED))

    steer = float(np.clip(steer, -MAX_STEER, MAX_STEER))
    if abs(speed) > 1e-6:
        speed = float(np.clip(speed, MIN_SPEED, MAX_SPEED))
    if speed <= 1e-6:
        accel_cmd = 0.0
    elif mpc_accel < -1e-6:
        accel_cmd = -MAX_ACCEL
    else:
        accel_cmd = MAX_ACCEL
    return np.array([steer, speed, accel_cmd, mpc_accel]), target_ind, oa, odelta, state, xref, ox, oy, False
#added
def _execute_pushing(action1, action2, idx1, idx2, sync_index_gap=2, slow_factor=0.5):
    if idx1 > idx2 + sync_index_gap:
        action1 = action1.copy()
        action1[1] *= slow_factor
        action1[2] = -MAX_ACCEL
    elif idx2 > idx1 + sync_index_gap:
        action2 = action2.copy()
        action2[1] *= slow_factor
        action2[2] = -MAX_ACCEL
    return np.concatenate((action1, action2))

#changed the func name run_carpool_simulation to run_car
def run_car(test_case, at_pushing_pose=True, path_tracking_config=None):
    #sim_env = PushingAmongObstaclesEnv(test_case=test_case)
    #rospy.init_node("qcar_ros", anonymous=True)
    if not rospy.core.is_initialized():
        rospy.init_node("qcar_ros", anonymous=True)
    # obs = sim_env.set_init_states()
    cfg = path_tracking_config or {}
    car1_pose_topic = cfg.get("car1_pose_topic", "/natnet_ros/RigidBody1/pose")
    car2_pose_topic = cfg.get("car2_pose_topic", "/natnet_ros/RigidBody2/pose")
    block_pose_topic = cfg.get("block_pose_topic", "/natnet_ros/RigidBody3/pose")
    car1_cmd_topic = cfg.get("car1_cmd_topic", "/qcar/mux/ackermann_cmd_mux/input/navigation")
    car2_cmd_topic = cfg.get("car2_cmd_topic", "/qcar2/mux/ackermann_cmd_mux/input/navigation")
    car1_ref_topic = cfg.get("car1_ref_topic", "/mpc/qcar1/reference_pose")
    car2_ref_topic = cfg.get("car2_ref_topic", "/mpc/qcar2/reference_pose")
    require_block_pose = bool(cfg.get("require_block_pose", True))

    data = Data()
    get_car1_pose = rospy.Subscriber(car1_pose_topic, PoseStamped, data.update_car1_pose)
    get_car2_pose = rospy.Subscriber(car2_pose_topic, PoseStamped, data.update_car2_pose)
    get_block_pose = rospy.Subscriber(block_pose_topic, PoseStamped, data.update_block_pose)
    rospy.sleep(1)
    
    give_command1 = rospy.Publisher(car1_cmd_topic, AckermannDriveStamped, queue_size=1)
    give_command2 = rospy.Publisher(car2_cmd_topic, AckermannDriveStamped, queue_size=1)
    ref_pose_pub1 = rospy.Publisher(car1_ref_topic, PoseStamped, queue_size=1)
    ref_pose_pub2 = rospy.Publisher(car2_ref_topic, PoseStamped, queue_size=1)
    command_subscriber_timeout = float(cfg.get("command_subscriber_timeout", 10.0))
    _wait_for_command_subscribers(
        give_command1,
        give_command2,
        car1_cmd_topic,
        car2_cmd_topic,
        command_subscriber_timeout,
    )

    car1_history, car2_history, block_history = [], [], []

    # collected_data = []
    # processed_data = np.array([])
    # reset_counter = 0
    
    # published_pose = False
    # backed_up = False
    # display_status_message = []
    # trial_start_time = time.time()
    # trial_max_time = 60.0
    # trial_curr_time = 0.0
    # rate = rospy.Rate(1/config.dt)
    # filenames = []
    # max_time = 1800
    # obs_t = []
    
    # # sim_loops = DT / config.dt
    print("Waiting for pose data...")
    rate = rospy.Rate(10)  # 10 Hz
    timeout = rospy.Time.now() + rospy.Duration(10.0)  # 10 second timeout
    
    while rospy.Time.now() < timeout:
        # Check if we've received valid data (not default values)
        #if (data.car1.quat != [0, 0, 0, 1]):
        if _pose_received(data, require_block_pose):
            print("Received all pose data!")
            break
        rate.sleep()
    else:
        print("WARNING: Timeout waiting for pose data!")
        if not data.car1.received or not data.car2.received:
            missing = []
            if not data.car1.received:
                missing.append(f"car1 ({car1_pose_topic})")
            if not data.car2.received:
                missing.append(f"car2 ({car2_pose_topic})")
            raise RuntimeError("Missing required pose data for " + ", ".join(missing))
        if require_block_pose and not data.block.received:
            raise RuntimeError(f"Missing required block pose data ({block_pose_topic})")
    #car1_start_pose = np.array([car1_theta[0], car1_theta[1], car1_theta[2]])
    

    print("Initial pose of car1:", data.car1.x, data.car1.y, data.car1.theta)
    print("Initial pose of car2:", data.car2.x, data.car2.y, data.car2.theta)
    if data.block.received:
        print("Initial pose of block:", data.block.x, data.block.y, data.block.theta)
    
    #object_goal_pose = sim_env.object_goal_pose
    #object_goal_pose = np.array([0.5, 0.5])  # Placeholder goal pose; replace with sim_env.object_goal_pose when available
    #print("Object goal pose:", object_goal_pose)
    object_goal_pose = cfg.get("object_goal_pose")
    #rate = Rate(1 / config.dt)

    #start_time = time.time()
    #max_time = 250
    #state_machine = ControlStateMachine(sim_env, objecta!")

    #car1_quat = [data.car1.x, data.car1.y, data.car1.quat[0], data.car1.quat[1], data.car1.quat[2], data.car1.quat[3]]
    

    #car1_theta = np.array([car1_quat[0], car1_quat[1], (np.pi - R.from_quat(car1_quat[2:6]).as_euler('xyz', degrees=False)[0]) % (2 * np.pi)])
    

    #car1_start_pose = np.array([car1_theta[0], car1_theta[1], car1_theta[2]])
    

    #print("Initial pose of car1:", data.car1.x, data.car1.y, data.car1.theta)
    
    #object_goal_pose = sim_env.object_goal_pose
    #object_goal_pose = np.array([0.5, 0.5])  # Placeholder goal pose; replace with sim_env.object_goal_pose when available
    #print("Object goal pose:", object_goal_pose)
    #object_goal_pose = None
    #rate = Rate(1 / config.dt)

    #start_time = time.time()
    #max_time = 250
    #state_machine = ControlStateMachine(sim_env, object_goal_pose, at_pushing_pose, path_tracking_config)
    
    #while state_machine.state != REACHED_GOAL and time.time() - start_time < max_time:
        #state_machine.update_poses(data=data)
        #action = state_machine.execute()
        
        # ADD: Record pose history (after update_poses)
        #car1_history.append(state_machine.car1_pose.copy())
        
        #added safety clamps
        #steer1 = float(np.clip(action[0], -MAX_STEER, MAX_STEER))
        #speed1 = float(np.clip(action[1], MIN_SPEED, MAX_SPEED))

        #drive_car1 = AckermannDrive(steering_angle=steer1, speed=speed1)
        
        #drive_msg1 = AckermannDriveStamped(drive=drive_car1)

        #give_command1.publish(drive_msg1)
        
        #rate.sleep()
        
    trajectory_type = cfg.get("trajectory_type", "circle")  # "circle", "straight", or "lemniscate"
    enable_live_plot = bool(cfg.get("enable_live_plot", True))
    car1_ref = _make_reference_path(data.car1, cfg, "car1")
    car2_ref = _make_reference_path(data.car2, cfg, "car2")
    block_ref = _make_reference_path(data.block, cfg, "block") if data.block.received else None
    cx, cy, cyaw, ck, sp = (
        car1_ref["cx"],
        car1_ref["cy"],
        car1_ref["cyaw"],
        car1_ref["ck"],
        car1_ref["sp"],
    )
    center_x = car1_ref["center_x"]
    center_y = car1_ref["center_y"]
    radius = car1_ref["radius"]
    dl = car1_ref["ds"]
    path_length = car1_ref["path_length"]
    end_progress_margin = float(cfg.get("goal_progress_margin", GOAL_DIS))
    print(f"car1 reference path length: {car1_ref['path_length']:.3f}m")
    print(f"car2 reference path length: {car2_ref['path_length']:.3f}m")
    print(f"Path-complete margin: {end_progress_margin:.3f}m")

    print("Generated car1 reference trajectory points (index, xref, yref, yawref, kref, vref):")
    for i in range(len(cx)):
        print(
            f"  ref[{i:03d}] xref={float(cx[i]):.3f}, yref={float(cy[i]):.3f}, "
            f"yawref={float(cyaw[i]):.3f}, kref={float(ck[i]):.4f}, vref={float(sp[i]):.3f}"
        )

    start_time = time.time()
    max_time = float(cfg.get("max_time", 250.0))

    rate = rospy.Rate(int(max(1, round(1.0 / DT))))
    
    target_ind1 = int(cfg.get("car1_start_index", 0))
    target_ind2 = int(cfg.get("car2_start_index", 0))
    oa1, odelta1 = None, None
    oa2, odelta2 = None, None

    print(f"DEBUG: car1 pose = ({data.car1.x:.3f}, {data.car1.y:.3f}, {data.car1.theta:.3f})")
    print(f"DEBUG: car2 pose = ({data.car2.x:.3f}, {data.car2.y:.3f}, {data.car2.theta:.3f})")
    print(f"Forward published acceleration command: +{MAX_ACCEL:.3f} m/s^2")
    run_label = f"QCar MPC - {trajectory_type.capitalize()} - {time.strftime('%H:%M:%S')}"
    plotter = (
        LivePlotter(
            cx,
            cy,
            cyaw,
            title=run_label,
            secondary_reference=(car2_ref["cx"], car2_ref["cy"], car2_ref["cyaw"]),
        )
        if enable_live_plot
        else _NullLivePlotter()
    )

    while not rospy.is_shutdown() and (time.time() - start_time) < max_time:
        action1, target_ind1, oa1, odelta1, state1, xref1, ox1, oy1, done1 = _mpc_action_for_pose(
            data.car1, car1_ref, target_ind1, oa1, odelta1, "car1", end_progress_margin
        )
        action2, target_ind2, oa2, odelta2, state2, xref2, ox2, oy2, done2 = _mpc_action_for_pose(
            data.car2, car2_ref, target_ind2, oa2, odelta2, "car2", end_progress_margin
        )
        _publish_reference(ref_pose_pub1, xref1)
        _publish_reference(ref_pose_pub2, xref2)

        if done1 and done2:
            print("Both cars completed their paths.")
            car1_history.append([state1.x, state1.y, state1.yaw, state1.v, 0.0, 0.0, time.time(), 0.0, 0.0])
            car2_history.append([state2.x, state2.y, state2.yaw, state2.v, 0.0, 0.0, time.time(), 0.0, 0.0])
            block_history.append(_block_history_row(data.block))
            plotter.update(
                state_x=state1.x,
                state_y=state1.y,
                state_yaw=state1.yaw,
                state_v=state1.v,
                ox=None,
                oy=None,
                xref=xref1,
                target_ind=target_ind1,
                elapsed_time=time.time() - start_time,
                force=True,
                secondary_state=(state2.x, state2.y, state2.yaw, state2.v),
                secondary_xref=xref2,
            )
            break

        action = _execute_pushing(action1, action2, target_ind1, target_ind2)
        steer1, speed1, accel1, mpc_accel1, steer2, speed2, accel2, mpc_accel2 = [
            float(v) for v in action
        ]

        print(
            f"t={time.time() - start_time:.2f}s | "
            f"car1 x={state1.x:.3f}, y={state1.y:.3f}, yaw={state1.yaw:.3f}, v={state1.v:.3f}, "
            f"cmd=(steer={steer1:.3f}, speed={speed1:.3f}, accel={accel1:.3f}, "
            f"mpc_accel={mpc_accel1:.3f}) | "
            f"car2 x={state2.x:.3f}, y={state2.y:.3f}, yaw={state2.yaw:.3f}, v={state2.v:.3f}, "
            f"cmd=(steer={steer2:.3f}, speed={speed2:.3f}, accel={accel2:.3f}, "
            f"mpc_accel={mpc_accel2:.3f})"
        )

        _publish_drive(give_command1, steer1, speed1, accel1)
        _publish_drive(give_command2, steer2, speed2, accel2)

        car1_history.append([
            state1.x, state1.y, state1.yaw, state1.v, steer1, speed1, time.time(), accel1, mpc_accel1
        ])
        car2_history.append([
            state2.x, state2.y, state2.yaw, state2.v, steer2, speed2, time.time(), accel2, mpc_accel2
        ])
        block_history.append(_block_history_row(data.block))
        plotter.update(
            state_x      = state1.x,
            state_y      = state1.y,
            state_yaw    = state1.yaw,
            state_v      = state1.v,
            ox           = ox1,
            oy           = oy1,
            xref         = xref1,
            target_ind   = target_ind1,
            elapsed_time = time.time() - start_time,
            secondary_state = (state2.x, state2.y, state2.yaw, state2.v),
            secondary_ox    = ox2,
            secondary_oy    = oy2,
            secondary_xref  = xref2,
        )

        rate.sleep()
    
    # ADD: Stop the robots
    for _ in range(5):
        _publish_drive(give_command1, 0.0, 0.0, -MAX_ACCEL)
        _publish_drive(give_command2, 0.0, 0.0, -MAX_ACCEL)
        rospy.sleep(0.05)

    if enable_live_plot:
        save_name = f"live_plot_{trajectory_type}_{time.strftime('%Y%m%d_%H%M%S')}.gif"
        plotter.save(save_name, extra_seconds=3.0)
    plotter.close()
    execution_time = time.time() - start_time
    original_path = None 
    reference_path = {
        "reference_x": np.array(cx),
        "reference_y": np.array(cy),
        "reference_yaw": np.array(cyaw),
        "reference_curvature": np.array(ck),
        "car1_reference_x": np.array(cx),
        "car1_reference_y": np.array(cy),
        "car1_reference_yaw": np.array(cyaw),
        "car1_reference_curvature": np.array(ck),
        "center_x": center_x,
        "center_y": center_y,
        "radius": radius,
        "target_speed": car1_ref["target_speed"],
        "ds": dl,
        "path_length": path_length,
        "car1_path_length": car1_ref["path_length"],
        "car2_path_length": car2_ref["path_length"],
        "goal_progress_margin": end_progress_margin,
        "circle_direction": car1_ref["circle_direction"],
        "max_time": max_time,
        "max_accel": MAX_ACCEL,
        "trajectory_type": trajectory_type,
        "straight_angle": float(cfg.get("straight_angle", data.car1.theta)),
        "block_history": np.array(block_history),
        "car2_history": np.array(car2_history),
        "car2_reference_x": np.array(car2_ref["cx"]),
        "car2_reference_y": np.array(car2_ref["cy"]),
        "car2_reference_yaw": np.array(car2_ref["cyaw"]),
        "car2_reference_curvature": np.array(car2_ref["ck"]),
        "block_reference_x": np.array(block_ref["cx"]) if block_ref is not None else np.array([]),
        "block_reference_y": np.array(block_ref["cy"]) if block_ref is not None else np.array([]),
        "block_reference_yaw": np.array(block_ref["cyaw"]) if block_ref is not None else np.array([]),
        "block_reference_curvature": np.array(block_ref["ck"]) if block_ref is not None else np.array([]),
    }

    return car1_history, car2_history, block_history, original_path, execution_time, object_goal_pose, reference_path

if __name__ == "__main__":
    import os    
    test_cases = [1]  # Your hardware test case
    num_runs = 1
    
    # Create directory for results
    for test_case in test_cases:
        results_dir = f'hardware_results_combined_test{test_case}' #change to hardware_results_test_straight
        os.makedirs(results_dir, exist_ok=True)
        try:
            #SWITCH TRAJECTORY 
            car1_hist, car2_hist, block_hist, orig_path, exec_time, goal, reference_path = run_car(test_case, True, path_tracking_config={
                    "trajectory_type": "straight",  # ← Change to "straight" for straight line
                    # "radius": RADIUS,
                    "ds": DS,
                    "target_speed": TARGET_SPEED,
                    #switch for cw or ccw
                    "circle_direction": "cw",
                    "max_time": MAX_TIME,
                    "length": LENGTH,  # For straight trajectory
                    "straight_angle": math.pi / 2.0,  # Vertical line in the +y direction.
                })
            
            # Save this run immediately
            np.savez_compressed(
                os.path.join(results_dir, f'run_{0:03d}_t_{time.time():.5f}.npz'),
                car1_history=np.array(car1_hist),
                car2_history=np.array(car2_hist),
                car_history_columns=np.array([
                    "x", "y", "yaw", "measured_speed", "steer_cmd", "speed_cmd",
                    "timestamp", "accel_cmd", "mpc_accel",
                ]),
                block_history=np.array(block_hist),
                original_path=np.array(orig_path) if orig_path is not None else np.array([]),
                execution_time=exec_time,
                object_goal_pose=goal,
                run_number=0,
                #tune
                radius=reference_path["radius"],
                target_speed=reference_path["target_speed"],
                center_x=reference_path["center_x"],
                center_y=reference_path["center_y"],
                ds=reference_path["ds"],
                path_length=reference_path["path_length"],
                goal_progress_margin=reference_path["goal_progress_margin"],
                circle_direction=reference_path["circle_direction"],
                max_time=reference_path["max_time"],
                max_accel=reference_path["max_accel"],
                trajectory_type=reference_path["trajectory_type"],
                straight_angle=reference_path["straight_angle"],
                reference_x=reference_path["reference_x"],
                reference_y=reference_path["reference_y"],
                reference_yaw=reference_path["reference_yaw"],
                reference_curvature=reference_path["reference_curvature"],
                car1_reference_x=reference_path["car1_reference_x"],
                car1_reference_y=reference_path["car1_reference_y"],
                car1_reference_yaw=reference_path["car1_reference_yaw"],
                car1_reference_curvature=reference_path["car1_reference_curvature"],
                car2_reference_x=reference_path["car2_reference_x"],
                car2_reference_y=reference_path["car2_reference_y"],
                car2_reference_yaw=reference_path["car2_reference_yaw"],
                car2_reference_curvature=reference_path["car2_reference_curvature"],
                block_reference_x=reference_path["block_reference_x"],
                block_reference_y=reference_path["block_reference_y"],
                block_reference_yaw=reference_path["block_reference_yaw"],
                block_reference_curvature=reference_path["block_reference_curvature"],
                car1_path_length=reference_path["car1_path_length"],
                car2_path_length=reference_path["car2_path_length"],
            )
        except Exception as e:
            print(f"  ✗ Run failed with error: {e}")
        
        # print(f"\n{'='*50}")
        # print(f"All results saved to {results_dir}/")
        # print(f"{'='*50}")
