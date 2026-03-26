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
from .mpcspeed_steercontrol import State, calc_ref_trajectory, iterative_linear_mpc_control, calc_nearest_index, calc_speed_profile, smooth_yaw

from qcar_params import MAX_SPEED, MIN_SPEED, MAX_STEER, MAX_DSTEER, MAX_ACCEL, DT, WB

from geometry_msgs.msg import(
    PoseStamped,
)
#from nav_msgs.msg import Path
#import rospy

REACHED_GOAL = 8

class _Pose():
    def __init__(self):
        self.reset()
    def reset(self): 
        self.x = 0
        self.y = 0
        self.theta = 0
        self.quat = [0, 0, 0, 1]
        self.worldx = 0
        self.worldy = 0
        #added pose received flag + speed estimate 
        self.received = False
        self.v = 0.0
        self._last_x = None
        self._last_y = None
        self._last_t = None

class Data():
    def __init__(self):
        self.car1 = _Pose()
    
    def update_car1_pose(self, msg):
        pose = msg.pose.position
        orientation = msg.pose.orientation
        self.car1.quat = [orientation.w, orientation.x,orientation.y,orientation.z]
        self.car1.theta = quat2euler([orientation.w, orientation.x,orientation.y,orientation.z])
        self.car1.x = pose.x
        self.car1.y = pose.y
        #added mark received + estimate speed
        self.car1.received = True
        now = time.time()
        if self.car1._last_t is not None:
            dt = max(now - self.car1._last_t, 1e-3)
            self.car1.v = float(math.hypot(self.car1.x - self.car1._last_x,
                                           self.car1.y - self.car1._last_y) / dt)
        self.car1._last_x = self.car1.x
        self.car1._last_y = self.car1.y
        self.car1._last_t = now

#changed the func run_carpool_simulation to run_car
def run_car(test_case, at_pushing_pose=True, path_tracking_config=None):
    #sim_env = PushingAmongObstaclesEnv(test_case=test_case)
    #rospy.init_node("qcar_ros", anonymous=True)
    if not rospy.core.is_initialized():
        rospy.init_node("qcar_ros", anonymous=True)
    # obs = sim_env.set_init_states()
    data = Data()
    get_car1_pose = rospy.Subscriber("/natnet_ros/qcar/pose", PoseStamped, data.update_car1_pose)
    rospy.sleep(1)
    
    give_command1 = rospy.Publisher("/qcar/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

    car1_history = []

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
        if data.car1.received:
            print("Received all pose data!")
            break
        rate.sleep()
    else:
        print("WARNING: Timeout waiting for pose data!")

    #car1_quat = [data.car1.x, data.car1.y, data.car1.quat[0], data.car1.quat[1], data.car1.quat[2], data.car1.quat[3]]
    

    #car1_theta = np.array([car1_quat[0], car1_quat[1], (np.pi - R.from_quat(car1_quat[2:6]).as_euler('xyz', degrees=False)[0]) % (2 * np.pi)])
    

    #car1_start_pose = np.array([car1_theta[0], car1_theta[1], car1_theta[2]])
    

    print("Initial pose of car1:", data.car1.x, data.car1.y, data.car1.theta)
    
    #object_goal_pose = sim_env.object_goal_pose
    #object_goal_pose = np.array([0.5, 0.5])  # Placeholder goal pose; replace with sim_env.object_goal_pose when available
    #print("Object goal pose:", object_goal_pose)
    object_goal_pose = None
    #rate = Rate(1 / config.dt)

    start_time = time.time()
    max_time = 250
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
        
    #added my mpc
    cfg = path_tracking_config or {}
    radius = float(cfg.get("radius", 1.5))
    dl = float(cfg.get("ds", 0.05))
    clockwise = bool(cfg.get("clockwise", False))
    center_x = float(cfg.get("center_x", 0.0))
    center_y = float(cfg.get("center_y", 0.0))
    target_speed = float(cfg.get("target_speed", 0.6))

    # Fallback circle waypoints (keeps shared.py light)
    circumference = 2.0 * math.pi * radius
    n_points = max(50, int(circumference / dl) + 1)
    theta = np.linspace(0.0, 2.0 * math.pi, n_points, endpoint=False)

    cx = (center_x + radius * np.cos(theta)).tolist()
    cy = (center_y + radius * np.sin(theta)).tolist()
    cyaw = ((theta + (-math.pi / 2.0 if clockwise else math.pi / 2.0) + math.pi) % (2.0 * math.pi) - math.pi).tolist()
    ck = (np.full(n_points, (-1.0 / radius) if clockwise else (1.0 / radius))).tolist()

    sp = calc_speed_profile(cx, cy, cyaw, target_speed)
    cyaw = smooth_yaw(cyaw)

    start_time = time.time()
    max_time = float(cfg.get("max_time", 250.0))

    rate = rospy.Rate(int(max(1, round(1.0 / DT))))

    target_ind, _ = calc_nearest_index(
        State(x=data.car1.x, y=data.car1.y, yaw=data.car1.theta, v=data.car1.v),
        cx, cy, cyaw, 0
    )
    oa, odelta = None, None

    while not rospy.is_shutdown() and (time.time() - start_time) < max_time:
        state = State(x=data.car1.x, y=data.car1.y, yaw=data.car1.theta, v=data.car1.v)

        xref, target_ind, dref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)
        x0 = [state.x, state.y, state.v, state.yaw]

        oa, odelta, *_ = iterative_linear_mpc_control(xref, x0, dref, oa, odelta)

        if oa is None or odelta is None:
            steer1, speed1 = 0.0, 0.0
        else:
            a_cmd = float(oa[0])                  # accel output
            steer1 = float(odelta[0])             # steer output
            speed1 = float(np.clip(state.v + a_cmd * DT, MIN_SPEED, MAX_SPEED))  # accel -> speed

        steer1 = float(np.clip(steer1, -MAX_STEER, MAX_STEER))
        speed1 = float(np.clip(speed1, MIN_SPEED, MAX_SPEED))

        drive_car1 = AckermannDrive(steering_angle=steer1, speed=speed1)
        drive_msg1 = AckermannDriveStamped()
        drive_msg1.header.stamp = rospy.Time.now()
        drive_msg1.drive = drive_car1
        give_command1.publish(drive_msg1)

        car1_history.append([state.x, state.y, state.yaw, state.v, steer1, speed1, time.time()])

        rate.sleep()
    
    # ADD: Stop the robots
    stop_drive = AckermannDrive(steering_angle=0.0, speed=0.0)
    #stop_msg = AckermannDriveStamped(drive=stop_drive)
    #give_command1.publish(stop_msg)
    
    stop_msg = AckermannDriveStamped()
    stop_msg.header.stamp = rospy.Time.now()
    stop_msg.drive = stop_drive
    for _ in range(5):
        give_command1.publish(stop_msg)
        rospy.sleep(0.05)

    # ADD: Calculate execution time and get original path
    execution_time = time.time() - start_time
    #original_path = state_machine.object_plan if hasattr(state_machine, 'object_plan') else None
    original_path = None 
    # ADD: Print summary
    if time.time() - start_time > max_time:
        print("Hardware experiment timed out.")
    else:
        print(f"Hardware experiment completed in {execution_time:.2f} seconds")
    
    # ADD: Return collected data
    return car1_history, original_path, execution_time, object_goal_pose

if __name__ == "__main__":
    import os    
    test_cases = [1]  # Your hardware test case
    num_runs = 1
    
    # Create directory for results
    for test_case in test_cases:
        results_dir = f'hardware_results_test{test_case}'
        os.makedirs(results_dir, exist_ok=True)
        try:
            car1_hist, orig_path, exec_time, goal = run_car(test_case, True, path_tracking_config={
                    "radius": 1.5,
                    "ds": 0.05,
                    "target_speed": 0.6,
                    "clockwise": False,
                    "max_time": 60.0,
                },)
            
            # Save this run immediately
            np.savez_compressed(
                os.path.join(results_dir, f'run_{0:03d}_t_{time.time():.5f}.npz'),
                car1_history=np.array(car1_hist),
                original_path=np.array(orig_path) if orig_path is not None else np.array([]),
                execution_time=exec_time,
                object_goal_pose=goal,
                run_number=0
            )
        except Exception as e:
            print(f"  ✗ Run failed with error: {e}")
        
        # print(f"\n{'='*50}")
        # print(f"All results saved to {results_dir}/")
        # print(f"{'='*50}")