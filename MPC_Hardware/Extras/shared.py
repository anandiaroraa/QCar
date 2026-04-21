import pdb
import time
from ..environments.hardware_environment import PushingAmongObstaclesEnv
from ..utils.load_config import multi_agent_config as config
from ..utils.rate_for_simulation import Rate
from ..utils.angle_utils import quat2euler_single as quat2euler
from ..controllers.state_machine_controller_hardware import ControlStateMachine
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import(
    PoseStamped,
)
from nav_msgs.msg import Path
import rospy

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

class Data():
    def __init__(self):
        self.car1 = _Pose()
        self.car2 = _Pose()
        self.block = _Pose()

    def update_block_pose(self, msg):
        # print("Block pose received, message:", msg)
        pose = msg.pose.position
        orientation = msg.pose.orientation
        self.block.quat = [orientation.w, orientation.x,orientation.y,orientation.z]
        self.block.theta = quat2euler([orientation.w, orientation.x,orientation.y,orientation.z])
        self.block.x = pose.x
        self.block.y = pose.y

    def update_car1_pose(self, msg):
        pose = msg.pose.position
        orientation = msg.pose.orientation
        self.car1.quat = [orientation.w, orientation.x,orientation.y,orientation.z]
        self.car1.theta = quat2euler([orientation.w, orientation.x,orientation.y,orientation.z])
        self.car1.x = pose.x
        self.car1.y = pose.y

    def update_car2_pose(self, msg):
        pose = msg.pose.position
        orientation = msg.pose.orientation
        self.car2.quat = [orientation.w, orientation.x,orientation.y,orientation.z]
        self.car2.theta = quat2euler([orientation.w, orientation.x,orientation.y,orientation.z])
        self.car2.x = pose.x
        self.car2.y = pose.y

def run_carpool_simulation(test_case, at_pushing_pose=True, path_tracking_config=None):
    sim_env = PushingAmongObstaclesEnv(test_case=test_case)
    rospy.init_node("mushr_ros", anonymous=True)
    # obs = sim_env.set_init_states()
    data = Data()
    get_car1_pose = rospy.Subscriber("/natnet_ros/mushr1/pose", PoseStamped, data.update_car1_pose)
    get_car2_pose = rospy.Subscriber("/natnet_ros/mushr2/pose", PoseStamped, data.update_car2_pose)
    get_block_pose = rospy.Subscriber("/natnet_ros/block/pose", PoseStamped, data.update_block_pose)
    rospy.sleep(1)

    give_command1 = rospy.Publisher("/mushr1/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
    give_command2 = rospy.Publisher("/mushr2/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

    data = Data()
    get_car1_pose = rospy.Subscriber("/natnet_ros/mushr1/pose", PoseStamped, data.update_car1_pose)
    get_car2_pose = rospy.Subscriber("/natnet_ros/mushr2/pose", PoseStamped, data.update_car2_pose)
    get_block_pose = rospy.Subscriber("/natnet_ros/block/pose", PoseStamped, data.update_block_pose)
    rospy.sleep(1)

    give_command1 = rospy.Publisher("/mushr1/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
    give_command2 = rospy.Publisher("/mushr2/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
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
        if (data.car1.quat != [0, 0, 0, 1] and
            data.car2.quat != [0, 0, 0, 1] and
            data.block.quat != [0, 0, 0, 1]):
            print("Received all pose data!")
            break
        rate.sleep()
    else:
        print("WARNING: Timeout waiting for pose data!")

    car1_quat = [data.car1.x, data.car1.y, data.car1.quat[0], data.car1.quat[1], data.car1.quat[2], data.car1.quat[3]]
    car2_quat = [data.car2.x, data.car2.y, data.car2.quat[0], data.car2.quat[1], data.car2.quat[2], data.car2.quat[3]]
    block_quat_pose = [data.block.x, data.block.y, data.block.quat[0], data.block.quat[1], data.block.quat[2], data.block.quat[3]]


    car1_theta = np.array([car1_quat[0], car1_quat[1], (np.pi - R.from_quat(car1_quat[2:6]).as_euler('xyz', degrees=False)[0]) % (2 * np.pi)])
    car2_theta = np.array([car2_quat[0], car2_quat[1], (np.pi - R.from_quat(car2_quat[2:6]).as_euler('xyz', degrees=False)[0]) % (2 * np.pi)])
    block_theta = np.array([block_quat_pose[0], block_quat_pose[1], (np.pi - R.from_quat(block_quat_pose[2:6]).as_euler('xyz', degrees=False)[0]) % (2 * np.pi)])

    car1_start_pose = np.array([car1_theta[0], car1_theta[1], car1_theta[2]])
    car2_start_pose = np.array([car2_theta[0], car2_theta[1], car2_theta[2]])
    block_start_pose = np.array([block_theta[0], block_theta[1], block_theta[2]])

    print("Initial pose of car1:", data.car1.x, data.car1.y, car1_theta[2])
    print("Initial pose of car2:", data.car2.x, data.car2.y, car2_theta[2])
    print("Initial pose of block:", data.block.x, data.block.y, block_theta[2])
    object_goal_pose = sim_env.object_goal_pose
    print("Object goal pose:", object_goal_pose)
    rate = Rate(1 / config.dt)

    start_time = time.time()
    max_time = 250
    state_machine = ControlStateMachine(sim_env, object_goal_pose, at_pushing_pose, path_tracking_config)

    while state_machine.state != REACHED_GOAL and time.time() - start_time < max_time:
        state_machine.update_poses(data=data)
        action = state_machine.execute()

        # ADD: Record pose history (after update_poses)
        car1_history.append(state_machine.car1_pose.copy())
        car2_history.append(state_machine.car2_pose.copy())
        block_history.append(state_machine.block_pose.copy())

        drive_car1 = AckermannDrive(steering_angle=action[0], speed=action[1])
        drive_car2 = AckermannDrive(steering_angle=action[2], speed=action[3])
        drive_msg1 = AckermannDriveStamped(drive=drive_car1)
        drive_msg2 = AckermannDriveStamped(drive=drive_car2)
        give_command1.publish(drive_msg1)
        give_command2.publish(drive_msg2)
        rate.sleep()

    # ADD: Stop the robots
    stop_drive = AckermannDrive(steering_angle=0.0, speed=0.0)
    stop_msg = AckermannDriveStamped(drive=stop_drive)
    give_command1.publish(stop_msg)
    give_command2.publish(stop_msg)

    # ADD: Calculate execution time and get original path
    execution_time = time.time() - start_time
    original_path = state_machine.object_plan if hasattr(state_machine, 'object_plan') else None

    # ADD: Print summary
    if time.time() - start_time > max_time:
        print("Hardware experiment timed out.")
    else:
        print(f"Hardware experiment completed in {execution_time:.2f} seconds")

    # ADD: Return collected data
    return car1_history, car2_history, block_history, original_path, execution_time, object_goal_pose

if __name__ == "__main__":
    import os
    test_cases = [3]  # Your hardware test case
    num_runs = 1

    # Create directory for results
    for test_case in test_cases:
        results_dir = f'hardware_results_test{test_case}'
        os.makedirs(results_dir, exist_ok=True)
        try:
            car1_hist, car2_hist, block_hist, orig_path, exec_time, goal = run_carpool_simulation(test_case, True)

            # Save this run immediately
            np.savez_compressed(
                os.path.join(results_dir, f'run_{0:03d}_t_{time.time():.5f}.npz'),
                car1_history=np.array(car1_hist),
                car2_history=np.array(car2_hist),
                block_history=np.array(block_hist),
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