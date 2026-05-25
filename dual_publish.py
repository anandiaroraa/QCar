#!/usr/bin/env python3
import argparse
import os
import threading
import time
import traceback

import numpy as np


QCAR1_CMD_TOPIC = "/qcar/mux/ackermann_cmd_mux/input/navigation"
QCAR2_CMD_TOPIC = "/qcar2/mux/ackermann_cmd_mux/input/navigation"
rospy = None
AckermannDrive = None
AckermannDriveStamped = None


def _load_ros_modules():
    global rospy, AckermannDrive, AckermannDriveStamped
    import rospy as rospy_module
    from ackermann_msgs.msg import AckermannDrive as AckermannDriveType
    from ackermann_msgs.msg import AckermannDriveStamped as AckermannDriveStampedType
    from MPC_Hardware import publish as qcar1_publish
    from MPC_Hardware.qcar_params import (
        DS as qcar1_ds,
        LENGTH as qcar1_length,
        MAX_TIME as qcar1_max_time,
        RADIUS as qcar1_radius,
        TARGET_SPEED as qcar1_target_speed,
    )
    from MPC_Hardware2 import publish as qcar2_publish
    from MPC_Hardware2.qcar_params import (
        DS as qcar2_ds,
        LENGTH as qcar2_length,
        MAX_TIME as qcar2_max_time,
        RADIUS as qcar2_radius,
        TARGET_SPEED as qcar2_target_speed,
    )

    rospy = rospy_module
    AckermannDrive = AckermannDriveType
    AckermannDriveStamped = AckermannDriveStampedType
    return {
        "qcar1_publish": qcar1_publish,
        "qcar1_ds": qcar1_ds,
        "qcar1_length": qcar1_length,
        "qcar1_max_time": qcar1_max_time,
        "qcar1_radius": qcar1_radius,
        "qcar1_target_speed": qcar1_target_speed,
        "qcar2_publish": qcar2_publish,
        "qcar2_ds": qcar2_ds,
        "qcar2_length": qcar2_length,
        "qcar2_max_time": qcar2_max_time,
        "qcar2_radius": qcar2_radius,
        "qcar2_target_speed": qcar2_target_speed,
    }


def _build_config(args, radius, ds, target_speed, max_time, length):
    return {
        "trajectory_type": args.trajectory,
        "radius": radius,
        "ds": ds,
        "target_speed": target_speed,
        "circle_direction": args.circle_direction,
        "max_time": max_time,
        "length": length,
        "enable_live_plot": args.live_plot,
        "require_pose": True,
        "start_barrier_timeout": args.start_barrier_timeout,
    }


def _save_run(results_dir, label, result):
    car_history, original_path, exec_time, goal, reference_path = result
    os.makedirs(results_dir, exist_ok=True)
    filename = os.path.join(results_dir, f"{label}_{time.time():.5f}.npz")
    np.savez_compressed(
        filename,
        car_history=np.array(car_history),
        original_path=np.array(original_path) if original_path is not None else np.array([]),
        execution_time=exec_time,
        object_goal_pose=goal,
        radius=reference_path["radius"],
        target_speed=reference_path["target_speed"],
        center_x=reference_path["center_x"],
        center_y=reference_path["center_y"],
        ds=reference_path["ds"],
        path_length=reference_path["path_length"],
        goal_progress_margin=reference_path["goal_progress_margin"],
        circle_direction=reference_path["circle_direction"],
        max_time=reference_path["max_time"],
        reference_x=reference_path["reference_x"],
        reference_y=reference_path["reference_y"],
        reference_yaw=reference_path["reference_yaw"],
        reference_curvature=reference_path["reference_curvature"],
    )
    print(f"{label}: saved {filename}")


def _stop_cars():
    pubs = [
        rospy.Publisher(QCAR1_CMD_TOPIC, AckermannDriveStamped, queue_size=1),
        rospy.Publisher(QCAR2_CMD_TOPIC, AckermannDriveStamped, queue_size=1),
    ]
    rospy.sleep(0.2)
    stop_msg = AckermannDriveStamped()
    stop_msg.header.stamp = rospy.Time.now()
    stop_msg.drive = AckermannDrive(steering_angle=0.0, speed=0.0)
    for _ in range(8):
        stop_msg.header.stamp = rospy.Time.now()
        for pub in pubs:
            pub.publish(stop_msg)
        rospy.sleep(0.05)


def _run_controller(label, runner, config, results_dir, failures):
    try:
        print(f"{label}: starting controller")
        result = runner.run_car(1, True, path_tracking_config=config)
        _save_run(results_dir, label, result)
        print(f"{label}: controller finished")
    except Exception:
        failures[label] = traceback.format_exc()
        print(f"{label}: controller failed")
        print(failures[label])
        rospy.signal_shutdown(f"{label} controller failed")


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Run QCar1 and QCar2 MPC controllers inside one ROS node."
    )
    parser.add_argument(
        "--trajectory",
        choices=("circle", "straight", "lemniscate"),
        default="lemniscate",
        help="Reference trajectory for both cars.",
    )
    parser.add_argument(
        "--circle-direction",
        choices=("cw", "ccw"),
        default="cw",
        help="Circle direction when --trajectory circle is used.",
    )
    parser.add_argument(
        "--live-plot",
        action="store_true",
        help="Enable each car's matplotlib live plotter. Off by default for shared-node runs.",
    )
    parser.add_argument(
        "--results-dir",
        default="hardware_results_dual",
        help="Directory for the two result npz files.",
    )
    parser.add_argument(
        "--start-barrier-timeout",
        type=float,
        default=60.0,
        help="Seconds each car waits at the shared start barrier.",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    modules = _load_ros_modules()
    rospy.init_node("qcar_dual_mpc", anonymous=False)

    qcar1_config = _build_config(
        args,
        radius=modules["qcar1_radius"],
        ds=modules["qcar1_ds"],
        target_speed=modules["qcar1_target_speed"],
        max_time=modules["qcar1_max_time"],
        length=modules["qcar1_length"],
    )
    qcar1_config["car_label"] = "QCar1"
    qcar2_config = _build_config(
        args,
        radius=modules["qcar2_radius"],
        ds=modules["qcar2_ds"],
        target_speed=modules["qcar2_target_speed"],
        max_time=modules["qcar2_max_time"],
        length=modules["qcar2_length"],
    )
    qcar2_config["car_label"] = "QCar2"

    start_barrier = threading.Barrier(2)
    qcar1_config["start_barrier"] = start_barrier
    qcar2_config["start_barrier"] = start_barrier

    failures = {}
    threads = [
        threading.Thread(
            target=_run_controller,
            args=("qcar1", modules["qcar1_publish"], qcar1_config, args.results_dir, failures),
        ),
        threading.Thread(
            target=_run_controller,
            args=("qcar2", modules["qcar2_publish"], qcar2_config, args.results_dir, failures),
        ),
    ]

    for thread in threads:
        thread.start()

    try:
        for thread in threads:
            while thread.is_alive() and not rospy.is_shutdown():
                thread.join(timeout=0.2)
    finally:
        _stop_cars()

    for thread in threads:
        thread.join(timeout=1.0)

    if failures:
        raise RuntimeError("One or more QCar controllers failed. See traceback above.")


if __name__ == "__main__":
    main()
