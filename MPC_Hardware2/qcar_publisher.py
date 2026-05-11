#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

#added for number of qcar subscribers expected.
NUM_CARS = 2

def main():
    rospy.init_node("pc_qcar_publisher")

    pub = rospy.Publisher(
        "/qcar/mux/ackermann_cmd_mux/input/navigation",
        AckermannDriveStamped,
        queue_size=10
    )

    # rate = rospy.Rate(10)  # 10 Hz

    # rospy.loginfo("Publishing to QCar...")
    # msg = AckermannDriveStamped()
    # while not rospy.is_shutdown():
    #     msg.drive.speed = 0.08     # forward speed
    #     msg.drive.steering_angle = 0.0  # straight
    #     pub.publish(msg)
    #     rate.sleep()
#added for waiting for subscribers to connect before publishing commands.
    rospy.loginfo("Waiting for %d subscribers to connect...", NUM_CARS)

    # Block here until both car nodes have subscribed
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        count = pub.get_num_connections()
        rospy.loginfo("Subscribers connected: %d / %d", count, NUM_CARS)
        if count >= NUM_CARS:
            break
        rate.sleep()

    rospy.loginfo("All cars connected — starting NOW")

    msg = AckermannDriveStamped()
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.drive.speed = 0.08
        msg.drive.steering_angle = 0.0
        pub.publish(msg)
        rate.sleep()
if __name__ == "__main__":
    main()