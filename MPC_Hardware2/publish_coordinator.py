#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

ready_count = [0]

def ready_cb(msg):
    if msg.data:
        ready_count[0] += 1
        print(f"Car ready: {ready_count[0]}/2")

rospy.init_node("qcar_coordinator")
rospy.Subscriber("/qcar/ready", Bool, ready_cb)
go_pub = rospy.Publisher("/qcar/go", Bool, queue_size=1)
rospy.sleep(1.0)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if ready_count[0] >= 2:
        print("All cars ready — GO!")
        rospy.sleep(0.1)  # let both scripts reach their wait loop
        for _ in range(10):  # publish repeatedly so both catch it
            go_pub.publish(Bool(data=True))
            rospy.sleep(0.05)
        break
    rate.sleep()