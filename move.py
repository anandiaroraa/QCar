#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('move')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #waiting
    rospy.sleep(1) 
    cmd = Twist()
    #qcar for straight
    cmd.linear.x = 0.1 #0.1 m/s 
    cmd.angular.z = 0.0 #no turn
    pub.publish(cmd) #vel cmd to my qcar
    rospy.sleep(4)
    #stop
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd) #stop cmd
    rospy.sleep(4)
    #back
    cmd.linear.x = -0.1 
    cmd.angular.z = 0.0
    pub.publish(cmd)
    rospy.sleep(4)
    #r
    cmd.linear.x = 0.0
    cmd.angular.z = -0.4
    pub.publish(cmd)
    rospy.sleep(4)
    #l
    cmd.linear.x = 0.0
    cmd.angular.z = 0.4
    pub.publish(cmd)
    rospy.sleep(4)
    #stop
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)    
    rospy.loginfo("Yayyyy!!!")
if __name__ == '__main__':
    main()