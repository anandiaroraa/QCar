import rospy
from geometry_msgs.msg import Twist #for giving velocity commands I wanna add
import sys, termios, tty #lib for keyboard

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings) #enter the normal mode 
    return key

def teleop():
    rospy.init_node('car1_teleop') #node
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #pub

    speed = 0.1 #m/s
    turn = 0.5 #rad/s

    print("w, s, a, d for moving and q to quit")

    while not rospy.is_shutdown():
        key = get_key()
        twist = Twist()

        if key == 'w':
            twist.linear.x = speed
        elif key == 's':
            twist.linear.x = -speed
        elif key == 'a':
            twist.angular.z = turn
        elif key == 'd':
            twist.angular.z = -turn
        elif key == 'q':
            break

        pub.publish(twist)

if __name__ == '__main__':
    teleop()