#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Define key mappings for movement
key_mapping = {'w': [0, 1], 's': [0, -1], 'a': [1, 0], 'd': [-1, 0]}

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtle_controller')

    turtles = ['/turtle1/cmd_vel', '/turtle2/cmd_vel', '/turtle3/cmd_vel', '/turtle4/cmd_vel']
    publishers = [rospy.Publisher(turtle, Twist, queue_size=1) for turtle in turtles]
    
    rospy.loginfo("Turtle controller node started.")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in key_mapping:
                twist = Twist()
                twist.linear.x = key_mapping[key][1]
                twist.angular.z = key_mapping[key][0]
                
                # Publish to all turtles
                for pub in publishers:
                    pub.publish(twist)
            elif key == '\x03':  # Ctrl-C to exit
                rospy.loginfo("Ctrl-C pressed. Exiting.")
                break
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Shutting down turtle controller.")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)