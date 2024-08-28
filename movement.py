#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

# Define key mappings for movement
key_mapping = {'w': [0, 1], 's': [0, -1], 'a': [1, 0], 'd': [-1, 0]}
attack_radius = 2.0  # Example attack radius
current_turtle = 'turtle1'  # Update this based on the current turtle
def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
def create_attack_message(turtle_name, x, y):
    """Generate an attack message."""
    return f"{turtle_name}:{x}:{y}"

def attack():
    """Publish an attack message."""
    attack_msg = create_attack_message(current_turtle, 0.0, 0.0)  # Example coordinates
    rospy.loginfo(f"Publishing attack message: {attack_msg}")
    attack_pub.publish(attack_msg)
if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtle_controller')
    
    turtles = ['/turtle1/cmd_vel', '/turtle2/cmd_vel', '/turtle3/cmd_vel', '/turtle4/cmd_vel']
    publishers = [rospy.Publisher(turtle, Twist, queue_size=1) for turtle in turtles]

    # Publisher for attack messages
    attack_pub = rospy.Publisher('attack_topic', String, queue_size=10)
    
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
            elif key.lower() == 'q':  # 'Q' key to attack
                attack()     
            elif key == '\x03':  # Ctrl-C to exit
                rospy.loginfo("Ctrl-C pressed. Exiting.")
                break
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Shutting down turtle controller.")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
