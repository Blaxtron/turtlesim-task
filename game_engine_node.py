#!/usr/bin/env python3

import math
import rospy
import turtlesim.msg
import geometry_msgs.msg
from std_msgs.msg import Float32

class GameEngineNode:
    def __init__(self):
        rospy.init_node('game_engine')
        self.turtles = {}  # Dictionary to store turtle information
        self.attack_duration = 1  # Attack duration in seconds
        self.attack_radius = 1  # Attack radius in meters
        self.max_attacks = 10

        # Subscribe to turtle topics
        for i in range(1, 5):  # Assuming 4 turtles
            turtle_name = 'turtle{}'.format(i)
            self.turtles[turtle_name] = {
                'health': 100,
                'attacks': self.max_attacks,
                'position': None,
                'is_attacking': False,
                'pub': rospy.Publisher(turtle_name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
            }
            rospy.Subscriber(turtle_name + '/pose', turtlesim.msg.Pose, self.pose_callback, turtle_name)

        # Timer for attack duration
        self.attack_timer = rospy.Timer(rospy.Duration(self.attack_duration), self.attack_timer_callback)

    def pose_callback(self, msg, turtle_name):
        self.turtles[turtle_name]['position'] = (msg.x, msg.y)  # Update turtle position
        # Check for collisions with other turtles
        for other_turtle_name, other_turtle in self.turtles.items():
            if turtle_name != other_turtle_name and self.is_colliding(turtle_name, other_turtle_name):
                # Handle collision (e.g., apply damage if attacking)
                if self.turtles[turtle_name].get('is_attacking', False):
                    other_turtle['health'] -= self.attack_damage
                    if other_turtle['health'] <= 0:
                        self.remove_turtle(other_turtle_name)
                else:
                    # Handle other collision scenarios (e.g., pushback)
                    pass
    def is_colliding(self, turtle1_name, turtle2_name):
    
        turtle1_pos = self.turtles[turtle1_name].get('position')
        turtle2_pos = self.turtles[turtle2_name].get('position')
        if turtle1_pos is not None and turtle2_pos is not None:
            distance = math.sqrt((turtle1_pos[0] - turtle2_pos[0]) ** 2 + (turtle1_pos[1] - turtle2_pos[1]) ** 2)
            return distance < self.attack_radius
        return False          
    def attack_timer_callback(self, event):
        for turtle_name, turtle in self.turtles.items():
            if  turtle['is_attacking']:
                turtle['is_attacking'] = False
                # Check for collisions and apply damage (if not already done)
                for other_turtle_name, other_turtle in self.turtles.items():
                    if turtle_name != other_turtle_name and self.is_colliding(turtle_name, other_turtle_name):
                        other_turtle['health'] -= self.attack_damage
                        if other_turtle['health'] <= 0:
                            self.remove_turtle(other_turtle_name)
                turtle['attacks'] -= 1
                if turtle['attacks'] == 0:
                    self.remove_turtle(turtle_name)

    def check_winner(self):
        all_attacks_done = all(turtle.attacks >= 10 for turtle in self.turtles.values())
    
        if all_attacks_done:
            # Game over, determine the winner based on health
            winner_name = max(self.turtles, key=lambda name: self.turtles[name].health)
            rospy.logwarn(f"Turtle {winner_name} wins with {self.turtles[winner_name].health} health!")
            self.game_over = True
        else:
            # Game continues
            rospy.loginfo("Game continues, not all turtles have completed 10 attacks yet.")


if __name__ == '__main__':
    game_engine = GameEngineNode()
    rospy.spin()
