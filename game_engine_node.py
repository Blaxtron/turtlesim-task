#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64, String
from turtlesim.msg import Pose
from math import sqrt

class Turtle:
    def __init__(self, name, initial_health=100, x=0.0, y=0.0):
        self.name = name
        self.health = initial_health
        self.x = x
        self.y = y
        self.attacks = 10
        self.publisher = rospy.Publisher(f"{self.name}/health", Int64, queue_size=10)

    def print_health(self):
        rospy.loginfo(f"{self.name}'s health={self.health}")

class GameEngineNode:
    def __init__(self):
        rospy.init_node('game_engine')
        self.turtles = {
            'turtle1': Turtle("turtle1", 100, 1.0, 2.0),
            'turtle2': Turtle("turtle2", 100, 3.0, 4.0),#creating the 4 turtles
            'turtle3': Turtle("turtle3", 100, -1.0, -2.0),
            'turtle4': Turtle("turtle4", 100, -3.0, -4.0),
        }
        self.attack_radius = 1.0  # Attack radius in meters
        self.attack_damage = 50  # Damage 

       
        for turtle_name in self.turtles:
            rospy.Subscriber(f'/{turtle_name}/pose', Pose, self.pose_callback, turtle_name)

        
        rospy.Subscriber('attack_topic', String, self.attack_callback)

    def pose_callback(self, msg, turtle_name):
        
        if turtle_name in self.turtles:
            self.turtles[turtle_name].x = msg.x
            self.turtles[turtle_name].y = msg.y

    def attack_callback(self, msg):
        attack_data = msg.data.split(":")
        attacker_name = attack_data[0]
        attacker = self.turtles.get(attacker_name)

        if attacker and attacker.attacks > 0:
            rospy.loginfo(f"{attacker_name} is attacking!")
            attacker.attacks -= 1

            eliminated_turtles = []

            for target_name, target in self.turtles.items():
                if target_name != attacker_name:
                    distance = sqrt((target.x - attacker.x) ** 2 + (target.y - attacker.y) ** 2)
                    if distance <= self.attack_radius:
                        target.health -= self.attack_damage
                        rospy.loginfo(f"{target_name} hit! New health: {target.health}")
                        target.publisher.publish(target.health)
                        if target.health <= 0:
                            rospy.loginfo(f"{target_name} eliminated!")
                            eliminated_turtles.append(target_name)

            # here i remove turtles that got killed 
            for turtle_name in eliminated_turtles:
                self.turtles.pop(turtle_name)

        
        self.check_winner()

    def check_winner(self):
        active_turtles = [t for t in self.turtles.values() if t.health > 0]
        all_attacks_exhausted = all(t.attacks == 0 for t in self.turtles.values())

        if len(active_turtles) == 1:
            rospy.logwarn(f"{active_turtles[0].name} wins with {active_turtles[0].health} health!")
            rospy.signal_shutdown("Game Over")
        elif not active_turtles or all_attacks_exhausted:
            rospy.logwarn("Game Over. All turtles have either been eliminated or run out of attacks.")
            rospy.signal_shutdown("Game Over")
        else:
            rospy.loginfo("Game continues.")

if __name__ == '__main__':
    game_engine = GameEngineNode()
    rospy.spin()
