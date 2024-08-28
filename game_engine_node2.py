#!/usr/bin/env python3

import geometry_msgs.msg
from std_msgs.msg import Float32, Int64
import rospy
import turtlesim.msg
from math import sqrt

class Turtle:
    def __init__(self, name, initial_health=100, x=0.0, y=0.0):
        self.name = name
        self.health = initial_health
        self.x = x
        self.y = y
        self.attacks = 10  # Add attacks attribute
        self.publisher = rospy.Publisher(f"{self.name}/health", Int64, queue_size=10)

    def print_health(self):
        rospy.loginfo(f"{self.name}'s health={self.health}")

    def attack(self, target_turtle):
        """
        Handles the attack logic for a turtle.

        Args:
            target_turtle (Turtle): The turtle being attacked.

        Returns:
            bool: True if the attack was successful, False otherwise.
        """

        if self.attacks <= 0:
            rospy.loginfo(f"{self.name} has no more attacks left.")
            return False

        distance = sqrt((target_turtle.x - self.x) ** 2 + (target_turtle.y - self.y) ** 2)
        if distance > attack_radius:
            rospy.loginfo(f"{self.name} is too far away to attack {target_turtle.name}.")
            return False

        # Apply attack damage
        target_turtle.health -= self.attack_damage
        rospy.loginfo(f"{self.name} attacked {target_turtle.name}. New health: {target_turtle.health}")

        # Check if target turtle is eliminated
        if target_turtle.health <= 0:
            rospy.loginfo(f"{target_turtle.name} eliminated!")
            self.game_engine.remove_turtle(target_turtle)

        self.attacks -= 1
        return True


attack_radius = 2.0


class GameEngineNode:
    def __init__(self):
        rospy.init_node('game_engine')
        self.turtles = {
            'turtle1': Turtle("turtle1", 100, 1.0, 2.0),
            'turtle2': Turtle("turtle2", 100, 3.0, 4.0),
            'turtle3': Turtle("turtle3", 100, -1.0, -2.0),
            'turtle4': Turtle("turtle4", 100, -3.0, -4.0),
        }
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
         

        # Subscribe to attack topic
        rospy.Subscriber('attack_topic', AttackMessage, self.attack_callback)  # Assuming message type is AttackMessage

    def attack_callback(self, msg):
        attacker_name = msg.data.name  # Assuming message has a 'name' field
        x = msg.data.x
        y = msg.data.y
        attacker = self.turtles.get(attacker_name)  # Access turtle object if it exists

        if attacker and attacker.attacks > 0:
            rospy.loginfo(f"{attacker_name} is attacking at ({x}, {y})")
            attacker.attacks -= 1
            for turtle_name, turtle in self.turtles.items():
                if turtle_name != attacker_name:
                    distance = sqrt((turtle.x - x) ** 2 + (turtle.y - y) ** 2)
                    if distance <= attack_radius:
                        turtle.health -= 50
                        rospy.loginfo(f"{turtle_name} hit! New health: {turtle.health}")
                        turtle.publisher.publish(turtle.health)
                        if turtle.health <= 0:
                            rospy.loginfo(f"{turtle_name} eliminated!")
                            self.turtles.pop(turtle_name)

if __name__ == '__main__':
    game_engine = GameEngineNode()
    rospy.spin()
