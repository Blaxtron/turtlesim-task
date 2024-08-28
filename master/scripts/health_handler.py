#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from math import sqrt

turtles = {
    'turtle1': {'health': 100, 'attacks': 10},
    'turtle2': {'health': 100, 'attacks': 10},
    'turtle3': {'health': 100, 'attacks': 10},
    'turtle4': {'health': 100, 'attacks': 10},
}


class Turtle:
    def __init__(self,name,initial_health=100):
        self.name = name
        self.health = initial_health
        self.publisher = rospy.Publisher(f"{self.name}/health", Int64, queue_size=10)

    def print_health(self):
        rospy.loginfo(f"{self.name}'s health={self.health}")
    attack_radius = 2.0

def attack_callback(msg):
    attacker_name, x, y = msg.data.split(':')
    x, y = float(x), float(y)
    if turtles[attacker_name]['attacks'] > 0:
        rospy.loginfo(f"{attacker_name} is attacking at ({x}, {y})")
        turtles[attacker_name]['attacks'] -= 1
        for turtle_name, data in turtles.items():
            if turtle_name != attacker_name:
                distance = sqrt((data['x'] - x) ** 2 + (data['y'] - y) ** 2)
                if distance <= attack_radius:
                    data['health'] -= 50
                    rospy.loginfo(f"{turtle_name} hit! New health: {data['health']}")
                    health_pub.publish(f"{turtle_name}:{data['health']}")
                    if data['health'] <= 0:
                        rospy.loginfo(f"{turtle_name} eliminated!")
                        turtles.pop(turtle_name)






def main():
    rospy.init_node("health_manager",anonymous=False)
    rospy.Subscriber('attack_topic', String, attack_callback)
    r = rospy.Rate(5)
    #assigns each turtle as a class objects
    turtle1 = Turtle("turtle1",100)
    turtle2 = Turtle("turtle2",100)
    turtle3 = Turtle("turtle3",100)
    turtle4 = Turtle("turtle4",100)
    turtleList = [turtle1,turtle2,turtle3,turtle4]
    while not rospy.is_shutdown():
        #publishes each turtle's health to /TURTLENAME/health topic
        for i in range(4):       
            turtleList[i].publisher.publish(turtleList[i].health)
            turtleList[i].print_health()
            #turtleList[i].health-=1 #demonstration of health decreasing
        
        r.sleep()    
        rospy.loginfo("\n")
        
            
        
        
            

    


if __name__ == '__main__':
    main()
