#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64

class Turtle:
    def __init__(self,name,initial_health=100):
        self.name = name
        self.health = initial_health

    def print_health(self):
        rospy.loginfo(f"{self.name}'s health={self.health}")



def main():
    rospy.init_node("health_manager",anonymous=False)
    healthPublisher = rospy.Publisher("health_node",Int64,queue_size=10)
    r = rospy.Rate(5)
    turtle1 = Turtle("turtle1",100)
    turtle2 = Turtle("turtle2",100)
    turtle3 = Turtle("turtle3",100)
    turtle4 = Turtle("turtle4",100)
    turtleList = [turtle1,turtle2,turtle3,turtle4]
    while not rospy.is_shutdown():
        for i in range(4):
            if turtleList[i].health < 0:
                turtleList[i].health = 0
            healthPublisher.publish(turtleList[i].health)
            turtleList[i].print_health()
            #turtleList[i].health-=1 #demonstration of health decreasing
        
        r.sleep()    
        rospy.loginfo("\n")
        
            
        
        
            

    


if __name__ == '__main__':
    main()
