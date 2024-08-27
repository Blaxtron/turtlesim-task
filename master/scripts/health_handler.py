#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64

class Turtle:
    def __init__(self,name,initial_health=100):
        self.name = name
        self.health = initial_health
        self.publisher = rospy.Publisher(f"{self.name}/health", Int64, queue_size=10)

    def print_health(self):
        rospy.loginfo(f"{self.name}'s health={self.health}")



def main():
    rospy.init_node("health_manager",anonymous=False)
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
