#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist

class TurtlesimControl(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.twist = Twist()
        self.publisher_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.5), self.callback_timer)
        
      
    def callback_timer(self, event):
    	self.twist.linear.x = 2.0
    	rospy.loginfo(f"[{self.node_name}] {self.twist}.")
    	self.publisher_.publish(self.twist) 
        
    def on_shutdown(self):
        self.twist.linear.x = 0.0
        self.publisher_.publish(self.twist)
        rospy.loginfo(f"[{self.node_name}] closed.")
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("turtlesim_control", anonymous = False)
    turtlesim_control = TurtlesimControl()
    rospy.on_shutdown(turtlesim_control.on_shutdown)
    rospy.spin()
