#!/usr/bin/env python3
import rospy, math
from duckiebot_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Twist

class ForwardTwistToDuckiebot(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.target_node =  rospy.get_param("~target_node", "joy_mapper_node")
        rospy.loginfo(f"[{self.node_name}] Initializing...")
        rospy.loginfo(f"[{self.node_name}] Start!")

        # ROS parameter
        self.ros_parameter = {
            "speed_gain": 0.41,
            "steer_gain": 8.3,
            "bicycle_kinematics": 0.0,
            "simulated_vehicle_length": 0.18, 
        }
        self.get_ros_parameter()

        # local variable
        self.twist_2d_stamped = Twist2DStamped()

        # subscriber node
        self.subscriber = rospy.Subscriber("~cmd_vel", Twist, self.callback_twist)

        # publisher node
        self.publisher = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # timer 
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.5), self.callback_timer)

    def get_ros_parameter(self):
        """Get ROS parameter from ROS Parameter Server
        """
        for key in self.ros_parameter:
            parameter_name = f"/{self.veh_name}/{self.target_node}/{key}"
            self.ros_parameter[key] = rospy.get_param(parameter_name, self.ros_parameter[key])

    def callback_timer(self, event):
        """Update parameter from ROS Parameter Server
        """
        for key in self.ros_parameter:
            parameter_name = f"/{self.veh_name}/{self.target_node}/{key}"
            temp = rospy.get_param(parameter_name)
            if self.ros_parameter[key] == temp:
                pass
            else:
                self.ros_parameter[key] = rospy.get_param([parameter_name])

    def callback_twist(self, msg):
        """Receive cmd_vel and ... 
        """
        self.twist_2d_stamped.header.stamp = rospy.get_rostime()
        self.twist_2d_stamped.v = msg.linear.x * self.ros_parameter["speed_gain"]
        if self.ros_parameter["bicycle_kinematics"] == 0.0:
            self.twist_2d_stamped.omega = msg.angular.z * self.ros_parameter["steer_gain"]
        else:
            steering_angle = msg.angular.z * self.ros_parameter["steer_gain"]
            self.twist_2d_stamped.omega = self.twist_2d_stamped.v / self.ros_parameter["simulated_vehicle_length"] * math.tan(steering_angle)
        self.publisher.publish(self.twist_2d_stamped)
        
    def on_shutdown(self): 
        self.twist_2d_stamped.header.stamp = rospy.get_rostime()
        self.twist_2d_stamped.v = 0.0
        self.twist_2d_stamped.omega = 0.0
        self.publisher.publish(self.twist_2d_stamped)
        rospy.loginfo(f"[{self.node_name}] closed.")
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("forward_twist_to_duckiebot", anonymous = False)
    forward_twist_to_duckiebot = ForwardTwistToDuckiebot()
    rospy.on_shutdown(forward_twist_to_duckiebot.on_shutdown)
    rospy.spin()