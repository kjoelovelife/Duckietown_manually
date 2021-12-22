#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from duckiebot_msgs.cfg import InverseKinematicConfig
from duckiebot_msgs.msg import Twist2DStamped

class InverseKinematicReconfigure(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.target_node = rospy.get_param("~target_node", "kinematics_node")
        self.initialize = True   
        ## wait for topic
        #self.wait_for_message = rospy.wait_for_message("/" + self.veh_name + "/joy_mapper_node/car_cmd", Twist2DStamped)
        rospy.loginfo(f"[{self.node_name}] Start!")

        # get parameter from ROS Parameter Server
        self.ros_parameter = {
            "baseline": 0.1,
            "k": 27.0,
            "omega_max": 8.0,
            "limit": 1.0,
            "radius": 0.0318,
            "gain": 1.0,
            "trim": 0.0,
        }
        
        # setup local variable
        self.initialize = True

        # start rqt_reconfig
        self.reconfigure = Server(InverseKinematicConfig, self.rqt_callback)

    def rqt_callback(self, config, level):
        for key in self.ros_parameter:
            parameter_name = f"/{self.veh_name}/{self.target_node}/{key}"
            if self.initialize == True:      
                self.ros_parameter[key] = rospy.get_param(parameter_name, self.ros_parameter[key])           
                config[key] = self.ros_parameter[key]
                self.initialize = False
            else:
                self.ros_parameter[key] = config[key]
                rospy.loginfo(f"[{self.node_name}] Set parameter {parameter_name} to {self.ros_parameter[key]}")
                rospy.set_param(parameter_name, self.ros_parameter[key])
        return config

    def on_shutdown(self): 
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("inverse_kinematic_reconfigure", anonymous = False)
    inverse_kinematic_reconfigure = InverseKinematicReconfigure()
    rospy.on_shutdown(inverse_kinematic_reconfigure.on_shutdown)
    rospy.spin()