#!/usr/bin/env python3
import os, sys, yaml
import rospy, rospkg
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from dynamic_reconfigure.server import Server
from adjust.cfg import lane_controller_nodeConfig
from adjust.msg import Twist2DStamped

class LANE_CONTROLLER_RECONFIGURE(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.initialize = True   
    
        ## wait for topic
        self.wait_for_message = rospy.wait_for_message( "/" + self.veh_name + "/lane_controller_node/car_cmd", Twist2DStamped)
        rospy.loginfo("[{}] Node Start! Please use rqt_reconfigure to adjust parameter for [lane_controller_node]!".format(self.node_name))

        # setup parameter


        # create service
        #self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration) 

        # start rqt_reconfig
        #self.reconfigure = Server(lane_controller_nodeConfig, self.rqt_callback)

    def rqt_callback(self, config, level):
        if self.initialize == True:
            for keys in self.yaml_dict:
                config[keys] = self.yaml_dict[keys]
            self.initialize = False
        else:
            for keys in self.yaml_dict:
                self.yaml_dict[keys] = config[keys]
        return config

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self): 
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("lane_controller_reconfigure", anonymous = False)
    lane_controller_reconfigure = LANE_CONTROLLER_RECONFIGURE()
    rospy.on_shutdown(lane_controller_reconfigure.on_shutdown)
    rospy.spin()

