#!/usr/bin/env python3
import os, sys, yaml
import rospy, rospkg
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from dynamic_reconfigure.server import Server
from duckiebot_msgs.cfg import LaneControllerNodeConfig
from duckiebot_msgs.msg import Twist2DStamped

class LaneControllerReconfigure(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.target_node_name =  "/" + self.veh_name + "/lane_controller_node"
        self.initialize = True   
    
        ## wait for topic
        self.wait_for_message = rospy.wait_for_message("/" + self.veh_name + "/lane_controller_node/car_cmd", Twist2DStamped)
        rospy.loginfo("[{}] Node Start! Please use rqt_reconfigure to adjust parameter for [lane_controller_node]!".format(self.node_name))

        # setup parameter
        self.lane_controller_node_param = {
            "d_offset": 0.0,
            "d_resolution": 0.0,
            "d_thres": 0.0,
            "integral_bounds/d/bot": 0.0,
            "integral_bounds/d/top": 0.0,
            "integral_bounds/phi/bot": 0.0,
            "integral_bounds/phi/top": 0.0,
            "k_Id": 0.0,
            "k_Iphi": 0.0,
            "k_d": 0.0,
            "k_theta": 0.0,
            "omega_ff": 0,
            "phi_resolution": 0.0,
            "stop_line_slowdown/end": 0.0,
            "stop_line_slowdown/start": 0.0,
            "theta_thres": 0.0,
            "v_bar": 0.0,
            "verbose": 0,
        }
        self.last_lane_controller_node_param = {}
        for key in self.lane_controller_node_param.keys():
            self.lane_controller_node_param[key] = rospy.get_param(self.target_node_name + "/" + key, self.lane_controller_node_param[key])          
            self.last_lane_controller_node_param[key] = self.lane_controller_node_param[key]
        # create service
        #self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration) 

        # start rqt_reconfig
        self.reconfigure = Server(LaneControllerNodeConfig, self.rqt_callback)

    def rqt_callback(self, config, level):
        if self.initialize == True:
            for keys in self.lane_controller_node_param.keys():
                if keys.find("/") >= 0:
                    list_keys = list(keys)
                    key_in_reconfigure = keys.replace("/", "_")
                    config[key_in_reconfigure] = self.lane_controller_node_param[keys]
                else:
                    config[keys] = self.lane_controller_node_param[keys]
            self.initialize = False
        else:
            for keys in self.lane_controller_node_param.keys():
                if keys.find("/") >= 0:
                    key_in_reconfigure = keys.replace("/", "_")
                    self.lane_controller_node_param[keys] = config[key_in_reconfigure]
                else:
                    self.lane_controller_node_param[keys] = config[keys]
                if self.lane_controller_node_param[keys] != self.last_lane_controller_node_param[keys]:
                    rospy.loginfo("[{}] set param {}: {}".format(self.node_name, keys, self.lane_controller_node_param[keys])) 
                    rospy.set_param(self.target_node_name + "/" + keys, self.lane_controller_node_param[keys])
                    self.last_lane_controller_node_param[keys] = self.lane_controller_node_param[keys]

        return config

    def on_shutdown(self): 
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("lane_controller_reconfigure", anonymous = False)
    lane_controller_reconfigure = LaneControllerReconfigure()
    rospy.on_shutdown(lane_controller_reconfigure.on_shutdown)
    rospy.spin()