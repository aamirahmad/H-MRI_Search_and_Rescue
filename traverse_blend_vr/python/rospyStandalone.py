# -*- coding: utf-8 -*-
"""
Created on Mon Jun  6 16:16:52 2016

@author: eruff

"""

import rosnode 
import re

try:
    import roslib
except ImportError as error:
    raise error
import rospy

from telekyb_msgs.msg import TKState

def getTKCores():
    uav_count = 0
    uavs = []
    # get list of current rosnodes
    for node in rosnode.get_node_names():
        # filter for tk core nodes and get the core id
        m = re.search('(?<=/TeleKybCore_)[0-9]{1,}',node)
        if m:
            m = m.group(0)
            # save the core id as object property
            uavs.append(m)
            uav_count += 1
    return uavs,uav_count
        
class ROSSubscriber():
    def __init__(self,topic,msg,uav_id):
        print("do nothing")
        rospy.Subscriber(topic, msg, self.callback)
        self.topic = topic
        self.uav_id = uav_id
    def callback(self,msg) :
        print("got msg",self.topic)
        real_position = [msg.pose.position.x,-msg.pose.position.y,-msg.pose.position.z]
        real_orientation = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        print(real_position,real_orientation)
        


if __name__ == '__main__':
    
    rospy.init_node('gettkstates', anonymous=False)
    uavs,uav_count = getTKCores()
    for u in uavs:
        topic = "/TeleKyb/TeleKybCore_"+str(u)+"/Sensor/TKState"
        print (topic)
        ROSSubscriber(topic,TKState,u)
    rospy.spin()