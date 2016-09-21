from bge import logic
import bpy
import sys
from mathutils import Vector
from random import random
import threading
import time
import rosnode 
import re

sys.path.append("/usr/local/lib/python3.4/dist-packages")

try:
    import roslib
except ImportError as error:
    raise error
import rospy

from telekyb_msgs.msg import TKState

cont = logic.getCurrentController()
own = cont.owner

scene = logic.getCurrentScene()


def prod(uav_id):
    # do something vaguely useful
    print(threading.current_thread().name, "Starting Subscriber for ",uav_id)
    
#    prod_vec = Vector((random() - 0.5, random() - 0.5, random() - 0.5))
#    print("Prodding", prod_vec)
#    scene.objects["Infantry"].position += prod_vec
    topic = "/TeleKyb/TeleKybCore_"+str(uav_id)+"/Sensor/TKState"
    print (topic)
    ROSSubscriber(topic,TKState,uav_id)    
    rospy.spin()
#    # finish

    print(threading.current_thread().name, "Exiting")

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
        print("Init RosSubscriber for UAV ",uav_id,"on topic",topic)
        self.topic = topic
        self.uav_id = uav_id       
        rospy.Subscriber(topic, msg, self.callback)
        
    def callback(self,msg) :
        real_position = [2*msg.pose.position.x,2*-msg.pose.position.y,2*-msg.pose.position.z]
        real_orientation = [msg.pose.orientation.w,msg.pose.orientation.x,-msg.pose.orientation.y,-msg.pose.orientation.z]
        for obj in scene.objects:
            if obj.name == "Sphere":
                if int(obj['uav_id']) == int(self.uav_id):
                    obj.position = real_position  
                    obj.orientation = real_orientation
    


def onexit():
    #terminate all threads    
    print("onexit")          
    rospy.signal_shutdown("END OF GAME ENGINE")
    print("Waiting for threads to finish...")

    for t in own['threads']:
        t.join()
        
    endGame = cont.actuators["endGame"]
    cont.activate(endGame)
                
if __name__ == "__main__":
    uavs,uav_count = getTKCores()
    print([(i,int(uav_id)) for i,uav_id in enumerate(uavs)])
        
    own['threads'] = [threading.Thread(name="Subscriber %d" % i, target=prod,args=(int(uav_id),)) for i,uav_id in enumerate(uavs)]
    print("Starting threads...")
    for t in own['threads']:
        t.start()
    

    
#    base = scene.objects['Base']
#
#    for prop in base.getPropertyNames():
#        if prop.startswith("uav_"):
#            core_id = base[prop]
#            #rospy.Subscriber("/TeleKyb/TeleKybCore_42/Sensor/TKState", TKState, pose_callback)
#            print("subscribe ",core_id)
#            try:
#                msg = rospy.client.wait_for_message("/TeleKyb/TeleKybCore_"+str(core_id)+"/Sensor/TKState", TKState, 0.2)
#                real_position = [msg.pose.position.x,-msg.pose.position.y,-msg.pose.position.z]
#                    
#                for obj in scene.objects:
#                    if obj.name == "Sphere":
#                            if obj['uav_id'] == core_id:
#                                obj.position = real_position
#                            
#            except:
#                pass
