from bge import logic as GameLogic
import bpy
import sys
sys.path.append("/usr/local/lib/python3.4/dist-packages/rospkg-1.0.35-py3.4.egg")
sys.path.append("/usr/local/lib/python3.4/dist-packages/catkin_pkg")

try:
    import roslib
except ImportError as error:
    raise error
import rospy

import rosnode 
import re

cont = GameLogic.getCurrentController()
own = cont.owner

uav_count = 0
# get list of current rosnodes
for node in rosnode.get_node_names():
    # filter for tk core nodes and get the core id
    m = re.search('(?<=/TeleKybCore_)[0-9]{1,}',node)
    if m:
        m = m.group(0)
        # save the core id as object property
        own['uav_' + str(uav_count)] = m
        uav_count += 1