#----------------------------------------------------------
# File objects.py
#----------------------------------------------------------
import bpy


import numpy as np

import sys
try:
    from time import *
except ImportError as err:
    print("Could not find time module")
    raise err

try:
    import roslib
except ImportError as error:
    print("Could not find ROS. source setup.[ba]sh ?")
    raise error
#import roslib; roslib.load_manifest('octomap_msgs')
#import roslib; roslib.load_manifest('std_msgs')
from octomap_msgs.msg import Octomap

from std_msgs.msg import Byte
import rospy

try:
    from octomap import  *
except ImportError as err:
    print("Could not find python-octomap")
    raise err
    
import atexit
from createMesh import MeshFromOcTree as MoFo


"""
    global variables
"""
global ocTree
ocTree = OcTree(.1)
global pathToMap
pathToMap = b'/home/eruff/workspace/arhms_telecmt/src/telekyb/packages/telekyb_users/tk_eruff/tk_eruff/maps/'
maps = [b'pillar_map.bt',b'fr_078_tidyup.bt',b'fr_campus.ot']

global ocPub

PUBLISH_BYTES = {"NONEWMAP": 0,
                 "FULLMAP" : 1,
                 "SMALLEST": 2,
                 "MEDIUM": 4,
                 "LARGE": 8}


# The worker thread pulls an item from the queue and processes it
def updateQPairs(output,a,x,y):
    proc = mp.current_process()

    print ('Starting:', proc.name, proc.pid)
    sys.stdout.flush()

    res = []
    for p in a:
        qList = list(p)
        if not qList[0] == qList[1]:
            for j in range(len(qList)):
                if qList[j] == y:                    
                    qList[j] = x
                if qList[j] > y:
                    qList[j] -= 1
            p = tuple(qList)

        sys.stdout.flush()
        res.append(p)
    output.put(res)
    print ('Exiting :', proc.name, proc.pid)
    sys.stdout.flush()    


    
def initSubscriber():
    mapTopic = '/octomap_binary'    
    rospy.logdebug("init Subscriber")
    rospy.Subscriber(mapTopic, Octomap, mapCallback)
    # spin() simply keeps python from exiting until this node is stopped



def mapCallback(msg):
    global ocTree
    # convert the bin ros msg to python represetation of the octomap octree       
    ocTree = binaryMsgToMap(msg)
    

def mapFromFile(filename):
    global ocTree
    global mapFromFile
    print("read octree from ",filename)
    ocTree.readBinary(filename)
    
    

def createMapFromOcTree(fromFile=True):
    global ocTree
    global ocPub

    print("number of nodes in the Tree",ocTree.calcNumNodes())
    print("ocTree Resolution ",ocTree.getResolution())
    print("create map from OcTree")
    
    if fromFile:
        blend_file = "/tmp/octomapFromFile"
    else:
        blend_file = "/tmp/octomapFromROSTopic"
        
    print("******************************************************************************")
    lvl = 16
    print("level: ",lvl)
    """    
        create the mesh from an ocTree
        pass the ocTree to the class and let the class do the rest
    """
    
    # call this with a subtree and therfore divide it into feasible junks
    
    num_nodes = ocTree.calcNumNodes()
    print("number of nodes",num_nodes)
    mofo = MoFo()
    if mofo.ocTreeToBlendFile(ocTree,lvl,blend_file):
        try:
            ocPub.publish(PUBLISH_BYTES["FULLMAP"])
        except:
            pass
    print("mode_set poll createMapFromOcTree",bpy.ops.object.mode_set.poll())
    print("********** ALL DONE EXITING NOW **********************************************")
    #rospy.signal_shutdown("read from file only once all done")
    return 
    
def all_done():
    global ocTree
    print("all done, shutting down")
    del ocTree
   
if __name__ == "__main__":
    oa_start_time = time()
    global ocPub
    global OCTOMAP_BLEND_FROM_FILE
    atexit.register(all_done)    

    progname = str(sys.argv[0])
    mapfile = str.encode(sys.argv[1])
    withrosnode = bool(int(sys.argv[2]))
    readFromFile = bool(int(sys.argv[3]))
    
    
    
    
    if len(sys.argv) < 4:
        print("to view arguments usage: " + progname + "<mapfile> <withrosenode[0,1]> <readfromfile[0,1]>")
        raise SystemError
    
    
    if(withrosnode):
        rospy.init_node("octomapToBlend", log_level=rospy.DEBUG, disable_signals=True) 
        ocPub = rospy.Publisher('mapSaved', Byte, queue_size=10)

    if not readFromFile and withrosnode:
        print("initialize subscriber")
        initSubscriber()
        r = rospy.Rate(.2)

    if withrosnode:        
        while not rospy.is_shutdown():
            
            if(readFromFile):
                mapFromFile(mapfile)
            # if the map is read from file we shut down the rosnode in the first loop            
            if ocTree.calcNumNodes() > 0:
                start_time = time()
                createMapFromOcTree(fromFile=readFromFile)         
                print("createMapFromOcTree took",time()-start_time,"s")
            
            print("number of occupied nodes",ocTree.calcNumNodes())
            print("tree resolution",ocTree.getResolution())
            if readFromFile and withrosnode:
                rospy.signal_shutdown("read from file only once all done")
            
            r.sleep()
            
    else:
        if(readFromFile):
            start_readOctomap = time()
            mapFromFile(mapfile)
            print("reading the file into an octree ",time()-start_readOctomap)
        if ocTree.calcNumNodes() > 0:
            start_time = time()
            print("createMapFromOctree")
            createMapFromOcTree(fromFile=readFromFile)         
            print("createMapFromOcTree took",time()-start_time,"s")
            
        print("number of occupied nodes",ocTree.calcNumNodes())
        print("tree resolution",ocTree.getResolution())            
         
    sys.stdout.flush()
    
    print("the comple program run took ",time()-oa_start_time)
