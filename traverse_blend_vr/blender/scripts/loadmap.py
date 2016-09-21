
from bge import logic,events
import bpy


from os import listdir
from os.path import isfile, join
import os.path

try:
    import roslib
except ImportError as error:
    print("Could not find ROS. source setup.[ba]sh ?")
    raise error
    
import rospy

from time import sleep
from std_msgs.msg import Byte

global controller    
    
def initSubscriber(contr):
    
    rospy.logdebug("init Subscriber")
    rospy.Subscriber('/loadmap', Byte, loadmapCallback)
    # spin() simply keeps python from exiting until this node is stopped

def loadmapCallback(msg):
    global controller
    # convert the bin ros msg to python represetation of the octomap octree       
    rospy.loginfo("I heard %s",msg.data)
    if int(msg.data) == 1:
        rospy.loginfo("loading a map")
        loadmap(controller)

#class Loadmap():
#    """Write here the general documentation of your actuator.
#    It will appear in the generated online documentation.
#    """
#    _name = "Loadmap"
#    _short_desc = "load an octomap octree as mesh"
#    
#    
#
#    # define here the data fields required by your actuator
#    # format is: field name, initial value, type, description
#    
#
#    def __init__(self):
##        sprint("%s initialization" % obj.name)
#        # Call the constructor of the parent class
#        
#
#        # Do here actuator specific initializations
#        self._fullmapfile = "octomapFromFile_part.blend"
#        self._fullmapfile = "octomapFromFile_noduplicates.blend"
#        self._fullmapfile = "octomapFromROSTopic_fromData_lvl16.blend"
#        self._octoMapFileParts = "octomapPart_"
#        self._fullmapfile = "octomapFromFile_fromData_lvl16.blend"
#        #self._fullmapfile = "octomapSimple2.blend"
#        
#        self.local_data['byte'] = 0
#        self._fullmapLib = []
#        self._partmapLib = []
#        
#        print('Component initialized')

def unique_libload(file_path):
    '''Loads a unique instance of a blend.
    INPUTS:
        - file_path - path to the blend file
    OUTPUTS:
        - added_objects, the object that were added
    '''
    scene = logic.getCurrentScene()
    
    old_objects = scene.objects + scene.objectsInactive

    if not isfile(file_path):
        print("no such file: " + file_path + " libloadfailed")
        return
        
    f = open(file_path, 'rb').read()
    identifier = file_path.split('.')[0] + "_lib" + str(.0)

    while identifier in logic.LibList():
        id = identifier.split('.')
        identifier = id[0] + '.' + str(int(id[1])+1)
#    print("libload %s " % identifier)
    logic.LibLoad(identifier, 'Scene', f, async=True)
    
    all_obj = scene.objects + scene.objectsInactive
    
    added_objects = []
    
    for o in all_obj:
        if o not in old_objects:
            added_objects.append(o)
            
    # set the physics of the loadad map to no colliiosn                
#    print(bpy.data.objects)
    for ob in bpy.context.scene.objects:
        if ob.type == 'MESH' and (ob.name.startswith("mapFrom")):
            #bpy.context.scene.objects.active = bpy.data.objects["mapFromData"]            
            ob.game.physics_type = 'STATIC'
#            ob.game.physics_type = 'NO_COLLISION'
    
    return identifier
  

'''
    if len(self._fullmapLib) < 1:
        print("no libs to remove for fullmap")
    else:
        for lib in self._fullmapLib:
            print("removing the current fullmap lib %s" %lib)
            logic.LibFree(lib)

    print("load the full map")
    self._fullmapLib.append(self.unique_libload(self._fullmapfile))
        
        
    if (self.local_data['byte'] & 2):
        pwd = bpy.path.abspath('/home/eruff/workspace/morse/morse_viz')
        map_files = [ f for f in listdir(pwd) if isfile(join(pwd,f)) and f.startswith(self._octoMapFileParts)]   
 
        print("mapfiles",map_files)               

        if len(self._partmapLib) < 1:
            print("no libs to remove for partial map")
        else:
            for lib in self._partmapLib:
                print("removing the current partial map lib %s" %lib)
                logic.LibFree(lib)

        for octomap in map_files:
            print("load the partial map ",octomap)
            self._partmapLib.append(self.unique_libload(octomap))
        sleep(1)            
    self.local_data['byte'] = 0
'''  

def loadmap(contr):
    owner = contr.owner
    DIR = "blend_files/octomaps/test_data/"
    # path joining version for other paths
    
    if owner['counter'] < len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]) :
        print (owner['counter'])
        prefix = "octomapFromFile_fromData_lvl16_"
        suffix = ".blend"
        path = DIR + prefix + str(owner['counter']) + suffix
        unique_libload(path)
        owner['counter'] += 1 # this is the property that keeps count
    

def main(contr):
    global controller
    controller = contr
    """ Main loop of the actuator.
    Implements the component behaviour
    """
    

    owner = contr.owner

    if not owner['initLoadmapNode']:
        print("Init the loadmap ros node only once")
        rospy.init_node('loadmapIntoBlender')
        initSubscriber(contr)
        owner['initLoadmapNode'] = True
    

        

                       

    
if __name__ == "__main__":
    
    main(logic.getCurrentController())