import logging
logger = logging.getLogger("morse." + __name__)


import morse.core.actuator
from morse.helpers.components import add_data
from morse.middleware.ros import mathutils


class Telekybsetstate(morse.core.actuator.Actuator):

    """
        Write here the general documentation of your actuator.
        It will appear in the generated online documentation.
    """
    _name = "Telekybstate"
    _short_desc = "Interface to set the position of a UAV from a TKState msg"

    # define here the data fields required by your actuator
    """
    the telekyb TKState msg looks like

    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    geometry_msgs/Pose pose
        geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
        geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
    geometry_msgs/Twist twist
        geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
        geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
    """

    # geometry_msgs/Pose part of the TKState msg
    # position pose/position
    add_data('x', 0.0, 'float', "x position")
    add_data('y', 0.0, 'float', "y position")
    add_data('z', 0.0, 'float', "z position")

    # orientation in quternion representation
    # pose/orientation
    add_data('qx', 0.0, 'float', "x part of rotation quaternion")
    add_data('qy', 0.0, 'float', "y part of rotation quaternion")
    add_data('qz', 0.0, 'float', "z part of rotation quaternion")
    add_data('qw', 0.0, 'float', "w part of rotation quaternion")

    # geometry_msgs/Twist part of the TKState msg
    # twist/linear
    add_data('vx', 0.0, 'float', "desired x velocity in m/s")
    add_data('vy', 0.0, 'float', "desired y velocity in m/s")
    add_data('vz', 0.0, 'float', "desired z velocity in m/s")
    # twist/angular
    add_data('vroll', 0.0, 'float', "angular velocity around x axis ")
    add_data('vpitch', 0.0, 'float', "angular velocity around y axis")
    add_data('vyaw', 0.0, 'float', "angular velocity around z axis")    


    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Do here actuator specific initializations
       
        self.local_data['x'] = 0.
        self.local_data['y'] = 0.
        self.local_data['z'] = 2.
       
        self.local_data['qx'] = 0.
        self.local_data['qy'] = 0.
        self.local_data['qz'] = 0.
        self.local_data['qw'] = 1.
       
        self.local_data['vx'] = 0.
        self.local_data['vy'] = 0.
        self.local_data['vz'] = 0.

        self.local_data['velroll'] = 0.
        self.local_data['velpitch'] = 0.
        self.local_data['velyaw'] = 0.
       
        logger.info('Component initialized')
        


    def default_action(self):
        """ Main loop of the actuator.

            Implements the component behaviour
        """
      
        """ Move the robot. """
           

        quaternion = mathutils.Quaternion((self.local_data['qw'], self.local_data['qx'], self.local_data['qy'],self.local_data['qz']))
        euler = quaternion.to_euler()
        
                # New parent position
        position = mathutils.Vector((self.local_data['x'],
                                     -self.local_data['y'],
                                     -self.local_data['z']))

        # New parent orientation
        orientation = mathutils.Euler([euler.x,
                                       -euler.y,
                                       -euler.z])

        self.robot_parent.force_pose(position, orientation.to_matrix())
        