#!/usr/bin/env python
# license removed for brevity

from __future__ import division, print_function
import rospy
import tf

from geometry_msgs.msg import PoseStamped


import math

import numpy

pub = rospy.Publisher('addFrameToGroundTruth', PoseStamped, queue_size=10)


# from ros transformations.py

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.435953, 0.310622, -0.718287, 0.444435])
    True

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis + 1
    j = _NEXT_AXIS[i+parity-1] + 1
    k = _NEXT_AXIS[i-parity] + 1

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = numpy.empty((4, ))
    if repetition:
        q[0] = cj*(cc - ss)
        q[i] = cj*(cs + sc)
        q[j] = sj*(cc + ss)
        q[k] = sj*(cs - sc)
    else:
        q[0] = cj*cc + sj*ss
        q[i] = cj*sc - sj*cs
        q[j] = cj*ss + sj*cc
        q[k] = cj*cs - sj*sc
    if parity:
        q[j] *= -1.0

    return q

def callback(data):
    global pub
    
#    data.header.stamp = rospy.Time.now() 
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.stamp)
    data.header.frame_id = "xtionTrav1"
    
    pub.publish(data)
    
    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.position.x, data.pose.position.y, data.pose.position.z),
                     (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w),
                     data.header.stamp,
                     data.header.frame_id,
                     "map")

    q = quaternion_from_euler(0.,numpy.pi/8,0.)
#    q = quaternion_from_euler(0.,0.,0.)
    
    br.sendTransform((0.,0.,0.),
                     (q[1],q[2],q[3],q[0]),
                     data.header.stamp,
                     "xtionTrav1_link",
                     data.header.frame_id)    


if __name__ == '__main__':
    print("Hello my dear friend how are you today?")
    
    rospy.init_node('fixViconTransfrom', anonymous=True)
    rospy.Subscriber("/TeleKyb/Vicon/Quadcopter_6/Quadcopter_6", PoseStamped, callback)
    
    rospy.spin()