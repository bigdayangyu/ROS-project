#!/usr/bin/env python

import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import exp_quat_func as eqf

listener = None

def return_rbt(trans, rot):
    """
    Prints out the 4x4 rigid body transformation matrix from quaternions

    Input:
        (3,) array - translation ector
        (4,) array - rotation vector in quaternions
    """

    #YOUR CODE HERE
    trans = np.array(trans)
    rot = np.array(rot)
    exp_form  =  eqf.quaternion_to_exp(rot)
    # print exp_form
    omega = exp_form[0]
    theta = exp_form[1]
    g = eqf.create_rbt(omega, theta, trans)
    return g


def compute_twist(rbt):
    """
    Computes the corresponding twist for the rigid body transform

    Input:
        rbt - (4,4) ndarray 

    Output:
        v - (3,) array
        w - (3,) array
    """
    #YOUR CODE HERE
    R = rbt[:3,:3]
    orientation = eqf.find_omega_theta(R)# omega/theta
    v = eqf.find_v(orientation[0], orientation[1], trans).reshape(3,)
    return (v, orientation[0])





if __name__=='__main__':
    rospy.init_node('ar_tags_subs')
    if len(sys.argv) < 4:
        print('Use: ar_tag_subs.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_tags = {}
    ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    print 'hey'
    listener = tf.TransformListener()
    print 'yo'

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['arZ'] + ' and ' + ar_tags['ar1'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['arZ'] + ' and ' + ar_tags['ar1'])
            print twist
        except Exception as e:
            print e
        #(trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))

        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['ar1'], rospy.Time(0))
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
            print twist
        except:
            print ''
            
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['arZ'], rospy.Time(0))
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
            print twist
        except:
            print ''

        rate.sleep()
