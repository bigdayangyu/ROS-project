#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Point, Quaternion, Pose, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import kin_func_skeleton as kfs
import exp_quat_func as eqf

lkp = {} # dictionary containing the last known position of AR tags

def callback(msg, ar_tags):
    for i in range(0, len(msg.transforms)):

        # YOUR CODE HERE
        # The code should look at the transforms for each AR tag
        # Then compute the rigid body transform between AR0 and AR1, 
        # AR0 and ARZ, AR1 and ARZ
        #  hint: use the functions you wrote in exp_quat_func
        #  note: you can change anything in this function to get it working
        #  note: you may want to save the last known position of the AR tag

        lkp[msg.transforms[i].child_frame_id] = msg.transforms[i].transform # position / orientation
       
        # print lkp
    
    tag_name = lkp.keys()
    marker1_pose = lkp['ar_marker_0']
    marker1_rot = np.array([marker1_pose.rotation.x, marker1_pose.rotation.y,marker1_pose.rotation.z, marker1_pose.rotation.w])
    marker1_trans = np.array([marker1_pose.translation.x, marker1_pose.translation.y,marker1_pose.translation.z])
    marker1_exp = eqf.quaternion_to_exp(marker1_rot)
    marker1_rbt = eqf.create_rbt(marker1_exp[0], marker1_exp[1], marker1_trans)

    marker2_pose = lkp['ar_marker_1']
    marker2_rot = np.array([marker2_pose.rotation.x, marker2_pose.rotation.y,marker2_pose.rotation.z, marker2_pose.rotation.w])
    marker2_trans = np.array([marker2_pose.translation.x, marker2_pose.translation.y,marker2_pose.translation.z])
    marker2_exp = eqf.quaternion_to_exp(marker2_rot)
    marker2_rbt = eqf.create_rbt(marker2_exp[0], marker2_exp[1], marker2_trans)

    marker3_pose = lkp['ar_marker_17']
    marker3_rot = np.array([marker3_pose.rotation.x, marker3_pose.rotation.y,marker3_pose.rotation.z, marker3_pose.rotation.w])
    marker3_trans = np.array([marker3_pose.translation.x, marker3_pose.translation.y,marker3_pose.translation.z])
    marker3_exp = eqf.quaternion_to_exp(marker3_rot)
    marker3_rbt = eqf.create_rbt(marker3_exp[0], marker3_exp[1], marker3_trans)



    g01 = eqf.compute_gab(marker1_rbt,marker2_rbt)
    g02 = eqf.compute_gab(marker1_rbt,marker3_rbt)
    g12 = eqf.compute_gab(marker2_rbt,marker3_rbt)
    print ('rot between tag 0 and 1')
    print g01
    print ('rot between tag 0 and 2')
    print g02
    print ('rot between tag 1 and 2')
    print g12


  
if __name__=='__main__':
    rospy.init_node('ar_tags_subs_manual')
    if len(sys.argv) < 4:
        print('Use: ar_tags_subs_manual.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_tags = {}
    ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]
    rospy.Subscriber('/tf', TFMessage, callback, ar_tags)
    print "hi"
    rospy.spin()
