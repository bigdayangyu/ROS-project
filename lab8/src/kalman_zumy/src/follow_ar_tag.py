#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist, Quaternion
import exp_quat_func as eqf
import kalman_filter as kf
from kalman_zumy.srv import NuSrv, NuSrvRequest

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
    trans = rbt[:3, 3]
    orientation = eqf.find_omega_theta(R)# omega/theta
    v = eqf.find_v(orientation[0], orientation[1], trans).reshape(3,)
    return (v, orientation[0])



def follow_ar_tag(zumy, ar_tags):

    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher('%s/cmd_vel' % zumy, Twist, queue_size=2)
    rate = rospy.Rate(10)
    print ar_tags
    
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar1'], ar_tags['arZ'], rospy.Time(0))
            rospy.wait_for_service('innovation')
            KF = rospy.ServiceProxy('innovation', NuSrv)
            transformMessage = Transform()
            transformMessage.translation = Vector3(trans[0], trans[1], trans[2])
            transformMessage.rotation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            kfMessage = NuSrvRequest()
            kfMessage.transform = transformMessage
            kfMessage.origin_tag = ar_tags['ar1']
            KF(kfMessage)
            (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
        except Exception as e:
            try:
                (trans, rot) = listener.lookupTransform('zumy1', ar_tags['ar1'], rospy.Time(0))
            except Exception as f:
                print f
            print e
            continue

        # YOUR CODE HERE
        #  The code should compute the twist given 
        #  the translation and rotation between arZ and ar1
        #  Then send it publish it to the zumy
        if np.linalg.norm(trans) > 0.02:
            rbt = return_rbt(trans=trans, rot=rot)
            velocity, omega = compute_twist(rbt=rbt)

            theta = math.atan(trans[1]/trans[0])
            if trans[0] < 0:
                theta = math.pi + theta
            print(theta)

            if abs(theta) < .05:
                twistMessage = Twist()
                l = Vector3()
                l.x = .2
                l.y = 0
                l.z = 0
                twistMessage.linear = l
                v = Vector3()
                v.x = 0
                v.y = 0
                v.z = 0
                twistMessage.angular = v
                zumy_vel.publish(twistMessage)
            else:
                print 'hi'
                twistMessage = Twist()
                l = Vector3()
                l.x = 0
                l.y = 0
                l.z = 0
                twistMessage.linear = l
                v = Vector3()
                v.x = 0
                v.y = 0
                v.z = theta * 0.5
                twistMessage.angular = v
                zumy_vel.publish(twistMessage)
        else:
            twistMessage = Twist()
            l = Vector3()
            l.x = 0
            l.y = 0
            l.z = 0
            twistMessage.linear = l
            v = Vector3()
            v.x = 0
            v.y = 0
            v.z = 0
            twistMessage.angular = v
            zumy_vel.publish(twistMessage)
        rate.sleep()
  
if __name__=='__main__':
    rospy.init_node('follow_ar_tag_twist')
    if len(sys.argv) < 4:
        print('Use: follow_ar_tag_manual.py [ zumy name ] [ AR tag number for goal] [ AR tag number for Zumy] ')
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['ar1'] = 'usb_cam'
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    rospy.spin()
