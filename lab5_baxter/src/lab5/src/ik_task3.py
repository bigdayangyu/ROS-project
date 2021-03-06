#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    flag = 0

    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        if flag == 0:
            #Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = 0.791
            request.ik_request.pose_stamped.pose.position.y = 0.631
            request.ik_request.pose_stamped.pose.position.z = -0.069
            request.ik_request.pose_stamped.pose.orientation.x = 0.786
            request.ik_request.pose_stamped.pose.orientation.y = 0.618
            request.ik_request.pose_stamped.pose.orientation.z = 0.012
            request.ik_request.pose_stamped.pose.orientation.w = 0.001
            flag = 1
        else:
            #Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = 0.873
            request.ik_request.pose_stamped.pose.position.y = 0.387
            request.ik_request.pose_stamped.pose.position.z = -0.094      
            request.ik_request.pose_stamped.pose.orientation.x = 0.954
            request.ik_request.pose_stamped.pose.orientation.y = 0.300
            request.ik_request.pose_stamped.pose.orientation.z = 0.016
            request.ik_request.pose_stamped.pose.orientation.w = 0.014
            flag = 0


        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            # group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

