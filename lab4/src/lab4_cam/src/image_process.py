#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from lab4_cam.srv import ImageSrv, ImageSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import *


# Nominal length of a tile side
TILE_LENGTH = 30.48 #cm

# Helper function to check computed homography
# This will draw dots in a grid by projecting x,y coordinates
# of tile corners to u,v image coordinates
def check_homography(image, H, nx, ny, length=TILE_LENGTH):
  # H should be a 3x3 numpy.array
  # nx is the number of tiles in the x direction
  # ny is the number of tiles in the y direction
  # length is the length of one side of a tile
  # image is an image array
  for i in range(nx+1):
    for j in range(ny+1):
      xbar = np.array([[i*length],[j*length],[1]])
      print(xbar.shape)
      ubar = np.dot(H,xbar).T[0]
      u = np.int(ubar[0]/ubar[2])
      v = np.int(ubar[1]/ubar[2])
      print 'Dot location: ' + str((u,v))
      cv2.circle(image, (u,v), 5, 0, -1)
  cv2.imshow('Check Homography', image)

# Create a CvBridge to convert ROS messages to OpenCV images
bridge = CvBridge()

# Converts a ROS Image message to a NumPy array to be displayed by OpenCV
def ros_to_np_img(ros_img_msg):
  return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

# Define the total number of clicks we are expecting (4 corners)
TOT_CLICKS = 6

if __name__ == '__main__':
  
  # Waits for the image service to become available
  rospy.wait_for_service('last_image')
  
  # Initializes the image processing node
  rospy.init_node('image_processing_node')
  
  # Creates a function used to call the 
  # image capture service: ImageSrv is the service type
  last_image_service = rospy.ServiceProxy('last_image', ImageSrv)

  # Create an empty list to hold the coordinates of the clicked points
  points = []

  # Callback function for 'cv2.SetMouseCallback' adds a clicked point to the
  # list 'points'
  def on_mouse_click(event,x,y,flag,param):
    if(event == cv2.EVENT_LBUTTONUP):
      point = (x,y)
      print "Point Captured: " + str(point)
      points.append(point)

  while not rospy.is_shutdown():
    try:
      # Waits for a key input to continue
      raw_input('Press enter to capture an image:')
    except KeyboardInterrupt:
      print 'Break from raw_input'
      break
    
    try:
      # Request the last image from the image service
      # And extract the ROS Image from the ImageSrv service
      # Remember that ImageSrv.image_data was
      # defined to be of type sensor_msgs.msg.Image
      ros_img_msg = last_image_service().image_data

      # Convert the ROS message to a NumPy image
      np_image = ros_to_np_img(ros_img_msg)

      # Display the CV Image
      cv2.imshow("CV Image", np_image)

      # Tell OpenCV that it should call 'on_mouse_click' when the user
      # clicks the window. This will add clicked points to our list
      cv2.setMouseCallback("CV Image", on_mouse_click, param=1)

      # Zero out list each time we have a new image
      points = []

      # Loop until the user has clicked enough points
      while len(points) < TOT_CLICKS:
        if rospy.is_shutdown():
          raise KeyboardInterrupt
        cv2.waitKey(10)

      # Convert the Python list of points to a NumPy array of the form
      #   | u1 u2 u3 u4 |
      #   | v1 v2 v3 v4 |
      uv = np.array(points).T

# === YOUR CODE HERE ===========================================================
      
      # This is placeholder code that will draw a 4 by 3 grid in the corner of
      # the image
      nx = 4
      ny = 3
      H = np.eye(3)

      xy = TILE_LENGTH*np.array([[0.,0.,1.,1.], [0.,1.,1.,0.]])
      x = xy[0, :]
      y = xy[1, :]
      u = uv[0, 0:4]
      v = uv[1, 0:4]

      A = np.vstack([np.hstack([x[0], y[0], 1, 0, 0, 0, (-u[0]*x[0]), (-u[0]*y[0])]),
          np.hstack([0, 0, 0, x[0], y[0], 1, (-v[0]*x[0]), (-v[0]*y[0])]),
          np.hstack([x[1], y[1], 1, 0, 0, 0, (-u[1]*x[1]), (-u[1]*y[1])]),
          np.hstack([0, 0, 0, x[1], y[1], 1, (-v[1]*x[1]), (-v[1]*y[1])]),
          np.hstack([x[2], y[2], 1, 0, 0, 0, (-u[2]*x[2]), (-u[2]*y[2])]),
          np.hstack([0, 0, 0, x[2], y[2], 1, (-v[2]*x[2]), (-v[2]*y[2])]),
          np.hstack([x[3], y[3], 1, 0, 0, 0, (-u[3]*x[3]), (-u[3]*y[3])]),
          np.hstack([0, 0, 0, x[3], y[3], 1, (-v[3]*x[3]), (-v[3]*y[3])])])

      B = np.vstack([u[0], v[0], u[1], v[1], u[2], v[2], u[3], v[3]])

      h = np.linalg.solve(A, B)

      H = np.vstack([np.hstack([h[0], h[1], h[2]]),
        np.hstack([h[3], h[4], h[5]]),
        np.hstack([h[6], h[7], 1])])

      Q = np.linalg.inv(H)

      ubar1 = np.array([[uv[0, 4]], [uv[1, 4]] ,[1]])
      xbar1 = np.dot(Q,ubar1).T[0]
      ubar2 = np.array([[uv[0, 5]], [uv[1, 5]],[1]])
      xbar2 = np.dot(Q,ubar2).T[0]

      x1 = np.int(xbar1[0]/xbar1[2])
      y1 = np.int(xbar1[1]/xbar1[2])
      x2 = np.int(xbar2[0]/xbar2[2])
      y2 = np.int(xbar2[1]/xbar2[2])

      xx = x2-x1
      yy = y2-y1

      xxyy = np.array([[xx], [yy]])

      print(np.linalg.norm(xxyy))




# ==============================================================================
      
      # Check the produced homography matrix
      check_homography(np_image, H, nx, ny)

      # Loop until the user presses a key
      key = -1
      while key == -1:
        if rospy.is_shutdown():
          raise KeyboardInterrupt
        key = cv2.waitKey(100)
      
      # When done, get rid of windows and start over
      # cv2.destroyAllWindows()

    except KeyboardInterrupt:
      print 'Keyboard Interrupt, exiting'
      break

    # Catch if anything went wrong with the Image Service
    except rospy.ServiceException, e:
      print "image_process: Service call failed: %s"%e
    
  cv2.destroyAllWindows()

