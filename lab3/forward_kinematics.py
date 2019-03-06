import numpy as np
# import scipy as sp
# from scipy import linalg
import math
import rospy
from sensor_msgs.msg import JointState

np.set_printoptions(precision=4,suppress=True)

#------------------------------------------------------- Prelab Functions ------
def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    #YOUR CODE HERE
    omega_hat = np.array([ [        0,-omega[2],  omega[1] ],
                           [ omega[2],        0, -omega[0] ],
                           [-omega[1], omega[0],         0 ]])

    return omega_hat

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    
    #YOUR CODE HERE
    rot = np.array([[math.cos(theta), -math.sin(theta)],
                    [math.sin(theta),  math.cos(theta)]])

    return rot

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    #YOUR CODE HERE
    w_hat = skew_3d(omega)
    w_mag = np.linalg.norm(omega)
    
    rot = np.eye(3) + w_hat/w_mag*math.sin(w_mag*theta) +\
          np.dot(w_hat,w_hat)/np.dot(w_mag,w_mag)*(1-math.cos(w_mag*theta))

    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    #YOUR CODE HERE
    xi_hat = np.array([[0    , -xi[2],  xi[0]],
                       [xi[2],      0,  xi[1]],
                       [0    ,      0,      0]])

 
    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    xi_hat = np.array([[     0, -xi[5],  xi[4], xi[0]],
                       [ xi[5],      0, -xi[3], xi[1]],
                       [-xi[4],  xi[3],      0, xi[2]],
                       [     0,      0,      0,    0 ]])

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    #YOUR CODE HERE
    w_theta = xi[2]*theta
    R = rotation_2d(w_theta)
    p1 = np.array([[(1-math.cos(w_theta)),     math.sin(w_theta)],
                   [   -math.sin(w_theta), (1-math.cos(w_theta))]])
    p2 = np.array([[0,-1],
                   [1, 0]])
    p3 = np.array([[xi[0]/xi[2]],
                   [xi[1]/xi[2]]])
    p = np.dot(np.dot(p1,p2),p3)
    g = np.vstack((np.hstack((R ,p)),[0,0,1]))
    return g


def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    omega = xi[3:]
    v = xi[:3]
    g1 = rotation_3d(omega, theta)
    w_mag = np.linalg.norm(omega)
    w_hat = skew_3d(omega)
    g2 = (1/(w_mag**2))*\
          (np.dot((np.eye(3)-g1),(np.dot(w_hat,v.reshape(3,1))))+ \
          np.dot(np.dot(omega.reshape(3,1),omega.reshape(3,1).T ),v.reshape(3,1))*theta)
    g3 = g2.reshape(3,1)
    g = np.vstack((np.hstack((g1, g3)),[0,0,0,1]))
 
    return g

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    #YOUR CODE HERE
    g = 1
    n = len(theta)
    for i in range(0,n):
        g = np.dot(g,homog_3d(xi[:,i], theta[i]))


    return g

#--------------------------------------------------------- Task 1 ---------
def velocity(omega, q):
    """
    Computes the velocity vector for a joint when given omega and q

    Args:
    omega - (3, 1) ndarray: vector for axis of rotation
    q - (3, 1) ndarray: vector for displacement in wider coordinate frame

    Returns:
    v - (3, 1) ndarray: the velocity vector
    """

    v = np.cross(-omega, q)

    return v


def Twist(v, omega):
    """
    Computes a twist with angle omega and velocity v

    Args:
    v - (3, 1) ndarray: vector for velocity
    omega - (3, 1) ndarray: vector for axis of rotation

    Returns:
    xi - (6, 1) ndarray: twist vector
    """
	

    xi = np.hstack( (v , omega))
    
    return xi



def task1(theta):
    """
    Computes the transformation matrix for baxter's arm given the joint angles
    for each joint

    Args:
    theta - (7, 1) ndarray: the displacement of each joint

    Returns:
    g - (4,4) ndarray: the resulting homogenous transformation matrix
    """

    if not theta.shape == (7,):
        raise TypeError('theta must be 7x1')

    omega = np.array([  [-.0059,.0113,.9999],
                    [-0.7077,0.7065,-0.0122],
                    [.7065,.7077,-.0038],
                    [-.7077,.7065,-.0122],
                    [.7065,.7077,-.0038],
                    [-.7077,.7065,-.0122],
                    [.7065,.7077,-.0038], 
                ])

    q = np.array([ [.0635,.2598,.1188],
               [0.1106, 0.3116,0.3885] ,
               [.1827,.3838,.3881],
               [.3682,.5684,.3181],
               [.4417,.6420,.3177],
               [.6332,.8337,.3067],
               [.7152,.9158,.3063],
             ])
    q_end = np.array([0.7957 , 0.9965, 0.3058])
    # xi = np.zeros(6, 7)
    # for i in range(0, len(theta)):
    # 	v = velocity(omega[i,:], q[i, :])
    # 	xi[:, i] = Twist(v, omega[i,:]).T

    xi = Twist(velocity(omega, q), omega).T
    g_initial = np.vstack((np.hstack((np.eye(3), q_end.reshape(3,1))),[0 ,0,0, 1]))

    g = np.dot(prod_exp(xi, theta),g_initial)

    return g


#-------------------------------------------------- Task 2 -------------------
def callback(message):
    """
    Plugs the joint angles into our simulator and prints
    """

    position = message.position
    theta = np.array(np.hstack([position[4:6], position[2:4], position[6:9]]))
    g = task1(theta)
    print(g)


def listener():
    """
    Creates a listener node that subscribes to the positions topic
    and plugs them into our joint simulator
    """

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("robot/joint_states", JointState, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

listener()
# theta = np.array([2*math.pi,2*math.pi,2*math.pi,2*math.pi,2*math.pi,2*math.pi,2*math.pi])
# g = task1(theta)
# q = np.array([.7957, .9965, .3058, 0])
# print q
# print g







