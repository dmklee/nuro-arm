import numpy as np
import cv2
import os
import nuro_arm

# all positional units are in meters, since this is the unit in the urdf
TVEC_WORLD2RIGHTFOOT = np.array((0.091, -0.0635, 0.0))

CALIBRATION_GRIDSIZE = 0.020
CALIBRATION_GRIDSHAPE = (9,7)

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

URDF_DIR = os.path.join(os.path.dirname(nuro_arm.__file__),
                        'assets/urdf')
XARM_CONFIG_FILE = os.path.join(os.path.dirname(nuro_arm.__file__),
                                'robot/configs.npy')
CAMERA_CONFIG_FILE = os.path.join(os.path.dirname(nuro_arm.__file__),
                                  'camera/configs.npy')

# this is the measure of the black square that contains the pattern
TAG_SIZE = 0.0188976

CUBE_SIZE = 0.0254
NORM_CUBE_VERTICES = 0.5 * np.array((( 1, 1, 1),
                                     ( 1,-1, 1),
                                     ( 1, 1,-1),
                                     ( 1,-1,-1),
                                     (-1, 1, 1),
                                     (-1,-1, 1),
                                     (-1, 1,-1),
                                     (-1,-1,-1)),
                                    dtype=np.float32)
CUBE_EDGES = ((0,1),(0,2),(0,4),(1,3),(1,5),(2,3),
              (2,6),(3,7),(4,5),(4,6),(5,7),(6,7))


# used to place camera in simulator if not using real camera
DEFAULT_CAM_POSE_MTX = np.array([
    [-0.49600867, -0.68981786,  0.5273772 ,  0.00390031],
    [-0.8682925 ,  0.39864808, -0.29520813,  0.08927076],
    [-0.00659807, -0.60434346, -0.79669658,  0.25612385],
    [0., 0., 0., 1.0],
])

CAM_MTX = np.array([[652.31611616,   0.        , 313.14329843],
                    [  0.        , 651.46937532, 253.54561384],
                    [  0.        ,   0.        ,   1.        ]])
CAM_DIST_COEFFS = np.array([[-0.45025704,  0.26170012, -0.00424988,
                             0.00268179, -0.09056137]])

FRAME_RATE = 20

##############################################
# common gripper states
##############################################
GRIPPER_CLOSED = 0
GRIPPER_OPENED = 1

##############################################
# useful pitch-roll tuples for RobotArm.move_hand_to
##############################################
TOP_DOWN_GRASP = np.array((-np.pi, 0.,))
STANDARD_GRASP = np.array((-2.6, 0.))
HORIZONTAL_GRASP = np.array((-np.pi/2, 0.))
