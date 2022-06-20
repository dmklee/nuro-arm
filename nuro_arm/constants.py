import numpy as np
import os
import nuro_arm

# all positional units are in meters, since this is the unit in the urdf
TVEC_WORLD2RIGHTFOOT = np.array((0.091, -0.0635, 0.0))

CALIBRATION_GRIDSIZE = 0.020 # 2 cm
CALIBRATION_GRIDSHAPE = (9, 7)

URDF_DIR = os.path.join(os.path.dirname(nuro_arm.__file__),
                        'assets/urdf')
IMAGES_DIR = os.path.join(os.path.dirname(nuro_arm.__file__),
                        'images')
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

CAM_RESOLUTION_WIDTH = 640
CAM_RESOLUTION_HEIGHT = 480
CAM_MTX = np.array([[669.69257956,   0.       , 322.45184341],
                    [  0.        , 669.4132531, 266.73265785],
                    [  0.        ,   0.       ,   1.        ]])
CAM_DIST_COEFFS = np.array([[-4.17660678e-01, 5.12390537e-02, -2.35830953e-03,
                             -2.73582069e-04, 3.05198891e-01]])
NEW_CAM_MTX = np.array([[560.95526123,   0.        , 321.88958533],
                        [  0.        , 562.58758545, 266.36252749],
                        [  0.        ,   0.        ,   1.        ]])
NEW_CAM_ROI = np.array((18, 25, 602, 429))

FRAME_RATE = 20

##############################################
# useful pitch-roll tuples for RobotArm.move_hand_to
##############################################
TOP_DOWN_GRASP = np.array((-np.pi, 0.,))
STANDARD_GRASP = np.array((-2.6, 0.))
HORIZONTAL_GRASP = np.array((-np.pi/2, 0.))
