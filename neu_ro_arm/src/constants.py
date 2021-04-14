import numpy as np
import cv2

tvec_world2rightfoot = np.array((63.5, 90.5, 0)) # mm

calibration_gridsize = 20 # mm
calibration_gridshape = (7,9)

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()
tag_size = 20 # mm

cube_size = 25.4 #mm
cube_vertices = 0.5 * cube_size * np.array((( 1, 1, 1),
                                            ( 1,-1, 1),
                                            ( 1, 1,-1),
                                            ( 1,-1,-1),
                                            (-1, 1, 1),
                                            (-1,-1, 1),
                                            (-1, 1,-1),
                                            (-1,-1,-1)),
                                           dtype=np.float32)
cube_edges = ((0,1),(0,2),(0,4),(1,3),(1,5),(2,3),
              (2,6),(3,7),(4,5),(4,6),(5,7),(6,7))

default_cam_pos = np.zeros(3)
default_cam_rot = np.zeros(3)

frame_rate = 20

