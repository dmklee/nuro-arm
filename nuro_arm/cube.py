import pybullet as pb
import os
from PIL import Image
import cv2
import numpy as np

from nuro_arm.constants import URDF_DIR, CUBE_SIZE

class Cube:
    def __init__(self,
                 pos,
                 rot=[0,0,0,1],
                 size=CUBE_SIZE,
                 pb_client=0,
                 tag_id=None,
                 rgba=(0,1,0,1)
                ):
        self.size = size
        self.client = pb_client

        cube_urdf_path = os.path.join(URDF_DIR, 'cube.urdf')
        self.id = pb.loadURDF(cube_urdf_path,
                              pos,
                              rot,
                              globalScaling=self.size,
                              physicsClientId=pb_client)
        if tag_id is not None:
            self.add_aruco_tag(tag_id, rgba)
        else:
            pb.changeVisualShape(self.id, -1,
                                 rgbaColor=rgba,
                                 physicsClientId=self.client)

    def reset_pose(self, pos, rot=[0,0,0,1]):
        '''Reset pose of cube to desired position and rotation, velocity is 0

        Parameters
        ----------
        pos : array_like
            x,y,z position
        rot : array_like, default to [0,0,0,1]
            rotation as a quaternion
        '''
        pb.resetBasePositionAndOrientation(self.id, pos, rot, self.client)
        pb.resetBaseVelocity(self.id, [0,0,0],[0,0,0], self.client)

    def get_pose(self):
        '''Get pose of cube

        Returns
        -------
        pos : array_like
            x,y,z position
        rot : array_like, default to [0,0,0,1]
            rotation as a quaternion
        '''
        pos, rot = pb.getBasePositionAndOrientation(self.id,
                                                    physicsClientId=self.client)
        return pos, rot

    def get_velocity(self):
        '''Get pose of cube

        Returns
        -------
        lin_vel : array_like
            linear velocity, length 3
        ang_vel : array_like
            angular velocity, length 3
        '''
        lin_vel, ang_vel = pb.getBaseVelocity(self.id, physicsClientId=self.client)
        return lin_vel, ang_vel

    def get_position(self):
        '''Return x,y,z position of the cube
        '''
        return self.get_pose()[0]

    def get_rotation(self):
        '''Return rotation of cube as a quaternion
        '''
        return self.get_pose()[1]

    def add_aruco_tag(self,
                      tag_id=0,
                      rgba=(0,1,0,1),
                      aruco_dict_id=cv2.aruco.DICT_4X4_50,
                     ):
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
        tag = cv2.aruco.drawMarker(aruco_dict,
                                   tag_id,
                                   aruco_dict.markerSize+2,
                                   borderBits=1)
        tag = np.pad(tag, (1,1), mode='constant', constant_values=255)

        tag_size = tag.shape[0]
        texture_img = np.zeros((4*tag_size, 4*tag_size,3),dtype=np.uint8)
        texture_img[...,:] = [255*c for c in rgba[:3]]

        # flip image so aruco tag appears properly
        texture_img[-tag_size:, -tag_size:,:] = tag[::-1,:,np.newaxis]

        img = Image.fromarray(texture_img, mode="RGB")
        tmp_fname = f'tmp_cube_tex{tag_id}.png'
        img.save(tmp_fname)
        texture_id = pb.loadTexture(tmp_fname)
        pb.changeVisualShape(self.id, -1,
                             rgbaColor=(1,1,1,rgba[-1]),
                             textureUniqueId=texture_id,
                             physicsClientId=self.client)
        os.remove(tmp_fname)

    def delete(self):
        pb.removeBody(self.id, physicsClientId=self.client)

    def __hash__(self):
        return self.id
