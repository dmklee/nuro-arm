import cv2
from nuro_arm.camera.camera import Camera
from nuro_arm.camera.gui import ShowCubes

def show_cubes():
    cam = Camera()
    modifier = ShowCubes(cam.configs, include_id=True)
    cam.gui.show(modifier_fns=[modifier])

if __name__ == "__main__":
    show_cubes()
