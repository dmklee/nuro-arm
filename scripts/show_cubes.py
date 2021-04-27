import cv2
from neu_ro_arm.camera.gui import ShowCubes

def show_cubes():
    cam = Camera()
    modifier = ShowCubes(cam.unpack_configs(False), include_id=True)
    cam.gui.show(modifier_fns=[modifier])

if __name__ == "__main__":
    show_cubes()
