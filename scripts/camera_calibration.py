import cv2
import time

from neu_ro_arm.camera.camera import Camera
from neu_ro_arm.camera.camera_utils import calc_distortion_matrix, convert_gray
from neu_ro_arm.constants import calibration_gridshape

def show_chessboard(original, canvas):
    gray = convert_gray(original)
    ret, corners = cv2.findChessboardCorners(gray, calibration_gridshape, None)
    if ret:
        canvas = cv2.drawChessboardCorners(canvas, calibration_gridshape,
                                           corners, ret)
    return canvas

def take_pictures(camera):
    images = []
    camera.gui.add_modifiers(show_chessboard)
    camera.show_feed()
    print('Press s to take picture. ESC to exit')
    while True:
        k = camera.gui.get_last_keypress()
        if k == ord('s'):
            print(f'img{len(images)} taken.')
            images.append(camera.get_image())
        elif k == 27:
            break
        time.sleep(1./camera.frame_rate)
    camera.hide_feed()
    good_images = reject_no_grid(images)
    print(f'Rejected {len(images)-len(good_images)} images that did not capture full grid')
    return good_images

def reject_no_grid(images):
    '''Rejects any images where the grid cannot be identified'''
    gw, gh = calibration_gridshape

    good_images = []
    for img in images:
        gray = convert_gray(img)

        ret, corners = cv2.findChessboardCorners(gray, (gw,gh), None)
        if ret:
            good_images.append(img)

    return good_images

if __name__ == "__main__":
    camera = Camera(2)
    imgs = take_pictures(camera)

    mtx, newcameramtx, roi, dist = calc_distortion_matrix(imgs, verbose=True)


