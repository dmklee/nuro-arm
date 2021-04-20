from neu_ro_arm.camera.camera import Camera, Capturer, GUI

def determine_camera_id():
    print('Performing scan over available cameras...')
    cap = Capturer()
    gui = GUI(cap)

    max_camera_id = 5
    for c_id in range(max_camera_id):
        is_valid = cap.set_camera_id(c_id)
        if is_valid:
            name = f"Camera{c_id}: is this the camera you want to use? [y/n]"
            k = gui.show(window_name=name,
                          exit_keys=[ord('y'), ord('n')]
                             )
            if k == ord('y'):
                print(f'Video capture enabled with camera{c_id}.')
                cap.release()
                return c_id
        # else:
            # print(f'  Camera{c_id} not available.')
    print('[ERROR] No other cameras were found.')
    cap.release()
    return None

def calc_camera_location(camera_id=None):
    camera = Camera(camera_id)

    print()
    print('Place checkerboard pattern in front of robot such that the semicircles'
          ' align with the suction cups')
    input('   hit enter when ready...')

    print()
    print('Position camera such that the view includes the checkerboard,'
          ' this may require adjusting the bolts on the top camera assembly.')
    print('Tighten collars firmly once camera view is good. We do not want'
          ' the camera to move around after we have calibrated its location.')
    camera.gui.show(window_name='Hit ESC once camera view is good.')

    ret, location_data = camera.calc_location()
    if ret:
        print()
        print('[SUCCESS] Camera location has been identified!')
        rvec, tvec, world2cam, cam2world = location_data
        camera._update_config_file({'camera_id': camera_id,
                                    'rvec' : rvec,
                                    'tvec' : tvec,
                                    'world2cam' : world2cam,
                                    'cam2world' : cam2world
                                   })
        print('Location information has been saved to config file.')
    else:
        print('[ERROR] Checkerboard pattern was not identified. Please try again.')

if __name__ == "__main__":
    camera_id = determine_camera_id()
    if camera_id is not None:
        calc_camera_location(camera_id)
