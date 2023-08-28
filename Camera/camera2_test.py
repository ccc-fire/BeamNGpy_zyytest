import random
import time
from matplotlib import pyplot as plt

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer
import numpy as np
import os

BNG_HOME = r'D:\RJY\BeamNG.tech.v0.28.2.0'
BNG_USER = r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28'

beamng = BeamNGpy('localhost', 64256, home=BNG_HOME, user=BNG_USER)
bng = beamng.open(launch=True)

vehicle_1 = Vehicle('vehicle_1', model='etk800', license='vehicle_1')     # 创建本车
scenario = Scenario('fresh', 'fresh')
scenario.add_vehicle(vehicle_1, pos=(2900, 316, 125.4), rot_quat=(0, 0, 1, 0))
scenario.make(bng)

# 获取当前时间
current_time = time.strftime("%Y%m%d%H%M%S", time.localtime())

# 创建文件夹路径
folder_path = r"C:\Users\HP\Desktop\camera_test"
output_folder = os.path.join(folder_path, current_time)

# 创建文件夹
os.mkdir(output_folder)

print("Storing OUTPUT to ", output_folder)

def store_images(camera_name, frame_id, image_color):
    # Convert images to arrays
    array_color = np.asarray(image_color.convert('RGB'))
    # Store

    output_file_color = os.path.join(
        output_folder,
        "_".join([camera_name, str(frame_id), "color"]) + ".npy"
    )
    with open(output_file_color, 'wb') as f:
        np.save(f, array_color)


    if frame_id % 5 == 0:
        # Store the images as JPEG
        output_file_color = os.path.join(
            output_folder,
            "_".join([camera_name, str(frame_id), "color"]) + ".jpeg"
        )
        plt.imsave(output_file_color, array_color)

        print("Stored to", output_file_color)

def main():


    bng.ui.hide_hud()
    bng.settings.set_deterministic(60)  # Set simulator to be deterministic, with 60 Hz temporal resolution


    # Load and start the scenario
    bng.scenario.load(scenario)
    bng.vehicles.spawn(3)
    bng.scenario.start()
    bng.control.pause()


    # put the cameras on the car

    camera_1 = Camera('camera_1', bng, vehicle_1,
                          pos=(0.0, -3, 1), dir=(0, -1, 0), field_of_view_y=120, resolution=(512, 512),
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True, is_render_instance=True)
    camera_2 = Camera('camera_2', bng, vehicle_1,
                          pos=(0.0, -3, 1), dir=(0, -1, 0), field_of_view_y=120, resolution=(512, 512),
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True, is_render_instance=True)
    camera_3 = Camera('camera_3', bng, vehicle_1,
                          pos=(0.0, -3, 1), dir=(0, -1, 0), field_of_view_y=120, resolution=(512, 512),
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True, is_render_instance=True)
    camera_4 = Camera('camera_4', bng, vehicle_1,
                          pos=(0.0, -3, 1), dir=(0, -1, 0), field_of_view_y=120, resolution=(512, 512),
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True, is_render_instance=True)
    camera_5 = Camera('camera_5', bng, vehicle_1,
                          pos=(0.0, -3, 1), dir=(0, -1, 0), field_of_view_y=120, resolution=(512, 512),
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True, is_render_instance=True)
    camera_6 = Camera('camera_6', bng, vehicle_1,
                          pos=(0.0, -3, 1), dir=(0, -1, 0), field_of_view_y=120, resolution=(512, 512),
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True, is_render_instance=True)

    # put radar on the car
    # # NOTE: Create sensor after scenario has started.
    # RANGE_MIN = 0.1
    # RANGE_MAX = 100.0
    # RESOLUTION = (200, 200)
    # FOV = 70
    # radar = Radar('radar1', bng, vehicle,
    #     requested_update_time=0.01,
    #     pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1),
    #     resolution=RESOLUTION, field_of_view_y=FOV, near_far_planes=(RANGE_MIN, RANGE_MAX),
    #     range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12, range_min_cutoff=0.5, range_direct_max_cutoff=RANGE_MAX)
    #
    # for _ in range(1000):
    #     sleep(5)
    #     readings_data = radar.poll() # Fetch the latest readings from the sensor, over either shared memory or the lua socket.
    #     radar.plot_data(readings_data, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200)
    #
    # radar.remove()





    plt.ion()
    # for frame_id in range(11):
    while 1:
        bng.step(20)
        cam1_data = camera_1.get_full_poll_request() # colour, annotation, instance, depth

        # front_cam_data = front_camera.poll()
        print(cam1_data)
        print(camera_1.get_position())
        plt.imshow(cam1_data['instance'])
        # plt.imshow(front_cam_data['colour'].convert('RGB'))
        plt.pause(0.01)


    #

#    assert vehicle_1.is_connected() # 判断是否连接









if __name__ == '__main__':
    main()