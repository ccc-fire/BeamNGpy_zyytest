import random
import time
from matplotlib import pyplot as plt

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer
import numpy as np
import os

# 获取当前时间
current_time = time.strftime("%Y%m%d%H%M%S", time.localtime())

# 创建文件夹路径
folder_path = r"C:\Users\HP\Desktop\camera_test"
output_folder = os.path.join(folder_path, current_time)

# 创建文件夹
os.mkdir(output_folder)

print("Storing OUTPUT to ", output_folder)

def store_images(camera_name, frame_id, image_color, image_depth, image_annotation):
    # Convert images to arrays
    array_color = np.asarray(image_color.convert('RGB'))
    array_depth = np.asarray(image_depth.convert('L'))
    array_annotation = np.asarray(image_annotation.convert('RGB'))
    # Store

    output_file_color = os.path.join(
        output_folder,
        "_".join([camera_name, str(frame_id), "color"]) + ".npy"
    )
    with open(output_file_color, 'wb') as f:
        np.save(f, array_color)

    output_file_depth = os.path.join(
        output_folder,
        "_".join([camera_name, str(frame_id), "depth"]) + ".npy"
    )
    with open(output_file_depth, 'wb') as f:
        np.save(f, array_depth)

    output_file_annotation = os.path.join(
        output_folder,
        "_".join([camera_name, str(frame_id), "annotation"]) + ".npy"
    )
    with open(output_file_annotation, 'wb') as f:
        np.save(f, array_annotation)

    if frame_id % 5 == 0:
        # Store the images as JPEG
        output_file_color = os.path.join(
            output_folder,
            "_".join([camera_name, str(frame_id), "color"]) + ".jpeg"
        )
        plt.imsave(output_file_color, array_color)

        output_file_depth = os.path.join(
            output_folder,
            "_".join([camera_name, str(frame_id), "depth"]) + ".jpeg"
        )
        plt.imsave(output_file_depth, array_depth)

        output_file_annotation = os.path.join(
            output_folder,
            "_".join([camera_name, str(frame_id), "annotation"]) + ".jpeg"
        )
        plt.imsave(output_file_annotation, array_annotation)

        print("Stored to", output_file_color, output_file_depth, output_file_annotation)


def main():
    random.seed(1703)

    set_up_simple_logging()

    # Plot  3x2 figure
    fig = plt.figure(1, figsize=(10, 5))
    axarr = fig.subplots(3, 3)

    a_colour = axarr[0, 0]
    b_colour = axarr[1, 0]
    c_colour = axarr[2, 0]

    a_depth = axarr[0, 1]
    b_depth = axarr[1, 1]
    c_depth = axarr[2, 1]

    a_annot = axarr[0, 2]
    b_annot = axarr[1, 2]
    c_annot = axarr[2, 2]

    plt.ion()

    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0', user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
    bng = beamng.open(launch=True)

    # Create a scenario in east_coast_usa
    # scenario = Scenario('east_coast_usa', 'tech_test', description='Random driving for research')

    # Set up first vehicle, with two cameras, gforces sensor, lidar, electrical
    # sensors, and damage sensors
    # vehicle_1 = Vehicle('vehicle_1', model='etk800', license='RED', color='Red')

    scenario = Scenario('fresh', 'demo')
    vehicle_1 = Vehicle('vehicle_1', model='etk800', license='vehicle_1')
    scenario.add_vehicle(vehicle_1, pos=(2900, 316, 125.4), rot_quat=(0, 0, 1, 0))

    # # Set up sensors
    # gforces = GForces()
    # electrics = Electrics()
    # damage = Damage()
    # timer = Timer()
    #
    # # Attach them
    # vehicle_1.attach_sensor('gforces', gforces)
    # vehicle_1.attach_sensor('electrics', electrics)
    # vehicle_1.attach_sensor('damage', damage)
    # vehicle_1.attach_sensor('timer', timer)

    # scenario.add_vehicle(vehicle_1, pos=(-426.68, -43.59, 31.11), rot_quat=(0, 0, 1, 0))

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(bng)

    # Start BeamNG and enter the main loop
    try:
        bng.ui.hide_hud()
        bng.settings.set_deterministic(60)  # Set simulator to be deterministic, with 60 Hz temporal resolution

        # Load and start the scenario
        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.pause()

        assert vehicle_1.is_connected()

        # put the cameras on the car
        pos = (0.0, -3, 1)
        direction = (0, -1, 0)
        fov = 120
        resolution = (512, 512)
        front_camera = Camera('front_camera', bng, vehicle_1,
            pos=pos, dir=direction, field_of_view_y=fov, resolution=resolution,
            is_render_colours=True, is_render_depth=True, is_render_annotations=True)

        pos = (0.0, 3, 1.0)
        direction = (0, 1, 0)
        fov = 90
        resolution = (512, 512)
        back_camera = Camera('back_camera', bng, vehicle_1,
            pos=pos, dir=direction, field_of_view_y=fov, resolution=resolution,
            is_render_colours=True, is_render_depth=True, is_render_annotations=True)

        # put the cameras on the scenario
        bird_cam_pos = (2900, 310, 200)
        bird_cam_dir = (0, 0, -1)
        bird_cam_fov = 60
        bird_cam_res = (512, 512) # 坐标:(-428,-59,60)
        bird_view_camera = Camera('bird_view_camera', bng, pos=bird_cam_pos,
                                  dir=bird_cam_dir, field_of_view_y=bird_cam_fov, resolution=bird_cam_res,
                                  is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                                  is_static=True)


        for frame_id in range(200): #
            # throttle = random.uniform(0.0, 1.0)
            # steering = random.uniform(-1.0, 1.0)
            # brake = random.choice([0, 0, 0, 1])
            # vehicle_1.control(throttle=throttle, steering=steering, brake=brake)
            # vehicle_1.ai_set_mode('span')
            # vehicle_1.ai_set_mode('traffic')

            bng.step(20) # 这里是1/3秒, 60代表一秒

            # # Retrieve sensor data and show the camera data.
            # vehicle_1.sensors.poll()
            # sensors = vehicle_1.sensors
            #
            # print('{} seconds passed.'.format(sensors['timer']['time']))


            front_cam_data = front_camera.poll()
            back_cam_data = back_camera.poll()
            bird_view_camera_data = bird_view_camera.poll()

            a_colour.imshow(front_cam_data['colour'].convert('RGB'))
            a_depth.imshow(front_cam_data['depth'].convert('L'))
            a_annot.imshow(front_cam_data['annotation'].convert('RGB'))

            b_colour.imshow(back_cam_data['colour'].convert('RGB'))
            b_depth.imshow(back_cam_data['depth'].convert('L'))
            b_annot.imshow(back_cam_data['annotation'].convert('RGB'))

            c_colour.imshow(bird_view_camera_data['colour'].convert('RGB'))
            c_depth.imshow(bird_view_camera_data['depth'].convert('L'))
            c_annot.imshow(bird_view_camera_data['annotation'].convert('RGB'))

            plt.pause(1.0)


            # 保存
            store_images('front_camera', frame_id, front_cam_data['colour'], front_cam_data['depth'], front_cam_data['annotation'])
            store_images('back_camera', frame_id, back_cam_data['colour'], back_cam_data['depth'], back_cam_data['annotation'])
            store_images('bird_view_camera', frame_id, bird_view_camera_data['colour'], bird_view_camera_data['depth'], bird_view_camera_data['annotation'])
    finally:
        bng.close()


if __name__ == '__main__':
    main()