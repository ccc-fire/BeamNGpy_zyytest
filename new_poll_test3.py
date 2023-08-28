import random
import numpy as np
from matplotlib import pyplot as plt
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer
from time import sleep

import time
import os
import threading
import queue
# 获取当前时间
current_time = time.strftime("%Y%m%d%H%M%S", time.localtime())

# 创建文件夹路径
folder_path = r"C:\Users\HP\Desktop\camera_test"
output_folder = os.path.join(folder_path, current_time)

# 创建文件夹
os.mkdir(output_folder)

print("Storing OUTPUT to ", output_folder)


fig1 = plt.figure(figsize=(5, 5))
ax1 = fig1.add_subplot(111)
plt.ion()
fig2 = plt.figure(figsize=(5, 5))
ax2 = fig2.add_subplot(111)
plt.ion()


def store_images(camera_name, frame_id, image_color):
# def store_images(camera_name, frame_id, image_color, image_depth, image_annotation):
    # Convert images to arrays
    array_color = np.asarray(image_color.convert('RGB'))
    # array_depth = np.asarray(image_depth.convert('L'))
    # array_annotation = np.asarray(image_annotation.convert('RGB'))
    # Store the arrays into target folder using camera_name and frame_id as identifiers

    # output_file_color = os.path.join(
    #     output_folder,
    #     "_".join([camera_name, str(frame_id), "color"]) + ".npy"
    # )
    # with open(output_file_color, 'wb') as f:
    #     np.save(f, array_color)
    #
    # output_file_depth = os.path.join(
    #     output_folder,
    #     "_".join([camera_name, str(frame_id), "depth"]) + ".npy"
    # )
    # with open(output_file_depth, 'wb') as f:
    #     np.save(f, array_depth)
    #
    # output_file_annotation = os.path.join(
    #     output_folder,
    #     "_".join([camera_name, str(frame_id), "annotation"]) + ".npy"
    # )
    # with open(output_file_annotation, 'wb') as f:
    #     np.save(f, array_annotation)

    if frame_id % 1 == 0:
        # Store the images as JPEG
        output_file_color = os.path.join(
            output_folder,
            "_".join([camera_name, str(frame_id), "color"]) + ".jpeg"
        )
        plt.imsave(output_file_color, array_color)

        # output_file_depth = os.path.join(
        #     output_folder,
        #     "_".join([camera_name, str(frame_id), "depth"]) + ".jpeg"
        # )
        # plt.imsave(output_file_depth, array_depth)
        #
        # output_file_annotation = os.path.join(
        #     output_folder,
        #     "_".join([camera_name, str(frame_id), "annotation"]) + ".jpeg"
        # )
        # plt.imsave(output_file_annotation, array_annotation)

        # print("Stored to", output_file_color, output_file_depth, output_file_annotation)


        print("Stored to", output_file_color)


def poll_camera(beamng: BeamNGpy):
    with beamng as bng:
        vehicle = Vehicle('ego_vehicle', model='etki', license='PYTHON', color='Green')  # Create a vehicle.

        scenario = Scenario('gridmap_v2', 'camera_test', description='Testing the camera sensor')  # Create a scenario.
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle, pos=(-122, -168, 100), rot_quat=(0, 0, 1, 0))
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        # Create some camera sensors in the simulation.
        print('Camera test start.')

        cam1 = Camera('camera1', bng, vehicle, is_using_shared_memory=True, pos=(-5, 0, 1), dir=(1, 0, 0),
                      field_of_view_y=70,
                      is_render_annotations=True, is_render_depth=True, requested_update_time=0.3, update_priority=0.5,
                      near_far_planes=(0.1, 1000),
                      resolution=(1920, 1080))  # 轮询时间：set_requested_update_time(0.3)，更新优先级：update_priority=0.5,
        # 设置最大待请求处理数
        cam1.set_max_pending_requests(20)

        # facing towards the vehicle.
        cam2 = Camera('camera2', bng, vehicle, is_using_shared_memory=False, pos=(5, 0, 1), dir=(-1, 0, 0),
                      field_of_view_y=70,
                      is_render_annotations=True, is_render_depth=True,
                      near_far_planes=(0.1, 1000), resolution=(512, 512))

        # Create a camera sensor which has an oblique angle to the world
        cam3 = Camera('camera3', bng, vehicle, is_using_shared_memory=False, pos=(0, 5, 1), dir=(0, -1, 0),
                      up=(1, 0, 1), field_of_view_y=70,
                      is_render_annotations=True, is_render_depth=True,
                      near_far_planes=(0.1, 1000), resolution=(512, 512))

        sleep(1)


        def cam1_c():

            cam1_rid = cam1.send_ad_hoc_poll_request()
            while (cam1.is_ad_hoc_poll_request_ready(cam1_rid) == False):
                sleep(0.1)
            images1 = cam1.collect_ad_hoc_poll_request(cam1_rid)
            # store_images('cam1', cam1_rid, images1['colour'])
            ax1.imshow(np.asarray(images1['colour'].convert('RGB')))
            plt.pause(0.1)

        def cam2_c():
            cam2_rid = cam2.send_ad_hoc_poll_request()
            while (cam2.is_ad_hoc_poll_request_ready(cam2_rid) == False):
                sleep(0.1)
            images2 = cam2.collect_ad_hoc_poll_request(cam2_rid)
            # store_images('cam1', cam1_rid, images1['colour'])
            ax2.imshow(np.asarray(images2['colour'].convert('RGB')))
            plt.pause(0.1)

        threads = []
        task1 = threading.Thread(target=cam1_c)
        threads.append(task1)
        task1.start()

        task2 = threading.Thread(target=cam2_c)
        threads.append(task2)
        task2.start()

        # event.wait()
        # 等待所有线程结束
        for thread in threads:
            thread.join()


        print("All threads have finished.")


if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppDate\Local\BeamNG.drive\0.28')
    poll_camera(beamng=beamng)
