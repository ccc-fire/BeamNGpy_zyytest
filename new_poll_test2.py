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
# # Plotting code setting up a 3x2 figure
# fig = plt.figure(1, figsize=(10, 5))
# axarr = fig.subplots(2, 3)
#
# ax11 = axarr[0, 0]
# ax12 = axarr[1, 0]
# ax21 = axarr[0, 1]
# ax22 = axarr[1, 1]
# ax31 = axarr[0, 2]
# ax32 = axarr[1, 2]


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

        # Create a camera sensor which uses shared memory. This is placed to the left of the vehicle, facing towards the vehicle.
        cam1 = Camera('camera1', bng, vehicle, is_using_shared_memory=True, pos=(-5, 0, 1), dir=(1, 0, 0),
                      field_of_view_y=70,
                      is_render_annotations=True, is_render_depth=True, requested_update_time=0.3, update_priority=0.5,
                      near_far_planes=(0.1, 1000),
                      resolution=(1920, 1080))  # 轮询时间：set_requested_update_time(0.3)，更新优先级：update_priority=0.5,
        # 设置最大待请求处理数
        cam1.set_max_pending_requests(20)

        # Create a camera sensor which does not use shared memory (data will be send back across the socket). This is placed to the right of the vehicle,
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
        vehicle.ai.set_mode('span')

        cam1_rid = cam1.send_ad_hoc_poll_request()
        while (cam1.is_ad_hoc_poll_request_ready(cam1_rid) == False):
            sleep(0.1)
        images1 = cam1.collect_ad_hoc_poll_request(cam1_rid)
        store_images('cam1', cam1_rid, images1['colour'])

        # cam1_id = []
        # start = time.time()
        # for _ in range(2):
        #     t0 = time.time()
        #     cam1_rid = cam1.send_ad_hoc_poll_request()
        #     # with lock:
        #     cam1_id.append(cam1_rid)
        #     # cam2_id.append(id)
        #     # cam3_id.append(id)
        #     while (cam1.is_ad_hoc_poll_request_ready(cam1_rid) == False):
        #         sleep(0.1)
        #     t = time.time() - t0
        #     print('Is ad-hoc request complete? ', cam1_rid, {t})
        # t1 = time.time()
        # print(f"send{20}: {t1 - start}")
        # # bng.ui.hide_hud()
        # def sc(id):
        #     images1 = cam1.collect_ad_hoc_poll_request(id)
        #     store_images('cam1', id, images1['colour'])
        #
        # p = []
        # for cam1_id in cam1_id:
        #     p1 = threading.Thread(target=sc, args=cam1_id)
        #     p1.start()
        #     p.append(p1)
        # for p1 in p:
        #     p1.join()
        #     print("finish p1")


        # for cam1_id in cam1_id:
        #     t0 = time.time()
        #     images1 = cam1.collect_ad_hoc_poll_request(cam1_id)  # Display the image data from request 2.
        #     # store_images('cam1', cam1_id, images1['colour'], images1['depth'], images1['annotation'])
        #     store_images('cam1', cam1_id, images1['colour'])
        #     print(f"save: {time.time()-t0}")

        # t2 = time.time()
        # print(f"collect: {t2 - start}")

        print("All threads have finished.")





if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppDate\Local\BeamNG.drive\0.28')
    poll_camera(beamng=beamng)
