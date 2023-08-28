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


def store_images(camera_name, frame_id, image_color, image_depth, image_annotation):
    # Convert images to arrays
    array_color = np.asarray(image_color.convert('RGB'))
    array_depth = np.asarray(image_depth.convert('L'))
    array_annotation = np.asarray(image_annotation.convert('RGB'))
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
        sleep(3)

        cam1_id = []
        lock = threading.Lock()
        # cam2_id = []
        # cam3_id = []
        # 事件对象
        event = threading.Event()
        # 线程安全队列用于在线程间传递参数
        cam1_id_queue = queue.Queue()
        start = time.time()
        def cam1_s():
            for _ in range(100):
                with lock:
                    cam1_rid = cam1.send_ad_hoc_poll_request()
                # with lock:
                cam1_id_queue.put(cam1_rid)
                # cam2_id.append(id)
                # cam3_id.append(id)
                while(cam1.is_ad_hoc_poll_request_ready(cam1_rid)==False):
                    sleep(0.1)
                with lock:
                    print('Is ad-hoc request complete? ', cam1.is_ad_hoc_poll_request_ready(cam1_rid), cam1_rid)
            cam1_id_queue.put(None)
            t1 = time.time()
            print(f"send: {t1 - start}")
            # event.set()

        def cam1_c():
            while True:
                cam1_id = cam1_id_queue.get()
                if cam1_id is None:
                    break
                with lock:
                    images1 = cam1.collect_ad_hoc_poll_request(cam1_id)  # Display the image data from request 2.
                store_images('cam1', cam1_id, images1['colour'], images1['depth'], images1['annotation'])
                cam1_id_queue.task_done()
            t2 = time.time()
            print(f"collect: {t2 - start}")
            # event.set()
        threads = []
        task1 = threading.Thread(target=cam1_s)
        threads.append(task1)
        task1.start()

        task2 = threading.Thread(target=cam1_c)
        threads.append(task2)
        task2.start()

        # event.wait()
        # 等待所有线程结束
        for thread in threads:
            thread.join()


        # cam1_id_queue.join()
        # event.wait()

        # 所有线程已结束，继续执行后续操作
        print("All threads have finished.")



        print("test__")

        # print('Ad-hoc poll request test.  The next 6 images come from ad-hoc requests sent to 2 camera sensors. They should contain scene data as before.')
        # # send a request on the shared memory sensor (data should come back over the socket, either way).
        # request_id_1 = cam1.send_ad_hoc_poll_request()
        # request_id_2 = cam2.send_ad_hoc_poll_request()      # send a request on the non shared memory sensor.
        # request_id_3 = cam3.send_ad_hoc_poll_request()
        # print('Ad-hoc poll requests sent. Unique request Id numbers: ', request_id_1, request_id_2, request_id_3)
        # for frame_id in range(3):
        #     sleep(3)
        #     print('Is ad-hoc request 1 complete? ', cam1.is_ad_hoc_poll_request_ready(request_id_1))
        #     print('Is ad-hoc request 2 complete? ', cam2.is_ad_hoc_poll_request_ready(request_id_2))
        #     print('Is ad-hoc request 3 complete? ', cam1.is_ad_hoc_poll_request_ready(request_id_3))
        #
        #     images1 = cam1.collect_ad_hoc_poll_request(request_id_1)        # Display the image data from request 1.
        #     store_images('cam1', frame_id, images1['colour'], images1['depth'], images1['annotation'])
        #     # ax11.imshow(np.asarray(images1['colour'].convert('RGB')))
        #     # plt.pause(0.1)
        #     # ax12.imshow(np.asarray(images1['annotation'].convert('RGB')))
        #     # plt.pause(0.1)
        #
        #     images2 = cam2.collect_ad_hoc_poll_request(request_id_2)        # Display the image data from request 2.
        #     store_images('cam2', frame_id, images2['colour'], images2['depth'], images2['annotation'])
        #
        #     images3 = cam3.collect_ad_hoc_poll_request(request_id_3)        # Display the image data from request 2.
        #     store_images('cam3', frame_id, images3['colour'], images3['depth'], images3['annotation'])
        #     # ax21.imshow(np.asarray(images2['colour'].convert('RGB')))
        #     # plt.pause(0.1)
        #     # ax22.imshow(np.asarray(images2['annotation'].convert('RGB')))
        #     # plt.pause(0.1)
        #
        # # Remove all the camera sensors from the simulation.
        # cam1.remove()
        # cam2.remove()
        # cam3.remove()
        #
        # sleep(3)
        # print('Camera test complete.')


if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppDate\Local\BeamNG.drive\0.28')
    poll_camera(beamng=beamng)
