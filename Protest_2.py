import random
import time
from time import sleep
from matplotlib import pyplot as plt

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer, Radar, AdvancedIMU, Lidar
import numpy as np
import os
import csv
import open3d as o3d
import multiprocessing
from datetime import datetime
import threading

from multiprocessing import Pool

def store_camera(camera_name, frame_id, image_color, image_depth, image_annotation, image_instance, output_folder):
    # Convert images to arrays
    array_color = np.asarray(image_color.convert('RGB'))
    array_depth = np.asarray(image_depth.convert('L'))
    array_annotation = np.asarray(image_annotation.convert('RGB'))
    array_instance = np.asarray(image_instance.convert('RGB'))

    output_file_color = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "color"]) + ".jpeg")
    plt.imsave(output_file_color, array_color)

    output_file_depth = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "depth"]) + ".jpeg")
    plt.imsave(output_file_depth, array_depth)

    output_file_annotation = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "annotation"]) + ".jpeg")
    plt.imsave(output_file_annotation, array_annotation)

    output_file_instance = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "instance"]) + ".jpeg")
    plt.imsave(output_file_instance, array_instance)

    print("Stored to", output_file_color, output_file_depth, output_file_annotation, output_file_instance)


def collect_camera(camera_name, frame_id, output_folder):
    # now = datetime.now()
    # frame_id = now.strftime("%Y-%m-%d-%H-%M-%S%f")
    print("start collect_________________")
    # while lock.locked():
    #     continue
    camera_data = camera_name.get_full_poll_request()
    store_camera(str(camera_name.name), frame_id, camera_data['colour'], camera_data['depth'],
                 camera_data['annotation'], camera_data['instance'], output_folder=output_folder)


# def main():
if __name__ == '__main__':

    # 获取当前时间
    current_time = time.strftime("%Y%m%d%H%M%S", time.localtime())
    # 创建文件夹路径
    folder_path = r"C:\Users\HP\Desktop\camera_test"
    output_folder = os.path.join(folder_path, current_time)
    # 创建文件夹
    os.mkdir(output_folder)
    print("Storing OUTPUT to ", output_folder)
    random.seed(1703)

    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
    bng = beamng.open(launch=True)

    # 网格地图

    scenario = Scenario('gridmap_v2', 'demo')
    vehicle_1 = Vehicle('vehicle_1', model='etk800', license='vehicle_1')
    origin = (-122, -188, 100.2)
    # origin = (2921, 316, 125.4)
    scenario.add_vehicle(vehicle_1, pos=origin, rot_quat=(0, 0, 1, 0))

    # 网格地图
    scenario.make(bng)

    # Start BeamNG and enter the main loop
    try:
        bng.ui.hide_hud()
        bng.settings.set_deterministic(60)  # Set simulator to be deterministic, with 60 Hz temporal resolution

        # Load and start the scenario
        bng.scenario.load(scenario)
        bng.scenario.start()
        assert vehicle_1.is_connected()

        # set camera _____________________________________________________________
        resolution = (1920, 1080)

        camera_front = Camera('camera_front', bng, vehicle_1,
                              pos=(0.000244140625, -0.049774169921875, 1.465557861328125), dir=(0, -1, 0),
                              field_of_view_y=120, resolution=resolution,
                              is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                              is_render_instance=True, is_using_shared_memory=True, is_visualised=True)
        bird_view_camera = Camera('bird_view_camera', bng, pos=(-120, -180, 120),
                                  dir=(0, 0, -1), field_of_view_y=60, resolution=resolution,
                                  is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                                  is_render_instance=True, is_static=True, is_using_shared_memory=True, is_visualised=True)

        bng.traffic.spawn(3)
        sleep(2)
        print("开始————————")
        start_time = time.time()
        tasks = [bird_view_camera, camera_front]

        for frame_id in range(2):
            p = []
            for task in tasks:
                p1 = multiprocessing.Process(target=collect_camera, args=[task, frame_id, output_folder])
                p1.start()
                p.append(p1)
                print("1")
            for p1 in p:
                print("开始结束一个线程")
                p1.join()
                print("结束一个线程")
            t = time.time()
            print(f"第{frame_id}轮采集完成，用时{t - start_time}秒")
        end_time = time.time()
        print(f"已经采集 {end_time - start_time} seconds")

    finally:
        bng.close()

# if __name__ == '__main__':
#     main()