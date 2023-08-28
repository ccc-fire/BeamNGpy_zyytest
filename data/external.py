from multiprocessing import Process, Queue
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera
import time
from matplotlib import pyplot as plt
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

def store_camera(camera_name, frame_id, image_color, image_depth, image_annotation, image_instance):

    # Convert images to arrays
    array_color = np.asarray(image_color.convert('RGB'))
    array_depth = np.asarray(image_depth.convert('L'))
    array_annotation = np.asarray(image_annotation.convert('RGB'))
    array_instance = np.asarray(image_instance.convert('RGB'))
    # Store

    output_file_color = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "color"]) + ".npy")
    with open(output_file_color, 'wb') as f:
        np.save(f, array_color)

    output_file_depth = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "depth"]) + ".npy")
    with open(output_file_depth, 'wb') as f:
        np.save(f, array_depth)

    output_file_annotation = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "annotation"]) + ".npy")
    with open(output_file_annotation, 'wb') as f:
        np.save(f, array_annotation)

    output_file_annotation = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "instance"]) + ".npy")
    with open(output_file_annotation, 'wb') as f:
        np.save(f, array_instance)


    if frame_id % 1 == 0:
        # Store the images as JPEG
        output_file_color = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "color"]) + ".jpeg")
        plt.imsave(output_file_color, array_color)

        output_file_depth = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "depth"]) + ".jpeg")
        plt.imsave(output_file_depth, array_depth)

        output_file_annotation = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "annotation"]) + ".jpeg")
        plt.imsave(output_file_annotation, array_annotation)

        output_file_instance = os.path.join(output_folder, "_".join([camera_name, str(frame_id), "instance"]) + ".jpeg")
        plt.imsave(output_file_instance, array_instance)

        # print("Stored to", output_file_color, output_file_depth, output_file_annotation, output_file_instance)

def main():
    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0', user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
    bng = beamng.open(launch=True)
    scenario = Scenario('fresh', 'demo')
    vehicle_1 = Vehicle('vehicle_1', model='etk800', license='vehicle_1')
    scenario.add_vehicle(vehicle_1, pos=(2900, 316, 125.4), rot_quat=(0, 0, 1, 0))
    scenario.make(bng)

    try:
        # Load and start the scenario
        bng.scenario.load(scenario)
        bng.scenario.start()

        # put the cameras on the scenario
        bird_view_camera = Camera('bird_view_camera', bng, pos=(2900, 310, 200),
                                  dir=(0, 0, -1), field_of_view_y=60, resolution=(1920, 1080),
                                  is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                                  is_render_instance=True, is_static=True)
        for frame_id in range(100):

            start_time = time.time()

            bird_view_camera_data = bird_view_camera.get_full_poll_request()
            end1_time = time.time()
            print(f"start: {start_time}, end:{end1_time}, poll_totol {end1_time - start_time} seconds")

            store_camera('bird_view_camera', frame_id, bird_view_camera_data['colour'], bird_view_camera_data['depth'],
                         bird_view_camera_data['annotation'], bird_view_camera_data['instance'])

            end2_time = time.time()
            print(f"start: {start_time}, end:{end2_time}, save_totol {end2_time - end1_time} seconds")
            print(f"totol {end2_time - start_time} seconds")

    finally:
        bng.close()

if __name__ == '__main__':
    main()