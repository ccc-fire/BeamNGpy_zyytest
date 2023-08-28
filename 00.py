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

# 获取当前时间
current_time = time.strftime("%Y%m%d%H%M%S", time.localtime())

# 创建文件夹路径
folder_path = r"C:\Users\HP\Desktop\camera_test"
output_folder = os.path.join(folder_path, current_time)

# 创建文件夹
os.mkdir(output_folder)

print("Storing OUTPUT to ", output_folder)


# lidar pcd.bin
# camera jpg
# Radar PCD

def store_radar(radar_name, frame_id, data):

    num_points = len(data)

    # 创建PLY文件头
    header = "ply\n"
    header += " A 6D point cloud of raw RADAR data, where each entry is (range, doppler velocity, azimuth angle, elevation angle, radar cross section, signal to noise ratio)\n"
    header += "format ascii 1.0\n"
    header += f"element vertex {num_points}\n"
    header += "property float range\n"
    header += "property float doppler_velocity\n"
    header += "property float azimuth_angle\n"
    header += "property float elevation_angle\n"
    header += "property float radar_cross_section\n"
    header += "property float signal_to_noise_ratio\n"
    header += "property float time\n"
    header += "end_header\n"
    output_file = os.path.join(output_folder, "_".join([radar_name, str(frame_id)]))
    # 将点云数据写入文件
    with open(output_file, 'w') as f:
        f.write(header)
        for point in data:
            range_distance, doppler_velocity, azimuth_angle, elevation_angle, radar_cross_section, signal_to_noise_ratio, time = point
            # x, y, z, distance, velocity, azimuth, elevation, RCS, SNR = point
            f.write(
                f"{range_distance:.6f} {doppler_velocity:.6f} {azimuth_angle:.6f} {elevation_angle:.6f} {radar_cross_section:.6f} {signal_to_noise_ratio:.6f} {time:.6f}\n" )
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

def store_IMU(IMU_name, data):
    output_file = os.path.join(output_folder, "_".join([IMU_name]) + ".txt")
    os.makedirs(os.path.dirname(output_file), exist_ok=True)  # 确保目录存在
    with open(output_file, 'a') as f:
        f.write(str(data) + "\n\n")

def store_lidar(lidar_name, frame_id , data):
    output_file = os.path.join(output_folder, "_".join([lidar_name, str(frame_id)]) + ".ply")
    point_cloud = data['pointCloud']
    # colors = data['colours']
    # Open3D库中的颜色数据仅支持RGB格式,Open3D期望颜色数据以(0, 0, 0)到(1, 1, 1)的浮点数形式表示
    # 这里的雷达颜色数据是RGBA格式

    # 将颜色数据从列表转换为NumPy数组
    colors = np.array(data['colours'], dtype=np.uint8)

    # 确保数组长度为3的倍数
    if len(colors) % 3 != 0:
        # raise ValueError("The length of colors array should be a multiple of 3.")
        # 如果长度不是3的倍数，则截断或填充数组
        remainder = len(colors) % 3

        if remainder > 0:
            # 截断多余的元素
            colors = colors[:-remainder]
        else:
            # 填充元素
            colors = np.pad(colors, ((0, 3 - remainder),), mode='constant')

    # 将颜色数组重新调整为形状为(3, N)的二维数组，并进行归一化处理
    colors = np.reshape(colors, (-1, 3)) / 255.0
    # 创建点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # o3d.io.write_point_cloud("point_cloud.ply", pcd)
    o3d.io.write_point_cloud(output_file, pcd)

    # # can can
    # # 从保存的PLY文件中加载点云数据
    # pcd = o3d.io.read_point_cloud(output_file)
    #
    # # 进行可视化
    # o3d.visualization.draw_geometries([pcd])

def main():

    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0', user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
    bng = beamng.open(launch=True)


    scenario = Scenario('fresh', 'demo')
    vehicle_1 = Vehicle('vehicle_1', model='etk800', license='vehicle_1')
    origin = (2900, 316, 125.4)
    # origin = (2921, 316, 125.4)
    scenario.add_vehicle(vehicle_1, pos=origin, rot_quat=(0, 0, 1, 0))


    scenario.make(bng)

    # Start BeamNG and enter the main loop
    try:
        bng.ui.hide_hud()
        bng.settings.set_deterministic(60)  # Set simulator to be deterministic, with 60 Hz temporal resolution

        # Load and start the scenario
        bng.scenario.load(scenario)
        bng.scenario.start()

        assert vehicle_1.is_connected()
        box = vehicle_1.get_bbox() #get vehicle box, origin = vehicle center
        # 对 x、y 和 z 分别求平均值
        x_avg = sum(point[0] for point in box.values()) / len(box)
        y_avg = sum(point[1] for point in box.values()) / len(box)
        z_avg = sum(point[2] for point in box.values()) / len(box)
        box_center = [x_avg, y_avg, z_avg]
        print(box_center)
        center_x = box['front_bottom_left'][0] - box['front_bottom_right'][0]
        center_y = box['front_top_right'][1] - box['rear_top_right'][1]
        center_z = box['front_top_left'][2] - box['front_bottom_left'][2]
        #
        #
        #
        #
        c0 = [2899.670166015625, 315.7689208984375, 125.39088439941406] #set 000 shihou de position
        # c0 = (2902.029052734375, 313.9044189453125, 125.3958740234375)
        bia = list(x - y for x, y in zip(box_center, c0))

        # 000位置=000+bia


        # camera resolution:1920,1080

        # set camera _____________________________________________________________
        resolution = (1920, 1080)

        # camera front position            dir未改
        c_pos0 = list(bia)
        c_pos0[2] += center_z / 2.0 + 0.3
        c_dir0 = (0, -1, 0)

        # camera front left position
        c_pos2 = list(bia)
        c_pos2[0] += center_x / 6.0
        c_pos2[2] += center_z / 2.0 + 0.3

        # camera front right position
        c_pos4 = list(bia)
        c_pos4[0] -= center_x / 6.0
        c_pos4[2] += center_z / 2.0 + 0.3

        # camera back left position
        c_pos1 = list(bia)
        c_pos1[0] += center_x / 5.0
        c_pos1[1] += center_y / 5.0
        c_pos1[2] += center_z / 2.0 + 0.3

        # camera back right position
        c_pos3 = list(bia)
        c_pos3[0] -= center_x / 5.0
        c_pos3[1] += center_y / 5.0
        c_pos3[2] += center_z / 2.0 + 0.3

        # camera back position
        c_pos5 = list(bia)
        c_pos5[1] += center_y / 3.0
        c_pos5[2] += center_z / 2.0 + 0.3
        c_dir5 = (0, 1, 0)

        camera_front = Camera('camera_front', bng, vehicle_1,
                          pos=tuple(c_pos0), dir=c_dir0, field_of_view_y=120, resolution=resolution,
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                          is_render_instance=True)
        camera_front_left = Camera('camera_front_left', bng, vehicle_1,
                          pos=tuple(c_pos2), dir=(0, -1, 0), field_of_view_y=120, resolution=resolution,
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                          is_render_instance=True)
        camera_front_right = Camera('camera_front_right', bng, vehicle_1,
                          pos=tuple(c_pos4), dir=(0, -1, 0), field_of_view_y=120, resolution=resolution,
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                          is_render_instance=True)
        camera_back_left = Camera('camera_back_left', bng, vehicle_1,
                          pos=tuple(c_pos1), dir=(0, 1, 0), field_of_view_y=120, resolution=resolution,
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                          is_render_instance=True)
        camera_back_right = Camera('camera_back_right', bng, vehicle_1,
                          pos=tuple(c_pos3), dir=(0, 1, 0), field_of_view_y=120, resolution=resolution,
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                          is_render_instance=True)
        camera_back = Camera('camera_back', bng, vehicle_1,
                          pos=tuple(c_pos5), dir=c_dir5, field_of_view_y=120, resolution=resolution,
                          is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                          is_render_instance=True)

        # put the cameras on the scenario  固定坐标位置
        bird_cam_pos = (2900, 310, 200)
        bird_cam_dir = (0, 0, -1)
        bird_cam_fov = 60
        bird_cam_res = (1920, 1080) # 坐标:(-428,-59,60)
        bird_view_camera = Camera('bird_view_camera', bng, pos=bird_cam_pos,
                                  dir=bird_cam_dir, field_of_view_y=bird_cam_fov, resolution=bird_cam_res,
                                  is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                                  is_render_instance=True, is_static=True)

        # put lidar _______________
        l_posTop = list(bia)
        l_posTop[1] += center_y / 4.0
        l_posTop[2] += center_z / 2.0 + 0.5
        lidar_Top = Lidar('lidar_Top', bng, vehicle_1, pos = l_posTop, requested_update_time=0.01, is_using_shared_memory=False)


        # set IMU
        I_pos = list(bia)
        I_pos[1] += center_y / 3.5
        I_pos[2] += center_z / 2.0 + 0.3

        IMU = AdvancedIMU('IMU', bng, vehicle_1, pos=I_pos, gfx_update_time=0.01)

        # set radar_______________________________________________________
        # radar front position
        r_pos0 = list(bia)
        r_pos0[1] -= center_y / 2

        # radar back left position
        r_pos1 = list(bia)
        r_pos1[0] += center_x / 4
        r_pos1[1] += center_y / 2

        # radar front left position
        r_pos2 = list(bia)
        r_pos2[0] += center_x / 2
        r_pos2[1] -= center_y / 4
        r_pos2[2] += center_z / 4

        # radar back right position
        r_pos3 = list(bia)
        r_pos3[0] -= center_x / 4
        r_pos3[1] += center_y / 2

        # radar front right position
        r_pos4 = list(bia)
        r_pos4[0] -= center_x / 2
        r_pos4[1] -= center_y / 4
        r_pos4[2] += center_z / 4

        # put radar
        RANGE_MIN = 0.1
        RANGE_MAX = 100.0
        RESOLUTION = (200, 200)
        FOV = 70

        radar_back_right = Radar('radar_back_right', bng, vehicle_1, requested_update_time=0.01,
                       pos=tuple(r_pos3), dir=(0, 1, 0), up=(0, 0, 1),
                       resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                       range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                       range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=True)
        radar_front_right = Radar('radar_front_right', bng, vehicle_1, requested_update_time=0.01,
                       pos=tuple(r_pos4), dir=(0, -1, 0), up=(0, 0, 1),
                       resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                       range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                       range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=True)

        radar_front_left = Radar('radar_front_left', bng, vehicle_1, requested_update_time=0.01,
                       pos=tuple(r_pos2), dir=(0, -1, 0), up=(0, 0, 1),
                       resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                       range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                       range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=True)
        radar_front = Radar('radar_front', bng, vehicle_1, requested_update_time=0.01,
                       pos=tuple(r_pos0), dir=(0, -1, 0), up=(0, 0, 1),
                       resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                       range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                       range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=True) # is_force_inside_triangle=True
        radar_back_left = Radar('radar_back_left', bng, vehicle_1, requested_update_time=0.01,
                       pos=tuple(r_pos1), dir=(0, 1, 0), up=(0, 0, 1),
                       resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                       range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                       range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=True)

        #__________________________________________________________________________
        bng.traffic.spawn(3)

        sleep(2)



        # 开始采集数据，时间为1秒：
        for frame_id in range(11):
            start_time = time.time()

            #save radar data 雷达
            radar_front_data = radar_front.poll()
            radar_back_left_data = radar_back_left.poll()
            radar_front_left_data = radar_front_left.poll()
            radar_back_right_data = radar_back_right.poll()
            radar_front_right_data = radar_front_right.poll()

            store_radar("radar_front", frame_id, radar_front_data)
            store_radar("radar_back_left", frame_id, radar_back_left_data)
            store_radar("radar_front_left", frame_id, radar_front_left_data)
            store_radar("radar_back_right", frame_id, radar_back_right_data)
            store_radar("radar_front_right", frame_id, radar_front_right_data)


            # print(radar_front_data)

            # save IMU data 惯性测量单元
            IMU_data = IMU.poll()
            store_IMU("IMU", IMU_data)
            # while 1:
            #     store_IMU("IMU", IMU_data)
            # print('IMU Data: ', IMU_data)

            # save lidar data 激光雷达
            lidar_Top_data = lidar_Top.poll()
            # print(lidar_Top_data)

            store_lidar("lidar_Top", frame_id, lidar_Top_data)
            # save camera data

            bird_view_camera_data = bird_view_camera.get_full_poll_request()
            camera_front_data = camera_front.get_full_poll_request()
            camera_front_left_data = camera_front_left.get_full_poll_request()
            camera_front_right_data = camera_front_right.get_full_poll_request()
            camera_back_left_data = camera_back_left.get_full_poll_request()
            camera_back_right_data = camera_back_right.get_full_poll_request()
            camera_back_data = camera_back.get_full_poll_request()
            store_camera('bird_view_camera', frame_id, bird_view_camera_data['colour'], bird_view_camera_data['depth'],
                         bird_view_camera_data['annotation'], bird_view_camera_data['instance'])
            store_camera('camera_front', frame_id, camera_front_data['colour'], camera_front_data['depth'],
                         camera_front_data['annotation'], camera_front_data['instance'])
            store_camera('camera_front_left', frame_id, camera_front_left_data['colour'], camera_front_left_data['depth'],
                         camera_front_left_data['annotation'], camera_front_left_data['instance'])
            store_camera('camera_front_right', frame_id, camera_front_right_data['colour'], camera_front_right_data['depth'],
                         camera_front_right_data['annotation'], camera_front_right_data['instance'])
            store_camera('camera_back_left', frame_id, camera_back_left_data['colour'], camera_back_left_data['depth'],
                         camera_back_left_data['annotation'], camera_back_left_data['instance'])
            store_camera('camera_back_right', frame_id, camera_back_right_data['colour'], camera_back_right_data['depth'],
                         camera_back_right_data['annotation'], camera_back_right_data['instance'])
            store_camera('camera_back', frame_id, camera_back_data['colour'], camera_back_data['depth'],
                         camera_back_data['annotation'], camera_back_data['instance'])
            end_time = time.time()
            execution_time = end_time - start_time  # 计算代码执行时间

            print(f"start: {start_time}, end:{end_time}, totol {execution_time} seconds")





            # print(bird_view_camera_data)
            # a_colour.imshow(bird_view_camera_data['colour'])
            # a_depth.imshow(bird_view_camera_data['depth'])
            # a_instance.imshow(bird_view_camera_data['instance'])
            # a_annotation.imshow(bird_view_camera_data['annotation'])
            # plt.pause(0.01)

            # readings_data = radar_front.poll()
            # # 根据轮次信息构造文件名
            # filename = r"C:\Users\HP\Desktop\radar_test\data_{}.csv".format(frame_id)
            #
            # # 保存数据到对应的 CSV 文件
            # with open(filename, 'w', newline='') as file:
            #     writer = csv.writer(file)
            #     writer.writerows(readings_data)
            #
            # print("数据已成功保存到{}文件中。".format(filename))
            # print(readings_data)
            # radar_front.plot_data(readings_data, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200)
            # plt.savefig('test.png', dpi=300)
            # plt.close()

            #     readings_data = radar.poll() # Fetch the latest readings from the sensor, over either shared memory or the lua socket.
            #     radar.plot_data(readings_data, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200)

            # camerafront_data = camera_front.get_full_poll_request()
            # print(camerafront_data)
            # plt.imshow(camerafront_data['colour'])
            # plt.pause(0.01)

    finally:
        bng.close()


if __name__ == '__main__':
    main()