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

beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                  user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
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
    box = vehicle_1.get_bbox()  # get vehicle box, origin = vehicle center
    # 对 x、y 和 z 分别求平均值
    x_avg = sum(point[0] for point in box.values()) / len(box)
    y_avg = sum(point[1] for point in box.values()) / len(box)
    z_avg = sum(point[2] for point in box.values()) / len(box)
    box_center = [x_avg, y_avg, z_avg]
    print(box_center)
    center_x = box['front_bottom_left'][0] - box['front_bottom_right'][0]
    center_y = box['front_top_right'][1] - box['rear_top_right'][1]
    center_z = box['front_top_left'][2] - box['front_bottom_left'][2]
    pos00 = Radar('c0', bng, vehicle_1, requested_update_time=0.01,
               pos=(0,0,0), dir=(0, -1, 0), up=(0, 0, 1),
               resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
               range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
               range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    pos00.get_position()


    c0 = [2899.670166015625, 315.7689208984375, 125.39088439941406]  # set 000 shihou de position

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
    c0 = Radar('c0', bng, vehicle_1, requested_update_time=0.01,
                        pos=tuple(c_pos0), dir=(0, -1, 0), up=(0, 0, 1),
                        resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                        range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                        range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)  # is_force_inside_triangle=True
    c2 = Radar('c2', bng, vehicle_1, requested_update_time=0.01,
                            pos=tuple(c_pos2), dir=(0, 1, 0), up=(0, 0, 1),
                            resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                            range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                            range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    c4 = Radar('c4', bng, vehicle_1, requested_update_time=0.01,
                             pos=tuple(c_pos4), dir=(0, -1, 0), up=(0, 0, 1),
                             resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                             range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                             range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    c1 = Radar('c1', bng, vehicle_1, requested_update_time=0.01,
                             pos=tuple(c_pos1), dir=(0, 1, 0), up=(0, 0, 1),
                             resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                             range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                                 range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    c3 = Radar('c3', bng, vehicle_1, requested_update_time=0.01,
                              pos=tuple(c_pos3), dir=(0, -1, 0), up=(0, 0, 1),
                              resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                              range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                                  range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    c5 = Radar('c5', bng, vehicle_1, requested_update_time=0.01,
                              pos=tuple(c_pos5), dir=(0, -1, 0), up=(0, 0, 1),
                              resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                              range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                              range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)


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

    radar_front = Radar('radar_front', bng, vehicle_1, requested_update_time=0.01,
                        pos=tuple(r_pos0), dir=(0, -1, 0), up=(0, 0, 1),
                        resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                        range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                        range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)  # is_force_inside_triangle=True
    radar_back_left = Radar('radar_back_left', bng, vehicle_1, requested_update_time=0.01,
                            pos=tuple(r_pos1), dir=(0, 1, 0), up=(0, 0, 1),
                            resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                            range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                            range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    radar_front_left = Radar('radar_front_left', bng, vehicle_1, requested_update_time=0.01,
                             pos=tuple(r_pos2), dir=(0, -1, 0), up=(0, 0, 1),
                             resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                             range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                             range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    radar_back_right = Radar('radar_back_right', bng, vehicle_1, requested_update_time=0.01,
                             pos=tuple(r_pos3), dir=(0, 1, 0), up=(0, 0, 1),
                             resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                             range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                                     range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)
    radar_front_right = Radar('radar_front_right', bng, vehicle_1, requested_update_time=0.01,
                              pos=tuple(r_pos4), dir=(0, -1, 0), up=(0, 0, 1),
                              resolution=(200, 200), field_of_view_y=70, near_far_planes=(0.1, 100.0),
                              range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
                              range_min_cutoff=0.5, range_direct_max_cutoff=100.0, is_snapping_desired=False)

    # __________________________________________________________________________
    vehicle_1.ai_set_mode('random')
    vehicle_1.ai_set_mode('traffic')
finally:
    bng.close()