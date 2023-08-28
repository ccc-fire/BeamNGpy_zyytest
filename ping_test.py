from beamngpy import BeamNGpy, Scenario, Vehicle
import numpy as np
import json
from beamngpy.api.beamng.vehicles import VehiclesApi
import json
from beamngpy.sensors import Radar
from time import sleep
from plyfile import PlyData, PlyElement

bng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0', user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
bng.open()

scenario = Scenario('fresh', 'demo')
vehicle_1 = Vehicle('vehicle_1', model='etk800', license='vehicle_1')
scenario.add_vehicle(vehicle_1, pos=(2926, 400, 125.4), rot_quat=(0, 0, 1, 0))

scenario.make(bng)
bng.scenario.load(scenario)
bng.scenario.start()

print('RADAR test start.')

# Create a RADAR sensor.
RANGE_MIN = 0.1
RANGE_MAX = 100.0
RESOLUTION = (200, 200)
FOV = 70
radar1 = Radar('radar1', bng, vehicle_1,
               pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1),
               resolution=RESOLUTION, field_of_view_y=FOV, near_far_planes=(RANGE_MIN, RANGE_MAX),
               range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12,
               range_min_cutoff=0.5, range_direct_max_cutoff=RANGE_MAX)

# Test the automatic polling functionality of the RADAR sensor, to make sure we retrieve the readings.
sleep(2)
sensor_readings = radar1.poll()
#print('RADAR readings (automatic polling): ', sensor_readings[0:10])
print(sensor_readings)
# save
def save_ply_file(points, filename):
    num_points = len(points)

    # 创建PLY文件头
    header = "ply\n"
    header += "A 6D point cloud of raw RADAR data, where each entry is (range, doppler velocity, azimuth angle, elevation angle, radar cross section, signal to noise ratio)\n"
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

    # 将点云数据写入文件
    with open(filename, 'w') as f:
        f.write(header)
        for point in points:
            range_distance, doppler_velocity, azimuth_angle, elevation_angle, radar_cross_section, signal_to_noise_ratio, time = point
            # x, y, z, distance, velocity, azimuth, elevation, RCS, SNR = point
            f.write(
                f"{range_distance:.6f} {doppler_velocity:.6f} {azimuth_angle:.6f} {elevation_angle:.6f} {radar_cross_section:.6f} {signal_to_noise_ratio:.6f} {time:.6f}\n" )

    print(f"PLY文件已保存为 {filename}")

save_ply_file(sensor_readings, "radar_data.ply")


# Plot the data.
radar1.plot_data(sensor_readings, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200)

# Test the ad-hoc polling functionality of the RADAR sensor. We send an ad-hoc request to poll the sensor, then wait for it to return.
sleep(1)
print('Ad-hoc poll request test.')
# send an ad-hoc polling request to the simulator.
request_id = radar1.send_ad_hoc_poll_request()
print('Ad-hoc poll requests sent. Unique request Id number: ', request_id)
sleep(3)
# Ensure that the data has been processed before collecting.
print('Is ad-hoc request complete? ', radar1.is_ad_hoc_poll_request_ready(request_id))
# Collect the data now that it has been computed.
sensor_readings_ad_hoc = radar1.collect_ad_hoc_poll_request(request_id)
print('RADAR readings (ad-hoc polling): ', sensor_readings_ad_hoc[0:10])
radar1.remove()
print('RADAR sensor removed.')

# Create a RADAR sensor which has a negative requested update rate, and ensure that no readings are computed from it.
radar2 = Radar('radar2', bng, vehicle_1, requested_update_time=-1.0)
print('Testing an ultrasonic sensor with a negative requested update time...')
sleep(2)
sensor_readings = radar2.poll()
print('RADAR readings (should be zeros): ', sensor_readings)
radar2.remove()

# Recreate the first RADAR sensor, with default parameters.
radar1 = Radar('radar1', bng, vehicle_1)

# Test that the property getter function return the correct data which was set.
sleep(1)
print(
    'Property getter test.  The displayed values should be the values which were set during the creation of the ultrasonic sensors.')
print('Sensor Name: ', radar1.name)
print('Position: ', radar1.get_position())
print('Direction: ', radar1.get_direction())
print('Requested update time: ', radar1.get_requested_update_time())
print('Priority: ', radar1.get_update_priority())
print('Max Pending Requests: ', radar1.get_max_pending_requests())

# Test that we can set the sensor core properties in the simulator from beamngpy.
sleep(1)
print('Property setter test.  The displayed property values should be different from the previous values.')
radar1.set_requested_update_time(0.3)
print('Newly-set Requested Update Time: ', radar1.get_requested_update_time())
radar1.set_update_priority(0.5)
print('Newly-set Priority: ', radar1.get_update_priority())
radar1.set_max_pending_requests(5)
print('Newly-set Max Pending Requests: ', radar1.get_max_pending_requests())

radar1.remove()

sleep(3)
print('RADAR test complete.')