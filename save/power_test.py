

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
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import PowertrainSensor


def powertrain_test(beamng: BeamNGpy):
    with beamng as bng:
        # Create a vehicle.
        vehicle = Vehicle('ego_vehicle', model='etki', license='PYTHON', color='Red')
        # Create a scenario.
        scenario = Scenario('smallgrid', 'powertrain_test', description='Testing the powertrain sensor')
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle)
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print('Powertrain test start.')

        # Create a default powertrain sensor.
        # will sample every ms with this setting.
        powertrain1 = PowertrainSensor('powertrain1', bng, vehicle,
                                       is_send_immediately=False, physics_update_time=0.001)

        # Test the automatic polling functionality of the powertrain sensor, to make sure we retrieve the readings data via shared memory.
        # sleep(2)
        # while True:
        #     sensor_readings = powertrain1.poll()
        #     print(sensor_readings[0])
        # print('powertrain readings (automatic polling): ', sensor_readings)


def powertrain_test2(beamng: BeamNGpy):
    with beamng as bng:
        # Create a vehicle.
        vehicle = Vehicle('ego_vehicle', model='etki', license='PYTHON', color='Red')
        # Create a scenario.
        scenario = Scenario('smallgrid', 'powertrain_test', description='Testing the powertrain sensor')
        # Add the vehicle to the scenario.
        scenario.add_vehicle(vehicle)
        scenario.make(bng)
        # Set simulator to 60hz temporal resolution
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print('Powertrain test start.')

        # Create a default powertrain sensor.
        # will sample every ms with this setting.
        powertrain1 = PowertrainSensor('powertrain1', bng, vehicle,
                                       is_send_immediately=False, physics_update_time=0.001)

        # Test the automatic polling functionality of the powertrain sensor, to make sure we retrieve the readings data via shared memory.
        # sleep(2)
        # while True:
        #     sensor_readings = powertrain1.poll()
        #     print(sensor_readings[0])
        # print('powertrain readings (automatic polling): ', sensor_readings)



if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppDate\Local\BeamNG.drive\0.28')
    powertrain_test(beamng=beamng)

    beamng2 = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppDate\Local\BeamNG.drive\0.28')
    powertrain_test(beamng=beamng2)


