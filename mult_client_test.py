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

import cProfile



def main():
    """
    Test that a second client can connect to a running instance, check for
    active vehicles, connect to one, and control it
    """
    random.seed(1703)

    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
    bng = beamng.open(launch=True)

    with bng as a_client:
        scenario = Scenario('smallgrid', 'multi_vehicle')
        first = Vehicle('first', model='etk800')
        scenario.add_vehicle(first, pos=(2, 2, 0))
        second = Vehicle('second', model='etki')
        scenario.add_vehicle(second, pos=(-2, -2, 0))
        scenario.make(a_client)

        a_client.scenario.load(scenario)
        a_client.scenario.start()

        b_client = BeamNGpy('localhost', 64256)
        #  Do not launch new process
        b_client.open(launch=False)
        vehicles = b_client.vehicles.get_current(include_config=False)
        assert 'second' in vehicles
        vehicle = vehicles['second']
        vehicle.connect(b_client)
        assert vehicle.is_connected()

        a_veh = second
        b_veh = vehicle

        # b_veh.control(throttle=1.0)
        #__________
        a_veh.control(throttle=0.5)

        # __________


        for _ in range(8):
            # Verify position updating in both clients
            a_veh.sensors.poll()
            b_veh.sensors.poll()
            a_ref = a_veh.sensors['state']['pos']
            b_ref = b_veh.sensors['state']['pos']
            b_client.control.step(100)
            a_veh.sensors.poll()
            b_veh.sensors.poll()
            a_new = a_veh.sensors['state']['pos']
            b_new = b_veh.sensors['state']['pos']

            assert a_ref[0] != a_new[0] or a_ref[1] != a_new[1]
            assert b_ref[0] != b_new[0] or b_ref[1] != b_new[1]

def multi_scenario():
    """
    Test that a second client can connect to a running instance and ask
    information about the loaded scenario.
    """
    random.seed(1703)

    set_up_simple_logging()

    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                      user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
    bng = beamng.open(launch=True)
    with bng as a_client:
        a_client.control.return_to_main_menu()  # if a scenario was running previously
        scenario = Scenario('gridmap_v2', 'multi_scenario')
        vehicle = Vehicle('vehicle', model='etk800')
        scenario.add_vehicle(vehicle, pos=(0, 0, 100))
        scenario.make(a_client)

        b_client = BeamNGpy('localhost', 64256)
        b_client.open(launch=False)

        a_client.scenario.load(scenario)
        a_client.scenario.start()

        running = b_client.scenario.get_current()
        if running.level == scenario.level:
            print("level======")
        if running.name == scenario.name:
            print("name======")


if __name__ == "__main__":
    multi_scenario()