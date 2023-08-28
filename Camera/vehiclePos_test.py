
from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat, sensors
from beamngpy.logging import BNGValueError
from beamngpy.types import Float3
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer, Radar
import numpy as np
import os


beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0',
                  user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
bng = beamng.open(launch=True)
scenario = Scenario('fresh', 'fresh1')
vehicle_a = Vehicle('vehicle_a', model='etk800')
vehicle_b = Vehicle('vehicle_b', model='etk800')
pos = (7073, 2764, 125.45)
scenario.add_vehicle(vehicle_a, pos=pos, rot_quat=angle_to_quat((0, 0, 45)))
pos = (7078, 2764, 125.45)
scenario.add_vehicle(vehicle_b, pos=pos, rot_quat=angle_to_quat((0, 0, 45)))
scenario.make(beamng)

bng.scenario.load(scenario)
bng.scenario.start()
bng.control.pause()

bbox_beg = vehicle_a.get_bbox()
vehicle_a.ai.set_mode('span')
bng.control.step(2000, wait=True)
bbox_end = vehicle_a.get_bbox()

for k, v in bbox_beg.items():
    assert k in bbox_end
    assert v != bbox_end[k]