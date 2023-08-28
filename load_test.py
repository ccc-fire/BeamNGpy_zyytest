import random
import time
from matplotlib import pyplot as plt

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer
import numpy as np
import os

BNG_HOME = r'D:\RJY\BeamNG.tech.v0.28.2.0'
BNG_USER = r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28'
with BeamNGpy('localhost', 64256, home=BNG_HOME,  user=BNG_USER) as bng:
    bng.set_steps_per_second(60)
    bng.settings.set_deterministic()

    bng.pause

    for i in range (1, 10):
        bng.step(30) #0.5秒

        time.sleep(2)


