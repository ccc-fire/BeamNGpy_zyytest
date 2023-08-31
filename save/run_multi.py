

import sys
import time
import math
import copy
import string
import random
import argparse

import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import Polygon
from shapely.geometry import Point

from beamngpy.beamngcommon import *
from beamngpy.beamngcommon import angle_to_quat

from beamngpy.sensors import Timer
from beamngpy.sensors import Lidar
from beamngpy.sensors import Damage
from beamngpy.sensors import Camera
from beamngpy.sensors import GForces
from beamngpy.sensors import Electrics

from beamngpy import Vehicle
from beamngpy import Scenario
from beamngpy import BeamNGpy
from beamngpy import setup_logging

from beamngpy.visualiser import LidarVisualiser

from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt
from shapely.geometry import Polygon


class BeamNG_Random_Test_Generation():
    # Init
    def __init__(self, beamng_location, number_traffic_vehicles, port_number=64256):

        # Define parameters about the world
        self.lanewidth = 3.75

        # Setup Beamng
        setup_logging()
        self.beamng = BeamNGpy('localhost', port_number, home=beamng_location)
        self.scenario = Scenario('west_coast_usa', 'Random Test Generation', description='Tests for science')

        # Define the number of traffic vehicles
        self.number_traffic_vehicles = number_traffic_vehicles

        # Create the vehicles
        self.traffic_vehicles = self.create_traffic_vehicles(self.number_traffic_vehicles)
        self.ego_vehicle = Vehicle('ego_vehicle', model='etk800', licence='Carlos', color='Red')
        self.target_vehicle = Vehicle('target_vehicle', model='hatch', licence='Target', color='Red')

        # Spawn the ego vehicle in a random lane
        lane = random.choice([1,2,3,4])
        self.spawn_ego_vehicle(lane, spawn=True)

        # Spawn the target vehicle at the target
        self.spawn_target_vehicle(target=[516.65, 728.85, 117.5])

        # Spawn the traffic vehicles
        self.spawn_traffic_vehicles()

        # Start the scene for the first time
        self.start_new_scenario()

    # Start a new scenario
    def start_new_scenario(self):

        # Make the scenario
        self.scenario.make(self.beamng)

        # Open beamng
        self.bng = self.beamng.open(launch=True)
        self.bng.load_scenario(self.scenario)

        # Configure and start beamng
        self.bng.set_steps_per_second(60)
        self.bng.set_deterministic()
        self.bng.hide_hud()
        self.bng.start_scenario()

        # Make sure you are focused on the ego vehicle
        self.bng.switch_vehicle(self.ego_vehicle)

        # Start the traffic
        self.bng.start_traffic(self.traffic_vehicles)

        # Start the AI
        self.start_ego_ai(max_speed=70, aggression=1)

        # Pause the scenario and wait for the traffic vehicles to spawn
        wait_time_per_vehicle = 4
        self.bng.step((wait_time_per_vehicle * len(self.traffic_vehicles)) / 0.05, wait=True) 
        self.bng.pause()

        # Restart the scenario incase it had already been run before
        self.bng.restart_scenario()

    # Used to run the scenario
    def run_scenario(self):

        # Start the scene for the first time
        self.bng.restart_scenario()
        self.update_positions()

        # Run for the duration requested
        while True:

            # Get the current state
            current_state = self.ego_vehicle.state

            # Step the simulation forward (1 step is 0.05 seconds)
            self.bng.step(1)

        # Close the file
        f.close()

        # Return true when done
        return True

    # Close beamng
    def stop(self):
        self.bng.close()
        exit()

    # Starts the AI for the ego vehicle
    def start_ego_ai(self, max_speed, aggression):
        # Tell it to chase the target vehicle
        self.ego_vehicle.ai_drive_in_lane(True)
        self.ego_vehicle.ai_set_target("target_vehicle", "chase")
        self.ego_vehicle.ai_set_mode('chase')

        # Set the speed limit and agression
        self.ego_vehicle.ai_set_speed(max_speed, mode='limit')
        self.ego_vehicle.ai_set_aggression(aggression)

    # Returns lane counted from left to right (1 through 4)
    def spawn_ego_vehicle(self, lane, spawn=False):
        # Make sure you are sending the correct lane information
        assert(1 <= lane <= 4)

        # Compute the correct lane adjustment
        lane_adjustment = (lane - 1) * self.lanewidth
        sp = {'pos': (-852.024 , -513.641 - lane_adjustment, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}

        if spawn:
            # Add the ego vehicle to the scenario
            self.scenario.add_vehicle(self.ego_vehicle, pos=sp['pos'], rot=sp['rot'], rot_quat=sp['rot_quat'])
        else:
            # Update the vehicles position
            self.ego_vehicle.pos = sp['pos']

    # Spawns the target vehicle at the target
    def spawn_target_vehicle(self, target):
        # Compute the correct lane adjustment
        sp = {'pos': (target[0] , target[1], target[2]), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}

        # Add the ego vehicle to the scenario
        self.scenario.add_vehicle(self.target_vehicle, pos=sp['pos'], rot=sp['rot'], rot_quat=sp['rot_quat'])

    # Spawn the traffic vehicles in a predefine box
    def spawn_traffic_vehicles(self):

        # Create the spawn box
        spawn_box = Polygon([(-811.3121337890625, -486.3587341308594), (-820.1568603515625, -478.5815734863281), (-778.927490234375, -437.0894775390625), (-771.5311889648438, -445.2535400390625)])
        spawn_height = 106.5

        # Used to keep track of previous spawn locations
        previous_spawn_locations = []

        # Get n random points inside the spawn box and spawn the vehicles
        for i in range(len(self.traffic_vehicles)):
            # Set the acceptable location flag to false
            acceptable_location = False

            # Keep trying until we have found acceptable locations
            while not acceptable_location:
                # Get a random point inside the box
                p = self.get_random_point_in_polygon(spawn_box)

                # assume the location is acceptable
                acceptable_location = True

                # Check if this fits all other locations
                for o in previous_spawn_locations:
                    # If this is too close to another point reject it
                    if p.distance(o) <= 4:
                        acceptable_location = False

            # Add the point to the previously accepted locations
            previous_spawn_locations.append(p)

            # Create the spawn point
            sp = {'pos': (p.x , p.y, spawn_height), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}

            # Get the vehicle
            t = self.traffic_vehicles[i]

            # Spawn the vehicle there
            print("Added vehicle {} in position {}".format(i, sp['pos']))
            self.scenario.add_vehicle(t, pos=sp['pos'], rot=sp['rot'], rot_quat=sp['rot_quat'])

    # Create and return the traffic vehicles
    def create_traffic_vehicles(self, number_traffic_vehicles):
        traffic_vehicles = []
        # Create the vehicles
        for i in range(number_traffic_vehicles):
            name = 'Traffic{}'.format(i)
            v = Vehicle(name, model='etk800', licence=name, color='White')
            traffic_vehicles.append(v)
        # Return the traffic vehicles
        return traffic_vehicles

    # Randomly places the ego and traffic vehicles in new positions
    def update_positions(self):
        lane = random.choice([1,2,3,4])
        lane_adjustment = (lane - 1) * self.lanewidth
        sp = {'pos': (-852.024 , -513.641 - lane_adjustment, 106.620), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}
        self.bng.teleport_vehicle(self.ego_vehicle, pos=sp['pos'], rot=sp['rot'], rot_quat=sp['rot_quat'])

        # Create the spawn box
        spawn_box = Polygon([(-811.3121337890625, -486.3587341308594), (-820.1568603515625, -478.5815734863281), (-778.927490234375, -437.0894775390625), (-771.5311889648438, -445.2535400390625)])
        spawn_height = 107

        # Used to keep track of previous spawn locations
        previous_spawn_locations = []

        # Get n random points inside the spawn box and spawn the vehicles
        for i in range(len(self.traffic_vehicles)):
            # Set the acceptable location flag to false
            acceptable_location = False

            # Keep trying until we have found acceptable locations
            while not acceptable_location:
                # Get a random point inside the box
                p = self.get_random_point_in_polygon(spawn_box)

                # assume the location is acceptable
                acceptable_location = True

                # Check if this fits all other locations
                for o in previous_spawn_locations:
                    # If this is too close to another point reject it
                    if p.distance(o) <= 4:
                        acceptable_location = False

            # Add the point to the previously accepted locations
            previous_spawn_locations.append(p)

            # Create the spawn point
            sp = {'pos': (p.x , p.y, spawn_height), 'rot': None, 'rot_quat': (0, 0, 0.926127, -0.377211)}

            # Get the vehicle
            t = self.traffic_vehicles[i]
            self.bng.teleport_vehicle(t, pos=sp['pos'], rot=sp['rot'], rot_quat=sp['rot_quat'])

    # Gets a random point inside a polygon
    def get_random_point_in_polygon(self, poly):
        minx, miny, maxx, maxy = poly.bounds
        while True:
            p = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
            if poly.contains(p):
                return p



# Get the file arguments
def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port_number',                type=int, default=64256,    help="The port number you want to use")
    parser.add_argument('--number_traffic_vehicles',    type=int, default=1,        help="The number of traffic vehicles")
    args = parser.parse_args()
    return args

# Main function
def main():

    args = get_arguments()

    # Init variables
    number_traffic_vehicles = args.number_traffic_vehicles

    # Create the beamng class
    bng_loc = 'C:\\Users\\ch6wd\\Documents\BeamNG.research.v1.7.0.1'
    bng_obj = BeamNG_Random_Test_Generation(beamng_location=bng_loc, number_traffic_vehicles=number_traffic_vehicles, port_number=args.port_number)

    # Run the scenario
    total_physical_accidents = bng_obj.run_scenario()

    # Close the program
    bng_obj.stop()

# If starting run the main function
if __name__ == '__main__':
    main()
