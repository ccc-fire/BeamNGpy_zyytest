
import random

from matplotlib import pyplot as plt

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera, Damage, Electrics, GForces, Timer


def main():
    random.seed(1703)

    set_up_simple_logging()

    # Plotting code setting up a 3x2 figure
    # fig = plt.figure(1, figsize=(10, 5))
    # axarr = fig.subplots(2, 3)
    #
    # a_colour = axarr[0, 0]
    # b_colour = axarr[1, 0]
    # a_depth = axarr[0, 1]
    # b_depth = axarr[1, 1]
    # a_annot = axarr[0, 2]
    # b_annot = axarr[1, 2]
    #
    # plt.ion()

    # 创建第一张图
    fig1 = plt.figure(figsize=(5, 5))
    ax1 = fig1.add_subplot(111)
    plt.ion()

    # 创建第二张图
    fig2 = plt.figure(figsize=(5, 5))
    ax2 = fig2.add_subplot(111)
    plt.ion()

    # fig3 = plt.figure(figsize=(5, 5))
    # ax3 = fig3.add_subplot(111)
    # plt.ion()


    beamng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0', user=r'C:\Users\HP\AppDate\Local\BeamNG.drive\0.28')
    bng = beamng.open(launch=True)

    # Create a scenario in east_coast_usa
    scenario = Scenario('east_coast_usa', 'tech_test', description='Random driving for research')

    # Set up first vehicle, with two cameras, gforces sensor, lidar, electrical
    # sensors, and damage sensors
    vehicle = Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    # Set up sensors
    gforces = GForces()
    electrics = Electrics()
    damage = Damage()
    timer = Timer()

    # Attach them
    vehicle.attach_sensor('gforces', gforces)
    vehicle.attach_sensor('electrics', electrics)
    vehicle.attach_sensor('damage', damage)
    vehicle.attach_sensor('timer', timer)

    scenario.add_vehicle(vehicle, pos=(-426.68, -43.59, 31.11), rot_quat=(0, 0, 1, 0))

    # Compile the scenario and place it in BeamNG's map folder
    scenario.make(bng)

    # Start BeamNG and enter the main loop
    try:
        bng.ui.hide_hud()
        bng.settings.set_deterministic(60)  # Set simulator to be deterministic, with 60 Hz temporal resolution

        # Load and start the scenario
        bng.scenario.load(scenario)
        bng.scenario.start()
        # Put simulator in pause awaiting further inputs
        bng.control.pause()

        assert vehicle.is_connected()

        pos = (0.0, -3, 1)
        direction = (0, -1, 0)
        fov = 120
        resolution = (512, 512)
        front_camera = Camera('front_camera', bng, vehicle,
            pos=pos, dir=direction, field_of_view_y=fov, resolution=resolution,
            is_render_colours=True, is_render_depth=True, is_render_annotations=True)

        pos = (0.0, 3, 1.0)
        direction = (0, 1, 0)
        fov = 90
        resolution = (512, 512)
        back_camera = Camera('back_camera', bng, vehicle,
            pos=pos, dir=direction, field_of_view_y=fov, resolution=resolution,
            is_render_colours=True, is_render_depth=True, is_render_annotations=True)

        bird_cam_pos = (2900, 310, 200)
        bird_cam_dir = (0, 0, -1)
        bird_cam_fov = 60
        bird_cam_res = (1920, 1080) # 坐标:(-428,-59,60)
        bird_view_camera = Camera('bird_view_camera', bng, pos=bird_cam_pos,
                                  dir=bird_cam_dir, field_of_view_y=bird_cam_fov, resolution=bird_cam_res,
                                  is_render_colours=True, is_render_depth=True, is_render_annotations=True,
                                  is_render_instance=True, is_static=True)



        # Send random inputs to vehice and advance the simulation 20 steps
        for _ in range(1024):
            throttle = random.uniform(0.0, 1.0)
            steering = random.uniform(-1.0, 1.0)
            brake = random.choice([0, 0, 0, 1])
            vehicle.control(throttle=throttle, steering=steering, brake=brake)

            # bng.control.step(10)

            # Retrieve sensor data and show the camera data.
            vehicle.sensors.poll()
            sensors = vehicle.sensors

            print('{} seconds passed.'.format(sensors['timer']['time']))

            front_cam_data = front_camera.poll()
            back_cam_data = back_camera.poll()
            # bird_view_camera_data = bird_view_camera.poll()


            ax1.imshow(front_cam_data['colour'].convert('RGB'))
            ax2.imshow(back_cam_data['colour'].convert('RGB'))
            # ax3.imshow(bird_view_camera_data["colour"].convert('RGB'))

            # a_colour.imshow(front_cam_data['colour'].convert('RGB'))
            # a_depth.imshow(front_cam_data['depth'].convert('L'))
            # a_annot.imshow(front_cam_data['annotation'].convert('RGB'))
            #
            # b_colour.imshow(back_cam_data['colour'].convert('RGB'))
            # b_depth.imshow(back_cam_data['depth'].convert('L'))
            # b_annot.imshow(back_cam_data['annotation'].convert('RGB'))

            plt.pause(0.1)
    finally:
        bng.close()


if __name__ == '__main__':
    main()
