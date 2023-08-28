from beamngpy import BeamNGpy, Scenario, Vehicle
import numpy as np
import json
from beamngpy.api.beamng.vehicles import VehiclesApi
import json


file_path = r"C:\Users\dell\Desktop\camera_test\state\file.json"

def main():
    bng = BeamNGpy('localhost', 64256, home=r'D:\RJY\BeamNG.tech.v0.28.2.0', user=r'C:\Users\HP\AppData\Local\BeamNG.drive\0.28')
    bng.open()

    scenario = Scenario('fresh', 'demo_test')
    vehicle_1 = Vehicle('vehicle_1', model='etk800', license='vehicle_1')
    scenario.add_vehicle(vehicle_1, pos=(2926, 400, 125.4), rot_quat=(0, 0, 1, 0))

    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()

    # set ai script action
    # with open(r'D:\RJY\BeamNGpy-master\BeamNGpy-master\zyy_test\code_rode_test.track.json', 'r') as file:
    #     # 加载JSON数据,获取path
    #     data = json.load(file)['recording']
    # vehicle_1.ai.set_script(data['path'],True)

## 采集车辆行驶中的数据
    try:
        while True:
            bng.control.step(20)
            vehicle_1.sensors.poll()
            print('vehicle_1 state: ', vehicle_1.state)
            state_list = {}
            state_list.append(vehicle_1.state)


    finally:
        with open(file_path,'w') as file:
            json.dump(state_list, file)
        print("111111111111111111111111111111111")


if __name__ == '__main__':
    main()




