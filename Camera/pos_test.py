import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 给定盒子坐标点数据
box = {
    'front_bottom_left': [2900.689697265625, 318.0464172363281, 125.1402359008789],
    'front_bottom_right': [2898.649658203125, 318.0461120605469, 125.14139556884766],
    'front_top_left': [2900.690673828125, 318.0617370605469, 126.53015899658203],
    'front_top_right': [2898.650634765625, 318.0614318847656, 126.53131866455078],
    'rear_bottom_left': [2900.690185546875, 313.3766784667969, 125.19161224365234],
    'rear_bottom_right': [2898.650146484375, 313.3763732910156, 125.1927719116211],
    'rear_top_left': [2900.691162109375, 313.3919982910156, 126.58153533935547],
    'rear_top_right': [2898.651123046875, 313.3916931152344, 126.58269500732422]
}

# 给定原点坐标
origin = np.array([2900, 316, 125.4])
#
# # 遍历盒子坐标点并相对于原点进行偏移
# for key, value in box.items():
#     offset = np.array(value) - origin
#     box[key] = offset.tolist()
#
# # 打印盒子坐标点相对于原点的偏移结果
# print(box)
#
# 提取盒子坐标点，并相对于原点进行偏移
points = np.array(list(box.values())) - origin

print(points)
# 创建 3D 图形对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制盒子的八个角点
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o')

# 设置坐标轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# # 显示图形
# plt.show()
#
# box = {
#     'front_bottom_left': [2900.689697265625, 318.0464172363281, 125.1402359008789],
#     'front_bottom_right': [2898.649658203125, 318.0461120605469, 125.14139556884766],
#     'front_top_left': [2900.690673828125, 318.0617370605469, 126.53015899658203],
#     'front_top_right': [2898.650634765625, 318.0614318847656, 126.53131866455078],
#     'rear_bottom_left': [2900.690185546875, 313.3766784667969, 125.19161224365234],
#     'rear_bottom_right': [2898.650146484375, 313.3763732910156, 125.1927719116211],
#     'rear_top_left': [2900.691162109375, 313.3919982910156, 126.58153533935547],
#     'rear_top_right': [2898.651123046875, 313.3916931152344, 126.58269500732422]
# }
#
# # 获取 front_bottom_left 的坐标
# front_bottom_left = box['front_bottom_left']
# print(front_bottom_left)  # 输出：[2900.689697265625, 318.0464172363281, 125.1402359008789]
#
# # 修改 front_bottom_right 的 x 坐标
# box['front_bottom_right'][0] = 2900.5
#
# # 添加一个新的角点坐标
# box['top_center'] = [2899.5, 320.0, 127.0]
#
# print(box)  # 输出整个修改后的字典