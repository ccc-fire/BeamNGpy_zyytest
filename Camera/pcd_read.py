import numpy as np
import os
import csv
import open3d as o3d

pcd = o3d.io.read_point_cloud(r"C:\Users\HP\Desktop\file_test\20230811104604\lidar_Top_8.ply")

# pcd = o3d.io.read_point_cloud(r"C:\Users\HP\Desktop\file_test\20230811104604\radar_back_left_0")
# 进行可视化
o3d.visualization.draw_geometries([pcd])