import numpy as np

#          Returns:
#             A 6D point cloud of raw RADAR data, where each entry is (range, doppler velocity, azimuth angle, elevation angle, radar cross section, signal to noise ratio).
def save_ply_file(points, filename):
    num_points = len(points)

    # 创建PLY文件头
    header = "ply\n"
    header += " A 6D point cloud of raw RADAR data, where each entry is (range, doppler velocity, azimuth angle, elevation angle, radar cross section, signal to noise ratio)\n"
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


# 示例点云数据（格式：x, y, z, 距离, 速度, 方位角, 俯仰角, 雷达散射截面, 信噪比）
# 原始雷达数据的6D点云
data = [
    [4.810201168060303, -0.000489684462081641, 0.5100605487823486, 0.35734784603118896, 2.9076114515191875e-05, 48467762216960.0, 0.0],
    [4.823272705078125, -0.0004883459769189358, 0.5161690711975098, 0.35734784603118896, 2.923435386037454e-05, 48205400113152.0, 0.0],
    [4.836462020874023, -0.0004870025150012225, 0.5222775936126709, 0.35734784603118896, 2.939445766969584e-05, 47942845071360.0, 0.0],
    [4.849778652191162, -0.0004856679879594594, 0.528386116027832, 0.35734784603118896, 2.9556549634435214e-05, 47679925125120.0, 0.0],
    [4.863200664520264, -0.0004843157948926091, 0.5344946384429932, 0.35734784603118896, 2.9720373277086765e-05, 47417105842176.0, 0.0],
    [4.876737117767334, -0.0004829598474316299, 0.5406031608581543, 0.35734784603118896, 2.9886054107919335e-05, 47154236227584.0, 0.0]
]

# 保存为PLY文件
save_ply_file(data, "radar_data.ply")
