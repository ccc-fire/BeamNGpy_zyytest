import os
from time import sleep
from multiprocessing import Pool
import time


def run(num, output):
    # if output is None:
    #     output = output_folder

    sleep(3)
    print(num, str(output))
    print("---")

if __name__ == '__main__':
    # 获取当前时间
    current_time = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    # 创建文件夹路径
    folder_path = r"C:\Users\HP\Desktop\camera_test"
    output_folder = os.path.join(folder_path, current_time)
    # 创建文件夹
    os.mkdir(output_folder)
    print("djwopfjowejkfew")


    print("Storing OUTPUT to ", output_folder)
    for frame_id in range(2):
        tasks = [[1, output_folder], [3, output_folder], [23, output_folder]]

        p = Pool(3)  # 创建一个大小等于cpu核数的进程池
        for task in tasks:  # 分配cpu核数个任务给每个进程并加入进程池
            p.apply_async(run, args=task)  # 这里用了异步非阻塞式

        p.close()  # 关闭进程池，不能再加入进程
        p.join()  # 主进程等进程池中所有子进程全结束再结束

    print("测试成功")
