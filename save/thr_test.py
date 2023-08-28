import threading
import time
from time import sleep

from multiprocessing import Pool

def t1():
    print("t1开始采集")
    sleep(2)
    print("t1采集完成")

def t2():
    print("t2开始采集")
    sleep(1.5)
    print("t2采集完成")
def t3():
    print("t3开始采集")
    sleep(0.8)
    print("t3采集完成")

# 创建要执行的任务列表
# tasks = [t1, t2, t3]

def run(num):
    print(num)
    sleep(0.5)


start_time = time.time()
print("开始采集数据——————————————————————————")

for frame_id in range(4):
    # 创建并启动进程池
    tasks = [[1], [3], [23]]
    pool = Pool(2)
    results = []

    for task in tasks:
        result = pool.apply_async(run, args=task)
        results.append(result)

    pool.close()
    pool.join()

    # 获取结果（如果有需要的话）
    for result in results:
        result.get()

    current_time = time.time()
    print(f"已经采集 {current_time-start_time} seconds，第{frame_id}轮")

print("All tasks finished")



# for frame_id in range(4):
#     # 创建并启动线程
#
#     tasks = [[1], [3], [23]]
#     threads = []
#     for task in tasks:
#         thread = threading.Thread(target=run, args=task)
#         threads.append(thread)
#         thread.start()
#
#     # 等待所有线程结束
#     for thread in threads:
#         thread.join()
#         print("结束一个线程")
#     current_time = time.time()
#     print(f"已经采集 {current_time-start_time} seconds，第{frame_id}轮")
#
# print("All tasks finished")