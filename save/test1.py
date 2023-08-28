import threading
import time
from time import sleep

def t1():
    print("t1开始采集")
    sleep(1.2)
    print("t1采集完成")

def t2():
    print("t2开始采集")
    sleep(2)
    print("t2采集完成")
def t3():
    print("t3开始采集")
    sleep(0.8)
    print("t3采集完成")

# 创建要执行的任务列表
tasks = [t1, t2, t3]

start_time = time.time()
print("开始采集数据——————————————————————————")
for frame_id in range(11):
    current_time = time.time()
    elapsed_time = current_time - start_time

    if elapsed_time >= 10:  # 设置采集时间为10秒钟
        print("数据采集完成，时间为30秒，程序退出")
        break

    # 创建并启动线程
    threads = []
    for task in tasks:
        thread = threading.Thread(target=task)
        threads.append(thread)
        thread.start()

    # 等待所有线程结束
    for thread in threads:
        thread.join()
    print(f"已经采集 {elapsed_time} seconds，第{frame_id}轮")

print("All tasks finished")