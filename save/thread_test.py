# import threading
# import time
# from time import sleep
#
#
#
# def t1():
#     for _ in range(3):
#         sleep(2)
#         print("t1采集完成")
#
# def t2():
#     sleep(1.5)
#     print("t2采集完成")
# def t3():
#     sleep(0.8)
#     print("t3采集完成")
#
# # 创建要执行的任务列表
# # tasks = [t1, t2, t3]
#
# start_time = time.time()
# tasks = [t1, t2, t3]
# print("开始采集数据——————————————————————————")
# for frame_id in range(2):
#
#
#
#     # 创建并启动线程
#     threads = []
#     for task in tasks:
#         thread = threading.Thread(target=task)
#         threads.append(thread)
#         thread.start()
#
#     # 等待所有线程结束
#     for thread in threads:
#         thread.join()
#     current_time = time.time()
#     print(f"已经采集 {current_time-start_time} seconds，第{frame_id}轮")
#
# print("All tasks finished")

import queue
import threading
import time

# 创建线程安全的队列
shared_queue = queue.Queue()

def producer():
    for i in range(5):
        data = i + 1
        shared_queue.put(data)
        print("Producer: produced", data)
        # 模拟生产数据的耗时操作
        time.sleep(0.5)

def consumer():
    while True:
        data = shared_queue.get()
        if data is None:
            break
        print("Consumer: consumed", data)
        # 模拟处理数据的耗时操作
        time.sleep(1)

# 创建并启动生产者线程
producer_thread = threading.Thread(target=producer)
producer_thread.start()

# 创建并启动消费者线程
consumer_thread = threading.Thread(target=consumer)
consumer_thread.start()

# 等待生产者线程结束
producer_thread.join()

# 在队列中放入结束标志（None），以通知消费者线程结束
shared_queue.put(None)

# 等待消费者线程结束
consumer_thread.join()

print("All threads have finished.")

