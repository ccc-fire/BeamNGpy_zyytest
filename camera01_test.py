import matplotlib.pyplot as plt

# 创建第一张图
fig1 = plt.figure(figsize=(5, 5))
ax1 = fig1.add_subplot(111)

# 创建第二张图
fig2 = plt.figure(figsize=(5, 5))
ax2 = fig2.add_subplot(111)
while True:


    # 在第一张图上进行绘图操作
    # ...
    plt.ion()



    # 在第二张图上进行绘图操作
    # ...

    plt.ion()
    plt.show()

    plt.pause(0.3)
