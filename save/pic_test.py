import cv2
import os

frame_rate = 10.0  # 视频帧率
image_folder = 'C:\\Users\\HP\Desktop\\camera_test\\20230825132812-2'  # 图片所在文件夹路径
video_name = os.path.join(image_folder, 'output_video.mp4')  # 输出视频文件名

# 获取图片文件夹下所有图片文件的路径
images = [os.path.join(image_folder, img) for img in os.listdir(image_folder) if img.endswith(".jpeg")]
images.sort()  # 根据文件名排序

frame = cv2.imread(images[0])
height, width, _ = frame.shape  # 获取图像的高度和宽度
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 视频编码器
video = cv2.VideoWriter(video_name, fourcc, frame_rate, (width, height))

for image in images:
    frame = cv2.imread(image)
    video.write(frame)

video.release()
cv2.destroyAllWindows()
