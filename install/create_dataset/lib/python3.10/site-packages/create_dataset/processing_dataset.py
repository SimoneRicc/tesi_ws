import numpy as np
from matplotlib import pyplot as plt
import cv2


dataset = np.load('/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_not_processed/pose_array.npy')
dataset = dataset[:200]
print(dataset.shape)
image = cv2.imread('/home/simone/tesi_ws/src/create_dataset/create_dataset/dataset_not_processed/image_0.jpg')
x = dataset[:, 1]
y = dataset[:, 2]
plt.figure()
plt.scatter(x, y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Pose Plot')
plt.show()
