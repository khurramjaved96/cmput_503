import cv2
import matplotlib.pyplot as plt
import numpy as np
img = cv2.imread("sample.jpg")

transformation = np.array([[-2.971073552408333e-05, -0.00029659365828275936, -0.15023274145630686],
                           [0.0008685905912974388, -1.3800993051245391e-05, -0.27470055927337944],
                           [-0.0003125735350258431, -0.0072342082539416175, 1.0]])

img_2 = cv2.warpPerspective(img, transformation.T, (img.shape[1], img.shape[0]))
plt.imshow(img_2)
plt.show()