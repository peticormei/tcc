import cv2

import numpy as np
import matplotlib.pyplot as plt

white_color = np.array([0,0,0])
gray_color = np.array([125,125,125])
black_color = np.array([255,255,255])

img = cv2.imread('output/kang_bw3.jpg')

black_area = cv2.inRange(img, white_color, gray_color)
white_area = cv2.inRange(img, gray_color, black_color)

# plt.imshow(black_area, cmap='gray')
# plt.show()

def color_proportion(area):
    proportion = (area > 0).mean()
    percentage = round(proportion * 100, 2)
    return percentage

print(color_proportion(black_area))
print(color_proportion(white_area))
