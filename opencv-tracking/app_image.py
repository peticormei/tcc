import cv2
import numpy as np

from pyzbar.pyzbar import decode

img = cv2.imread('qr_test.png')

for qr in decode(img):
    pts = np.array([qr.polygon], np.int32)
    pts = pts.reshape((-1,1,2))

    cv2.polylines(img, [pts], True, (255, 0, 255), 3)

cv2.imshow('image', img)

cv2.waitKey(0)
cv2.destroyAllWindows()
