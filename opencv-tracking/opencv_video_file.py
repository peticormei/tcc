import cv2
import numpy as np

from pyzbar.pyzbar import decode

centroids = []
polygons = []

cap = cv2.VideoCapture('IMG_2338.mp4')

def centeroidnp(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return int(sum_x/length), int(sum_y/length)

if (cap.isOpened()== False):
    print("Error opening video stream or file")

while (cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        resize = cv2.resize(frame, None, fx=0.35, fy=0.35, interpolation=cv2.INTER_LINEAR)

        for qr in decode(resize):
            pts = np.array([qr.polygon], np.int32)
            pts = pts.reshape((-1,1,2))

            polygons.append(pts)
            centroids.append(centeroidnp(np.array(qr.polygon)))

        # for i in range(0, len(polygons)):
        #     cv2.fillPoly(resize, [polygons[i]], color=(255, 255, 255))

        cv2.polylines(resize, [pts], True, (255, 0, 255), 3)

        for i in range(1, len(centroids)):
            cv2.line(resize, centroids[i - 1], centroids[i], (0, 0, 255), 10)

        cv2.imshow('Frame', resize)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break


cv2.imwrite('output/kang_og.jpg', resize)

height, width, channels = resize.shape 
img_BLACK = np.zeros((height, width, channels), np.uint8)
img_BLACK2 = img_BLACK.copy()

for i in range(0, len(polygons)):
    cv2.fillPoly(img_BLACK, [polygons[i]], color=(255, 255, 255))

colours, counts = np.unique(img_BLACK.reshape(-1,3), axis=0, return_counts=1)

for index, colour in enumerate(colours):
    count = counts[index]
    proportion = (100 * count) / (height * width)
    print(f"   Colour: {colour}, count: {count}, proportion: {proportion:.2f}%")

cv2.imwrite('output/kang_bw.jpg', img_BLACK)

for i in range(1, len(centroids)):
    cv2.line(img_BLACK2, centroids[i - 1], centroids[i], (0, 0, 255), 10)

cv2.imwrite('output/kang_route.jpg', img_BLACK2)

cap.release()
cv2.destroyAllWindows()
