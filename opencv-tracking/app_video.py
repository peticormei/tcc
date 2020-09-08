import cv2
import numpy as np

from pyzbar.pyzbar import decode

cap = cv2.VideoCapture(0)

cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, frame = cap.read()

    for qr in decode(frame):
        pts = np.array([qr.polygon], np.int32)
        pts = pts.reshape((-1,1,2))

        cv2.polylines(frame, [pts], True, (255, 0, 255), 3)

    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.release()
cv2.destroyAllWindows()
