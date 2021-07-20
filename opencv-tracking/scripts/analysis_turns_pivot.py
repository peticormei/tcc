import cv2
import math
import numpy as np

import matplotlib.pyplot as plt

Q1 = 1
Q2 = 2
Q3 = 3
Q4 = 4

state_q = {
    1: False,
    2: False,
    3: False,
    4: False,
}

# cap = cv2.VideoCapture("http://192.168.0.37:4747/video")

# filename = 'pivot_90_turn_5.csv'

# cap = cv2.VideoCapture("scripts/new_videos/pivot_90_turn_1.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_90_turn_2.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_90_turn_3.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_90_turn_4.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_90_turn_5.avi")

# filename = 'data/pivot_180_turn_5.csv'

# cap = cv2.VideoCapture("scripts/new_videos/pivot_180_turn_1.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_180_turn_2.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_180_turn_3.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_180_turn_4.avi")
# cap = cv2.VideoCapture("scripts/new_videos/pivot_180_turn_5.avi")

def quadrant(x, y):
    if (x < 320):
        if (y < 240):
            return Q1
        return Q2
    elif (y < 240):
        return Q4
    return Q3

def angleT(points):
    (x1, y1) = points[0]
    (x2, y2) = points[1]

    q = quadrant(x1, y1)
    
    try:
        angle = int(math.atan((y1-y2)/(x2-x1))*180/math.pi)

        # if (q == Q3):
        #     angle = 180 + angle
        # elif (q == Q4):
        #     angle = -180 + angle


        if (q == Q1):
            state_q[Q1] = True
            state_q[Q2] = False
            if state_q[Q4]:
                state_q[Q4] = False
        elif (q == Q2):
            state_q[Q1] = False
            state_q[Q2] = True
            if state_q[Q3]:
                state_q[Q3] = False
        elif (q == Q3 ):
            state_q[Q3] = True
            if state_q[Q4] and state_q[Q2]:
                state_q[Q4] = False
        elif (q == Q4):
            if state_q[Q3] and state_q[Q1]:
                state_q[Q3] = False
            state_q[Q4] = True

        if (state_q[Q1]):
            if (state_q[Q4]):
                if (state_q[Q3]):
                    return abs(angle) + 180, q
                return 180 - abs(angle), q
            return abs(angle), q

        if (state_q[Q2]):
            if (state_q[Q3]):
                if (state_q[Q4]):
                    return abs(angle) + 180, q
                return 180 - abs(angle), q
            return abs(angle), q

        return abs(angle), q
    except:
        if (y < 120):
            return -90, q
        elif (y > 360):
            return 90, q
        elif (q == Q4):
            return 180, q
        return 90, q

initP = None

ans = []

reference_point = (0,0)
angle_point = (0,0)

while True:
    ret, frame = cap.read()

    if ret == True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        parameters = cv2.aruco.DetectorParameters_create()
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if corners:
            for tuple in zip(corners, ids):
                c, i = tuple
                aruco_id = np.int0(i)

                if (aruco_id[0] == 1):
                    reference_corners = np.int0(c)

                    rect = cv2.minAreaRect(reference_corners[0])
                    (reference_x, reference_y), (w, h), angle = rect

                    reference_point = (int(reference_x), int(reference_y))

                    cv2.circle(frame, reference_point, 2, (0, 0, 255), -1)
                    cv2.putText(frame, f'{aruco_id}', reference_point, cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 0])
                else:
                    int_corners = np.int0(c)
                    rect = cv2.minAreaRect(int_corners[0])
                    (x,y), (w, h), angle = rect

                    angle_point = (int(x), int(y))

                    cv2.polylines(frame, int_corners, True, (0, 255, 0), 5)
                    cv2.putText(frame, f'{aruco_id}', angle_point, cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 0])

            points = [angle_point, reference_point]

            an, q = angleT(points)

            ans.append((an, (cap.get(cv2.CAP_PROP_POS_MSEC) / 1000), (int(x), int(y))))

            # if (len(ans) == 0 or ans[-1] != an):
            #     ans.append(an)
            #     print(ans)

            cv2.putText(frame, f'{an} {q} {angle}', (50, 50), cv2.FONT_HERSHEY_PLAIN, 1, [255, 255, 0])

        cv2.circle(frame, (320, 240), 2, (0, 0, 255), -1)

        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1)

        if key == 27:
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()

t = []
v = []
e = []
r = []


with open(filename, 'w') as file:
    for i in range(len(ans)):
        file.write(f'{ans[i][0]},{ans[i][1]}\n')
        # t.append(ans[i][0])
        # v.append(ans[i][1])
        # e.append(ans[i][2][0])
        # r.append(ans[i][2][1])

# plt.plot(v, t)
# plt.ylim([0, 120])
# plt.ylabel('Ângulo')
# plt.xlabel('Tempo (Segundos)')
# plt.grid()
# plt.show()

# print(len(e))
# print(len(r))

# plt.plot(e, r)
# plt.ylabel('some numbers')
# plt.xlim([0, 1280])
# plt.ylim([0, 720])
# plt.show()
