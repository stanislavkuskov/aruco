import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import math
import serial as ser
import time
import os
import argparse

class aruco_detector():
    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()
        self.serial_port=None
        # self.KOFF=0.424242
        self.KOFF=0.19444

    def serial_init(self):
        # os.stat('/dev/ttyACM0')
        os.chmod('/dev/ttyAMA0', 0777)
        self.serial_port = ser.Serial(
            port='/dev/ttyAMA0',
            baudrate=9600,
            dsrdtr=1,
            timeout=0,
            parity=ser.PARITY_NONE,
            stopbits=ser.STOPBITS_ONE,
            bytesize=ser.EIGHTBITS
        )

        time.sleep(1)

    def serial_send(self,string):
        self.serial_port.write(string)
        # self.serial.write(helm_angle_cmd)
        pass

    def detect_marker(self,frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # print(parameters)

        '''    detectMarkers(...)
            detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
            mgPoints]]]]) -> corners, ids, rejectedImgPoints
            '''
        # lists of ids and the corners beloning to each id
        corners, marker_id, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        marker_id_res=marker_id[0]
        corner_res=corners[0]

        i=0
        end=False

        for marker in marker_id:
            if marker>50 and marker<81:
                marker_id_res=marker_id[i]
                corner_res=corners[i]
                end=True
            if end==True:
                break
            i=i+1


        return marker_id_res,corner_res

    def calculate_points(self,corners,marker_id,frame):

        c=corners
        rect = cv.minAreaRect(c)

        # # center

        marker_center = np.array([int(rect[0][0]),int(rect[0][1])])
        marker_angle = np.array([int(c[0][0][0]), int(c[0][0][1])])
        xa=int(c[0][0][0])
        xb=int(c[0][1][0])
        ya=int(c[0][0][1])
        yb=int(c[0][1][1])

        xc=(xa+xb)/2
        yc=(ya+yb)/2
        marker_angle_center = np.array([xc,yc])

        return marker_center, marker_angle, marker_angle_center


    def calculate_angle(self,center, marker_angle):

        angle = np.degrees(np.arctan2((float(marker_angle[0])-float(center[0])),(float(marker_angle[1])-float(center[1]))))
        #angle=(-angle)
        # angle = np.arctan((float(marker_angle[1])-float(center[1]))/(float(marker_angle[0])-float(center[0])))
        #if angle < 0:angle = np.fabs(angle)
        #else: angle = 360 - angle
        #angle_in_360=(angle+180)-90
        # print 'Angle : ',
        # print angle
        # print
        return angle


    def calculate_position(self,marker_id,marker_xy,center_marker_position,frame_center,angle):
        # print(marker_xy,center)
        k=0.1


        xc=int(frame_center)  # center kadra in pixel
        yc=int(frame_center)  #rabotaet

        x=marker_xy[0]
        y=marker_xy[1]

        x,y=y,x  #Ya oshibsya s osymi, po etomu tak
        #
        # print 'Center marcera na pole',
        # print x,
        # print ' : ',
        # print y

        xt=center_marker_position[0]   #centr markera v sisteme kadra in pixel
        yt=center_marker_position[1]   #vrode rabotaet

        #smeshay nachalo sistemu koordinat v centr kadra
        # i perevoju v santimetry
        #xr=float((xt-xc)*self.KOFF)
        #yr=float((yt-yc)*self.KOFF)
        xr=float(xt-xc)
        yr=float(yt-yc)  # rabotaet

        #povorachivau sistemu koordinat
        print 'Angle : ',
        print angle

        angle=float(np.radians(angle))

        xr2 = float((xr*np.cos(angle)-yr*np.sin(angle))*self.KOFF)
        yr2 = float((xr*np.sin(angle)+yr*np.cos(angle))*self.KOFF)

        #polucau koordinaty centra kadra v globalnoy sisteme coordinat
        xr3 = int(x-xr2)
        yr3 = int(y-yr2)
        print marker_id,
        print '   Coordinate ',
        print (xr3,yr3)
        print

        position=np.array([xr3,yr3])

        return position


    def show_resilts(self,frame,marker_center,marker_angle_point,frame_center):
        cv.circle(frame, (marker_center[0], marker_center[1]), 3, (0, 0, 255), -1)
        cv.circle(frame, (marker_angle_point[0], marker_angle_point[1]), 3, (0, 255, 255), -1)
        cv.circle(frame, (frame_center, frame_center), 3, (255, 255, 0), -1)
        cv.imshow("frame",frame)
        # cv.circle(frame, (int(frame_center), int(frame_center)), 2, (255, 255, 255), thickness=1, lineType=8,
                  # shift=0)



markers=np.array([[51,52,53],
                  [54,55,56],
                  [57,58,59],
                  [60,61,62],
                  [63,64,65],
                  [66,67,68],
                  [69,70,71],
                  [72,73,74],
                  [75,76,77],
                  [78,79,80]
                  ])

marker_xy=np.array([[[22, 45],   [22, 75],   [22, 105]],
                    [[48, 45],   [48, 75],   [48, 105]],
                    [[74, 45],   [74, 75],   [74, 105]],
                    [[100, 45], [100, 75],   [100, 105]],
                    [[126, 45], [126, 75],   [126, 105]],
                    [[152, 45], [152, 75],   [152, 105]],
                    [[178, 45], [178, 75],   [178, 105]],
                    [[204, 45], [204, 75],   [204, 105]],
                    [[230, 45], [230, 75],   [230, 105]],
                    [[256, 45], [256, 75],   [256, 105]]])
AR=aruco_detector()
cam_ids=np.array([0,1,2,3,4,5,6,7,8,9])
cam_finded = False

try:
    ap = argparse.ArgumentParser()
    ap.add_argument("-cam", "--camera", required=True,
                            help="camera id")
    args = vars(ap.parse_args())
    if int(args["camera"]):
        cap = cv.VideoCapture(int(args["camera"]))
except:
    cap = cv.VideoCapture(0)
# cap = cv.VideoCapture(1)

frame_width = 640
frame_height=480

cap.set(3, frame_width)
cap.set(4, frame_height)
frame_center=frame_height/2

try:
    AR.serial_init()
except: print "serial is empty"

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = frame[0:frame_height, int((frame_width - frame_height) / 2):int(frame_width-(int(frame_width - frame_height)/ 2))]

    try:
        marker_id, corners=AR.detect_marker(frame)

        if marker_id:

            center, marker_angle, marker_angle_center = AR.calculate_points(corners, marker_id, frame)



            i, j = np.where(markers == int(marker_id))

            marker_i=i[0]
            marker_j=j[0]


            angle = int(AR.calculate_angle(center, marker_angle_center))

            position=AR.calculate_position(marker_id,marker_xy[marker_i][marker_j], center,frame_center,angle)

            if position[0]<99:
                px=str("0"+str(position[0]))
                if position[0]<10:
                    px="0"+px
            else: px=position[0]
            if position[1]<99:
                py = str("0" + str(position[1]))
                if position[1]<10:
                    py="0"+py
            else:py=position[1]
            data_string = "00/" + str(px) + "/" + str(py) + "/" + str(angle) + '\n'
            print(data_string)
            print(int(angle))
            # time.sleep(1)

            AR.show_resilts(frame, center, marker_angle_center, frame_center)

            if (AR.serial_port):
                AR.serial_send(data_string)
                time.sleep(0.05)
                # print AR.serial_port

            else:
                 print "serial port is empty"


        # calculate angle
        #     AR.calculate_angle(corners)


        # print(center,marker_angle)

    except:
        print("Marker is empty")
        data_string = "-5/0-5/0-5/-5" + '\n'
        #data_string = "-5/-5/-5/-5" + "\n"

        if (AR.serial_port):
           AR.serial_send(data_string)
           time.sleep(0.05)

        else:
           print "serial port is empty"


    # cv.imshow("frame", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()

