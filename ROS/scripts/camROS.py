#!/usr/bin/python3
import numpy as np 
import cv2 as cv 
import rospy
import time
from std_msgs.msg import Int32
from threading import Thread, Lock

global mid_x
mid_x = Int32()
global mid_y
mid_y = Int32()

#use of python threading to increase camera decode performance
class WebcamVideoStream :
    def __init__(self, src = 0,width=176, height=144) :
        self.stream = cv.VideoCapture(src)
        self.stream.set(cv.CAP_PROP_FRAME_WIDTH,width)
        self.stream.set(cv.CAP_PROP_FRAME_HEIGHT,height)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.read_lock = Lock()
    
    def start(self) :
        if self.started :
            print ("already started!!")
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self) :
        while self.started :
            (grabbed, frame) = self.stream.read()
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

    def read(self) :
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self) :
        self.started = False
        if self.thread.is_alive():
           self.thread.join()

    def __exit__(self, exc_type, exc_value, traceback) :
        self.stream.release()


def detection(data):
    img = data
    h,w,c = img.shape
    # changing color space to HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # setting threshold limits for yellow color filter
    lower = np.array([20, 100, 100])
    upper = np.array([50, 255, 255])

    # creating mask
    mask = cv.inRange(hsv, lower, upper)

    m = cv.moments(mask, False)
    try:
        cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
    except ZeroDivisionError:
        cy, cx = int(h / 2), int(w / 2)

    # Publish centroid
    mid_x.data = cx
    mid_y.data = cy
    cv.circle(img, (mid_x.data, mid_y.data), 7, (255, 0, 0), -1)
    pub.publish(mid_x)
    # plotting results
    try:
        #cv2.imshow("original", orig)
        #cv2.imshow("yellow mask", mask)
        #cv.imshow("plotting centroid", img)
        #cv.waitKey(1)
        dummy=1

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pub = rospy.Publisher('/centroid_NEW', Int32, queue_size=1)
    rospy.init_node("cam_NEW",anonymous=True)
    cap = WebcamVideoStream().start()
    num_frames = 0

    while not rospy.is_shutdown():
        image = cap.read()
        if num_frames == 0:
            start = time.time()
            print('RUNNING')
            print(image.shape)
        detection(image)
        num_frames = num_frames + 1

    #calculate FPS
    end = time.time()
    sec = end - start
    print(" FPS: " + str(int(num_frames/sec)))
    cap.stop()
    cv.destroyAllWindows()

