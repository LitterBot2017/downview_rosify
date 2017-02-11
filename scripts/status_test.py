#!/usr/bin/env python

import rospy
from downview_cam.msg import po 
import numpy as np
import cv2
import math

# Get capture stream
frameWidth=1280
frameHeight=720
cameraFparm= 813.0
cap = cv2.VideoCapture(1)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, frameWidth)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, frameHeight)

fgbg = cv2.BackgroundSubtractorMOG()
#20  80
itemWidthMin = 10
itemHeightMin = 10
itemWidthMax = 300
itemHeightMax = 300
distanceThreshold = 50
historicalRectangles = []
maxHistory = 5 # Remember only last five frames


# this function will publish the position coordiantes provided that 
# the position is a positive pixel number
def pub_position(position,pub):

    centered_thres = 50

    msg = po()

    if (position[0] == -1) or (position[1] == -1):
        msg.x = position[0]
        msg.y = position[1]
        msg.command = "not_detected"
    else:
        msg.x = position[0]
        msg.y = position[1]
        msg.command = "detected"
        print "errors" + repr((abs(position[0]-frameWidth/2.0))) + " " + (repr((abs(position[1]-(500.0/2)))))
        # if the can is sufficiently centered
        if (abs(position[0]-frameWidth/2.0)<centered_thres) and (abs(position[1]-(500.0/2))<centered_thres):
            msg.command = 'centered'
    # rospy.loginfo(msg)
    pub.publish(msg)

def deleteBackground(frame):
    return fgbg.apply(frame)

def blurFrame(frame):
    blurSize = 13
    return cv2.blur(frame, (blurSize, blurSize))

def threshold(frame):
    img_mean = np.mean(frame)
    img_std = np.std(frame)
    ret, frame = cv2.threshold(frame, img_mean-2*img_std, 255, cv2.THRESH_BINARY)
    # frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,21,2)
    return frame

def getContours(frame):
    contours, heirarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return contours

def getRectangles(contours):
    rectangles = []
    for contour in enumerate(contours):
        x, y, w, h = cv2.boundingRect(contour[1])
        if w > itemWidthMin and h > itemHeightMin:
            rectangles.append((x, y, w, h))
    return rectangles

def drawRectangles(frame, rectangles):
    for rectangle in rectangles:
        x = rectangle[0]
        y = rectangle[1]
        w = rectangle[2]
        h = rectangle[3]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255,0,0), 2)
    return frame

def getRectangleCenter(rectangle):
    x = rectangle[0]
    y = rectangle[1]
    w = rectangle[2]
    h = rectangle[3]
    return ((x + (w/2)), (y + (h/2)))

def mergeRectangle(rectangle1, rectangle2):
    x1 = rectangle1[0]
    y1 = rectangle1[1]
    w1 = rectangle1[2]
    h1 = rectangle1[3]
    x2 = rectangle2[0]
    y2 = rectangle2[1]
    w2 = rectangle2[2]
    h2 = rectangle2[3]

    xDirection = [x1, (x1 + w1), x2, (x2 + w2)]
    yDirection = [y1, (y1 + h1), y2, (y2 + h2)]
    xDirection.sort()
    yDirection.sort()
    return (xDirection[0], yDirection[0], (xDirection[3] - xDirection[0]), (yDirection[3] - yDirection[0]))

def mergeRectangles(rectangles):
    for firstIndex, firstRectangle in enumerate(rectangles):
        for secondIndex, secondRectangle in enumerate(rectangles[firstIndex + 1:]):
            rect1 = getRectangleCenter(firstRectangle)
            rect2 = getRectangleCenter(secondRectangle)
            dist = math.sqrt((rect1[0] - rect2[0])**2 + (rect1[1] - rect2[1])**2)
            if dist < distanceThreshold:
                realSecondIndex = firstIndex + secondIndex + 1
                newRectangles = rectangles[0:firstIndex]
                newRectangles += rectangles[firstIndex + 1:realSecondIndex]
                newRectangles += rectangles[realSecondIndex + 1:]
                newRectangles += [ mergeRectangle(firstRectangle, secondRectangle) ]
                return mergeRectangles(newRectangles)
    return rectangles

def filterBigRectangles(rectangles):
    filteredRectangles = []
    for rectangle in rectangles:
        if (rectangle[2] < itemWidthMax) and (rectangle[3] < itemHeightMax):
            filteredRectangles.append(rectangle)
    return filteredRectangles

def anglefind(rectangle):
    rect=getRectangleCenter(rectangle)
    temp=(frameWidth/2-rect[0])/cameraFparm
    angle= math.degrees(math.atan(temp))
    return angle

if __name__ == '__main__':

    #initialize node stuff here
    rospy.init_node('downviewcam', anonymous=True)

    pub = rospy.Publisher('down_cam_msg', po)

    rate = rospy.Rate(10) # 10hz

    try:
        while not rospy.is_shutdown():
            ret, origFrame = cap.read()
            origFrame_cut = origFrame[0:480,:]

            if (ret == True):
                # frame = deleteBackground(origFrame_cut)
                #frame = blurFrame(frame)
                down_frame = cv2.pyrDown(origFrame_cut)
                down_frame = cv2.pyrDown(down_frame)
                down_frame = cv2.pyrMeanShiftFiltering(down_frame, 5, 75)
                down_frame = cv2.pyrUp(down_frame)
                down_frame = cv2.pyrUp(down_frame)
                # frame = threshold(frame)
                # cv2.imshow('down_frame_upped',down_frame)
                down_frame = cv2.cvtColor(down_frame,cv2.cv.CV_RGB2GRAY)
                # cv2.imshow('down_frame_upped',down_frame)
                down_frame = threshold(down_frame)
                cv2.imshow('down_frame_upped',down_frame)
                
                # contours = getContours(frame)
                contours = getContours(down_frame)
                # cv2.drawContours(down_frame, contours, -1, (0,255,0), 10)
                # cv2.imshow('down_frame_upped',down_frame)

                # if len(historicalRectangles) == maxHistory:
                #     historicalRectangles = historicalRectangles[1:]

                rectangles = getRectangles(contours)
                rectangles = mergeRectangles(rectangles)
                rectangles = filterBigRectangles(rectangles)

                # historicalRectangles.append(rectangles)

                # mergedHistoricalRectanges = []
                # for rectangles in historicalRectangles:
                #     for rectangle in rectangles:
                #         mergedHistoricalRectanges.append(rectangle)

                # mergedHistoricalRectanges = mergeRectangles(mergedHistoricalRectanges)
                # mergedHistoricalRectanges = filterBigRectangles(mergedHistoricalRectanges)
                # if len(mergedHistoricalRectanges) > 0:
                if len(rectangles) > 0:
                  #angle=anglefind(mergedHistoricalRectanges[0])
                  #print(angle)
                  # position=getRectangleCenter(mergedHistoricalRectanges[0])
                  position = getRectangleCenter(rectangles[0])
                  pub_position(position, pub)
                  #if (position[0]>500) & (position[0]<800) &(position[1] > 300) &( position[1] < 420):
                      #print("stop!")

                      #pub_position(position)
                else:
                  position= [-1]*2
                  pub_position(position, pub)

                # frame = drawRectangles(origFrame_cut, mergedHistoricalRectanges)
                frame = drawRectangles(origFrame_cut, rectangles)
                #font = cv2.FONT_HERSHEY_SIMPLEX
                #cv2.putText(frame, repr(angle), (100, 130), font, 1, (200, 255, 155), 2, cv2.CV_AA)
                cv2.imshow('frame', frame)
                # cv2.imshow('frame',frame)

            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break

            rate.sleep()

        cap.release()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException: pass
