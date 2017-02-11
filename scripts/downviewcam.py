#!/usr/bin/env python

import rospy
from beginner_tutorials.msg import po 
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
itemWidthMin = 30
itemHeightMin = 30
itemWidthMax = 160
itemHeightMax = 160
distanceThreshold = 500
historicalRectangles = []
maxHistory = 5 # Remember only last five frames

def pub_position(position):

    pub = rospy.Publisher('po_chatter', po)
    rospy.init_node('downviewcam', anonymous=True)                                                                 
    rate = rospy.Rate(10) # 10hz
    msg = po()
    msg.x = position[0]
    msg.y = position[1]
    msg.command = "Pick the obects"
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

def deleteBackground(frame):
    return fgbg.apply(frame)

def blurFrame(frame):
    blurSize = 13
    return cv2.blur(frame, (blurSize, blurSize))

def threshold(frame):
    ret, frame = cv2.threshold(frame, 200, 255, cv2.THRESH_BINARY)
    return frame

def getContours(frame):
    contours, heirarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
    try:
        while (1):
            ret, origFrame = cap.read()
            origFrame_cut = origFrame[0:360,:]

            if (ret == True):
                frame = deleteBackground(origFrame_cut)
                #frame = blurFrame(frame)
                frame = threshold(frame)
                contours = getContours(frame)

                if len(historicalRectangles) == maxHistory:
                    historicalRectangles = historicalRectangles[1:]

                rectangles = getRectangles(contours)
                rectangles = mergeRectangles(rectangles)
                rectangles = filterBigRectangles(rectangles)

                historicalRectangles.append(rectangles)

                mergedHistoricalRectanges = []
                for rectangles in historicalRectangles:
                    for rectangle in rectangles:
                        mergedHistoricalRectanges.append(rectangle)

                mergedHistoricalRectanges = mergeRectangles(mergedHistoricalRectanges)
                mergedHistoricalRectanges = filterBigRectangles(mergedHistoricalRectanges)
                if len(mergedHistoricalRectanges) > 0:
                  #angle=anglefind(mergedHistoricalRectanges[0])
                  #print(angle)
                  position=getRectangleCenter(mergedHistoricalRectanges[0])
                  pub_position(position)
                  #if (position[0]>500) & (position[0]<800) &(position[1] > 300) &( position[1] < 420):
                      #print("stop!")

                      #pub_position(position)
                else:
                  position=None

                frame = drawRectangles(origFrame_cut, mergedHistoricalRectanges)
                #font = cv2.FONT_HERSHEY_SIMPLEX
                #cv2.putText(frame, repr(angle), (100, 130), font, 1, (200, 255, 155), 2, cv2.CV_AA)
                cv2.imshow('frame', frame)

            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException: pass
