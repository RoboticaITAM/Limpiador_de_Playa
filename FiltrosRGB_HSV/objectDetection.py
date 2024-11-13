# -*- coding: utf-8 -*-
"""
Created on Wed Oct  2 11:42:06 2024

@author: LARojas
"""
#%%Setup
import os
from time import time
import cv2 as cv
import numpy as np

os.chdir(os.path.dirname(os.path.abspath(__file__)))

#%%
Winname = "HSV:"
def nothing(x):
    pass

def getTrackbars():
    arr = np.zeros((14))
    arr[0] = cv.getTrackbarPos('BL', 'HSV:')
    arr[1] = cv.getTrackbarPos('GL', 'HSV:')
    arr[2] = cv.getTrackbarPos('RL', 'HSV:')
    arr[3] = cv.getTrackbarPos('BH', 'HSV:')
    arr[4] = cv.getTrackbarPos('GH', 'HSV:')
    arr[5] = cv.getTrackbarPos('RH', 'HSV:')
    arr[6] = cv.getTrackbarPos('H', 'HSV:')
    arr[7] = cv.getTrackbarPos('S', 'HSV:')
    arr[8] = cv.getTrackbarPos('V', 'HSV:')
    arr[9] = cv.getTrackbarPos('H2', 'HSV:')
    arr[10] = cv.getTrackbarPos('S2', 'HSV:')
    arr[11] = cv.getTrackbarPos('V2', 'HSV:')
    arr[12] = cv.getTrackbarPos('CL', 'HSV:')
    arr[13] = cv.getTrackbarPos('CH', 'HSV:')
    return arr

#%%Main Loop


cv.namedWindow('HSV:')
cv.resizeWindow('HSV:',300,600)

cv.createTrackbar('BL',Winname,0,255,nothing)
cv.createTrackbar('GL',Winname,0,255,nothing)
cv.createTrackbar('RL',Winname,0,255,nothing)
cv.createTrackbar('BH',Winname,255,255,nothing)
cv.createTrackbar('GH',Winname,255,255,nothing)
cv.createTrackbar('RH',Winname,255,255,nothing)
cv.createTrackbar('H',Winname,0,255,nothing)
cv.createTrackbar('S',Winname,0,255,nothing)
cv.createTrackbar('V',Winname,0,255,nothing)
cv.createTrackbar('H2',Winname,255,255,nothing)
cv.createTrackbar('S2',Winname,255,255,nothing)
cv.createTrackbar('V2',Winname,255,255,nothing)
cv.createTrackbar('CL',Winname,100,255,nothing)
cv.createTrackbar('CH',Winname,200,255,nothing)

cam = cv.VideoCapture(0)
loop_time = time()

while(True):
    
    res, screenshot = cam.read()
    bw = cv.cvtColor(screenshot, cv.COLOR_BGR2GRAY)
    blue,green,red = cv.split(screenshot)
    hsv = cv.cvtColor(screenshot, cv.COLOR_BGR2HSV)
    tb = getTrackbars()
    hsv_lower_boundary = np.array(tb[6:9])
    hsv_upper_boundary = np.array(tb[9:12])
    hsvmask = cv.inRange(hsv, hsv_lower_boundary, hsv_upper_boundary)
    
    f,bmask_low = cv.threshold(blue, tb[0], 255, cv.THRESH_BINARY)
    f,bmask_high = cv.threshold(blue, tb[3], 255, cv.THRESH_BINARY_INV)
    bmask = cv.bitwise_and(bmask_low, bmask_high)
    f,gmask_low = cv.threshold(green, tb[1], 255, cv.THRESH_BINARY)
    f,gmask_high = cv.threshold(green, tb[4], 255, cv.THRESH_BINARY_INV)
    gmask = cv.bitwise_and(gmask_low, gmask_high)
    f,rmask_low = cv.threshold(red, tb[2], 255, cv.THRESH_BINARY)
    f,rmask_high = cv.threshold(red, tb[5], 255, cv.THRESH_BINARY_INV)
    rmask = cv.bitwise_and(rmask_low, rmask_high)
    colormask = cv.bitwise_and(bmask, gmask)
    colormask = cv.bitwise_and(colormask, rmask)
    
    finalmask = cv.bitwise_and(colormask, hsvmask)
    
    final = cv.bitwise_and(screenshot,screenshot, mask= finalmask)
    edges = cv.Canny(final, tb[12], tb[13])
    
    center = np.round([np.average(indices) for indices in np.where(edges >= 255)]).astype(int)
    screenshot = cv.rectangle(screenshot,(center[1],center[0]),(center[1]+10,center[0]+10),color =(0,255,0),thickness =2)
    #print(center)
    
    
    cv.imshow("hsvmask",hsvmask)
    cv.imshow('cam',screenshot)
    cv.imshow("final:", final)
    cv.imshow("edges",edges)
    cv.imshow("colormask",colormask)
    
    print('FPS {}'.format(1 / (time() - loop_time)))
    loop_time = time()
    if cv.waitKey(1) == ord(' '):
        cv.imwrite("screenshot.png",screenshot)
        cv.imwrite("capture.png",screenshot)
        cv.imwrite("capture.png",screenshot)
        cv.imwrite("capture.png",screenshot)
        
    if cv.waitKey(1) == ord('q'):
        cv.destroyAllWindows()
        break

print('Done.')