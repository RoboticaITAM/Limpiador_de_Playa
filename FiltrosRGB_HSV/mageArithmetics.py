# -*- coding: utf-8 -*-
"""
Created on Thu Sep  5 11:41:16 2024

@author: RoboCub
"""
import cv2 as cv
# Load two images
img1 = cv.imread('cp.png')
img2 = cv.imread('logo.png')
assert img1 is not None, "file could not be read, check with os.path.exists()"
assert img2 is not None, "file could not be read, check with os.path.exists()"
 
# I want to put logo on top-left corner, So I create a ROI
rows,cols,channels = img2.shape
roi = img1[0:rows, 0:cols]
print(rows)
print(cols)
# Now create a mask of logo and create its inverse mask also
img2gray = cv.cvtColor(img2,cv.COLOR_BGR2GRAY)
ret, mask = cv.threshold(img2gray, 75,255, cv.THRESH_BINARY_INV)

print(mask[60,150])
print(mask[0,0])
mask_inv = cv.bitwise_not(mask)
cv.imshow('res1',img2gray)
cv.waitKey(0)
cv.destroyAllWindows()
 
# Now black-out the area of logo in ROI
img1_bg = cv.bitwise_and(roi,roi,mask = mask_inv)
cv.imshow('res2',img1_bg)
cv.waitKey(0)
cv.destroyAllWindows()
 
# Take only region of logo from logo image.
img2_fg = cv.bitwise_and(img2,(255,255,255),mask = mask)
cv.imshow('res3',img2_fg)
cv.waitKey(0)
cv.destroyAllWindows()
 
# Put logo in ROI and modify the main image
dst = cv.add(img1_bg,img2_fg)
img1[0:rows, 0:cols ] = dst
 
cv.imshow('res4',img1)
cv.waitKey(0)
cv.destroyAllWindows()




































