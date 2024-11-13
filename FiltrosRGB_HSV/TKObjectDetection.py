# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 09:56:42 2024

@author: luis2
"""
import tkinter
import screeninfo
import cv2 as cv
from PIL import ImageTk, Image
import os
import numpy as np
os.chdir(os.path.dirname(os.path.abspath(__file__)))
#%%
from screeninfo import get_monitors
for m in get_monitors():
    print(str(m))
    
#%%
with open("Captures/n.txt","w") as f:
    f.write("0")
    
    
#%%
with open("Captures/n.txt","r") as f:
    captures = int(f.read())
    print(captures)
    
captures = captures + 1

with open("Captures/n.txt","w") as f:
    f.write(str(captures))
    print(captures)

#%%
root = tkinter.Tk()
cam = cv.VideoCapture(1)
res, screenshot = cam.read()
showCenter = True
capturePath = "Captures"
captures = 0
# with open("Captures/n.txt","r") as f:
#     captures = int(f.read())

# def capture():
    
#     with open(capturePath+"/cap"+ str(captures) + ".txt", "w") as f:
#         f.write(str(captures))
#     captures = captures+1
    
def getTrackbars():
    arr = np.zeros((14))
    for i in range(len(sliderFrame.sliders)):
        arr[i] = sliderFrame.sliders[i].get()
    return arr

def mainLoop():
    res, screenshot = cam.read()
    #bw = cv.cvtColor(screenshot, cv.COLOR_BGR2GRAY)
    blue,green,red = cv.split(screenshot)
    hsv = cv.cvtColor(screenshot, cv.COLOR_BGR2HSV)
    tb = getTrackbars()
    hsv_lower_boundary = np.array([tb[8],tb[10],tb[12]])
    hsv_upper_boundary = np.array([tb[9],tb[11],tb[13]])
    hsvmask = cv.inRange(hsv, hsv_lower_boundary, hsv_upper_boundary)
    
    f,bmask_low = cv.threshold(blue, tb[0], 255, cv.THRESH_BINARY)
    f,bmask_high = cv.threshold(blue, tb[1], 255, cv.THRESH_BINARY_INV)
    bmask = cv.bitwise_and(bmask_low, bmask_high)
    f,gmask_low = cv.threshold(green, tb[2], 255, cv.THRESH_BINARY)
    f,gmask_high = cv.threshold(green, tb[3], 255, cv.THRESH_BINARY_INV)
    gmask = cv.bitwise_and(gmask_low, gmask_high)
    f,rmask_low = cv.threshold(red, tb[4], 255, cv.THRESH_BINARY)
    f,rmask_high = cv.threshold(red, tb[5], 255, cv.THRESH_BINARY_INV)
    rmask = cv.bitwise_and(rmask_low, rmask_high)
    colormask = cv.bitwise_and(bmask, gmask)
    colormask = cv.bitwise_and(colormask, rmask)
    
    finalmask = cv.bitwise_and(colormask, hsvmask)
    
    final = cv.bitwise_and(screenshot,screenshot, mask= finalmask)
    edges = cv.Canny(final, tb[6], tb[7])
    center = np.round([np.average(indices) for indices in np.where(edges >= 255)]).astype(int)
    screenshot = cv.rectangle(screenshot,(center[1],center[0]),(center[1]+10,center[0]+10),color =(0,255,0),thickness =2)
    
    
    root.ssIm = ImageTk.PhotoImage(master = root,image = Image.fromarray(cv.resize(\
                                cv.cvtColor(screenshot,cv.COLOR_BGR2RGB),(int(w1/3),502))))
    camLb.configure(image = root.ssIm)
    camLb.pack()
    
    root.cMIm = ImageTk.PhotoImage(master = root,image = Image.fromarray(cv.resize(\
                                cv.cvtColor(colormask,cv.COLOR_BGR2RGB),(int(w1/3),502))))
    cMLb.configure(image = root.cMIm)
    cMLb.pack()
    
    root.hMIm = ImageTk.PhotoImage(master = root,image = Image.fromarray(cv.resize(\
                                cv.cvtColor(hsvmask,cv.COLOR_BGR2RGB),(int(w1/3),502))))
    hMLb.configure(image = root.hMIm)
    hMLb.pack()
    
    root.eIm = ImageTk.PhotoImage(master = root,image = Image.fromarray(cv.resize(\
                                edges,(int(w1/3),502))))
    eLb.configure(image = root.eIm)
    eLb.pack()
    
    root.fIm = ImageTk.PhotoImage(master = root,image = Image.fromarray(cv.resize(\
                                cv.cvtColor(final,cv.COLOR_BGR2RGB),(int(w1/3),502))))
    fLb.configure(image = root.fIm)
    fLb.pack()
   


def updateImages():
    mainLoop()
    root.update()
    root.after(0,updateImages)

# specify resolutions of both windows
w0, h0 = 1920, 1080
w1, h1 = 1920, 1080

root.geometry(f"{w1}x{h1}+0+0") # <- this is the key, offset to the right by w0
#root.geometry(f"{w1}x{h1}+{w0}+0") # <- this is the key, offset to the right by w0
root.state("zoomed")

#
#sstk = ImageTk.PhotoImage(master = root,image = Image.fromarray(cv.resize(pls,(int(w1/3),502))))
camFrame = tkinter.Frame(root, width = int(w1/3), height = 502,bg = "gray")
camLb = tkinter.Label(camFrame,borderwidth=0,highlightthickness=0)
camLb.pack()
camFrame.place(x = 0, y = 0)

colorMaskFrame = tkinter.Frame(root, width = int(w1/3), height = 501, bg = "red")
cMLb = tkinter.Label(colorMaskFrame,borderwidth=0,highlightthickness=0)
cMLb.pack()
colorMaskFrame.place(x = 0, y = 502)

hsvMaskFrame = tkinter.Frame(root, width = int(w1/3), height = 501, bg = "green")
hMLb = tkinter.Label(hsvMaskFrame,borderwidth=0,highlightthickness=0)
hMLb.pack()
hsvMaskFrame.place(x = int(w1/3), y = 502)

finalFrame = tkinter.Frame(root, width = int(w1/3), height = 502, bg = "blue")
fLb = tkinter.Label(finalFrame,borderwidth=0,highlightthickness=0)
fLb.pack()
finalFrame.place(x = int(w1/3), y = 0)

sliderFrame = tkinter.Frame(root, width = int(w1/3), height = 502,borderwidth=0,highlightthickness=0)
sliderFrame.sliders = []
tkinter.Label(sliderFrame, text = "Blue Low").grid(column = 0,row = 0)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[0].grid(column=1,row=0)
tkinter.Label(sliderFrame, text = "Blue High").grid(column = 0,row = 1)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[1].grid(column=1,row=1)
sliderFrame.sliders[1].set(255)
tkinter.Label(sliderFrame, text = "Green Low").grid(column = 0,row = 2)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[2].grid(column=1,row=2)
tkinter.Label(sliderFrame, text = "Green High").grid(column = 0,row = 3)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[3].grid(column=1,row=3)
sliderFrame.sliders[3].set(255)
tkinter.Label(sliderFrame, text = "Red Low").grid(column = 0,row = 4)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[4].grid(column=1,row=4)
tkinter.Label(sliderFrame, text = "Red High").grid(column = 0,row = 5)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[5].grid(column=1,row=5)
sliderFrame.sliders[5].set(255)
tkinter.Label(sliderFrame, text = "Canny Low").grid(column = 0,row = 6)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[6].grid(column=1,row=6)
sliderFrame.sliders[6].set(100)
tkinter.Label(sliderFrame, text = "Canny High").grid(column = 0,row = 7)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[7].grid(column=1,row=7)
sliderFrame.sliders[7].set(200)

tkinter.Label(sliderFrame, text = "Hue Low").grid(column = 2,row = 0)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[8].grid(column=3,row=0)
tkinter.Label(sliderFrame, text = "Hue High").grid(column = 2,row = 1)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[9].grid(column=3,row=1)
sliderFrame.sliders[9].set(255)
tkinter.Label(sliderFrame, text = "Saturation Low").grid(column = 2,row = 2)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[10].grid(column=3,row=2)
tkinter.Label(sliderFrame, text = "Saturation High").grid(column = 2,row = 3)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[11].grid(column=3,row=3)
sliderFrame.sliders[11].set(255)
tkinter.Label(sliderFrame, text = "Value Low").grid(column = 2,row = 4)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[12].grid(column=3,row=4)
tkinter.Label(sliderFrame, text = "Value High").grid(column = 2,row = 5)
sliderFrame.sliders.append(tkinter.Scale(sliderFrame, from_=0, to = 255,orient = tkinter.HORIZONTAL,length=200))
sliderFrame.sliders[13].grid(column=3,row=5)
sliderFrame.sliders[13].set(255)

sliderFrame.place(x = int(2*w1/3), y = 0)

edgeFrame = tkinter.Frame(root, width = int(w1/3), height = 501, bg = "black")
eLb = tkinter.Label(edgeFrame,borderwidth=0,highlightthickness=0)
eLb.pack()
edgeFrame.place(x = int(2*w1/3), y = 502)

root.after(0,updateImages())
root.mainloop()






