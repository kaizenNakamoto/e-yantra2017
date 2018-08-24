# Team ID:       Harvester Bot-eYRC#492

# Authors' List:            Kiran RS
                                            Chinmai R
		  Karthik S Nayak
		  Mithun M R

# Theme: 	  Harvester Bot(eYRC 2017)

import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
global ar
fruit_flag='n'
flist={'Medium Orange','Large Apple','Medium Apple','Small Apple','Large Blueberry','Medium Blueberry','Small Blueberry'}
typef='No Fruit'
pin1=12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin1,GPIO,intial=0)
GPIO.output(pin1,GPIO.LOW)
cap=cv2.VideoCapture(1);
cap.set(cv2.CAP_PROP_FRAME_WIDTH,160 );
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120);
#10.50.0.128.156.207,OPEN
def biggestContourI(contours):
    maxVal = 0
    maxI = None
    area=0
    for i in range(0, len(contours)):
        area=cv2.contourArea(contours[i])
        if(area>maxVal):
            maxVal=area;
            maxI=i;
    #print("ar",area)
    global ar
    ar=area;
    return maxI



def Orange_filter():
    _,f2=cap.read()
    f = f2[17:142, 56:144]
   # 0.29.0.255.0.125
    lowH = 0
    lowS = 0
    lowV = 0
    higH = 255
    higS = 144
    higV = 255
    lr=np.array([lowH,lowS,lowV]);
    hr = np.array([higH, higS, higV]);
    f1 = cv2.cvtColor(f, cv2.COLOR_BGR2HSV);
    blur = cv2.GaussianBlur(f1, (11, 11), 0);
    ker = np.ones((3,3));
    mask = cv2.inRange(blur, lr, hr);
    er = cv2.erode(mask, ker, iterations=1);
    _,er2=cv2.threshold(er,0,255,cv2.THRESH_BINARY_INV)
    global contours
    im2, contours, hierarchy = cv2.findContours(er2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    global cmax
    cmax = biggestContourI(contours)
    global fruit_flag
    if ar>130:
            fruit_flag='o';
            drawCont(fruit_flag,cmax,ar)
    else:
         Blue_filter()
def Apple_filter():
    _, f2 = cap.read()
    f = f2[17:142, 56:144]
    # 0.29.0.255.0.125
    lowH = 0
    lowS = 0
    lowV = 0
    higH = 255
    higS = 255
    higV = 255
    lr = np.array([lowH, lowS, lowV]);
    hr = np.array([higH, higS, higV]);
    f1 = cv2.cvtColor(f, cv2.COLOR_BGR2HSV);
    blur = cv2.GaussianBlur(f1, (11, 11), 0);
    ker = np.ones((3, 3));
    mask = cv2.inRange(blur, lr, hr);
    er = cv2.erode(mask, ker, iterations=1);
    _, er2 = cv2.threshold(er, 0, 255, cv2.THRESH_BINARY_INV)
    global contours
    im2, contours, hierarchy = cv2.findContours(er2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    global cmax
    cmax = biggestContourI(contours)
    global fruit_flag
    if ar>190:
        fruit_flag='a';
        drawCont(fruit_flag,cmax,ar)
    else:
        fruit_flag='n'
        drawCont(fruit_flag,cmax,ar)
def Blue_filter():
    _, f2 = cap.read()
    f = f2[17:142, 56:144]
    # 0.29.0.255.0.125
    lowH = 0
    lowS = 0
    lowV = 0
    higH = 255
    higS = 59
    higV = 255
    lr = np.array([lowH, lowS, lowV]);
    hr = np.array([higH, higS, higV]);
    f1 = cv2.cvtColor(f, cv2.COLOR_BGR2HSV);
    blur = cv2.GaussianBlur(f1, (11, 11), 0);
    ker = np.ones((3, 3));
    mask = cv2.inRange(blur, lr, hr);
    di = cv2.erode(mask, ker, iterations=1);
    _, er2 = cv2.threshold(di, 0, 255, cv2.THRESH_BINARY_INV)
    global contours
    im2, contours, hierarchy = cv2.findContours(er2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    global cmax
    cmax = biggestContourI(contours)
    M=cv2.moments(cmax)
    if(M['m00']!=0):
       cx=int(M['m10']/M['m00'])
       cy=int(M['m01']/M['m00'])
    else:
        cx=0
        cy=0
    col=f[cx,cy]
    print('bl---',col);
    global fruit_flag
    if ar>130:
        if(col[0]<150 and col[1]<150 and col[2]<150):
            fruit_flag='b';
            drawCont(fruit_flag,cmax,ar)
        else:
             Apple_filter()
    else:
        Apple_filter()

def drawCont(fruit_flag,cmax,ar):
    if(fruit_flag=='o'):
        typef='Orange'
    elif(fruit_flag=='a'):
        typef='Apple'
    elif(fruit_flag=='b'):
        typef='Blue Berry'
    elif(fruit_flag=='n'):
        typef='No Fruit'

    if(ar<=200):
        size='Small'
    elif(ar>=1150):
        size='Large'
    else:
        size='Medium'
    ftext=size+' '+typef
    if(typef!='No Fruit'):
        reqFruit(ftext)
        cv2.drawContours(f,contours,cmax,(0,0,0),1);
        cv2.putText(f,ftext,(25,30),cv2.FONT_HERSHEY_SIMPLEX, .3,(0,0,0),1)
        cv2.imshow('finale', f);
    else:
        cv2.putText(f,'No Fruit',(25,30),cv2.FONT_HERSHEY_COMPLEX,.3,(0,0,0),1)
        cv2.imshow('finale',f)

def reqFruit(ftext):
    if ftext in flist:
        pinON();
    else
        pinOF()
while(True):
    _, f2 = cap.read()
    f = f2[17:142, 56:144]
    fruit_flag='n'
    size='none'
    Orange_filter()
    print(fruit_flag)
    time.sleep(0.01)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
        break;

cap.release();
cv2.destroyAllWindows();
