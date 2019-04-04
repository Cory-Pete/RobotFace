# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import maestro
from time import sleep
from threading import Timer
import socket
import threading
import queue

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')

faceSize = 10000
faceSizeTolerance = 2500
faceTolerance = 40

timer = None
t = 5.0
key = None

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

centerx = 640 / 2
centery = 480 / 2

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

degreeTot = 0
direction = 1

globalVar = ""

key = None
def setKey():
    global key
    print("setKey was called")

def detectFaces(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    #print(faces)
    thereAreFaces = False
    for (x,y,w,h) in faces:
        cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        thereAreFaces = True
        #roi_gray = gray[y:y+h, x:x+w]
        #roi_color = img[y:y+h, x:x+w]
        #eyes = eye_cascade.detectMultiScale(roi_gray)
    if(thereAreFaces):
        return faces
    else:
        return False

class ClientSocket(threading.Thread):
    def __init__(self, IP, PORT):
        super(ClientSocket, self).__init__()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((IP, PORT))
  
        print ('connected')
        self.alive = threading.Event()
        self.alive.set()

    def recieveData(self):
        global globalVar
        try:
            data = self.s.recv(105)
            print (data)
            globalVar = data
        except IOError as e:
            if e.errno == errno.EWOULDBLOCK:
                pass

    def sendData(self, sendingString):
        print ('sending')
        sendingString += "\n"
        self.s.send(sendingString.encode('UTF-8'))
        print ('done sending')

    def run(self):
        global globalVar
        while self.alive.isSet():
            data = self.s.recv(105)
            print (data)
            globalVar = data
            if(data == "0"):
                self.killSocket()
            
           
            
    def killSocket(self):
        self.alive.clear()
        self.s.close()
        print("Goodbye")
        exit()
            

IP = '10.200.35.21'
PORT = 5010
client = ClientSocket(IP, PORT)
#print(client)
##client.start()
speech = "It goes it goes it goes it goes it goes yuh"

class Control():
    def __init__(self):
        self.tango = maestro.Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.resetSearchPan = False
        self.resetSearchTilt = False
    
    def resetHead(self):
        self.headTilt = 6000
        self.headTurn = 6000
        self.tango.setTarget(HEADTURN, self.headTurn)
        self.tango.setTarget(HEADTILT, self.headTilt)
        print("RESET HEAD")

    def turnLeft(self):
        self.headTurn += 250
        if(self.headTurn > 7900):
            self.headTurn = 7900
        self.tango.setTarget(HEADTURN, self.headTurn)   
        print("TURN HEAD RIGHT")
    def turnRight(self):
        self.headTurn -= 250
        if(self.headTurn < 1510):
            self.headTurn = 1510
        self.tango.setTarget(HEADTURN, self.headTurn)
        print('TURN HEAD LEFT')        
    def tiltUp(self):
        self.headTilt += 500
        if(self.headTilt > 7900):
            self.headTilt = 7900
        self.tango.setTarget(HEADTILT, self.headTilt)
    def tiltDown(self):
        self.headTilt -= 500
        if(self.headTilt < 1510):
            self.headTilt = 1510
        self.tango.setTarget(HEADTILT, self.headTilt)
    def left(self):
        self.turn += 1200
        self.tango.setTarget(TURN, self.turn)
        print("LEFT")
        time.sleep(.25)
        self.turn -= 1200
        self.tango.setTarget(TURN, self.turn)
    def right(self):
        self.turn -= 1200
        self.tango.setTarget(TURN, self.turn)
        print("RIGHT")
        time.sleep(.25)
        self.turn += 1200
        self.tango.setTarget(TURN, self.turn)
    def forward(self):
        self.motors = 5000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.3)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
    def backward(self):
        self.motors = 7000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.3)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
        
    def searchForFaces(self):
        print("searching")
        headTilt = self.headTilt
        headTurn = self.headTurn
        if(headTilt >= 7900):
            self.resetSearchTilt = True
        if(headTilt <= 1510):
            self.resetSearchTilt = False

        resetPan = self.resetSearchPan
        resetTilt = self.resetSearchTilt

        if(headTurn < 7900 and resetPan is False):
            self.turnLeft()
        elif(headTurn >= 7900 and resetPan is False):
            self.resetSearchPan = True
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnRight()
        elif(headTurn > 1510 and resetPan is True):
            self.turnRight()
        elif(headTurn <= 1510 and resetPan is True):
            self.resetSearchPan = False
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnLeft()

    def alignMoveToFace(self, face):
        global faceSize, faceSizeTolerance, faceTolerance, centerx, centery
        largest = 0
        current = None
        for (x,y,w,h) in faces:
            if(w*h) > largest:
                largest = w*h
                current = [int(x+(w/2)), int(y+(h/2))]
        if(current[0] - centerx > faceTolerance): #too far right
            print("too far left, turn body right")
            self.right()
        elif(centerx - current[0] > faceTolerance): #too far left
            print("too far right, turn body left")
            self.left()
        elif(self.headTurn < 5900):
            print("head is turned right, turning it left now yeet")
            self.turnLeft()
        elif(self.headTurn > 6100):
            print("head is turned left, turning it right now")
            self.turnRight()
        else:
            if(current[1] - centery > faceTolerance):
                self.tiltDown()
            elif(current[1] - centery < -faceTolerance):
                self.tiltUp()
            else:
                if(largest > faceSize + faceSizeTolerance): #human too big
                    print("Go backward")
                    self.backward()
                elif(largest < faceSize - faceSizeTolerance):
                    print("Go forward")
                    self.forward()
                else:
                    return False
        return True

    


controller = Control()

time.sleep(2)

while(True):
    controller.resetHead()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
        img = frame.array
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        else:
            faces = detectFaces(img)
            if faces is not False:
                timer = None
                break
            else:
                controller.searchForFaces()
                time.sleep(0.5)
        cv.imshow("Image", img)
        rawCapture.truncate(0)

    rawCapture.truncate(0)
    if key == ord("q") or key == 27:
        break

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
        img = frame.array
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

        faces = detectFaces(img)
        if(faces is False):
            controller.searchForFaces()
            time.sleep(0.5)
        elif(controller.alignMoveToFace(faces) is False):
            time.sleep(1)
            print(speech)
            print(client)
            client.sendData(speech)
            break
        cv.imshow("Image", img)
        rawCapture.truncate(0)
    
    print("start of third loop")
    rawCapture.truncate(0)        

    if key == ord("q") or key == 27:
        break

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to check if human has left frame
        img = frame.array
        if key == ord("q") or key == 27:
            break
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif(timer is None):
            print("reached this one")
            timer = Timer(t, setKey)
            timer.start()
            print("reached this too")
        elif(not timer.is_alive()):
            break
        else:
            print("reached the other loop")
            faces = detectFaces(img)
            if(faces is not False):
                print("reset timer")
                timer.cancel()
                timer = Timer(t, setKey)
                timer.start()
        if key == ord("q") or key == 27:
            break
        cv.imshow("Image", img)
        rawCapture.truncate(0)
    rawCapture.truncate(0)

    if key == ord("q") or key == 27:
        break
cv.destroyAllWindows()
