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

timer = None
t = 5.0
key = None

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3


MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

degreeTot = 0
direction = 1

globalVar = ""

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
            

IP = '10.200.28.12'
PORT = 5010
client = ClientSocket(IP, PORT)
##client.start()
speech = "It goes it goes it goes it goes it goes YUH"

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

    def turnRight(self):
        self.headTurn += 500
        if(self.headTurn > 7900):
            self.headTurn = 7900
        self.tango.setTarget(HEADTURN, self.headTurn)   
    def turnLeft(self):
        self.headTurn -= 500
        if(self.headTurn < 1510):
            self.headTurn = 1510
        self.tango.setTarget(HEADTURN, self.headTurn)
    def tiltUp(self):
        self.headTilt += 500
        if(self.headTilt > 7900):
            self.headTilt = 7900
        self.tango.setTarget(HEADTILT, self.headTilt)
    def tiltDown(self):
        self.headTilt -= 500
        if(self.headTilt < 2000):
            self.headTilt = 2000
        self.tango.setTarget(HEADTILT, self.headTilt)
    def left(self):
        self.turn += 1000
        self.tango.setTarget(TURN, self.turn)
        print("LEFT")
        time.sleep(.25)
        self.turn -= 1000
        self.tango.setTarget(TURN, self.turn)
    def right(self):
        self.turn -= 1000
        self.tango.setTarget(TURN, self.turn)
        print("RIGHT")
        time.sleep(.25)
        self.turn += 1000
        self.tango.setTarget(TURN, self.turn)
    def forward(self):
        self.motors = 5000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.5)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
    def backward(self):
        self.motors = 7000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.5)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
        
    def searchForFaces(self):
        headTilt = self.headTilt
        headTurn = self.headTurn
        if(headTilt >= 7900):
            self.resetSearchTilt = True
        if(headTilt <= 2000):
            self.resetSearchTilt = False

        resetPan = self.resetSearchPan
        resetTilt = self.resetSearchTilt

        if(headTurn < 7900 and resetPan is False):
            self.turnRight()
        elif(headTurn >= 7900 and resetPan is False):
            self.resetSearchPan = True
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnLeft()
        elif(headTurn > 1510 and resetPan is True):
            self.turnLeft()
        elif(headTurn <= 1510 and resetPan is True):
            self.resetSearchPan = False
            if(resetTilt is False):
                self.tiltUp()
            else:
                self.tiltDown()
            self.turnRight()


controller = Control()


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
                searchForFaces()
                time.sleep(0.5)

    if key == ord("q") or key == 27:
        break

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to look for humans
        img = frame.array
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

        faces = detectFaces(img)
        if(faces is False):
            searchForFaces()
            time.sleep(0.5)
        elif(human too far left):
            controller.left()
            controller.turnRight()
        elif(human too far right):
            controller.turnRight()
            controller.left()
        else:
            if(human too high up):
                controller.tiltUp()
            elif(human too low down):
                controller.tiltDown()
            else:
                if(human too big):
                    controller.backward()
                elif(human too small):
                    controller.forward()
                else:
                    time.sleep(1)   
                    client.sendData(speech)
                    break

    if key == ord("q") or key == 27:
        break

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): #Loop to check if human has left frame
        img = frame.array
        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif(timer = None):
            timer = Timer(t, break)
            timer.start()
        else:
            faces = detectFaces(img)
            if(faces is not False):
                timer = Timer(t, break)
                timer.start()

    if key == ord("q") or key == 27:
        break

    
def detectFaces(self, frame):
    
    return
    #scan for faces
    #return dimensions of face if found, false otherwise


