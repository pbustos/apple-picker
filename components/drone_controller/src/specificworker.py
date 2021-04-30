#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
from dt_apriltags import Detector
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
import imutils


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        
        # Drone
        self.state = 'idle'
        self.x = 0
        self.y = 0
        self.pose = {}
        self.pose_home = {0.0022, 0.0000026, 0.00073, 0}
        self.appleCatched = False
        self.treeCatched = False


        #AprilTags
        self.center = 0
        self.tags = {}
        self.n_tags = 0
        self.at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        # Image
        self.image = []
        self.depth = []
        self.camera_name = "frontCamera"
        self.data = {}
        self.depth_array = []
        self.color = 0

        #Timer
        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def draw_image(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        #Marker is on X: 256 Y:256
        # cv.drawMarker(color, (int(color_.width/2), int(color_.height/2)),  (0, 255, 0), cv.MARKER_CROSS, 25, 2)
        cv.drawMarker(color, (200, 200),  (0, 0, 255), cv.MARKER_CROSS, 25, 2)
        plt.imshow(color)


    def draw_camera(self, color_, title):
        plt.figure(1)
        plt.clf()
        plt.imshow(color_)
        plt.title('Front Camera ')
        plt.pause(.00001)

    @QtCore.Slot()
    def compute(self):
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        self.image = all.image
        self.depth = all.depth
        self.draw_image(self.image)

        # Depth processing
        # Max value: 10.0       Min value from apple: between 0.18 and 0.19
        if len(self.depth.depth) <= 1048576:
            self.depth_array = np.frombuffer(self.depth.depth,dtype=np.float32).reshape(self.depth.height, 
                self.depth.width)

        if self.treeCatched == False:
            self.x, self.y, self.color = self.colorDetect(self.image, 36, 0, 0, 86, 255, 255) #green)
        elif self.appleCatched == False:
            self.x, self.y, self.color = self.colorDetect(self.image, 0, 50, 120, 10, 255, 255) #red)
        else:
            self.ar_detection(self.image)

        try: 
            self.appleSwitch()

        except Ice.Exception as e:
            print(e)

    
    def appleSwitch(self):
        
        if self.state == 'idle':
            self.idle()
        elif self.state == 'movex':
            self.movex()
        elif self.state == 'movey':
            self.movey()
        elif self.state == 'advance':
            self.advance()
        elif self.state == 'drop':
            self.drop()
        elif self.state == 'stop':
            self.stop()
        elif self.state == 'reverse':
            self.reverse()         
        elif self.state == 'turnleft': 
            self.turn_left()
        elif self.state == 'turnright':
            self.turn_right()
        elif self.state == 'tag':
            self.tag()      
        elif self.state == 'tree':
            self.tree() 
        
        else:
            self.error()

    # =============== Drone Movements ===================
    # ===================================================

    # PoseType parameters for movements
    # x = adv           rx: None
    # y = width         ry: None
    # z = height        rz: rotate
    def moveDummy(self, x_=0, y_=0, z_=0, rz_=0):
        self.pose = RoboCompCoppeliaUtils.PoseType(x=x_,y=y_,z=z_,rz=rz_)
        headCamera = RoboCompCoppeliaUtils.TargetTypes.HeadCamera
        self.coppeliautils_proxy.addOrModifyDummy(headCamera, "Quadricopter_target", self.pose)

    #State: idle
    def idle(self):
        print("[ IDLE ] depth = ", self.depth_array[200][200])

        if self.depth_array[200][200] < 1:
            self.moveDummy(x_=-0.002)
        elif self.depth_array[200][200] > 9:
            self.moveDummy(x_= 0.002)

        if self.x == 0:
            print("Looking for tree")
            self.state = 'tree'
        if self.x > 0 and self.color == 86:     #tree detected
            print("Tree detected")
            self.treeCatched = True
        if self.x > 0 and self.color == 10:     #apple detected
            print("Apple detected")
            self.state = 'movex'                #state = move x
    #State: move coordinate x
    def movex(self):
        if self.x > 260:
            self.moveDummy(y_=-0.001)
        if self.x < 260:
            self.moveDummy(y_=0.001)
        if self.x >= 250 or self.x <= 260:  
            print("[MOVE X] = ", self.x)  
            self.state = 'movey'    #state = move y

    #State: move coordinate y
    def movey(self):
        if self.y > 235:
            self.moveDummy(z_=0.001)
        if self.y < 235:
            self.moveDummy(z_=-0.001)
        if self.y >= 220 or self.y <= 235:
            print("[MOVE Y] = ", self.y)
            self.state = 'advance'  #state = advance

    #State: advance until the suctionPad catches the apple
    def advance(self):
        print("[ADVANCE] depth = ", self.depth_array[200][200])
        depth = self.depth_array[200][200]
        if depth > 0.30: 
            self.moveDummy(x_=0.01)
        else:
            self.moveDummy(x_=0.005)
        if (depth > 0.17 and depth < 0.185):
            print("DEPTH < ", depth)
            self.state = 'reverse'  #state = reverse
        else:                
            self.state = 'movex'    #state = move x
    
    # State: drop
    def drop(self):
        print("[ DROP ] depth: ", self.depth_array[200][200])
        self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Hand, "suctionPad_Dummy", None)
        self.state = 'turnright' 
        self.appleCatched = False
        self.treeCatched = False
    
    # State: stop
    def stop(self):
        print("[ STOP ]")

    # State: REVERSE
    def reverse(self):
        print("[ REVERSE ] depth: ", self.depth_array[200][200])
        self.moveDummy(x_=-0.002)
        if self.depth_array[200][200] >= 0.35:
            self.state = 'turnleft'
        self.appleCatched = True

     # State: turnleft
    def turn_left(self):
        print("[ TURN LEFT ] ", self.depth_array[200][200])
        if(self.n_tags == 0):        
            self.moveDummy(rz_=-0.007)
            self.n_tags = 0
        else:
            print("             + TAG SPOTTED")
            self.state = 'tag'

    # State: turnright
    def turn_right(self):
        print("[ TURN RIGHT ] depth", self.depth_array[200][200])
        if(self.depth_array[200][200] > 0.5):
            self.moveDummy(rz_=0.005)
        else:
            self.state = 'idle'

    # State: tag
    def tag(self):
        print("[ TAG ] ")
        # print("Centro X del tag: ",self.center[0])
        # print("Centro Y del tag: ",self.center[1])
        self.state = 'drop'

    # State: home POSE: 0	-2.0e-01	+6.50e-01
    #             ORI: 0	0	0
    def tree(self):
        print("[ TREE ] ")
        # self.moveDummy(x_=0.003)
        self.treeCatched = False
        self.state = 'idle'


    def error(self):
        print("Error en el switch")


    # =============== Object detection ===================
    # ====================================================
    def colorDetect(self, color_, low_h, low_s, low_v, high_h, high_s, high_v):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        x = 0
        y = 0
        color = cv.cvtColor(color, cv.COLOR_RGB2BGR)
        
        hsv = cv.cvtColor(color, cv.COLOR_BGR2HSV)
        lower_red_range = np.array([low_h, low_s, low_v])
        upper_red_range = np.array([high_h, high_s, high_v])
        mask = cv.inRange(hsv, lower_red_range, upper_red_range)
        # Find contours
        cnts = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts) 
        # Iterate through contours and filter by the number of vertices 
        for c in cnts:
            area = cv.contourArea(c)
            if area > 700: 
                cv.drawContours(color,[c],-1,(0,255,0), 2)
                M = cv.moments(c)
                x = int(M["m10"]/ M["m00"])
                y = int(M["m01"]/ M["m00"])

                cv.circle(color,(x,y),2,(0,255,0), 2)
                # cv.putText(color, "rojo", (x-20, y-20), cv.FONT_HERSHEY_SIMPLEX,2, (255,255,255), 2)

        color = cv.cvtColor(color, cv.COLOR_BGR2RGB)

        self.draw_camera(color, 'Drone Camera')
        return (x, y, high_h)

    def ar_detection(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        gray = cv.cvtColor(color, cv.COLOR_RGB2GRAY)
        k = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        cameraMatrix = np.array(k).reshape((3,3))
        camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

        tag_size = 0.065
        # if(self.n_tags == 0):
        self.tags = self.at_detector.detect(gray, False, camera_params, tag_size)
        if(len(self.tags) == 1):
            print("[ INFO ] {} total AprilTags detected".format(len(self.tags)))
            self.n_tags = 1
            
        if(self.n_tags == 1):
            for tag in self.tags:
                for idx in range(len(tag.corners)):
                    cv.line(color, tuple(tag.corners[idx-1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)), (0, 255, 0),thickness=2)
                    cv.circle(color, tuple(tag.center.astype(int)), 2, (0,255,0), 2)
                    self.center = tag.center.astype(int)
                    
                    print("Centro X del tag: ",self.center[0])
                    print("Centro Y del tag: ",self.center[1])  
            
        self.draw_camera(color, 'Drone Camera')

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)




    # =============== Slots methods for State Machine ===================
    # ===================================================================

    #
    # sm_initialize
    #
    @QtCore.Slot()
    def sm_initialize(self):
        #print("Entered state initialize")
        self.t_initialize_to_compute.emit()
        pass
    

    #
    # sm_compute
    #
    @QtCore.Slot()
    def sm_compute(self):
        #print("Entered state compute")
        self.compute()
        pass


    #
    # sm_finalize
    #
    @QtCore.Slot()
    def sm_finalize(self):
        print("Entered state finalize")
        pass

 
