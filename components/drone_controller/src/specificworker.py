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
        self.state = 1
        self.x = 0
        self.y = 0
        self.pose = {}
        self.switch = {
            1:self.idle(),
            2:self.movex()
        }
        # Image
        self.image = []
        self.depth = []
        self.camera_name = "frontCamera"
        self.data = {}
        self.depth_array = []
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


    @QtCore.Slot()
    def compute(self):
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        self.image = all.image
        self.depth = all.depth
        self.x, self.y = self.circleDetect(self.image)
        # print("Value X: " ,self.x)
        # print("Value Y: " ,self.y)
    

        # Depth processing
        # Max value: 10.0       Min value from apple: 0.19
        if len(self.depth.depth) <= 1048576:
            self.depth_array = np.frombuffer(self.depth.depth,dtype=np.float32).reshape(self.depth.height, 
                self.depth.width)
            
        try: 
            
            if self.state == 1:
                self.idle()
            elif self.state == 2:
                self.movex()
            elif self.state == 3:
                self.movey()
            elif self.state == 4:
                self.advance()
            elif self.state == 5:
                self.drop()
            elif self.state == 6:
                self.stop()                
            else:
                self.error()
            

        except Ice.Exception as e:
            print(e)

        # RoboCompCoppeliaUtils.CoppeliaUtils.addOrModifyDummy(self, self.headCamera, self.camera_name, self.pt) 
        # code to send data to drone_pyrep. See ~/robocomp/interfaces/IDSLs/JoystickAdapter.idsl
        # joy_data = RoboCompJoystickAdapter.TData()
        # self.joystickadapter_proxy.sendData()
        
    # =============== Drone Movements ===================
    # ===================================================================

    # PoseType parameters
    # x = adv           rx: None
    # y = width         ry: None
    # z = height        rz: rotate
    def moveDummy(self, x_=0, y_=0, z_=0, rz_=0):
        self.pose = RoboCompCoppeliaUtils.PoseType(x=x_,y=y_,z=z_,rz=rz_)
        headCamera = RoboCompCoppeliaUtils.TargetTypes.HeadCamera
        self.coppeliautils_proxy.addOrModifyDummy(headCamera, "Quadricopter_target", self.pose)

    def idle(self):
        print("[ IDLE ]")
        if self.x == 0:
            self.moveDummy(x_=0.002)
        if self.x > 0:
            self.state = 2   #state = move x

    def movex(self):
        print("[ MOVE X ]")
        if self.x > 260:
            self.moveDummy(y_=-0.001)
        if self.x < 260:
            self.moveDummy(y_=0.001)
        if self.x >= 250 or self.x <= 260:    
            self.state = 3  #state = 

    def movey(self):
        print("[ MOVE Y ]")
        if self.y > 225:
            self.moveDummy(z_=0.001)
        if self.y < 225:
            self.moveDummy(z_=-0.001)
        if self.y >= 209 or self.y <= 226:
            self.state = 4  #state = 
 
    def advance(self):
        print("[ ADVANCE ]")
        depth = self.depth_array[225][225]
        print("depht_array:  ", self.depth_array[225][225])
        self.moveDummy(x_=0.005)
        if depth <= 0.19 and self.x >= 230 and self.y >= 226:
            self.moveDummy(x_=-0.002)
            plt.pause(.1)
            self.state = 5  #state = drop
        else:
            self.state = 2  #state = move x

    def drop(self):
        print("[ DROP ]")
        
        
        self.moveDummy(rz_=-0.005)

        if self.depth_array[200][200] >= 10:
            print("depht_array:  ", self.depth_array[200][200])
            self.state = 6
    
    def stop(self):
        print("[ STOP ]")

    def error(self):
        print("Error en el switch")

        
    # ===================================================================
    def draw_image(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        plt.figure(1)
        plt.clf()
        plt.imshow(color)
        plt.title('Front Camera ')
        plt.pause(.1)

    def circleDetect(self, color_):
        x = 0
        y = 0
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        #Marker is on X: 256 Y:256
        cv.drawMarker(color, (int(color_.width/2), int(color_.height/2)),  (0, 255, 0), cv.MARKER_CROSS, 25, 2)
        plt.imshow(color)
        color = cv.cvtColor(color, cv.COLOR_RGB2BGR)
        
        hsv = cv.cvtColor(color, cv.COLOR_BGR2HSV)
        lower_red_range = np.array([0, 50, 120])
        upper_red_range = np.array([10, 255, 255])
        mask = cv.inRange(hsv, lower_red_range, upper_red_range)
        # Find contours
        cnts = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts) 
        # Iterate through contours and filter by the number of vertices 
        for c in cnts:
            area = cv.contourArea(c)
            if area > 1000: 
                cv.drawContours(color,[c],-1,(0,255,0), 2)
                M = cv.moments(c)
                x = int(M["m10"]/ M["m00"])
                y = int(M["m01"]/ M["m00"])

                cv.circle(color,(x,y),2,(0,255,0), 2)
                # cv.putText(color, "rojo", (x-20, y-20), cv.FONT_HERSHEY_SIMPLEX,2, (255,255,255), 2)

        color = cv.cvtColor(color, cv.COLOR_BGR2RGB)
        plt.figure(1)
        plt.clf()
        plt.imshow(color)
        plt.title('OpenCV Camera ')
        plt.pause(.001)
        return (x, y)

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

 
