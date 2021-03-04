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
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
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

        # image
        self.image = []
        self.depth = []
        self.camera_name = "frontCamera"
        self.data = {}
        
        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.start(self.Period)
            self.defaultMachine.start()
            self.destroyed.connect(self.t_compute_to_finalize)

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
        print("Entered state compute")
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        #self.draw_image(all.image)
        self.circleDetect(all.image)
        
        # code to send data to drone_pyrep. See ~/robocomp/interfaces/IDSLs/JoystickAdapter.idsl 
        #joy_data = RoboCompJoystickAdapter.TData()
        #self.joystickadapter_proxy.sendData()
        
       
        
        
    # ===================================================================
    def draw_image(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        plt.figure(1);
        plt.clf()
        plt.imshow(color)
        plt.title('Front Camera ')
        plt.pause(.1)

    def circleDetect(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        cv.drawMarker(color, (int(color_.width/2), int(color_.height/2)),  (0, 255, 0), cv.MARKER_CROSS, 25, 2)
        plt.imshow(color);
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
        plt.figure(1);
        plt.clf()
        plt.imshow(color);
        plt.title('OpenCV Camera ')
        plt.pause(.001)

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

######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompJoystickAdapter you can publish calling this methods:
    # self.joystickadapter_proxy.sendData(...)

    ######################
    # From the RoboCompJoystickAdapter you can use this types:
    # RoboCompJoystickAdapter.AxisParams
    # RoboCompJoystickAdapter.ButtonParams
    # RoboCompJoystickAdapter.TData