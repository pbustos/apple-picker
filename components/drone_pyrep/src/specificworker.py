#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
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

from genericworker import *
import time
from pyrep import PyRep
from pyrep.robots.mobiles.turtlebot import TurtleBot
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.shape import Object
from pyrep.objects.joint import Joint
from pyrep.backend import simConst
import numpy as np
import cv2
import matplotlib.pyplot as plt

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        SCENE_FILE = '../../etc/basic_scene.ttt'

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        
        self.robot = TurtleBot()
        self.drone = Shape('Quadricopter')
        cam = VisionSensor("frontCamera")
        self.camera = { "handle": cam,
                        "id": 0,
                        "angle": np.radians(cam.get_perspective_angle()),
                        "width": cam.get_resolution()[0],
                        "height": cam.get_resolution()[1],
                        "focal": (cam.get_resolution()[0] / 2) / np.tan(np.radians(cam.get_perspective_angle() / 2)),
                        "rgb": np.array(0),
                        "depth": np.ndarray(0)}

        self.joystick_newdata = []
        self.speed_robot = []
        self.speed_robot_ant = []
        self.last_received_data_time = 0

        self.once = False

    #@QtCore.Slot()
    def compute(self):
        cont = 0
        start = time.time()
        while True:
            self.pr.step()
            self.read_camera(self.camera)
            self.read_joystick()

            elapsed = time.time()-start
            if elapsed < 0.05:
                time.sleep(0.05-elapsed)
            cont += 1
            if elapsed > 1:
                print("Freq -> ", cont)
                cont = 0
                start = time.time()

    ###########################################
    def read_camera(self, cam):
        image_float = cam["handle"].capture_rgb()
        depth = cam["handle"].capture_depth(True)
        image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"], width=cam["width"], height=cam["height"],
                                                     depth=3, focalx=cam["focal"], focaly=cam["focal"],
                                                     alivetime=time.time(), image=image.tobytes())
        cam["depth"] = RoboCompCameraRGBDSimple.TDepth(cameraID=cam["id"], width=cam["handle"].get_resolution()[0],
                                                       height=cam["handle"].get_resolution()[1],
                                                       focalx=cam["focal"], focaly=cam["focal"],
                                                       alivetime=time.time(), depthFactor=1.0, depth=depth.tobytes())

        # try:
        #     self.camerargbdsimplepub_proxy.pushRGBD(cam["rgb"], cam["depth"])
        # except Ice.Exception as e:
        #     print(e)

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata: #and (time.time() - self.joystick_newdata[1]) > 0.1:
            datos = self.joystick_newdata[0]
            adv = 0.0
            rot = 0.0
            side = 0.0
            height = 0.0
            for x in datos.axes:
                if x.name == "advance":
                    adv = x.value/20 if np.abs(x.value) > 0.001 else 0
                if x.name == "rotate":
                    side = x.value/20 if np.abs(x.value) > 0.001 else 0
                if x.name == "tilt":
                    height = x.value/20 if np.abs(x.value) > 0.001 else 0
                if x.name == "side":
                    rot = x.value/15 if np.abs(x.value) > 0.001 else 0

            print("Joystick ", adv, side, height, rot)
            self.move_quad_target([adv, side, height, rot])
            self.joystick_newdata = None
            self.last_received_data_time = time.time()

    #################################################################################
    def move_quad_target(self, vels):
        target = Shape('Quadricopter_target')
        adv, side, height, rot = vels
        current_pos = target.get_position(self.drone)
        current_ori = target.get_orientation(self.drone)
        target.set_position([current_pos[0]-adv, current_pos[1]-side, current_pos[2]-height], self.drone)
        target.set_orientation([current_ori[0] - adv, current_ori[1] - side, current_ori[2] - rot], self.drone)

    ##################################################################################
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    ###################################################################################
    def JoystickAdapter_sendData(self, data):
        self.joystick_newdata = [data, time.time()]

    ##################################################################################
    #                       Methods for CameraRGBDSimple
    # ===============================================================================
    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self, camera):
        return RoboCompCameraRGBDSimple.TRGBD(self.cameras[camera]["rgb"], self.cameras[camera]["depth"])

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self, camera):
        return self.cameras[camera]["depth"]
    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self, camera):
        return self.cameras[camera]["rgb"]



