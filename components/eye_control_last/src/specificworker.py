#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
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
from rich.console import Console
from genericworker import *
import time
from threading import Thread, Lock
import interfaces as ifaces
import numpy as np
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)
import math
import random

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

MIN_VELOCITY = 0.05
TOLERANCE = 0.02

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 33
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
        self.motor = ifaces.RoboCompJointMotorSimple.MotorState()
        self.people = []
        self.ids = []
        self.k1 = 0.8
        self.k2 = 0.6
        self.k3 = 1
        self.k4 = 0.4
        self.k5 = 4
        self.k6 = 1
        self.k7 = 1
        self.sacadic = False
        self.last_sacadic = False
        self.objetive = None
        self.objetive_id = None
        self.last_puntoMedioX = 0
        self.movement_data = None
        self.sacadic_mode = True
        self.actived_threads = True
        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        goal.maxSpeed = 0
        goal.position = 0
        self.jointmotorsimple_proxy.setPosition("", goal)
        self.obtencion_datos_thread = Thread(target=self.obtencion_datos,
                                    name="obtencion_datos", daemon=True)
        self.obtencion_datos_thread.start()
        self.set_objetive_thread = Thread(target=self.set_objetive,
                                    name="set_objetive", daemon=True)
        self.set_objetive_thread.start()
        
        
        

    def __del__(self):
        """Destructor"""
        self.actived_threads = False
        self.obtencion_datos_thread.join()
        self.set_objetive_thread.join()

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        if self.objetive != None:
            puntoMedioX = self.objetive.roi.topX + self.objetive.roi.width/2
            # print("DEPTH:",self.people[0].roi.depth)
            if(abs(puntoMedioX - self.last_puntoMedioX) > 2):
                objetive = 0.5*self.last_puntoMedioX + 0.5*puntoMedioX
                self.last_puntoMedioX = objetive
                
                error = objetive - 240
                # print("ERROR RADSE:", round(np.rad2deg(error_rads)))
                # Rotational speed given by odometry
                # act_rot_speed = robot_node.attrs["robot_ref_rot_speed"].value
                # print("POSICION ASIGNADA A SERVO: ", goal_rad)
                error_rads = 0.6 * np.arctan2(error, 388.04)
                self.movement_data =  {
                    "error_rads": error_rads,
                    "goal_rad" : self.motor.pos - error_rads,
                    # "act_rot_speed": act_rot_speed,
                    "distance": self.objetive.roi.depth*1000,
                }
                goal = self.tracker_camera(self.movement_data)
                # if abs(self.last_goal.position-goal.position)>0.02 and abs(goal.position) < (math.pi/2):
                if abs(goal.position) < (math.pi/2) and not self.last_sacadic:
                    self.jointmotorsimple_proxy.setPosition("", goal)
                    if self.sacadic:
                        self.last_sacadic = True

    def set_objetive(self):       
        while self.actived_threads:  
            try:
                if len(self.ids) > 0:
                    if self.sacadic_mode:
                        self.objetive_id = random.choice(self.ids)
                        print("CHOSEN ID:", self.objetive_id)
                        start_time = time.time()
                        while time.time() < start_time + 3:
                            for person in self.people:
                                if person.id == self.objetive_id:
                                    self.objetive = person
                                    break
                    else:
                        self.objetive_id = self.following_objetive
                        for person in self.people:
                            if person.id == self.objetive_id:
                                self.objetive = person
                                break
                else:
                    goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
                    goal.position = 0
                    goal.maxSpeed = 0
                    self.jointmotorsimple_proxy.setPosition("", goal)
            except:
                print("CAN'T GET SERVO DATA")
                        
    def tracker_camera(self, data):
        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        goal.position = data["goal_rad"]
        if abs(data["error_rads"]) > 0.3:
            # print("SACADIC")
            goal.position += 0.33*data["goal_rad"]
            self.sacadic = True
            goal.maxSpeed = 0
        else:
            # print("NOT SACADIC")
            self.last_sacadic = False
            self.sacadic = False
            # goal.maxSpeed = self.k5 *abs(abs((((data["error_rads"] + MIN_VELOCITY) * 2.5)) * (2000/data["distance"])) - 0.3 * abs(data["act_rot_speed"]))
            if data["distance"] > 0:
                goal.maxSpeed = self.k5 *abs(abs((((data["error_rads"] + MIN_VELOCITY) * 2.5)) * (1500/data["distance"])))
            else:
                goal.maxSpeed = self.k5 *abs(abs((((data["error_rads"] + MIN_VELOCITY) * 2.5))))
        return goal

    def obtencion_datos(self):
        # try:
        while self.actived_threads:
            try:
                servo_data = self.jointmotorsimple_proxy.getMotorState("")
                self.motor.pos = servo_data.pos
                # self.motor.vel = servo_data.vel
                # self.motor.isMoving = servo_data.isMoving
                # print("POSITION:", self.motor.pos)
                # if not self.motor.isMoving:
                try:
                    self.people = self.humancamerabody_proxy.newPeopleData().peoplelist
                    self.ids = []
                    for person in self.people:
                        self.ids.append(person.id)       
                except:
                    pass
            except:
                print("CAN'T GET SERVO DATA")

    def EyeControl_getServoAngle(self):
        return float(self.motor.pos)

    def EyeControl_setFollowedPerson(self, id):
        if id == 0:
            self.sacadic_mode = True
            return
        self.sacadic_mode = False
        for person in self.people:
            if person.id == id:
                print("PERSON SETTED:", id)
                self.following_objetive = id
                
                return

    def startup_check(self):
        print(f"Testing RoboCompHumanCameraBody.TImage from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TImage()
        print(f"Testing RoboCompHumanCameraBody.TGroundTruth from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TGroundTruth()
        print(f"Testing RoboCompHumanCameraBody.KeyPoint from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.KeyPoint()
        print(f"Testing RoboCompHumanCameraBody.Person from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.Person()
        print(f"Testing RoboCompHumanCameraBody.PeopleData from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.PeopleData()
        print(f"Testing RoboCompHumanCameraBody.TConnection from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TConnection()
        print(f"Testing RoboCompHumanCameraBody.TJointData from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TJointData()
        print(f"Testing RoboCompJointMotorSimple.MotorState from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorState()
        print(f"Testing RoboCompJointMotorSimple.MotorParams from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorParams()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalPosition from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalVelocity from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalVelocity()
        QTimer.singleShot(200, QApplication.instance().quit)




    ######################
    # From the RoboCompHumanCameraBody you can call this methods:
    # self.humancamerabody_proxy.getJointData(...)
    # self.humancamerabody_proxy.newPeopleData(...)

    ######################
    # From the RoboCompHumanCameraBody you can use this types:
    # RoboCompHumanCameraBody.TImage
    # RoboCompHumanCameraBody.TGroundTruth
    # RoboCompHumanCameraBody.KeyPoint
    # RoboCompHumanCameraBody.Person
    # RoboCompHumanCameraBody.PeopleData
    # RoboCompHumanCameraBody.TConnection
    # RoboCompHumanCameraBody.TJointData

    ######################
    # From the RoboCompJointMotorSimple you can call this methods:
    # self.jointmotorsimple_proxy.getMotorParams(...)
    # self.jointmotorsimple_proxy.getMotorState(...)
    # self.jointmotorsimple_proxy.setPosition(...)
    # self.jointmotorsimple_proxy.setVelocity(...)
    # self.jointmotorsimple_proxy.setZeroPos(...)

    ######################
    # From the RoboCompJointMotorSimple you can use this types:
    # RoboCompJointMotorSimple.MotorState
    # RoboCompJointMotorSimple.MotorParams
    # RoboCompJointMotorSimple.MotorGoalPosition
    # RoboCompJointMotorSimple.MotorGoalVelocity


