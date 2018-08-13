#!usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import baxter_interface
import argparse
import sys
import struct
import copy
import rospkg
import baxter_external_devices
from baxter_interface import CHECK_VERSION
from baxter_interface import Gripper
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from baxter_core_msgs.srv import (CloseCamera, ListCameras, OpenCamera)
import cv_bridge
import time
from baxter_interface import CameraController
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float32MultiArray, Float32
from PySide.QtCore import *
from PySide.QtGui import *
from collections import namedtuple
import logging
import os
from datetime import datetime
import controllerTraditionalPlanning
import controllerToI
import subprocess
import  csv
from decimal import *
getcontext().prec = 3
from time import sleep
preASP_domain_file = 'baxter_code/pre_ASP_files/preASP_Domain.txt'
asp_World_file = 'baxter_code/ASP_World.sp'
history_marker = '%% *_*_*'
import domain_info



# Defines the pose that the arm should take to reach a zone
class Zone():
    def __init__(self, name, x_coordinate, y_coordinate):
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate
        self.name = name
        self.pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = x_coordinate,
                        y = y_coordinate,
                        z = -0.07,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }
        self.mid_pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = x_coordinate,
                        y = y_coordinate,
                        z = -0.01,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }
        self.elevated_pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = x_coordinate,
                        y = y_coordinate,
                        z = 0.1,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }




# Defines the pose that the arm should take to reach an object
class Object():
    def __init__(self, name, x_coordinate, y_coordinate):
        self.name = name
        self.pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = x_coordinate,
                        y = y_coordinate,
                        z = -0.07,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }
        self.elevated_pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = x_coordinate,
                        y = y_coordinate,
                        z = 0.1,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }

    def set_location(self, x_coordinate, y_coordinate):
        self.pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = x_coordinate,
                        y = y_coordinate,
                        z = -0.07,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }
        self.elevated_pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = x_coordinate,
                        y = y_coordinate,
                        z = 0.1,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }



# Defines the typical BGR values of a colour object e.g. red
class BGR_colour():
    def __init__(self, B_value, G_value, R_value, allowed_deviation):
        self.B_value = B_value
        self.G_value = G_value
        self.R_value = R_value
        self.allowed_deviation = allowed_deviation

    # determine whether a particular pixel matches this colour
    def match(self, pixel_x, pixel_y, camera_image):
        pixel = camera_image[pixel_x, pixel_y]
        difference = abs(pixel[0] - self.B_value) + abs(pixel[1] - self.G_value) + abs(pixel[2] - self.R_value)
        if difference < self.allowed_deviation:
            return True
        else:
            return False



# Defines the refined components of each coarse object constant
class Components():
    def __init__(self, name, components):
        self.name = name
        self.components = components



# Defines the sorts that may be included in a zoomed description
class Sort():
    def __init__(self, name, constants):
        self.name = name
        self.constants = constants
        self.rel_constants = []

    def add(self, constant):
        self.rel_constants.append(constant)




class Executer():

    domain = domain_info.DomainInfo()



    def __init__(self):
        
        # initialize ROS node
        rospy.init_node("rsdk_ik_service_client")

        # create an instance of baxter_interface's Limb class
        limb_object = baxter_interface.Limb('left')
        self.limb_object = limb_object

        # initialise and calibrate gripper object
        gripper = Gripper('left')
        gripper.calibrate()
        gripper.set_holding_force(100)
        self.gripper = gripper

        # get the arm's current joint angles
        angles = limb_object.joint_angles()
        angles['left_s0']=0.0
        angles['left_s1']=0.0
        angles['left_e0']=0.0
        angles['left_e1']=0.0
        angles['left_w0']=0.0
        angles['left_w1']=0.0
        angles['left_w2']=0.0
        self.angles = angles

        # initialize locations
        c1 = Zone('c1', 0.4, -0.1)
        c2 = Zone('c2', 0.4, -0.0)
        c3 = Zone('c3', 0.45, -0.1)
        c4 = Zone('c4', 0.45, -0.0)
        c5 = Zone('c5', 0.4, 0.1)
        c6 = Zone('c6', 0.4, 0.2)
        c7 = Zone('c7', 0.45, 0.1)
        c8 = Zone('c8', 0.45, 0.2)
        c9 = Zone('c9', 0.4, 0.3)
        c10 = Zone('c10', 0.4, 0.4)
        c11 = Zone('c11', 0.45, 0.3)
        c12 = Zone('c12', 0.45, 0.4)
        self.cells = [c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12]
        self.objects = []

        # set camera resolution and set camera
        self.cam_res_one = 640
        self.cam_res_two = 400
        left_camera = CameraController('left_hand_camera')
        left_camera.resolution = (self.cam_res_one, self.cam_res_two)
        left_camera.open()

        # get camera image and convert it to OpenCV format with bgr8 encoding
        self.camera_image = None
        camera_subscriber = rospy.Subscriber('cameras/left_hand_camera/image', Image, self.get_img)
        while self.camera_image is None: pass

        # initialise other values
        self.current_refined_location = 'unknown_cell'
        self.current_location = 'unknown'
        self.currently_holding = 'nothing'
        self.object_placed = False
        self.RealValues = self.getInitialConditions()
        reader = open(preASP_domain_file, 'r')
        pre_asp = reader.read()
        reader.close()
        self.pre_asp_split = pre_asp.split('\n')
        self.history_marker_index = self.pre_asp_split.index(history_marker) + 1
        self.executedSteps = 0
        self.history = []
	


    def executeAction(self, action):

        print ('\nexecuting refined action: ' + action)

        # execute move actions
        if 'move' in action:
            for cell in self.cells:
                if (action[10 : len(action)-1] == cell.name):
                    self.move(cell)
                    return True

        # execute pickup actions
        elif 'pickup' in action:
            for obj in self.objects:
                if (obj.name == action[12 : len(action)-1]):
                    self.pickup(obj)
                    return True

        # execute put down actions
        elif 'put_down' in action:
            for cell in self.cells:
                if (self.current_refined_location == cell.name):
                    self.put_down(cell)
                    return True

        # execute test actions
        elif 'test' in action:
            hpd = self.test(action)
            if (self.current_refined_location == 'c1') or (self.current_refined_location == 'c2') or (self.current_refined_location == 'c3') or (self.current_refined_location == 'c4'): self.current_location = 'zoneR'
            elif (self.current_refined_location == 'c5') or (self.current_refined_location == 'c6') or (self.current_refined_location == 'c7') or (self.current_refined_location == 'c8'): self.current_location = 'zoneG'
            elif (self.current_refined_location == 'c9') or (self.current_refined_location == 'c10') or (self.current_refined_location == 'c11') or (self.current_refined_location == 'c12'): self.current_location = 'zoneY'
            self.RealValues[self.domain.LocationRobot_index] = self.current_location
            if 'green_box' in self.currently_holding: self.RealValues[self.domain.LocationGreenBox_index] = self.current_location
            elif 'blue_box' in self.currently_holding: self.RealValues[self.domain.LocationBlueBox_index] = self.current_location
            return hpd

        # warn user if the plan contains an action that this function isn't programmed for
        else: print ('action not recognised')




    def getTheseObservations(self,indexes):
        observableValues = list(self.RealValues)
        robotLocation = self.RealValues[self.domain.LocationRobot_index]
        observations = []
        if(self.RealValues[self.domain.LocationGreenBox_index] != robotLocation): observableValues[self.domain.LocationGreenBox_index] = 'unknown'
        if(self.RealValues[self.domain.LocationBlueBox_index] != robotLocation): observableValues[self.domain.LocationBlueBox_index] = 'unknown'
        for index in indexes: observations.append([index,observableValues[index]])
        return observations



    def getRealValues(self):
        observations =[] 
        for index, val in enumerate(self.RealValues): observations.append([index, val])
        return observations
	


    def getRobotLocation(self):
        return self.RealValues[self.LocationRobot_index]



    def getGoalFeedback(self):
        print ('Has the goal been achieved? (y/n)')
        response = raw_input()
        if response == 'y': return True
        else: return False



    def getExecutedSteps(self):
        return self.executedSteps



    def __del__(self):
        print('deleting executer ')



    def __getRealValues_as_obsList(self,step):
        obsList = []
        if(self.RealValues[self.domain.LocationRobot_index] != 'unknown'): 
            obsList.append('obs(loc(rob1,'+str(self.RealValues[self.domain.LocationRobot_index])+'),true,'+str(step)+').')
        if(self.RealValues[self.domain.LocationGreenBox_index] != 'unknown'):
            obsList.append('obs(loc(green_box,'+str(self.RealValues[self.domain.LocationGreenBox_index])+'),true,'+str(step)+').')
        if(self.RealValues[self.domain.LocationBlueBox_index] != 'unknown'): 
            obsList.append('obs(loc(blue_box,'+str(self.RealValues[self.domain.LocationBlueBox_index])+'),true,'+str(step)+').')
        if(self.RealValues[self.domain.In_handGreenBox_index] != 'unknown'): 
            obsList.append('obs(in_hand(rob1,green_box),'+self.RealValues[self.domain.In_handGreenBox_index]+','+str(step)+').')
        if(self.RealValues[self.domain.In_handBlueBox_index] != 'unknown'): 
            obsList.append('obs(in_hand(rob1,blue_box),'+self.RealValues[self.domain.In_handBlueBox_index]+','+str(step)+').')
        return obsList



    def __getActionObservations(self, action):
        relevant_indexes = set()
        if(action[0:4] == 'move'):
            relevant_indexes.add(self.domain.LocationRobot_index)

        if(action == 'pickup(rob1,green_box)' or action == '+put_down(rob1,green_box)'):
            relevant_indexes.add(self.domain.In_handGreenBox_index)
            relevant_indexes.add(self.domain.LocationGreenBox_index)

        if(action == 'pickup(rob1,blue_box)' or action == '+put_down(rob1,blue_box)'):
            relevant_indexes.add(self.domain.In_handBlueBox_index)
            relevant_indexes.add(self.domain.LocationBlueBox_index)

        return self.getTheseObservations(relevant_indexes)



    def getInitialConditions(self):

        # determine the location of the green box
        green = BGR_colour(50, 80, 50, 10)
        x_coordinate, y_coordinate, green_box_zone = self.locate_object(green)
        print ('The green box is in:')
        print (green_box_zone)

        # initialise the green_box object
        green_box = Object('green_box', x_coordinate, y_coordinate)
        green_box_top = Object('green_box_top', x_coordinate, y_coordinate)

        # determine the location of the blue box
        blue = BGR_colour(200, 0, 30, 180)
        x_coordinate, y_coordinate, blue_box_zone = self.locate_object(blue)
        print ('The blue box is in:')
        print (blue_box_zone)

        # initialise the blue_box object
        blue_box = Object('blue_box', x_coordinate, y_coordinate)
        blue_box_top = Object('blue_box_top', x_coordinate, y_coordinate)

        self.objects = [green_box, green_box_top, blue_box, blue_box_top]

        initial_conditions = ['above', green_box_zone, blue_box_zone, 'false', 'false']
        return initial_conditions




    # this function executes a move action
    def move(self, zone):
        self.ik_move(zone.elevated_pose)



    # this function executes a pickup action
    def pickup(self, obj):
        self.ik_move(obj.elevated_pose)
        self.gripper.open()
        self.ik_move(obj.pose)
        self.gripper.close()
        time.sleep(0.5)
        self.ik_move(obj.elevated_pose)



    # this function executes a put down action
    def put_down(self, zone):
        if not self.object_placed: self.ik_move(zone.pose)
        else: self.ik_move(zone.mid_pose)
        self.gripper.open()
        self.ik_move(zone.elevated_pose)



    def get_img(self, msg):
        camera_image = self.msg_to_cv(msg)
        self.camera_image = camera_image
  

    def msg_to_cv(self, msg): return cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')



    # function for determining the location of an object
    def locate_object(self, colour):

        # this pose is used to look down at the workspace and detect objects
        vertical_viewing_pose = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x = 0.4,
                        y = 0.26,
                        z = 0.4,
                    ),
                    orientation=Quaternion(
                        x = 0.0,
                        y = -2.0,
                        z = 0.0,
                        w = 0.0,
                    ),
                ),
            ),
        }

        # move arm to the desired coordinates
        self.ik_move(vertical_viewing_pose)

        # detect object
        image_coordinate_x, image_coordinate_y = self.object_detection(colour)

        # if the object has been detected...
        if (not (image_coordinate_x == 0 and image_coordinate_y == 0)):

            print ('2D coordinates are...')
            print  ('x coordinate: ' + str(image_coordinate_x))
            print  ('y coordinate: ' + str(image_coordinate_y))

            # use the image coordinates to determine the world coordinates of the object
            offset_x = 0.675
            offset_y = 0.77
            scale = -0.0015
            x_coordinate = image_coordinate_x * scale + offset_x
            y_coordinate = image_coordinate_y * scale + offset_y
            print ('\n3D coordinates are...')
            print ('x coordinate: ' + str(x_coordinate))
            print ('y coordinate: ' + str(y_coordinate))
            print ('\n\n')

        # warn user when object is not detected
        else:
            print ('object not detected')
            return 0, 0, 'unknown'

        # determine which zone the box is in - EDIT this section to change the zones
        if (y_coordinate < 0):
            object_zone = 'zoneR'
        elif (y_coordinate < 0.2):
            object_zone = 'zoneG'
        else:
            object_zone = 'zoneY'

        return x_coordinate, y_coordinate, object_zone



    # move arm to desired coordinates using inverse kinematics
    def ik_move(self, poses):
        ns = "ExternalTools/left/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()   
        ikreq.pose_stamp.append(poses['left'])
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
        if (resp.isValid[0]):
            # move to desired coordinates using the angles given by inverse kinematics
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            self.limb_object.move_to_joint_positions(limb_joints)
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        pass



    # function for object detection
    def object_detection(self, colour):
        # iterate through pixels in image and detect ones of a specific colour
        for pixel_x in range(self.cam_res_two):
            for pixel_y in range(self.cam_res_one):
                pixel_matches_colour = colour.match(pixel_x, pixel_y, self.camera_image)
                if pixel_matches_colour:
                    return pixel_x, pixel_y
        return 0, 0




    # this function executes a test action
    def test(self, action):

        # parse the fluent
        if 'true' in action:
            fluent = action[10:len(action)-6]
        elif 'false' in action:
            fluent = action[10:len(action)-7]

        # moving the robot's arm never fails
        if ('loc' in fluent) and ('rob1' in fluent):
            self.current_refined_location = fluent[9:len(fluent)-1]
            real_value = 'true'
            hpd = True
            print ('the robot is in cell ' + self.current_refined_location + '\n')

        # object location must be tested before attempting a pickup action
        elif 'loc' in fluent:
            # if there are blue pixels, the blue box is in the current location
            if 'blue' in fluent:
                blue_pixel_count = self.tally_pixels(200, 0, 30, 180)
                if blue_pixel_count > 50:
                    print ('the blue_box is in ' + self.current_refined_location)
                    self.RealValues[self.domain.LocationBlueBox_index] = self.current_location
                    real_value = 'true'
                    hpd = True
                    for cell in self.cells:
                        if cell.name == self.current_refined_location:
                            for obj in self.objects:
                                if 'blue' in obj.name: obj.set_location(cell.x_coordinate, cell.y_coordinate)
                else:
                    print ('the blue_box is not in ' + self.current_refined_location)
                    self.RealValues[self.domain.LocationBlueBox_index] = 'unknown'
                    real_value = 'false'
                    hpd = False
            # if there are green pixels, the green box is in the current location
            elif 'green' in fluent:
                green_pixel_count = self.tally_pixels(50, 80, 50, 10)
                if green_pixel_count > 50:
                    print ('the green_box is in ' + self.current_refined_location)
                    self.RealValues[self.domain.LocationGreenBox_index] = self.current_location
                    real_value = 'true'
                    hpd = True
                    for cell in self.cells:
                        if cell.name == self.current_refined_location:
                            for obj in self.objects:
                                if 'green' in obj.name: obj.set_location(cell.x_coordinate, cell.y_coordinate)
                else:
                    print ('the green_box is not in ' + self.current_refined_location)
                    global searched_cells
                    global search_complete
                    searched_cells.append(self.current_refined_location)
                    if search_complete: self.RealValues[self.domain.LocationGreenBox_index] = 'unknown'
                    real_value = 'false'
                    hpd = False

        # putting an object down never fails
        elif ('false' in action and 'in_hand' in fluent):
            self.currently_holding = 'nothing'
            self.object_placed = True
            real_value = 'false'
            hpd = True
            print ('the object is not in hand\n')
            if 'blue_box' in fluent:
                self.RealValues[self.domain.LocationBlueBox_index] = self.current_location
                self.RealValues[self.domain.In_handBlueBox_index] = 'false'
            elif 'green_box' in fluent:
                self.RealValues[self.domain.LocationGreenBox_index] = self.current_location
                self.RealValues[self.domain.In_handGreenBox_index] = 'false'

        # picking up an object can fail and needs to be tested
        else:
            hpd = False
            real_value = 'false'
            # test if there are pixels of the relevant colour close to the camera
            if 'blue_box' in fluent:
                blue_pixel_count = self.tally_pixels(200, 0, 30, 180)
                if blue_pixel_count > 0:
                    real_value = 'true'
                    hpd = True
                    self.currently_holding = fluent[13:len(fluent)-1]
                    print ('the object is in hand\n')
                    self.RealValues[self.domain.LocationBlueBox_index] = self.current_location
                    self.RealValues[self.domain.In_handBlueBox_index] = 'true'
                else:
                    print ('the object is not in hand\n')
                    self.RealValues[self.domain.LocationBlueBox_index] = 'unknown'
                    self.RealValues[self.domain.In_handBlueBox_index] = 'false'
            elif 'green_box' in fluent:
                green_pixel_count = self.tally_pixels(150, 250, 150, 180)
                if green_pixel_count > 0:
                    real_value = 'true'
                    hpd = True
                    self.currently_holding = fluent[13:len(fluent)-1]
                    print ('the object is in hand\n')
                    self.RealValues[self.domain.LocationGreenBox_index] = self.current_location
                    self.RealValues[self.domain.In_handGreenBox_index] = 'true'
                else:
                    print ('the object is not in hand\n')
                    self.RealValues[self.domain.LocationGreenBox_index] = 'unknown'
                    self.RealValues[self.domain.In_handGreenBox_index] = 'false'

        # record action observation
        action_observation = 'holds(directly_observed(rob1,'+fluent+','+real_value+'),1)'

        return hpd, action_observation




    # function for checking whether object is in hand
    def tally_pixels(self, b, g, r, variance):
        colour = BGR_colour(b, g, r, variance)
        pixel_count = 0
        for pixel_x in range(400):
            for pixel_y in range(640):
                pixel_matches_colour = colour.match(pixel_x, pixel_y, self.camera_image)
                if (pixel_x < 250) and (pixel_x > 100) and (pixel_y < 450) and (pixel_y > 300) and pixel_matches_colour: pixel_count = pixel_count + 1
        return pixel_count



    # this function is used for testing purposes only and should not be called during proper execution
    def examine_image(self, b, g, r, variance):
        print ('Examining Image')
        colour = BGR_colour(b, g, r, variance)
        camera_image_edited = self.camera_image
        for pixel_x in range(400):
            for pixel_y in range(640):
                pixel_matches_colour = colour.match(pixel_x, pixel_y, self.camera_image)
                if pixel_matches_colour:
                    print (self.camera_image[pixel_x, pixel_y])
                    camera_image_edited[pixel_x,pixel_y] = (255, 255, 255)
        cv2.imshow("temp", camera_image_edited)
        cv2.waitKey(0)
        return



    def getMyRefinedLocation(self):
        return self.current_refined_location



    def getTheseCoarseObservations(self,indexes):
        observableValues = list(self.RealValues)
        robotLocation = self.RealValues[self.domain.LocationRobot_index]
        observations = []
        if(self.RealValues[self.domain.LocationGreenBox_index] != robotLocation): observableValues[self.domain.LocationGreenBox_index] = 'unknown'
        if(self.RealValues[self.domain.LocationBlueBox_index] != robotLocation): observableValues[self.domain.LocationBlueBox_index] = 'unknown'
        for index in indexes: observations.append([index,observableValues[index]])
        return observations

