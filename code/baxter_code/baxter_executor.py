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
preASP_domain_file = 'preASP_Domain.txt'
asp_World_file = 'ASP_World.sp'
history_marker = '%% *_*_*'



# Defines the pose that the arm should take to reach a zone
class Zone():
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

    LocationRobot_index = 0
    LocationGreenBox_index = 1
    LocationBlueBox_index = 2
    In_handGreenBox_index = 3
    In_handBlueBox_index = 4



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
        self.green_box_in_hand = False # TODO remove
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
        input = self.__getRealValues_as_obsList(0) + ['hpd('+ action +',0).']
        refinedPlan = self.getRefinedPlan(action)
        happened = self.executeRefinedPlan(refinedPlan)
        self.executedSteps += 1
        if not happened: self.history.append(action + " (FAILED) ")
        else:
            self.history.append(action)
        return (self.__getActionObservations(action))



    def getTheseObservations(self,indexes):
        observableValues = list(self.RealValues)
        robotLocation = self.RealValues[self.LocationRobot_index]
        observations = []
        if(self.RealValues[self.LocationGreenBox_index] != robotLocation): observableValues[self.LocationGreenBox_index] = 'unknown'
        if(self.RealValues[self.LocationBlueBox_index] != robotLocation): observableValues[self.LocationBlueBox_index] = 'unknown'
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
        if(self.RealValues[self.LocationRobot_index] != 'unknown'): 
            obsList.append('obs(loc(rob1,'+str(self.RealValues[self.LocationRobot_index])+'),true,'+str(step)+').')
        if(self.RealValues[self.LocationGreenBox_index] != 'unknown'):
            obsList.append('obs(loc(green_box,'+str(self.RealValues[self.LocationGreenBox_index])+'),true,'+str(step)+').')
        if(self.RealValues[self.LocationBlueBox_index] != 'unknown'): 
            obsList.append('obs(loc(blue_box,'+str(self.RealValues[self.LocationBlueBox_index])+'),true,'+str(step)+').')
        if(self.RealValues[self.In_handGreenBox_index] != 'unknown'): 
            obsList.append('obs(in_hand(rob1,green_box),'+self.RealValues[self.In_handGreenBox_index]+','+str(step)+').')
        if(self.RealValues[self.In_handBlueBox_index] != 'unknown'): 
            obsList.append('obs(in_hand(rob1,blue_box),'+self.RealValues[self.In_handBlueBox_index]+','+str(step)+').')
        return obsList



    def __getActionObservations(self, action):
        relevant_indexes = set()
        if(action[0:4] == 'move'):
            relevant_indexes.add(self.LocationRobot_index)

        if(action == 'pickup(rob1,green_box)' or action == '+put_down(rob1,green_box)'):
            relevant_indexes.add(self.In_handGreenBox_index)
            relevant_indexes.add(self.LocationGreenBox_index)

        if(action == 'pickup(rob1,blue_box)' or action == '+put_down(rob1,blue_box)'):
            relevant_indexes.add(self.In_handBlueBox_index)
            relevant_indexes.add(self.LocationBlueBox_index)

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
#        blue = BGR_colour(200, 0, 30, 180)
#        x_coordinate, y_coordinate, blue_box_zone = self.locate_object(blue)
#        print ('The blue box is in:')
#        print (blue_box_zone)

        # initialise the blue_box object
#        blue_box = Object('blue_box', x_coordinate, y_coordinate)
#        blue_box_top = Object('blue_box_top', x_coordinate-0.03, y_coordinate+0.02)
        blue_box_zone = 'zoneR' # TODO detect blue box properly

        self.objects = [green_box, green_box_top]

        initial_conditions = ['above', green_box_zone, blue_box_zone, 'false', 'false']
        return initial_conditions



    # this function uses the refined_baxter_domain.txt file and SPARC to get a refined action plan
    def getRefinedPlan(self, action):
  
        # use action and history to figure out the transition (initial_state, action, final_state)
        # the location of the robot is relevant for move transitions
        if 'move' in action:
            initial_state = ['coarse_loc(rob1,' + self.RealValues[self.LocationRobot_index] + ')']
            final_state = ['coarse_loc(rob1,' + action[10:len(action)-1] + ')']

        # the location of the robot and object, and the in_hand status of the object, are relevant for pickup transitions
        elif 'pickup' in action:
            obj = action[12:len(action)-1]
            # include the in_hand status of the object
            initial_state = ['-coarse_in_hand(rob1,' + obj + ')']
            final_state = ['coarse_in_hand(rob1,' + obj + ')']
            # include the location of the robot
            rob_loc = 'coarse_loc(rob1,' + self.RealValues[self.LocationRobot_index] + ')'
            # include the location of the object
            if 'green' in obj: obj_loc = 'coarse_loc(' + obj + ',' + self.RealValues[self.LocationGreenBox_index] + ')'
            elif 'blue' in obj: obj_loc = 'coarse_loc(' + obj + ',' + self.RealValues[self.LocationBlueBox_index] + ')'
            initial_state.append(rob_loc)
            initial_state.append(obj_loc)
            final_state.append('coarse_loc(rob1,' + obj_loc[12+len(obj):len(obj_loc)-1] + ')')
            final_state.append(obj_loc)

        # the in_hand status of the object is relevant for put_down transitions
        elif 'put_down' in action:
            initial_state = ['coarse_in_hand(rob1,' + action[14:len(action)-1] + ')']
            final_state = ['-coarse_in_hand(rob1,' + action[14:len(action)-1] + ')']

        # edit refined_asp to get temporary zoomed_asp file
        self.zoom(initial_state, action, final_state)

        # get refined answer set
        os.system('java -jar sparc.jar zoomed_baxter_domain.txt -A > tmp')
        refined_answer = open('tmp', 'r').read()
        os.remove('tmp')

        # stop running code if refined answer set is empty
        if refined_answer == '\n': raw_input('No refined answer set, press enter if you wish to continue.\n')

        refined_plans = (refined_answer.split('{'))[1:]
        for i in range(len(refined_plans)):
            refined_plans[i] = refined_plans[i].strip('{')
            refined_plans[i] = refined_plans[i].strip('\n')
            refined_plans[i] = refined_plans[i].strip('}')

        refined_plan = refined_plans[0]

        return refined_plan



    def executeRefinedPlan(self, refined_plan):

        # parse refined plan to get an ordered list of refined actions
        refined_plan = refined_plan.replace(' ','')
        refined_actions = refined_plan.split('occurs')
        refined_actions.remove('')
        for i in range(len(refined_actions)):
            if not (i == len(refined_actions)-1):
                refined_actions[i] = refined_actions[i][0:len(refined_actions[i])-1]
        ordered_actions = []
        for i in range(len(refined_actions)):
            for action in refined_actions:
                if (','+str(i)+')') in action: ordered_actions.append(action[1:len(action)-3])

        # execute each action in refined plan and get relevant outputs
        happened = True
        for action in ordered_actions:
            happened_refined = self.execute_refined_action(action)
            if not happened_refined:
                happened = False
                break
        return happened




    # this function parses the intended action and calls the relevant refined function
    def execute_refined_action(self, action):

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
            self.RealValues[self.LocationRobot_index] = self.current_location
            if 'green_box' in self.currently_holding: self.RealValues[self.LocationGreenBox_index] = self.current_location
            elif 'blue_box' in self.currently_holding: self.RealValues[self.LocationBlueBox_index] = self.current_location
            return hpd

        # warn user if the plan contains an action that this function isn't programmed for
        else: print ('action not recognised')






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
        self.green_box_in_hand = True



    # this function executes a put down action
    def put_down(self, zone):
        if not self.object_placed: self.ik_move(zone.pose)
        else: self.ik_move(zone.mid_pose)
        self.gripper.open()
        self.ik_move(zone.elevated_pose)
        self.green_box_in_hand = False



    def get_img(self, msg):
        camera_image = self.msg_to_cv(msg)
        self.camera_image = camera_image
  

    def msg_to_cv(self, msg):
        return cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')



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



    # function for object detection
    def object_detection(self, colour):
        # iterate through pixels in image and detect ones of a specific colour
        for pixel_x in range(self.cam_res_two):
            for pixel_y in range(self.cam_res_one):
                pixel_matches_colour = colour.match(pixel_x, pixel_y, self.camera_image)
                if pixel_matches_colour:
                    return pixel_x, pixel_y
        return 0, 0




    # this action writes a zoomed ASP file
    def zoom(self, initial_state, action, final_state):

        # clean up typos
        for i in range(len(initial_state)):
            if '))' in initial_state[i]: initial_state[i] = initial_state[i][0:len(initial_state[i])-1]
        for i in range(len(final_state)):
            if '))' in final_state[i]: final_state[i] = final_state[i][0:len(final_state[i])-1]

        # EDIT these lists to change the domain
        coarse_places = Sort('coarse_place', ['zoneR', 'zoneG', 'zoneY', 'above'])
        coarse_objects = Sort('coarse_object', ['yellow_box', 'blue_box', 'green_box'])
        places = Sort('place', ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'unknown_cell'])
        objects = Sort('object', ['yellow_box_top', 'blue_box_top', 'green_box_top'])
        coarse_things = Sort('coarse_thing', ['#coarse_object', '#robot'])
        things = Sort('thing', ['#object', '#robot'])
        coarse_components = Sort('coarse_component', ['#coarse_place', '#coarse_object'])
        refined_components = Sort('refined_component', ['#place', '#object'])
        robots = Sort('robot', ['rob1'])
        sorts = [coarse_places, coarse_objects, places, objects, coarse_things, things, coarse_components, refined_components, robots]
        inertial_fluents = ['loc(#thing,#place)', 'in_hand(#robot,#object)']
        defined_fluents = ['coarse_loc(#coarse_thing,#coarse_place)', 'coarse_in_hand(#robot,#coarse_object)']
        actions = ['move(#robot,#place)', 'pickup(#robot,#object)', 'put_down(#robot,#object)']

        # EDIT these instantiations of the Components class to change which refined objects are associated with which coarse ones
        zoneR_components = Components('zoneR', ['c1', 'c2', 'c3', 'c4'])
        zoneG_components = Components('zoneG', ['c5', 'c6', 'c7', 'c8'])
        zoneY_components = Components('zoneY', ['c9', 'c10', 'c11', 'c12'])
        above_components = Components('above', ['unknown_cell'])
        yellow_box_components = Components('yellow_box', ['yellow_box_top'])
        blue_box_components = Components('blue_box', ['blue_box_top'])
        green_box_components = Components('green_box', ['green_box_top'])
        refinements = [zoneR_components, zoneG_components, zoneY_components, yellow_box_components, blue_box_components, green_box_components, above_components]

        # initialise relevance lists
        rel_initial_conditions = []
        rel_final_conditions = []
        rel_conditions = []
        rel_obj_consts = []
        rel_sorts = []
        rel_sort_names = ['#coarse_thing', '#thing']
        rel_inertial_fluents = []
        rel_defined_fluents = []
        rel_actions = ['test(#robot,#physical_inertial_fluent,#outcome)']

        # initialise irrelevance lists - EDIT to include new objects or zones or cells
        irrelevant_sort_names = ['#coarse_place', '#coarse_object', '#place', '#object']
        irrelevant_obj_consts = ['zoneR', 'zoneG', 'zoneY', 'above', 'yellow_box', 'blue_box', 'green_box', 'c1,', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'unknown_cell', 'yellow_box_top', 'blue_box_top', 'green_box_top']
        irrelevant_fluents = ['coarse_loc', 'coarse_in_hand', 'loc', 'in_hand']
        irrelevant_actions = ['move', 'pickup', 'put_down']

        # determine which initial conditions are relevant
        for condition in initial_state:
            if not condition in final_state: # conditions that change are relevant
                rel_initial_conditions.append(condition)
                rel_conditions.append(condition)
            elif ('rob1' in condition) and ('loc' in condition) and ('pickup' in action): # the robot's location is relevant for pickup actions
                rel_initial_conditions.append(condition)
                rel_conditions.append(condition)
    
        # refine initial conditions
        for i in range(len(rel_initial_conditions)):
            if ('loc' in rel_initial_conditions[i]) and ('rob1' in rel_initial_conditions[i]):
                rel_initial_conditions[i] = 'loc(rob1,' + self.current_refined_location + ')'
            if ('in_hand' in rel_initial_conditions[i]) and (not '-' in rel_initial_conditions[i]):
                rel_initial_conditions[i] = 'in_hand(rob1,' + self.currently_holding + ')'

        # determine which final conditions are relevant
        for condition in final_state:
            if not condition in initial_state:
                rel_final_conditions.append(condition)
                rel_conditions.append(condition)

        # determine which object constants are relevant
        for condition in rel_conditions:
            for index in range(len(condition)):
                if condition[index] == '(': opening_bracket = index
                elif condition[index] == ')': closing_bracket = index
            obj_consts = condition[opening_bracket+1:closing_bracket].split(',')
            for const in obj_consts:
                rel_obj_consts.append(const)
                if const in irrelevant_obj_consts: irrelevant_obj_consts.remove(const)
        rel_obj_consts = list(set(rel_obj_consts)) # remove duplicates

        # add refined components of relevant object constants
        for const in rel_obj_consts:
            for refinement in refinements:
                if const == refinement.name:
                    for refined_const in refinement.components:
                        rel_obj_consts.append(refined_const)  
                        if refined_const == 'c1': refined_const = 'c1,'
                        if refined_const in irrelevant_obj_consts: irrelevant_obj_consts.remove(refined_const)

        # sort relevant objects into types
        for sort in sorts:
            for const in sort.constants:
                if const in rel_obj_consts: sort.add(const)

        # determine which sorts should be included in the zoomed description
        for sort in sorts:
            if len(sort.rel_constants) != 0:
                rel_sorts.append(sort)
                rel_sort_names.append('#'+sort.name)
                if ('#'+sort.name) in irrelevant_sort_names: irrelevant_sort_names.remove('#'+sort.name)

        # add relevant sorts to sorts of sorts (coarse_things, things, coarse_components and refined_components)
        for sort in rel_sorts:
            for sort_of_sorts in sorts:
                if ('#'+sort.name) in sort_of_sorts.constants: sort_of_sorts.add('#'+sort.name)

        # determine which inertial fluents are relevant
        for fluent in inertial_fluents:
            fluent_relevant = True
            for index in range(len(fluent)):
                if fluent[index] == '(': opening_bracket = index
                elif fluent[index] == ')': closing_bracket = index
            fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
            for sort in fluent_sorts:
                if not sort in rel_sort_names: fluent_relevant = False
            if fluent_relevant:
                rel_inertial_fluents.append(fluent)
                if fluent[0:opening_bracket] in irrelevant_fluents: irrelevant_fluents.remove(fluent[0:opening_bracket])

        # determine which defined fluents are relevant
        for fluent in defined_fluents:
            fluent_relevant = True
            for index in range(len(fluent)):
                if fluent[index] == '(': opening_bracket = index
                elif fluent[index] == ')': closing_bracket = index
            fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
            for sort in fluent_sorts:
                if not sort in rel_sort_names: fluent_relevant = False
            if fluent_relevant:
                rel_defined_fluents.append(fluent)
                if fluent[0:opening_bracket] in irrelevant_fluents: irrelevant_fluents.remove(fluent[0:opening_bracket])

        # determine which actions are relevant
        for act in actions:
            action_relevant = True
            for index in range(len(act)):
                if act[index] == '(': opening_bracket = index
                elif act[index] == ')': closing_bracket = index
            action_sorts = act[opening_bracket+1:closing_bracket].split(',')
            for sort in action_sorts:
                if not sort in rel_sort_names: action_relevant = False
            if action_relevant:
                rel_actions.append(act)
                if act[0:opening_bracket] in irrelevant_actions: irrelevant_actions.remove(act[0:opening_bracket])

        # determine what the goal of the refined ASP should be
        goal = 'goal(I) :- '
        for condition in rel_final_conditions:
            if '-' in condition:
                condition = condition.replace('-', '')
                goal = goal + '-holds(' + condition + ',I), '
            else: goal = goal + 'holds(' + condition + ',I), '
        goal = goal[0:len(goal)-2] + '.\n'

        # make temporary copy of refined ASP file that can be edited
        original_asp = open('refined_baxter_domain.txt', 'r')
        zoomed_asp = open('zoomed_baxter_domain.txt', 'w')
        for line in original_asp:
            if line == '%% GOAL GOES HERE\n': # put goal in
                zoomed_asp.write(goal)
            elif line == '%% INITIAL STATE GOES HERE\n': # put initial conditions in
                for condition in rel_initial_conditions:
                    if '-' in condition:
                        condition = condition.replace('-', '')
                        zoomed_asp.write('-holds(' + condition + ', 0).\n')
                    else:
                        zoomed_asp.write('holds(' + condition + ', 0).\n')
            elif ('cannot test in the first step' in line) and ('pickup' in action): # remove this axiom for pickup actions, as the object's location needs to be tested at step zero
                pass
            elif 'ZOOM THIS SORT:' in line: # add relevant constants to sort
                zoomed_sort = ''
                for sort in sorts:
                    if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
                        zoomed_sort = '#' + sort.name + ' = {'
                        for const in sort.rel_constants:
                            zoomed_sort = zoomed_sort + const + ', '
                        zoomed_sort = zoomed_sort[0:len(zoomed_sort)-2] + '}.\n'
                        zoomed_asp.write(zoomed_sort)
            elif 'ZOOM THIS SORT OF SORTS' in line: # add relevant sorts
                zoomed_sort = ''
                for sort in sorts:
                    if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
                        zoomed_sort = '#' + sort.name + ' = '
                        for const in sort.rel_constants:
                            zoomed_sort = zoomed_sort + const + ' + '
                        zoomed_sort = zoomed_sort[0:len(zoomed_sort)-3] + '.\n'
                        zoomed_asp.write(zoomed_sort)
            elif 'ZOOM INERTIAL FLUENTS' in line: # add relevant inertial fluents
                inertial_fluent_sort = '#physical_inertial_fluent = '
                for fluent in rel_inertial_fluents:
                    inertial_fluent_sort = inertial_fluent_sort + fluent + ' + '
                inertial_fluent_sort = inertial_fluent_sort[0:len(inertial_fluent_sort)-3] + '.\n'
                zoomed_asp.write(inertial_fluent_sort)
            elif 'ZOOM DEFINED FLUENTS' in line: # add relevant defined fluents
                defined_fluent_sort = '#physical_defined_fluent = '
                for fluent in rel_defined_fluents:
                    defined_fluent_sort = defined_fluent_sort + fluent + ' + '
                defined_fluent_sort = defined_fluent_sort[0:len(defined_fluent_sort)-3] + '.\n'
                zoomed_asp.write(defined_fluent_sort)
            elif 'ZOOM ACTIONS' in line: # add relevant actions
                action_sort = '#action = '
                for act in rel_actions:
                    action_sort = action_sort + act + ' + '
                action_sort = action_sort[0:len(action_sort)-3] + '.\n'
                zoomed_asp.write(action_sort)
            else:
                line_relevant = True
                for sort in irrelevant_sort_names: # don't include predicates with irrelevant sorts
                    if sort in line:
                        line_relevant = False
                for const in irrelevant_obj_consts: # don't include attributes with irrelevant object constants
                    if const in line:
                        line_relevant = False
                for fluent in irrelevant_fluents: # don't include axioms with irrelevant fluents
                    if fluent in line:
                        line_relevant = False
                for act in irrelevant_actions: # don't include axioms with irrelevant actions
                    if act in line:
                        line_relevant = False
                if line_relevant:
                    zoomed_asp.write(line)
        original_asp.close()
        zoomed_asp.close()




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
            hpd = True
            print ('the robot is in cell ' + self.current_refined_location + '\n')

        # object location must be tested before attempting a pickup action
        elif 'loc' in fluent:
            # if there are blue pixels, the blue box is in the current location
            if 'blue' in fluent:
                blue_pixel_count = self.tally_pixels(200, 0, 30, 180)
                if blue_pixel_count > 50:
                    print ('the blue_box is in ' + self.current_location)
                    self.RealValues[self.LocationBlueBox_index] = self.current_location
                    hpd = True
                else:
                    print ('the blue_box is not in ' + self.current_location)
                    self.RealValues[self.LocationBlueBox_index] = 'unknown'
                    hpd = False
            # if there are green pixels, the green box is in the current location
            elif 'green' in fluent:
                green_pixel_count = self.tally_pixels(150, 250, 150, 180)
                if green_pixel_count > 50:
                    print ('the green_box is in ' + self.current_location)
                    self.RealValues[self.LocationGreenBox_index] = self.current_location
                    hpd = True
                else:
                    print ('the green_box is not in ' + self.current_location)
                    self.RealValues[self.LocationGreenBox_index] = 'unknown'
                    hpd = False

        # putting an object down never fails
        elif ('false' in action and 'in_hand' in fluent):
            self.currently_holding = 'nothing'
            self.object_placed = True
            hpd = True
            print ('the object is not in hand\n')
            if 'blue_box' in fluent:
                self.RealValues[self.LocationBlueBox_index] = self.current_location
                self.RealValues[self.In_handBlueBox_index] = 'false'
            elif 'green_box' in fluent:
                self.RealValues[self.LocationGreenBox_index] = self.current_location
                self.RealValues[self.In_handGreenBox_index] = 'false'

        # picking up an object can fail and needs to be tested
        else:
            hpd = False
            # test if there are pixels of the relevant colour close to the camer
            if 'blue_box' in fluent:
                blue_pixel_count = self.tally_pixels(200, 0, 30, 180)
                if blue_pixel_count > 0:
                    hpd = True
                    self.currently_holding = fluent[13:len(fluent)-1]
                    print ('the object is in hand\n')
                    self.RealValues[self.LocationBlueBox_index] = self.current_location
                    self.RealValues[self.In_handBlueBox_index] = 'true'
                else:
                    print ('the object is not in hand\n')
                    self.RealValues[self.LocationBlueBox_index] = 'unknown'
                    self.RealValues[self.In_handBlueBox_index] = 'false'
            elif 'green_box' in fluent:
                green_pixel_count = self.tally_pixels(150, 250, 150, 180)
                if green_pixel_count > 0:
                    hpd = True
                    self.currently_holding = fluent[13:len(fluent)-1]
                    print ('the object is in hand\n')
                    self.RealValues[self.LocationGreenBox_index] = self.current_location
                    self.RealValues[self.In_handGreenBox_index] = 'true'
                else:
                    print ('the object is not in hand\n')
                    self.RealValues[self.LocationGreenBox_index] = 'unknown'
                    self.RealValues[self.In_handGreenBox_index] = 'false'

        return hpd




    # function for checking whether object is in hand
    def tally_pixels(self, b, g, r, variance):
        colour = BGR_colour(b, g, r, variance)
        pixel_count = 0
        for pixel_x in range(400):
            for pixel_y in range(640):
                pixel_matches_colour = colour.match(pixel_x, pixel_y, self.camera_image)
                if pixel_matches_colour: pixel_count = pixel_count + 1
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
































# Obscelete functions, undeleted in case I want to use them again later

'''
# this function tests the fluents relevant to the goal
# EDIT this section if different objects are included in the goal (e.g. if the goal involves a purple oject, add a section for testing the purple object's location etc)
def test_goal(newGoal, relevant_observations):
    print ('testing goal: ' + newGoal)

    # if there are blue pixels, the blue box is in the current location
    if ('blue' in newGoal):
        blue_pixel_count = tally_pixels(200, 0, 30, 250)
        if blue_pixel_count > 0:
            print ('the blue_box is in ' + global_variables.current_location)
            relevant_observations.append([2, global_variables.current_location])
        else:
            print ('the blue_box is not in ' + global_variables.current_location)
            relevant_observations.append([2, 'unknown'])

    # if there are green pixels, the green box is in the current location
    if ('green' in newGoal):
        green_pixel_count = tally_pixels(50, 250, 50, 180)
        if green_pixel_count > 0:
            print ('the green_box is in ' + global_variables.current_location)
            relevant_observations.append([5, global_variables.current_location])
        else:
            print ('the green_box is not in ' + global_variables.current_location)
            relevant_observations.append([5, 'unknown'])

    # if there are lots of blue pixels, the blue box must be close to the wrist camera (and, therefore, in_hand)
    if ('blue' in newGoal):
        blue_pixel_count = tally_pixels(200, 0, 30, 150)
        if blue_pixel_count > 100:
            print ('the blue_box is in_hand')
            relevant_observations.append([4, 'true'])
        else:
            print ('the blue_box is not in_hand')
            relevant_observations.append([4, 'false'])

    # if there are lots of green pixels, the green box must be close to the wrist camera (and, therefore, in_hand)
    if ('green' in newGoal):
        green_pixel_count = tally_pixels(0, 100, 0, 100)
        #if green_pixel_count > 10:
        if global_variables.green_box_in_hand: # TODO remove this, it's for testing purposes only
            print ('the green_box is in_hand')
            relevant_observations.append([6, 'true'])
        else:
            print ('the green_box is not in_hand')
            relevant_observations.append([6, 'false'])

    return relevant_observations



# MAIN
if __name__ == "__main__":

    # go back to initial position
    initial_pose = {
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
    ik_move(initial_pose)
'''
