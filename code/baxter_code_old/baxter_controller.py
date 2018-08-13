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
import global_variables
from time import sleep



# Checks with the user about whether or not the assumption that the goal has been achieved is correct
def getGoalFeedback():
    print ('Has the goal been achieved? (y/n)')
    response = raw_input()
    if response == 'y':
        return True
    else:
        return False



# Defines the pose that the arm should take to reach a zone
class Zone():
    def __init__(self, name, x_coordinate, y_coordinate, arm):
        self.name = name
        self.pose = {
            arm: PoseStamped(
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
            arm: PoseStamped(
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
            arm: PoseStamped(
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
    def __init__(self, name, x_coordinate, y_coordinate, arm):
        self.name = name
        self.pose = {
            arm: PoseStamped(
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
            arm: PoseStamped(
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
            arm: PoseStamped(
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
            arm: PoseStamped(
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
    def match(self, pixel_x, pixel_y):
        pixel = global_variables.camera_image[pixel_x, pixel_y]
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



# this function is used for testing purposes only and should not be called during proper execution
def examine_image(b, g, r, variance):
    print ('Examining Image')
    colour = BGR_colour(b, g, r, variance)
    camera_image_edited = global_variables.camera_image
    for pixel_x in range(400):
        for pixel_y in range(640):
            pixel_matches_colour = colour.match(pixel_x, pixel_y)
            if pixel_matches_colour:
                print (global_variables.camera_image[pixel_x, pixel_y])
                camera_image_edited[pixel_x,pixel_y] = (255, 255, 255)
    cv2.imshow("temp", camera_image_edited)
    cv2.waitKey(0)
    return



# function for object detection
def object_detection(cam_res_one, cam_res_two, colour):
    # iterate through pixels in image and detect ones of a specific colour
    for pixel_x in range(cam_res_two):
        for pixel_y in range(cam_res_one):
            pixel_matches_colour = colour.match(pixel_x, pixel_y)
            if pixel_matches_colour:
                return pixel_x, pixel_y
    return 0, 0



# function for determining the location of an object
def locate_object(cam_res_one, cam_res_two, arm, limb_object, colour):

    # this pose is used to look down at the workspace and detect objects
    vertical_viewing_pose = {
        arm: PoseStamped(
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
    ik_move(vertical_viewing_pose, arm, limb_object)

    # detect object
    image_coordinate_x, image_coordinate_y = object_detection(cam_res_one, cam_res_two, colour)

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

    # determine which zone the box is in - EDIT this section to change the zones
    if (y_coordinate < 0):
        object_zone = 'zoneR'
    elif (y_coordinate < 0.2):
        object_zone = 'zoneG'
    else:
        object_zone = 'zoneY'

    return x_coordinate, y_coordinate, object_zone



# function for checking whether object is in hand
def tally_pixels(b, g, r, variance):
    colour = BGR_colour(b, g, r, variance)
    pixel_count = 0
    for pixel_x in range(400):
        for pixel_y in range(640):
            pixel_matches_colour = colour.match(pixel_x, pixel_y)
            if pixel_matches_colour:
                pixel_count = pixel_count + 1
    return pixel_count



# this function tests the fluents relevant to the goal
# EDIT this section if different objects are included in the goal (e.g. if the goal involves a purple oject, add a section for testing the purple object's location etc)
def test_goal(newGoal, relevant_observations, current_location):
    print ('testing goal: ' + newGoal)

    # if there are blue pixels, the blue box is in the current location
    blue_pixel_count = tally_pixels(200, 0, 30, 250)
    if blue_pixel_count > 0:
        print ('the blue_box is in ' + current_location)
        relevant_observations.append([2, current_location])
    else:
        print ('the blue_box is not in ' + current_location)
        relevant_observations.append([2, 'unknown'])

    # if there are green pixels, the green box is in the current location
    green_pixel_count = tally_pixels(150, 250, 150, 80)
    if green_pixel_count > 0:
        print ('the green_box is in ' + current_location)
        relevant_observations.append([5, current_location])
    else:
        print ('the green_box is not in ' + current_location)
        relevant_observations.append([5, 'unknown'])

    # if there are lots of blue pixels, the blue box must be close to the wrist camera (and, therefore, in_hand)
    blue_pixel_count = tally_pixels(200, 0, 30, 150)
    if blue_pixel_count > 100:
        print ('the blue_box is in_hand')
        relevant_observations.append([4, 'true'])
    else:
        print ('the blue_box is not in_hand')
        relevant_observations.append([4, 'false'])

    # if there are lots of blue pixels, the green box must be close to the wrist camera (and, therefore, in_hand)
    green_pixel_count = tally_pixels(150, 250, 150, 80)
    if green_pixel_count > 100:
        print ('the green_box is in_hand')
        relevant_observations.append([6, 'true'])
    else:
        print ('the green_box is not in_hand')
        relevant_observations.append([6, 'false'])

    return relevant_observations



# move arm to desired coordinates using inverse kinematics
def ik_move(poses, limb, limb_object):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()   
    ikreq.pose_stamp.append(poses[limb])
    #try:
    rospy.wait_for_service(ns, 5.0)
    resp = iksvc(ikreq)
    #except (rospy.ServiceException, rospy.ROSException), e:
    #    rospy.logerr("Service call failed: %s" % (e,))
    if (resp.isValid[0]):
        # move to desired coordinates using the angles given by inverse kinematics
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limb_object.move_to_joint_positions(limb_joints)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")



# this function executes a move action
def move(zone, arm, limb_object):
    ik_move(zone.elevated_pose, arm, limb_object)



# this function executes a pickup action
def pickup(obj, arm, limb_object, gripper):
    ik_move(obj.elevated_pose, arm, limb_object)
    gripper.open()
    ik_move(obj.pose, arm, limb_object)
    gripper.close()
    time.sleep(0.5)
    ik_move(obj.elevated_pose, arm, limb_object)



# this function executes a put down action
def put_down(zone, arm, limb_object, gripper):
    if not global_variables.object_placed:
        ik_move(zone.pose, arm, limb_object)
    else:
        ik_move(zone.mid_pose, arm, limb_object)
    gripper.open()
    ik_move(zone.elevated_pose, arm, limb_object)



# this function executes a test action
def test(action, current_location, current_refined_location, currently_holding, observations):
    
    relevant_observations = []

    # parse the fluent
    if 'true' in action:
        fluent = action[10:len(action)-6]
    elif 'false' in action:
        fluent = action[10:len(action)-7]

    # moving the robot's arm never fails
    if ('loc' in fluent) and ('rob1' in fluent):
        current_refined_location = fluent[9:len(fluent)-1]
        hpd = True
        print ('the robot is in cell ' + current_refined_location + '\n')

    # object location must be tested before attempting a pickup action - EDIT add sections for testing for pixels matching new objects
    elif 'loc' in fluent:
        # if there are blue pixels, the blue box is in the current location
        if 'blue' in fluent:
            blue_pixel_count = tally_pixels(200, 0, 30, 180)
            if blue_pixel_count > 50:
                print ('the blue_box is in ' + current_location)
                relevant_observations.append([2, current_location])
                hpd = True
            else:
                print ('the blue_box is not in ' + current_location)
                relevant_observations.append([2, 'unknown'])
                hpd = False
        # if there are green pixels, the green box is in the current location
        elif 'green' in fluent:
            green_pixel_count = tally_pixels(150, 250, 150, 180)
            if green_pixel_count > 50:
                print ('the green_box is in ' + current_location)
                relevant_observations.append([5, current_location])
                hpd = True
            else:
                print ('the green_box is not in ' + current_location)
                relevant_observations.append([5, 'unknown'])
                hpd = False

    # putting an object down never fails
    elif ('false' in action and 'in_hand' in fluent):
        currently_holding = 'nothing'
        global_variables.object_placed = True
        hpd = True
        print ('the object is not in hand\n')
        if 'yellow_box' in fluent:
            observations[1][1] = current_location
            relevant_observations.append(observations[1])
            observations[3][1] = 'false'
            relevant_observations.append(observations[3])
        elif 'blue_box' in fluent:
            observations[2][1] = current_location
            relevant_observations.append(observations[2])
            observations[4][1] = 'false'
            relevant_observations.append(observations[4])
        elif 'green_box' in fluent:
            observations[5][1] = current_location
            relevant_observations.append(observations[5])
            observations[6][1] = 'false'
            relevant_observations.append(observations[6])

    # picking up an object can fail and needs to be tested - EDIT this section to add tests for new objects
    else:
        hpd = False
        # test if there are pixels of the relevant colour close to the camer
        if 'yellow_box' in fluent:
            yellow_pixel_count = tally_pixels(0, 210, 210, 270)
            if yellow_pixel_count > 0:
                hpd = True
                currently_holding = fluent[13:len(fluent)-1]
                print ('the object is in hand\n')
                observations[1][1] = current_location
                relevant_observations.append(observations[1])
                observations[3][1] = 'true'
                relevant_observations.append(observations[3])
            else:
                currently_holding = 'nothing'
                print ('the object is not in hand\n')
                observations[1][1] = 'unknown'
                relevant_observations.append(observations[1])
                observations[3][1] = 'false'
                relevant_observations.append(observations[3])
        elif 'blue_box' in fluent:
            blue_pixel_count = tally_pixels(200, 0, 30, 180)
            if blue_pixel_count > 0:
                hpd = True
                currently_holding = fluent[13:len(fluent)-1]
                print ('the object is in hand\n')
                observations[2][1] = current_location
                relevant_observations.append(observations[2])
                observations[4][1] = 'true'
                relevant_observations.append(observations[4])
            else:
                currently_holding = 'nothing'
                print ('the object is not in hand\n')
                observations[2][1] = 'unknown'
                relevant_observations.append(observations[2])
                observations[4][1] = 'false'
                relevant_observations.append(observations[4])
        elif 'green_box' in fluent:
            green_pixel_count = tally_pixels(150, 250, 150, 180)
            if green_pixel_count > 0:
                hpd = True
                currently_holding = fluent[13:len(fluent)-1]
                print ('the object is in hand\n')
                observations[5][1] = current_location
                relevant_observations.append(observations[5])
                observations[6][1] = 'true'
                relevant_observations.append(observations[6])
            else:
                currently_holding = 'nothing'
                print ('the object is not in hand\n')
                observations[5][1] = 'unknown'
                relevant_observations.append(observations[5])
                observations[6][1] = 'false'
                relevant_observations.append(observations[6])

    return current_refined_location, currently_holding, hpd, relevant_observations



# this function parses the intended action and calls the relevant refined function
def execute_refined_action(action, objects, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding):

    observations = [[0,'unknown'], [1,'unknown'], [2,'unknown'], [3,'unknown'], [4,'unknown'], [5,'unknown'], [6,'unknown']]
    relevant_observations = []

    print ('\nexecuting refined action: ' + action)

    # execute move actions
    if 'move' in action:
        for cell in cells:
            if (action[10 : len(action)-1] == cell.name):
                move(cell, arm, limb_object)
                return current_refined_location, current_location, currently_holding, relevant_observations, 10, True

    # execute pickup actions
    elif 'pickup' in action:
        for obj in objects:
            if (obj.name == action[12 : len(action)-1]):
                pickup(obj, arm, limb_object, gripper)
                return current_refined_location, current_location, currently_holding, relevant_observations, 2, True

    # execute put down actions
    elif 'put_down' in action:
        for cell in cells:
            if (current_refined_location == cell.name):
                put_down(cell, arm, limb_object, gripper)
                return current_refined_location, current_location, currently_holding, relevant_observations, 2, True

    # execute test actions
    elif 'test' in action:
        current_refined_location, currently_holding, hpd, relevant_observations = test(action, current_location, current_refined_location, currently_holding, observations)
        if (current_refined_location == 'c1') or (current_refined_location == 'c2') or (current_refined_location == 'c3') or (current_refined_location == 'c4'):
            current_location = 'zoneR'
        elif (current_refined_location == 'c5') or (current_refined_location == 'c6') or (current_refined_location == 'c7') or (current_refined_location == 'c8'):
            current_location = 'zoneG'
        elif (current_refined_location == 'c9') or (current_refined_location == 'c10') or (current_refined_location == 'c11') or (current_refined_location == 'c12'):
            current_location = 'zoneY'
        observations[0][1] = current_location
        relevant_observations.append(observations[0])
        return current_refined_location, current_location, currently_holding, relevant_observations, 1, hpd

    # warn user if the plan contains an action that this function isn't programmed for
    else:
        print ('action not recognised')



# this function executes a refined action plan
def execute_refined_plan(refined_plan, objects, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding):

    # parse refined plan to get an ordered list of refined actions
    refined_plan = refined_plan[1:len(refined_plan)-1]
    refined_plan = refined_plan.replace(' ','')
    refined_actions = refined_plan.split('occurs')
    refined_actions.remove('')
    for i in range(len(refined_actions)):
        if not (i == len(refined_actions)-1):
            refined_actions[i] = refined_actions[i][0:len(refined_actions[i])-1]
    ordered_actions = []
    for i in range(len(refined_actions)):
        for action in refined_actions:
            if (','+str(i)+')') in action:
                ordered_actions.append(action[1:len(action)-3])

    # execute each action in refined plan and get relevant outputs
    relevant_observations = []
    time = 0
    happened = True
    for action in ordered_actions:
        current_refined_location, current_location, currently_holding, observations_refined, time_refined, happened_refined = execute_refined_action(action, objects, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding)
        for observation in observations_refined:
            for relevant_observation in relevant_observations:
                if observation[0] == relevant_observation[0]:
                    relevant_observations.remove(relevant_observation)
            relevant_observations.append(observation)
        time += time_refined
        if not happened_refined:
            happened = False
            break

    return current_refined_location, current_location, currently_holding, relevant_observations, time, happened, objects



# this function uses the refined_baxter_domain.txt file and SPARC to get a refined action plan
def refine(action, history, current_step, version, current_refined_location, currently_holding):

    # convert ToI history into traditional format so that they can be processed in the same way
    history_formatted = []
    if version == 'ToI':
        for i in range(len(history)):
            if ('hpd' in history[i]) and ('true' in history[i]):
                if ('0)' in history[i]) or ('1)' in history[i]) or ('2)' in history[i]) or ('3)' in history[i]) or ('4)' in history[i]) or ('5)' in history[i]) or ('6)' in history[i]) or ('7)' in history[i]) or ('8)' in history[i]) or ('9)' in history[i]):
                    history_formatted.append(history[i][0:len(history[i])-8] + history[i][len(history[i])-3:len(history[i])])
                else:
                    history_formatted.append(history[i][0:len(history[i])-9] + history[i][len(history[i])-4:len(history[i])])
            elif not 'hpd' in history[i]:
                history_formatted.append(history[i])
    else: history_formatted = history
  
    # use action and history to figure out the transition (initial_state, action, final_state)
    # the location of the robot is relevant for move transitions
    if 'move' in action:
        # get the initial state from step zero
        for observation in history_formatted:
            if ('obs' in observation) and ('rob1' in observation) and ('loc' in observation):
                initial_state = ['coarse_loc(rob1,' + observation[13:len(observation)-10] + ')']
        # if actions have happened since step zero, initial state may need to be updated
        for observation in history_formatted:
            if ('hpd' in observation) and ('move' in observation) and (not 'exo' in observation):
                if ('0)' in observation) or ('1)' in observation) or ('2)' in observation) or ('3)' in observation) or ('4)' in observation) or ('5)' in observation) or ('6)' in observation) or ('7)' in observation) or ('8)' in observation) or ('9)' in observation):
                    initial_state = ['coarse_loc(rob1,' + observation[14:len(observation)-5] + ')']
                else:
                    initial_state = ['coarse_loc(rob1,' + observation[14:len(observation)-6] + ')']
        final_state = ['coarse_loc(rob1,' + action[10:len(action)-1] + ')']

    # the location of the robot and object, and the in_hand status of the object, are relevant for pickup transitions
    elif 'pickup' in action:
        obj = action[12:len(action)-1]
        # include the in_hand status of the object
        initial_state = ['-coarse_in_hand(rob1,' + obj + ')']
        final_state = ['coarse_in_hand(rob1,' + obj + ')']
        # include the location of the robot
        for observation in history_formatted:
            if ('obs' in observation) and ('rob1' in observation) and ('loc' in observation):
                rob_loc = 'coarse_loc(rob1,' + observation[13:len(observation)-10] + ')'
        for observation in history_formatted:
            if ('hpd' in observation) and ('move' in observation):
                rob_loc = 'coarse_loc(rob1,' + observation[14:len(observation)-5] + ')'
        # include the location of the object
        for observation in history_formatted:
            if ('obs' in observation) and (obj in observation) and ('loc' in observation):
                obj_loc = 'coarse_loc(' + obj + ',' + observation[9+len(obj):len(observation)-10] + ')'
            if ('exo' in observation) and (obj in observation):
                obj_loc = 'coarse_loc(' + obj + ',' + observation[14+len(obj):len(observation)-9] + ')'
        initial_state.append(rob_loc)
        initial_state.append(obj_loc)
        final_state.append('coarse_loc(rob1,' + obj_loc[12+len(obj):len(obj_loc)-1] + ')')
        final_state.append(obj_loc)

    # the in_hand status of the object is relevant for put_down transitions
    elif 'put_down' in action:
        initial_state = ['coarse_in_hand(rob1,' + action[14:len(action)-1] + ')']
        final_state = ['-coarse_in_hand(rob1,' + action[14:len(action)-1] + ')']

    # edit refined_asp to get temporary zoomed_asp file
    zoom(initial_state, action, final_state, current_refined_location, currently_holding)

    # get refined answer set
    os.system('java -jar sparc.jar zoomed_baxter_domain.txt -A > tmp')
    refined_answer = open('tmp', 'r').read()
    os.remove('tmp')

    # stop running code if refined answer set is empty
    if refined_answer == '\n':
        raw_input('No refined answer set, press enter if you wish to continue.\n')

    # refined_plan = parse answer set
    refined_plan = refined_answer
    return refined_plan



# this action writes a zoomed ASP file
def zoom(initial_state, action, final_state, current_refined_location, currently_holding):

    # clean up typos
    for i in range(len(initial_state)):
        if '))' in initial_state[i]:
            initial_state[i] = initial_state[i][0:len(initial_state[i])-1]
    for i in range(len(final_state)):
        if '))' in final_state[i]:
            final_state[i] = final_state[i][0:len(final_state[i])-1]

    # EDIT these lists to change the domain
    coarse_places = Sort('coarse_place', ['zoneR', 'zoneG', 'zoneY', 'unknown'])
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
    unknown_components = Components('unknown', ['unknown_cell'])
    yellow_box_components = Components('yellow_box', ['yellow_box_top'])
    blue_box_components = Components('blue_box', ['blue_box_top'])
    green_box_components = Components('green_box', ['green_box_top'])
    refinements = [zoneR_components, zoneG_components, zoneY_components, yellow_box_components, blue_box_components, green_box_components, unknown_components]

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
    irrelevant_obj_consts = ['zoneR', 'zoneG', 'zoneY', 'unknown', 'yellow_box', 'blue_box', 'green_box', 'c1,', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'yellow_box_top', 'blue_box_top', 'green_box_top']
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
            rel_initial_conditions[i] = 'loc(rob1,' + current_refined_location + ')'
        if ('in_hand' in rel_initial_conditions[i]) and (not '-' in rel_initial_conditions[i]):
            rel_initial_conditions[i] = 'in_hand(rob1,' + currently_holding + ')'

    # determine which final conditions are relevant
    for condition in final_state:
        if not condition in initial_state:
            rel_final_conditions.append(condition)
            rel_conditions.append(condition)

    # determine which object constants are relevant
    for condition in rel_conditions:
        for index in range(len(condition)):
            if condition[index] == '(':
                opening_bracket = index
            elif condition[index] == ')':
                closing_bracket = index
        obj_consts = condition[opening_bracket+1:closing_bracket].split(',')
        for const in obj_consts:
            rel_obj_consts.append(const)
            if const in irrelevant_obj_consts:
                irrelevant_obj_consts.remove(const)
    rel_obj_consts = list(set(rel_obj_consts)) # remove duplicates

    # add refined components of relevant object constants
    for const in rel_obj_consts:
        for refinement in refinements:
            if const == refinement.name:
                for refined_const in refinement.components:
                    rel_obj_consts.append(refined_const)  
                    if refined_const == 'c1':
                        refined_const = 'c1,'
                    if refined_const in irrelevant_obj_consts:
                        irrelevant_obj_consts.remove(refined_const)

    # sort relevant objects into types
    for sort in sorts:
        for const in sort.constants:
            if const in rel_obj_consts:
                sort.add(const)

    # determine which sorts should be included in the zoomed description
    for sort in sorts:
        if len(sort.rel_constants) != 0:
            rel_sorts.append(sort)
            rel_sort_names.append('#'+sort.name)
            if ('#'+sort.name) in irrelevant_sort_names:
                irrelevant_sort_names.remove('#'+sort.name)

    # add relevant sorts to sorts of sorts (coarse_things, things, coarse_components and refined_components)
    for sort in rel_sorts:
        for sort_of_sorts in sorts:
            if ('#'+sort.name) in sort_of_sorts.constants:
                sort_of_sorts.add('#'+sort.name)

    # determine which inertial fluents are relevant
    for fluent in inertial_fluents:
        fluent_relevant = True
        for index in range(len(fluent)):
            if fluent[index] == '(':
                opening_bracket = index
            elif fluent[index] == ')':
                closing_bracket = index
        fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
        for sort in fluent_sorts:
            if not sort in rel_sort_names:
                fluent_relevant = False
        if fluent_relevant:
            rel_inertial_fluents.append(fluent)
            if fluent[0:opening_bracket] in irrelevant_fluents:
                irrelevant_fluents.remove(fluent[0:opening_bracket])

    # determine which defined fluents are relevant
    for fluent in defined_fluents:
        fluent_relevant = True
        for index in range(len(fluent)):
            if fluent[index] == '(':
                opening_bracket = index
            elif fluent[index] == ')':
                closing_bracket = index
        fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
        for sort in fluent_sorts:
            if not sort in rel_sort_names:
                fluent_relevant = False
        if fluent_relevant:
            rel_defined_fluents.append(fluent)
            if fluent[0:opening_bracket] in irrelevant_fluents:
                irrelevant_fluents.remove(fluent[0:opening_bracket])

    # determine which actions are relevant
    for act in actions:
        action_relevant = True
        for index in range(len(act)):
            if act[index] == '(':
                opening_bracket = index
            elif act[index] == ')':
                closing_bracket = index
        action_sorts = act[opening_bracket+1:closing_bracket].split(',')
        for sort in action_sorts:
            if not sort in rel_sort_names:
                action_relevant = False
        if action_relevant:
            rel_actions.append(act)
            if act[0:opening_bracket] in irrelevant_actions:
                irrelevant_actions.remove(act[0:opening_bracket])

    # determine what the goal of the refined ASP should be
    goal = 'goal(I) :- '
    for condition in rel_final_conditions:
        if '-' in condition:
            condition = condition.replace('-', '')
            goal = goal + '-holds(' + condition + ',I), '
        else:
            goal = goal + 'holds(' + condition + ',I), '
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



# MAIN
if __name__ == "__main__":

    # initialize global variables
    global_variables.init()

    # initialize ROS node
    rospy.init_node("rsdk_ik_service_client")

    current_location = 'unknown'
    current_refined_location = 'unknown_cell'
    currently_holding = 'nothing'

    # set which arm is to be used (left or right)
    arm = 'left'

    # create an instance of baxter_interface's Limb class
    limb_object = baxter_interface.Limb(arm)

    # initialise and calibrate gripper object
    gripper = Gripper(arm)
    gripper.calibrate()
    gripper.set_holding_force(100)

    # get the arm's current joint angles
    angles = limb_object.joint_angles()

    # initialize zones
    zoneR = Zone('zoneR', 0.4, -0.1, arm)
    zoneG = Zone('zoneG', 0.4, 0.1, arm)
    zoneY = Zone('zoneY', 0.4, 0.3, arm)
    zones = [zoneR, zoneG, zoneY]
    c1 = Zone('c1', 0.4, -0.1, arm)
    c2 = Zone('c2', 0.4, -0.0, arm)
    c3 = Zone('c3', 0.45, -0.1, arm)
    c4 = Zone('c4', 0.45, -0.0, arm)
    c5 = Zone('c5', 0.4, 0.1, arm)
    c6 = Zone('c6', 0.4, 0.2, arm)
    c7 = Zone('c7', 0.45, 0.1, arm)
    c8 = Zone('c8', 0.45, 0.2, arm)
    c9 = Zone('c9', 0.4, 0.3, arm)
    c10 = Zone('c10', 0.4, 0.4, arm)
    c11 = Zone('c11', 0.45, 0.3, arm)
    c12 = Zone('c12', 0.45, 0.4, arm)
    cells = [c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12]
 
   # set starting position to zero
    if (arm == 'left'):
        angles['left_s0']=0.0
        angles['left_s1']=0.0
        angles['left_e0']=0.0
        angles['left_e1']=0.0
        angles['left_w0']=0.0
        angles['left_w1']=0.0
        angles['left_w2']=0.0
    else:
        angles['right_s0']=0.0
        angles['right_s1']=0.0
        angles['right_e0']=0.0
        angles['right_e1']=0.0
        angles['right_w0']=0.0
        angles['right_w1']=0.0
        angles['right_w2']=0.0

    # set camera resolution and set camera
    cam_res_one = 640
    cam_res_two = 400
    left_camera = CameraController('left_hand_camera')
    left_camera.resolution = (cam_res_one, cam_res_two)
    left_camera.open()

    # get camera image and convert it to OpenCV format with bgr8 encoding
    def get_img(msg):
        global_variables.camera_image = msg_to_cv(msg)
  
    def msg_to_cv(msg):
        return cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')

    camera_subscriber = rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_img)

    while global_variables.camera_image is None:
        pass



    # EDIT: add code to determine the locations of new objects here    

    # determine the location of the green box
    green = BGR_colour(150, 250, 150, 80)
    x_coordinate, y_coordinate, green_box_zone = locate_object(cam_res_one, cam_res_two, arm, limb_object, green)

    # initialise the green_box object
    global_variables.green_box = Object('green_box', x_coordinate, y_coordinate, arm)
    global_variables.green_box_top = Object('green_box_top', x_coordinate-0.03, y_coordinate+0.03, arm)

    # determine the location of the blue box
    blue = BGR_colour(200, 0, 30, 160)
    x_coordinate, y_coordinate, blue_box_zone = locate_object(cam_res_one, cam_res_two, arm, limb_object, blue)

    # initialise the blue_box object
    global_variables.blue_box = Object('blue_box', x_coordinate, y_coordinate, arm)
    global_variables.blue_box_top = Object('blue_box_top', x_coordinate-0.03, y_coordinate+0.02, arm)

    # determine the location of the yellow box
    #yellow = BGR_colour(0, 210, 210, 270)
    #x_coordinate, y_coordinate, yellow_box_zone = locate_object(cam_res_one, cam_res_two, arm, limb_object, yellow)

    # initialise the yellow_box object
    #global_variables.yellow_box = Object('yellow_box', x_coordinate, y_coordinate, arm)
    #global_variables.yellow_box_top = Object('yellow_box_top', x_coordinate, y_coordinate, arm)

    objects = [global_variables.blue_box, global_variables.blue_box_top, global_variables.green_box, global_variables.green_box_top]
    goal = 'holds(loc(blue_box,zoneY),I), holds(loc(green_box,zoneY),I), -holds(in_hand(rob1,blue_box),I), -holds(in_hand(rob1,green_box),I).'



    # initialize variables to pass to ASP functions
    maxPlanLength = 13
    initial_conditions = [[0,'unknown'], [2,blue_box_zone], [3,'false'], [4,'false'], [5,green_box_zone], [6,'false']] # EDIT to include new objects

    # set this variable to decide whether to use traditional planning or theory of intentions
    version = 'traditional'

    # run traditional planning
    if version == 'traditional':
        start_time_trad = datetime.now()
        history_traditional, plans_trad, time_executing_trad  = controllerTraditionalPlanning.controllerTraditionalPlanning(goal, maxPlanLength, initial_conditions, objects, zones, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding)
        end_time_trad = datetime.now()
        time_planning_trad = (end_time_trad - start_time_trad).total_seconds()
    
    # run theory of intentions    
    elif version == 'ToI':
        start_time_toi = datetime.now()
        history_toi, numberPlans_toi  = controllerToI.controllerToI(goal, maxPlanLength, initial_conditions, objects, zones, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding)
        end_time_toi = datetime.now()
        time_planning_toi = (end_time_toi - start_time_toi).total_seconds()

    # go back to initial position
    initial_pose = {
        arm: PoseStamped(
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
    ik_move(initial_pose, arm, limb_object)
