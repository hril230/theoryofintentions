# -*- coding: utf-8 -*-

from datetime import datetime
import controllerTraditionalPlanning
import baxterControllerToI as controllerToI
#from baxter_executor import Executer
from baxter_code import baxter_executor
import subprocess
from baxter_code.domain_info import DomainInfo

# initialize variables
sparcPath = "baxter_code/sparc.jar"
goal = 'holds(loc(green_box,zoneG),I), -holds(in_hand(rob1,green_box),I).'
max_plan_length = 17
history_trad = [""]
history_toi = [""]       
executer = baxter_executor.Executer()
ASP_subfolder = 'baxter_code/'
domain_info = DomainInfo()
known_world_state = executer.getInitialConditions()
refined_location = 'unknown_cell'

# run controller
version = 'ToI'
if version == 'traditional':
    controller = controllerTraditionalPlanning.ControllerTraditionalPlanning(sparcPath, ASP_subfolder, domain_info, executer, refined_location, known_world_state, goal, max_plan_length)
    history, numberPlans, goal_correction = controller.run()
elif version == 'ToI':
    controller = controllerToI.ControllerToI(sparcPath, ASP_subfolder, domain_info, executer, refined_location, known_world_state, goal, max_plan_length)
    history_toi, numberPlans_toi, goal_correction_toi = controller.run()
