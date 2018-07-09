# -*- coding: utf-8 -*-

from datetime import datetime
from realWorld import World
import controllerTraditionalPlanning
import controllerToI
from baxter_executor import Executer
import subprocess

# initialize variables
sparcPath = "$HOME/baxter_code/sparc.jar"
goal = 'holds(loc(green_box,zoneG),I), -holds(in_hand(rob1,green_box),I).'
maxPlanLength = 17
history_trad = [""]
history_toi = [""]       
executer = Executer()

# run controller
version = 'traditional'
if version == 'traditional': history_trad, plans_trad, goal_correction_trad = controllerTraditionalPlanning.controllerTraditionalPlanning(sparcPath, goal, maxPlanLength, executer)
elif version == 'ToI': history_toi, numberPlans_toi, goal_correction_toi = controllerToI.controllerToI(sparcPath, goal, maxPlanLength, executer)
