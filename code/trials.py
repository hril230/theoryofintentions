'''Python program to run experimental trials in the agent reasoning programs (with toi and traditional planning
Objectives:
1. Select all possible valid initial states for the objects in the domain.
2. Run two pairwise for each initial state.

How I model different scenarios in realWorld.py:

# scenario 1 - just planning, everything goes fine, no unexpected changes in the world.
# scenario 2 - unexpected achievement of goal.
# scenario 3 - unexpected false observation that leads to not expected achievement of goal, diagnosis and replan.
# scenario 4 - unexpected true observation that leads to not expected achievement of goal, diagnosis and replan.
# scenario 5 - Failure to achieve goal, diagnosis, and re-planning.
# scenario 6 - Failure to execute, diagnosis and replaning.
'''


from datetime import datetime
from simulation.realWorld import World
from controllerTraditionalPlanning import ControllerTraditionalPlanning
from controllerToI import ControllerToI
from simulation.executer import Executer
import subprocess
import  csv
import random
from simulation.domain_info import DomainInfo
subfolder_path = 'simulation/'
results_file_name = "simulation/results/scenario_"


sparcPath = "$HOME/work/solverfiles/sparc.jar"

goal = "holds(loc(book1,library),I), holds(loc(book2,library),I), -holds(in_hand(rob1,book1),I), -holds(in_hand(rob1,book2),I) ."

maxPlanLength = 17
scenario = None

textfile = None
csvfile = None
writer = None
initial_state = []

locations = ['office1', 'office2', 'kitchen', 'library']
boolean = ['true', 'false']


runCount = 0

def run_and_write(scenario, initial_conditions_index):
	print 'initial_conditions_index: ' +str(initial_conditions_index)
	domain_info = DomainInfo()

	randomSeed = runCount
	world_trad = World(sparcPath,initial_state,scenario,randomSeed,domain_info)
	executer = Executer(world_trad)

	controllerToI = ControllerToI(sparcPath,goal, maxPlanLength, executer,domain_info, subfolder_path)
	history_toi, numberPlans_toi, goal_correction_toi = controllerToI.run()





def createConditionsAndRun(scenario):
	global initial_state


	initial_conditions_index = 0

	controlledRun = False
	controlledRunConditions = 2

	if(scenario == 'random'):
		controlledRun = True
		controlledRunConditions = random.randrange(1,97,1)
		controllerRunConditions = 42

	#Cases when rob1 is holding book1 (16 possible combinations)
	for robot_location in locations:
		for book2_location in locations:
			initial_conditions_index +=1
			if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue
			book1_location = robot_location
			in_handBook1 = 'true'
			in_handBook2 = 'false'
			initial_state = [robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
			run_and_write(scenario, initial_conditions_index)

	#Cases when rob1 is holding book2 (16 possible combinations)
	for robot_location in locations:
		for book1_location in locations:
			initial_conditions_index +=1
			if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue
			book2_location = robot_location
			in_handBook1 = 'false'
			in_handBook2 = 'true'
			initial_state = [robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
			run_and_write(scenario, initial_conditions_index)

	#Cases when rob1 is not holding any book (64 possible combinations)
	for robot_location in locations:
		for book1_location in locations:
			for book2_location in locations:
				initial_conditions_index +=1
				if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue
				in_handBook1 = 'false'
				in_handBook2 = 'false'
				initial_state = [robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
				run_and_write(scenario, initial_conditions_index)

if __name__ == "__main__":
	'''
	The main function, creates two results file.
	1. experimental_results.csv: A csv that contains the time taken for the reason during running for traditional planing vs theory of intentions .
	2. experimental_results.txt: A detailed text file that contains initial state, plans and history.
	'''
	scenarios = ['random']
	numberRunsRandomScenario = 1

	for s in scenarios:
		runCount = 0
		createConditionsAndRun(s)
		if(s == 'random'):
			for x in range (1,numberRunsRandomScenario): createConditionsAndRun(s)
