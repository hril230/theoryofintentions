# -*- coding: utf-8 -*-
'''Python program to run trials in the agent reasoning programs with ToI
1. Select all possible valid initial states for the objects in the domain.
2. Run each initial state.

How I classify trial results according to different scenarios in realWorld.py:

# scenario 1 - just planning, everything goes fine, no unexpected changes in the world.
# scenario 2 - unexpected achievement of goal.
# scenario 3 - unexpected false observation that leads to not expected achievement of goal, diagnosis and replan.
# scenario 4 - unexpected true observation that leads to not expected achievement of goal, diagnosis and replan.
# scenario 5 - Failure to achieve goal, diagnosis, and re-planning.
# scenario 6 - Failure to execute, diagnosis and replaning.
'''


from datetime import datetime
from realWorld import World
import controllerTraditionalPlanning
import controllerToI
from executer import Executer
import subprocess
import random

global goal
results_folder = 'results/'
sparcPath = "$HOME/work/solverfiles/sparc.jar"


maxPlanLength = 17
scenario = None

textfile = None
locations = ['office1', 'office2', 'kitchen', 'library']
boolean = ['true', 'false']


runCount = 0

def run_and_write(scenario, initial_conditions_index, initial_state):
	global runCount
	runCount +=1
	#print("$$$$$$$$$$$$$$$$$$$   Run number " + str(runCount) +"    $$$$$$$$$$$$$$$$$$$")
	history_toi = [""]
	time_planning_toi = 0
	time_executing_toi = 0
	scenarioRecreated_toi = 0
	length = 0


	randomSeed = runCount
	start_time_toi = datetime.now()
	world_toi = World(sparcPath,initial_state,scenario,randomSeed)
	executer = Executer(world_toi)
	history_toi, numberPlans_toi, goal_correction_toi = controllerToI.controllerToI(sparcPath,goal, maxPlanLength, executer)
	end_time_toi = datetime.now()
	time_planning_toi = (end_time_toi - start_time_toi).total_seconds()
	historyWorld_toi = world_toi.getHistory()
	time_executing_toi = world_toi.getExecutionTimeUnits()
	steps_executed_toi = world_toi.getExecutedSteps()
	exo_action = str(world_toi.get_exo_action_happened())[0]
	achieved_goal_toi = (world_toi.achievedGoal()[0]).capitalize()


	toi_exo_action = ''
	toi_exo_move_book = ''
	toi_exo_move_room = ''
	toi_exo_lock = ''
	for item in historyWorld_toi:
		if('exo' in item):
			toi_exo_action = item
			break

	if('move' in toi_exo_action):
		toi_exo_move_book = item[9:14]
		toi_exo_move_room = item[15:-1]
	if('lock' in toi_exo_action): toi_exo_lock = 'true'



	activityInfo=[]
	historyInfo=[]
	firstActivityLength = 0
	secondActivity = False
	firstStop = 0

	##working out what scenario it has been run.
	for item in history_toi:
		if('activity' in item or 'finish' in item or 'selected_goal_holds' in item or 'Goal is futile' in item or 'Goal holds' in item or 'unobserved' in item):
			activityInfo.append(item)
			if('activity_length(1' in item): firstActivityLength = int(item[item.rfind(',')+1:-2])
		else:
			if('attempt(stop(1),' in item): firstStop = int(item[16:-2])
			if('attempt(start(2),' in item): secondActivity = True
			split_item = [''] * 2

			split_item[0] = item[:item.rfind(',')]
			split_item[1] = item[item.rfind(',')+1:-2]
			historyInfo.append(split_item)
	if(firstActivityLength + 2 == firstStop): scenarioRecreated_toi = 5
	elif(secondActivity == False and steps_executed_toi < firstActivityLength and achieved_goal_toi): scenarioRecreated_toi = 2
	elif(steps_executed_toi == firstActivityLength and secondActivity == False): scenarioRecreated_toi = 1

	elif(firstStop > 0 and firstStop < firstActivityLength + 2 and secondActivity == True):
		#print('it has stopped before end of plan, and started a second activity')
		if(toi_exo_lock == 'true'):
			scenarioRecreated_toi = 6
		elif(toi_exo_move_book != ''):
			#print('the exo actio is a move of this book: ' + toi_exo_move_book +  ' to this room: '+ toi_exo_move_room)
			#print('looking for observation: ' + str(['obs(loc('+toi_exo_move_book+','+toi_exo_move_room+'),true' , str(firstStop)]))
			if(['obs(loc('+toi_exo_move_book+','+toi_exo_move_room+'),true' , str(firstStop)] in historyInfo):
				scenarioRecreated_toi = 4
			else:
				scenarioRecreated_toi = 3

	historyInfo.sort(key=lambda x:x[0])
	historyInfo.sort(key=lambda x:int(x[1]))
	historyInfo = [','.join(item)+')' for item in historyInfo]
	history_toi = historyInfo + activityInfo


	#Writing to txt
	textfile.write("$$$$$$$$$$$$$$$$$$$   Run number " + str(runCount) +"   $$$$$$$$$$$$$$$$$$$\n")
	textfile.write("Running Scenario "+str(scenarioRecreated_toi)+"\n")
	textfile.write("Goal: "+goal+"\n")
	textfile.write("Initial Conditions: "+str(initial_state)+"\n")
	textfile.write("Initial Conditions Index: "+str(initial_conditions_index)+"\n")
	textfile.write("\n\n")
	textfile.write("Achieved goal ToI: "+ str(achieved_goal_toi)+"\n")
	textfile.write("History World: " +str(historyWorld_toi) + "\n")
	textfile.write("History ToI: "+ "\n"+ str(history_toi)+"\n")


	if('Goal is futile' in history_toi):
		textfile.write("Goal is futile\n")

	elif('Goal holds' in history_toi):
		textfile.write("Goal holds\n")

	textfile.write('\n--------------------------------------------------------------\n\n')




def createConditionsAndRunAll(scenario):
	initial_conditions_index = 0

	controlledRun = False
	controlledRunConditions = 2

	if(scenario == 'random'):
		controlledRun = True
		controlledRunConditions = random.randrange(1,193,1)

	#Cases when rob1 is holding book1
	for locked in boolean:
		for robot_location in locations:
			for book2_location in locations:
				initial_conditions_index +=1
				if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue
                                book1_location = robot_location
                                in_handBook1 = 'true'
                                in_handBook2 = 'false'
				initial_state = [locked, robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
				run_and_write(scenario, initial_conditions_index, initial_state)

	#Cases when rob1 is holding book2
	for locked in boolean:
		for robot_location in locations:
			for book1_location in locations:
				initial_conditions_index +=1
				if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue
                                book2_location = robot_location
                                in_handBook1 = 'false'
                                in_handBook2 = 'true'
  				initial_state = [locked, robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
				run_and_write(scenario, initial_conditions_index, initial_state)

	#Cases when rob1 is not holding any book
	for locked in boolean:
		for robot_location in locations:
			for book1_location in locations:
			     	for book2_location in locations:
					initial_conditions_index +=1
					if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue
                                	in_handBook1 = 'false'
                                	in_handBook2 = 'false'
	  				initial_state = [locked, robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
				run_and_write(scenario, initial_conditions_index, initial_state)

if __name__ == "__main__":
	global goal
	textfile = open(results_folder +'results_ToI_test.txt', 'w')

	goal = "holds(loc(book1,library),I), holds(loc(book2,library),I), -holds(in_hand(rob1,book1),I), -holds(in_hand(rob1,book2),I) ."
	#initial_state = [locked, robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
	initial_state = ['false', 'library', 'library', 'kitchen', 'true', 'false']

	run_and_write('random_exo_actions',0,initial_state)

	#createConditionsAndRunAll('random_exo_actions')