# -*- coding: utf-8 -*-
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
from realWorld import World
import controllerTraditionalPlanning
import controllerToI
from executer import Executer
import subprocess
import  csv
import random

sparcPath = "$HOME/work/solverfiles/sparc.jar"

goal = "holds(loc(book1,library),I), holds(loc(book2,library),I), -holds(in_hand(rob1,book1),I), -holds(in_hand(rob1,book2),I) ."

maxPlanLength = 17
reRuns = None
scenario = None

textfile = None
csvfile = None
writer = None
initial_state = []

locations = ['office1', 'office2', 'kitchen', 'library']
boolean = ['true', 'false']


runCount = 0

def run_and_write(scenario, initial_conditions_index):  
	global runCount
	
	runCount +=1
	print("$$$$$$$$$$$$$$$$$$$   Run number " + str(runCount) +"    $$$$$$$$$$$$$$$$$$$")
	history_trad = [""]
	history_toi = [""]
	time_planning_trad = 0
	time_planning_toi = 0
	time_executing_trad = 0
	time_executing_toi = 0  
	scenarioRecreated_toi = 0      
        length = 0

	start_time_trad = datetime.now()
	randomSeed = runCount
	world_trad = World(sparcPath,initial_state,scenario,randomSeed)
	executer = Executer(world_trad)
	history_trad, plans_trad, goal_correction_trad = controllerTraditionalPlanning.controllerTraditionalPlanning(sparcPath,goal, maxPlanLength, executer)
	end_time_trad = datetime.now()
	time_planning_trad = (end_time_trad - start_time_trad).total_seconds()
	numberPlans_trad = 0
	for item in plans_trad:
		if(item[0] != 'No Plan'): numberPlans_trad += 1

	achieved_goal_trad = (world_trad.achievedGoal()[0]).capitalize()
	historyWorld_trad = world_trad.getHistory()
	time_executing_trad = world_trad.getExecutionTimeUnits()
	steps_executed_trad = world_trad.getExecutedSteps()

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
	
	trad_exo_action = ''
	for item in historyWorld_trad:
		if('exo' in item): trad_exo_action = item

	same_exo_action = 'F'
	if(trad_exo_action == toi_exo_action): same_exo_action = 'T'


	activityInfo=[]
	historyInfo=[]
	firstActivityLength = 0
	secondActivity = False
	firstStop = 0
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
		print('it has stopped before end of plan, and started a second activity')
		if(toi_exo_lock == 'true'): 
			scenarioRecreated_toi = 6		
		elif(toi_exo_move_book != ''):
			print('the exo actio is a move of this book: ' + toi_exo_move_book +  ' to this room: '+ toi_exo_move_room) 
			print('looking for observation: ' + str(['obs(loc('+toi_exo_move_book+','+toi_exo_move_room+'),true' , str(firstStop)]))
			if(['obs(loc('+toi_exo_move_book+','+toi_exo_move_room+'),true' , str(firstStop)] in historyInfo): 
				scenarioRecreated_toi = 4
			else: 
				scenarioRecreated_toi = 3

	historyInfo.sort(key=lambda x:x[0])
	historyInfo.sort(key=lambda x:int(x[1]))
	historyInfo = [','.join(item)+')' for item in historyInfo]
	history_toi = historyInfo + activityInfo
		
	

	for i in range (reRuns):
		start_time_trad = datetime.now()
		world_toi = World(sparcPath,initial_state,scenario)
		executer = Executer(world_toi)
		history_trad, plans_trad, goal_correction_trad   = controllerTraditionalPlanning.controllerTraditionalPlanning(goal, maxPlanLength, executer)
		end_time_trad = datetime.now()
		another_time_planning_trad = (end_time_trad - start_time_trad).total_seconds()

		start_time_toi = datetime.now()
		world_toi = World(sparcPath,nitial_state,scenario)
		executer = Executer(world_toi)
		history_toi, numberPlans_toi, goal_correction_toi = controllerToI.controllerToI(goal, maxPlanLength, executer, scenario)
		end_time_toi = datetime.now()
		another_time_planning_toi = (end_time_toi - start_time_toi).total_seconds()

		time_planning_trad = min(time_planning_trad,another_time_planning_trad)
		time_planning_toi = min(time_planning_toi,another_time_planning_toi)
		

		
	

	#Writing to txt
	textfile.write("$$$$$$$$$$$$$$$$$$$   Run number " + str(runCount) +"   $$$$$$$$$$$$$$$$$$$\n")
        textfile.write("Running Scenario "+str(scenarioRecreated_toi)+"\n")
	textfile.write("Goal: "+goal+"\n")
	textfile.write("Initial Conditions: "+str(initial_state)+"\n")
	textfile.write("Initial Conditions Index: "+str(initial_conditions_index)+"\n")
	textfile.write("\n\n")

        textfile.write("Achieved goal Traditional: "+ str(achieved_goal_trad)+"\n")
       	textfile.write("History World: " +str(historyWorld_trad) + "\n")
	textfile.write("Plans Traditional: \n")
        for plan in plans_trad:
                textfile.write(str(plan) + "\n")
	textfile.write("History Traditional: " + str(history_trad)+"\n")
 	textfile.write("\n\n")
 

        textfile.write("Achieved goal ToI: "+ str(achieved_goal_toi)+"\n")
       	textfile.write("History World: " +str(historyWorld_toi) + "\n")
	textfile.write("History ToI: "+ "\n"+ str(history_toi)+"\n")


	l=[initial_conditions_index]
	l.append(round(time_planning_trad+5*time_executing_trad,2))
	l.append(round(time_planning_toi+5*time_executing_toi,2))
        l.append(numberPlans_trad)
        l.append(numberPlans_toi)
	l.append(round(time_planning_trad,2))
	l.append(round(time_planning_toi,2))
	l.append(time_executing_trad)
        l.append(time_executing_toi)
	l.append(steps_executed_trad)
	l.append(steps_executed_toi)
	l.append(achieved_goal_trad)
	l.append(achieved_goal_toi)
	l.append(goal_correction_trad)
	l.append(goal_correction_toi)
	l.append(exo_action)
	l.append(scenarioRecreated_toi)
	l.append(same_exo_action)
	l.append(firstStop-2)


	

	if('Goal is futile' in history_toi):
		textfile.write("Goal is futile\n")
		l.append("Goal is futile")

	elif('Goal holds' in history_toi):
		textfile.write("Goal holds\n")	



	textfile.write('\n--------------------------------------------------------------\n\n')
	writer.writerow(l)
	csvfile.flush()



def createConditionsAndRun(scenario):
	global initial_state
	

	initial_conditions_index = 0
	
	controlledRun = False
	controlledRunConditions = 2

	if(scenario == 'random'):
		controlledRun = True
		controlledRunConditions = random.randrange(1,193,1)

	#Cases when rob1 is holding book1
	for locked_or_not in boolean:
		for robot_location in locations:			
			for book2_location in locations:
				initial_conditions_index +=1
				if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue	
                                book1_location = robot_location
                                in_handBook1 = 'true'
                                in_handBook2 = 'false'
				initial_state = [locked_or_not, robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
				run_and_write(scenario, initial_conditions_index)

	#Cases when rob1 is holding book2
	for locked_or_not in boolean:
		for robot_location in locations:
			for book1_location in locations:
				initial_conditions_index +=1
				if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue	
                                book2_location = robot_location
                                in_handBook1 = 'false'
                                in_handBook2 = 'true'
  				initial_state = [locked_or_not, robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
				run_and_write(scenario, initial_conditions_index)

	#Cases when rob1 is not holding any book
	for locked_or_not in boolean:
		for robot_location in locations:
			for book1_location in locations:
			     	for book2_location in locations:
					initial_conditions_index +=1
					if(controlledRun == True and initial_conditions_index != controlledRunConditions): continue	
                                	in_handBook1 = 'false'
                                	in_handBook2 = 'false'
	  				initial_state = [locked_or_not, robot_location , book1_location , book2_location, in_handBook1, in_handBook2]
					run_and_write(scenario, initial_conditions_index)

if __name__ == "__main__":
	'''
	The main function, creates two results file. 
	1. experimental_results.csv: A csv that contains the time taken for the reason during running for traditional planing vs theory of intentions . 
	2. experimental_results.txt: A detailed text file that contains initial state, plans and history. 
	'''
	reRuns = 0
	scenarios = [1,2,3,4,5,6,'random']
	numberRunsRandomScenario = 1000

	for s in scenarios:
		runCount = 0
		results_file_name = "scenario_" + str(s)
		textfile = open(results_file_name+'.txt', 'w')
		csvfile = open(results_file_name+'.csv', 'w')
		writer = csv.writer(csvfile)
		writer.writerow(['Index', ' Total Time Trad', ' Total Time ToI' , ' Number Plans Trad', ' Number Plans ToI', ' Time Planning Trad', ' Time Planning ToI',' Exec TU Trad', ' Exec TU ToI', ' Steps Executed Trad', ' Steps Executed ToI', ' Achieved Goal Trad', ' Achieved Goal ToI', ' Goal Correction Trad', ' Goal Correction ToI', ' Exo-action', ' Scenario Recreated', ' Same Exo-action', ' Actions Before Replanning'])


		createConditionsAndRun(s)
		if(s == 'random'): 
			for x in range (1,numberRunsRandomScenario): createConditionsAndRun(s)
			

			

