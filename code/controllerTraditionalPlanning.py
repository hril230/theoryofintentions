'''
This program represnets the Controller of the agent that uses traidtional planning.
It needs three arguments:
1. The goal to be achieved (in terms of the fluents of the domain description).
2. The length of the longest possible plan of the domain.
3. An instance of the Executer that will execute the plan and obtain observations from the world.

Once the controller has run the simmulation, it will return:
1. The history of his actions and observations (as he believes them).
2. All the plans that has created during the run.
3. A boolean specifiying if it has been rectified for having produced, during diagnosis, the false assumption that the goal has been reached when it was not.


The initial coniditions of the knowledge representation of the domain will be observed and given by the executer. In this simmulations, we create it so that the executer can observe the state of the whole domain, and therefore the knowledge of the initial conditions is alwasy complete and True.
'''


from datetime import datetime
import subprocess
from simulation.realWorld import World
import re
import random

goal = None  # given when initialised
maxPlanLength = None  #given when initialised
history = None  #sarting with initial conditions taken from world and adding occurrence
currentStep = None
belief = None

allPlans = None
currentPlan = None
possibleDiagnosis = []
goal_correction = None

planning_goal_marker = '%% @_@_@'
planning_history_marker = '%% #_#_#'
asp_planning_file = 'simulation/ASP_Traditional.sp'
preASP_planning_file = 'simulation/preASP_Traditional.txt'
preASP_planning_split = None
history_index_planning_asp = None


belief_history_marker = '%% *_*_*'
asp_belief_file = 'simulation/ASP_Believe.sp'
asp_diagnosing_file = 'simulation/ASP_Diagnosis.sp'
preASP_domain_file = 'simulation/preASP_Domain.txt'

preASP_belief_split = None
history_index_domain_asp = None
executer = None

def controllerTraditionalPlanning(thisPath,newGoal, maxPlanLen, new_executer):
	global belief
	global executer
	global maxPlanLength
	global goal
	global allPlans
	global currentPlan
	global possibleDiagnosis
	global currentStep
	global history
	global 	goal_correction
	global sparcPath

	allPlans = []
	currentPlan = []
	currentStep = 0
	goal_correction = 0
	maxPlanLength = maxPlanLen
	executer = new_executer
	goal = newGoal
	belief = ['unknown']*6
	sparcPath = thisPath

	initialConditions = list(executer.getRealValues())
	history = observations_to_obsList(initialConditions,0)

	preparePreASP_string_lists()
	needNewPlan = True
	while(needNewPlan == True):
		diagnose()
		updateBeliefWithDiagnosis()
		currentPlan = findPlan()
		needNewPlan = False
		if(len(currentPlan) > 1 and currentPlan[1] == "Goal holds" and executer.getGoalFeedback() == False):
			goal_correction += 1
			while(possibleDiagnosis and currentPlan[1] == "Goal holds"):
				updateBeliefWithDiagnosis()
				currentPlan = findPlan()

		if(currentPlan[0] == "No Plan"): break

		for action in currentPlan:
			print('Next action (traditional planning) : ' + str(action))
			relevantObservations = executer.executeAction(action)
			happened = updateBelief_fromAction(action,relevantObservations)
			if(happened == True):
				history.append('hpd('+action+','+str(currentStep)+').')
				currentStep += 1
				history = history + observations_to_obsList(relevantObservations, currentStep)
				possibleDiagnosis = []
			else:
				print('Inconsistent observations, action did not happen, need to call planner')
				history = history + observations_to_obsList(relevantObservations, currentStep)
				needNewPlan = True
				break
	print('%%%%%%%%%%%%%%  Finish Plan Traditional %%%%%%%%%%%%%%%% ')
	return (history, allPlans, goal_correction)


def diagnose():
	global possibleDiagnosis

	asp_diagnosing_split = preASP_belief_split[:history_index_domain_asp] + history + preASP_belief_split[history_index_domain_asp+1:]
	asp_diagnosing_split[0] = "#const numSteps = "+str(currentStep)+"."
	asp = '\n'.join(asp_diagnosing_split)
	f1 = open(asp_diagnosing_file, 'w')
	f1.write(asp)
	f1.close()
	output = subprocess.check_output('java -jar '+sparcPath + ' ' + asp_diagnosing_file +' -A',shell=True)
	output_split = output.rstrip().split('\n\n')
	possibleDiagnosis = [a.strip('}').strip('{').split(', ') for a in output_split]


def updateBeliefWithDiagnosis():
	chosenDiagnosis = possibleDiagnosis.pop()
	fluents = []
	for item in chosenDiagnosis:
		if('holds' in item): fluents.append(item)
		elif(item[0:4] == 'diag'):
			item = item.replace('diag', 'hpd')
			commaIndex = item.rfind(',')
			item = item[0:commaIndex+1] + 'true,'+item[commaIndex+1:]
	updateBelief_fromAnswer(', '.join(fluents))


def findPlan():
	global currentStep
	global currentPlan
	global preASP_planning_split
	global allPlans

	newPlan = []
	numSteps = 4

	asp_planning_split = preASP_planning_split[:history_index_planning_asp] + getBelief_as_obsList(0) + preASP_planning_split[history_index_planning_asp+1:]

	answerSet = '\n'
	while(answerSet == '\n' and numSteps < maxPlanLength):
		asp_planning_split[0] = '#const numSteps = '+ str(numSteps) + '.'
		asp_planning = '\n'.join(asp_planning_split)
        	f1 = open(asp_planning_file, 'w')
		f1.write(asp_planning)
		f1.close()
		print('Looking for next plan (Trad) - numberSteps ' + str(numSteps))
	        answerSet = subprocess.check_output('java -jar '+sparcPath + ' ' +asp_planning_file+' -A ',shell=True)
		numSteps +=1

	if(answerSet == "\n"):
		newPlan = ["No Plan","Futile Goal/Inconsistent"]
	elif("{}" in answerSet):
		newPlan = ["No Plan","Goal holds"]
	elif(answerSet == ""):
		newPlan = ["No Plan","Unable to process"]
	else:
		plans = answerSet.rstrip().split('\n\n')
    		plans = [plan for plan in plans if plan!='' and plan!='{}']
		#chosenPlan = random.choice(plans)
		chosenPlan = plans[0]
		plan_in_actions = chosenPlan.strip('}').strip('{').split(', ')
		splitActionsList = [action[:-1].split('),') for action in plan_in_actions]
		splitActionsList.sort(key=lambda x:int(x[1]))
 		plan_in_actions = ['),'.join(action)+ ')' for action in splitActionsList]
		for a in plan_in_actions:
			newPlan.append(a[7:a.rfind(',')])
	allPlans.append(newPlan)
	return newPlan


def updateBelief_fromAction(action, observations):
	global history_index_domain_asp
	global preASP_belief_split

	input = getBelief_as_obsList(0) + observations_to_obsList(observations,1) + ['hpd('+ action +',0).']
	asp_belief_split = preASP_belief_split[:history_index_domain_asp] + input + preASP_belief_split[history_index_domain_asp+1:]
	asp = '\n'.join(asp_belief_split)
	f1 = open(asp_belief_file, 'w')
	f1.write(asp)
	f1.close()
	print('Next, checking belief-observations consistency ')
	output = subprocess.check_output('java -jar '+ sparcPath + ' ' + asp_belief_file +' -A',shell=True)
	output = output.strip('}').strip('{')
	if 'holds' in output:
		updateBelief_fromAnswer(output)
		return True
	else: return False


def preparePreASP_string_lists():
    	global preASP_planning_split
    	global preASP_belief_split
	global history_index_domain_asp
	global history_index_planning_asp
	global goal

	#preparing preASP_planning_split and history_index_planning_asp
    	reader = open(preASP_planning_file, 'r')
    	preASP_planning = reader.read()
    	reader.close()
	preASP_planning_split = preASP_planning.split('\n')
	preASP_planning_split[0] = "#const numSteps = "+ str(maxPlanLength)+"."

   	index_goal = preASP_planning_split.index(planning_goal_marker)
    	goal_formatted = "goal(I) :- "+ goal
    	preASP_planning_split.insert(index_goal+1,  goal_formatted)
    	history_index_planning_asp = preASP_planning_split.index(planning_history_marker)

	#preparing preASP_belief_split and history_index_domain_asp
    	reader = open(preASP_domain_file, 'r')
    	preASP_belief = reader.read()
    	reader.close()
	preASP_belief_split = preASP_belief.split('\n')
    	history_index_domain_asp = preASP_belief_split.index(belief_history_marker)


def updateBelief_fromAnswer(answer):
	global belief
	belief = ['unknown'] * 6
	for holds in answer.split(', '):
		if holds[0] == '-':
			fluent = holds[7:holds.rfind(',')]
			if(fluent[0:8] == 'in_hand('):
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'book1'): belief[World.In_handBook1_index] = 'false'
				if(split_fluent[1] == 'book2'): belief[World.In_handBook2_index] = 'false'
			if(fluent[0:7] == 'locked('):
				belief[World.LibraryLocked_index] = 'false'

		else:
			fluent = holds[6:holds.rfind(',')]
			if(fluent[0:7] == 'locked('):
				belief[World.LibraryLocked_index] = 'true'
			elif(fluent[0:4] == 'loc('):
				fluent = fluent[4:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[0] == 'rob1'): belief[World.LocationRobot_index] = split_fluent[1]
				elif(split_fluent[0] == 'book1'): belief[World.LocationBook1_index] = split_fluent[1]
				elif(split_fluent[0] == 'book2'): belief[World.LocationBook2_index] = split_fluent[1]
			elif(fluent[0:8] == 'in_hand('):
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'book1'): belief[World.In_handBook1_index] = 'true'
				if(split_fluent[1] == 'book2'): belief[World.In_handBook2_index] = 'true'

def getBelief(index):
	return belief[index]


def getBelief_as_obsList(step):
	obsList = []
	global belief
	if(belief[World.LibraryLocked_index] != 'unknown'): obsList.append('obs(locked(library),'+ belief[World.LibraryLocked_index] + ','+str(step)+').')
	if(belief[World.LocationRobot_index] != 'unknown'): obsList.append('obs(loc(rob1,' +str(belief[World.LocationRobot_index])+ '),true,'+str(step)+').')
	if(belief[World.LocationBook1_index] != 'unknown'): obsList.append('obs(loc(book1,' +str(belief[World.LocationBook1_index])+ '),true,'+str(step)+').')
	if(belief[World.LocationBook2_index] != 'unknown'): obsList.append('obs(loc(book2,' +str(belief[World.LocationBook2_index])+ '),true,'+str(step)+').')
	if(belief[World.In_handBook1_index] != 'unknown'): obsList.append('obs(in_hand(rob1,book1),' + belief[World.In_handBook1_index]+ ','+str(step)+').')
	if(belief[World.In_handBook2_index] != 'unknown'): obsList.append('obs(in_hand(rob1,book2),' + belief[World.In_handBook2_index]+ ','+str(step)+').')
	return obsList


def observations_to_obsList(observations, step):
	obsList = []
	for observation in observations:
		if (observation[0] == World.LibraryLocked_index and observation[1] != 'unknown'):
			obsList.append('obs(locked(library),'+ observation[1] + ',' + str(step) +').')
		if (observation[0] == World.LocationRobot_index and observation[1] != 'unknown'):
			obsList.append('obs(loc(rob1,'+str(observation[1])+ '),true,'+ str(step) +').')
		if (observation[0] == World.LocationBook1_index):
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(book1,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(book1,' +str(executer.getRobotLocation())+ '),false,'+ str(step) +').')
		if (observation[0] == World.LocationBook2_index):
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(book2,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(book2,' +str(executer.getRobotLocation())+ '),false,'+ str(step) +').')
		if (observation[0] == World.In_handBook1_index and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book1),' + observation[1]+ ','+ str(step) +').')
		if (observation[0] == World.In_handBook2_index and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book2),' + observation[1]+ ','+ str(step) +').')
	return obsList
