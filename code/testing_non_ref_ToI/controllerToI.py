#this controller assumes only one exogeneous action will occur during the run.
from datetime import datetime
import subprocess
from realWorld import World
from sets import Set

import re
import numpy
import random

goal = None  # given when initialised
maxPlanLength = None  #given when initialised
numberSteps = None
toi_history = None  #starting with initial conditions taken from world and adding occurrence
currentStep = None
numberActivities = None
inputForPlanning = None
believes_goal_holds = False


toi_goal_marker = '%% @_@_@'
toi_beginning_history_marker = '%% #_#_# beginning'
toi_end_history_marker = '%% #_#_# end'
toi_current_step_marker = '%% *_*_*'
preASP_toi_file = 'pre_ASP_files/preASP_ToI.txt'

asp_toi_file = 'ASP_files/ASP_ToI_Planning.sp'
preASP_toi_split = None
toi_beginning_history_index = None
toi_current_step_index = None

asp_toi_diagnosing_file = 'ASP_files/ASP_ToI_Diagnosis.sp'


executer = None
goal_correction = None


def controllerToI(thisPath,newGoal, maxPlanLen, new_executer, initialKnowledge):
	global executer
	global maxPlanLength
	global numberSteps
	global goal
	global numberActivities
	global currentStep
	global toi_history
	global inputForPlanning
	global goal_correction
	global currentDiagnosis
	global sparcPath
	global currentInitialStateAssumptions #this conditions are created by the ASP, and include the initialKnowledge. The value of some fluents may have to be assumed by choice.
	sparcPath = thisPath
	numberActivities = 1
	numberSteps = 4
	maxPlanLength = maxPlanLen
	executer = new_executer
	goal = newGoal
	goal_correction = 0
	currentDiagnosis = Set()
	inputForPlanning = []
	currentInitialStateAssumptions = Set()

	print 'initial knowledge: ' + str(initialKnowledge)
	preparePreASP_string_lists()
	toi_history = observations_to_obsList(initialKnowledge,0)
	toi_history.append("hpd(select(my_goal), true,0).")

   	currentStep = 1

	diagnose()
	print 'Initial conditions assumptions! ' + str(currentInitialStateAssumptions)

	finish = False
	while(finish == False):

		nextAction = runToIPlanning(inputForPlanning)
		print('\nNext action with ToI: ' +str(nextAction) +'  at step '+ str(currentStep))
		actionObservations = []
		if(nextAction == 'finish'):
			if(executer.getGoalFeedback() == True):
				toi_history.append('finish')
				finish = True
				break
			else:
				goal_correction += 1
				while(nextAction == 'finish'):
					toi_history.append('obs(my_goal,false,'+str(currentStep)+').')
					print('wrong assumption - goal not reached !!!!! ')
					diagnose()
					nextAction = runToIPlanning(inputForPlanning)

		if(nextAction == None):
	    		toi_history.append("Goal is futile")
			finish = True
			break


		toi_history.append('attempt('+nextAction+','+str(currentStep)+').')
		currentStep += 1
		if(nextAction[0:4] == 'stop'):
			numberActivities += 1
			numberSteps += 1
		elif(nextAction[0:5] == 'start'): pass
		else:
			actionObservations = executer.executeAction(nextAction)
			relevantObservations = actionObservations + executer.getTheseObservations(getIndexesRelevantToGoal())
			latest_observations = list(set(observations_to_obsList(relevantObservations,currentStep)))
			print 'Latest_observations: ' + ' '.join(latest_observations)
			toi_history = toi_history + latest_observations

		lastDiagnosis = currentDiagnosis
		lastInitialAssumptions = currentInitialStateAssumptions
		diagnose()
		if(lastInitialAssumptions != currentInitialStateAssumptions):
			print ('\nWrong Assumptions!')
			print('New assumption: ' + ' '.join(currentInitialStateAssumptions))
	if(currentDiagnosis): toi_history + list(currentDiagnosis)
	return (toi_history, numberActivities, goal_correction)




def getIndexesRelevantToGoal():
	return [World.LocationBook1_index, World.LocationBook2_index, World.In_handBook1_index, World.In_handBook2_index]

def runToIPlanning(input):
	global numberSteps
	global preASP_toi_split
	global toi_history
	global maxPlanLength
	global currentStep
	global believes_goal_holds
	nextAction = None
	print('running ToI planning ')


	current_asp_split = preASP_toi_split[:toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	current_asp_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
	current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
	current_asp_split[2] = "#const max_name = " + str(numberActivities) + "."
	asp = '\n'.join(current_asp_split)
        f1 = open(asp_toi_file, 'w')
	f1.write(asp)
	f1.close()

	answerSet = subprocess.check_output('java -jar '+sparcPath + ' ' + asp_toi_file +' -A ',shell=True)
	while( "intended_action" not in answerSet and "selected_goal_holds" not in answerSet and numberSteps < currentStep + maxPlanLength+3):
		current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
		asp = '\n'.join(current_asp_split)
        	f1 = open(asp_toi_file, 'w')
		f1.write(asp)
		f1.close()
		print('Looking for next action (ToI) - numberSteps ' + str(numberSteps))
		answerSet = subprocess.check_output('java -jar '+sparcPath + ' ' + asp_toi_file +' -A ',shell=True)
		numberSteps +=1

        possibleAnswers = answerSet.rstrip().split('\n\n')

	chosenAnswer = possibleAnswers[0]
	split_answer = chosenAnswer.strip('}').strip('{').split(', ')
	toi_history = []
	believes_goal_holds = False
	new_activity_name = None
	for line in split_answer:
		if("intended_action" in line):
			nextAction = line[16:line.rfind(',')]
			if 'start' in nextAction:
				new_activity_name = nextAction[nextAction.find('(')+1:nextAction.find(')')]
		elif("selected_goal_holds" in line):
			believes_goal_holds = True
		else:
			toi_history.append(line + '.')

	if new_activity_name:
		activity_info = []
		for line in split_answer:
			if("activity" in line and '('+new_activity_name+',' in line): activity_info.append(line)
		print '\nNew Activity: '+' '.join(activity_info)
	return nextAction



def diagnose():
	global inputForPlanning
	global currentDiagnosis
	global currentInitialStateAssumptions
	inputForPlanning = []
	possibleDiagnosis = []
	input = list(toi_history)
	input.append("explaining("+str(currentStep)+").")
	current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	current_asp_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
	current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
	current_asp_split[2] = "#const max_name = " + str(numberActivities) + "."
	asp = '\n'.join(current_asp_split)
        f1 = open(asp_toi_diagnosing_file, 'w')
	f1.write(asp)
	f1.close()

	# running only diagnosis
	answerSet = subprocess.check_output('java -jar '+sparcPath + ' ' + asp_toi_diagnosing_file +' -A ',shell=True)
	answers = answerSet.rstrip().split('\n\n')

	chosenAnswer = None
	for a in answers:
		if all([v in a for v in currentInitialStateAssumptions]) and all([v in a for v in currentDiagnosis]):  chosenAnswer = a
	if not chosenAnswer:
		for a in answers:
			if all([v in a for v in currentInitialStateAssumptions]): chosenAnswer = a
	if not chosenAnswer:
		for a in answers:
			if all([v in a for v in currentDiagnosis]): chosenAnswer = a
	if not chosenAnswer: chosenAnswer = answers[0]

	currentDiagnosis = Set()
	currentInitialStateAssumptions = Set()
 	split_diagnosis = chosenAnswer.strip('}').strip('{').split(', ')
	for line in split_diagnosis:
		if("unobserved" in line):
			newLine = line.replace("unobserved", "occurs") + '.'
			inputForPlanning.append(newLine)
			currentDiagnosis.add(line)
		elif("selected_goal_holds" in line): pass
		elif(line == ""): pass
		elif('assumed_initial_condition' in line):
			currentInitialStateAssumptions.add(line)
			inputForPlanning.append(line.replace('assumed_initial_condition','holds')[:-1] + ',0).')
		else:
			inputForPlanning.append(line + '.')
		inputForPlanning.append('planning('+str(currentStep)+').')
	print 'current diagnosis: ' + str(currentDiagnosis)
	return


def preparePreASP_string_lists():
	global toi_goal_marker
	global toi_beginning_history_marker
	global toi_current_step_marker
	global preASP_toi_file
	global goal

	global preASP_toi_split
	global toi_beginning_history_index
	global toi_current_step_index


	#preparing preASP_toi_split and toi_beginning_history_index
    	reader = open(preASP_toi_file, 'r')
    	preASP_toi = reader.read()
    	reader.close()
	preASP_toi_split = preASP_toi.split('\n')

   	index_goal = preASP_toi_split.index(toi_goal_marker)
    	preASP_toi_split.insert(index_goal+1,  "holds(my_goal,I) :- "+ goal)
    	toi_beginning_history_index = preASP_toi_split.index(toi_beginning_history_marker)
	toi_current_step_index = preASP_toi_split.index(toi_current_step_marker)


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
