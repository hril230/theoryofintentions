
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
toi_history = None  #sarting with initial conditions taken from world and adding occurrence
currentStep = None
numberActivities = None
inputForPlanning = None
believes_goal_holds = False
currentDiagnosis = None

toi_goal_marker = '%% @_@_@'
toi_beginning_history_marker = '%% #_#_# beginning'
toi_end_history_marker = '%% #_#_# end'
toi_current_step_marker = '%% *_*_*'
preASP_toi_file = 'preASP_ToI.txt'

asp_toi_file = 'ASP_ToI.sp'
preASP_toi_split = None
toi_beginning_history_index = None
toi_current_step_index = None

asp_toi_diagnosing_file = 'ASP_TOI_Diagnosis.sp'


executer = None
goal_correction = None


def controllerToI(thisPath,newGoal, maxPlanLen, new_executer):
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

	sparcPath = thisPath
	numberActivities = 1
	numberSteps = 4
	maxPlanLength = maxPlanLen
	executer = new_executer 
	goal = newGoal	
	goal_correction = 0	
	initialConditions = list(executer.getRealValues())
	currentDiagnosis = ''
	inputForPlanning = []

	preparePreASP_string_lists()	
	toi_history = observations_to_obsList(initialConditions,0)
    	toi_history.append("hpd(select(my_goal), true,0).")

   	currentStep = 1
	diagnose()
    	finish = False
	while(finish == False):
		nextAction = runToIPlanning(inputForPlanning)

		print(' $$$$$$$$$$$$$$$$$    next action with ToI : ' +str(nextAction) +'  at step '+ str(currentStep))

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

	
	
		actionObservations = []
		toi_history.append('attempt('+nextAction+','+str(currentStep)+').')

		if(nextAction[0:4] == 'stop'):
			numberActivities += 1 
			numberSteps += 1
		elif(nextAction[0:5] == 'start'): pass
		else:
			print('action : '+ str(nextAction))
			actionObservations = executer.executeAction(nextAction)
		currentStep += 1
		relevantObservations = actionObservations + executer.getTheseObservations(getIndexesRelevantToGoal())
		toi_history = toi_history + list(set(observations_to_obsList(relevantObservations,currentStep)))				
                diagnose()
	
	if(currentDiagnosis != ''): toi_history.append(currentDiagnosis)
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
	for line in split_answer:	
		if("intended_action" in line):      		
			nextAction = line[16:line.rfind(',')] 
		#elif("number_unobserved" in line): continue	
		elif("selected_goal_holds" in line): 
			believes_goal_holds = True
		else:
			toi_history.append(line + '.')	
	return nextAction



def diagnose():

	global inputForPlanning
	global currentDiagnosis

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


	if currentDiagnosis in answerSet:
		for a in answers:
			if(currentDiagnosis in a): chosenAnswer = a
	else:
		chosenAnswer = answers[0]

 	split_diagnosis = chosenAnswer.strip('}').strip('{').split(', ') 
	for line in split_diagnosis:
		if("number_unobserved" in line):
			newLine =line.replace("number_unobserved","explanation") 
			inputForPlanning.append(newLine + '.') 
		elif("unobserved" in line):
			newLine = line.replace("unobserved", "occurs") + '.'
			inputForPlanning.append(newLine)
			currentDiagnosis = line
		elif("selected_goal_holds" in line): pass
		elif(line == ""): pass
		else:
			inputForPlanning.append(line + '.')

			
	print('current diagnosis: '+str(currentDiagnosis))
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
	



