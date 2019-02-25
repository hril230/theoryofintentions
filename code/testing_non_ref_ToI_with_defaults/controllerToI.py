#this controller assumes only one exogeneous action will occur during the run.
from datetime import datetime
import subprocess
from realWorld import World
from sets import Set

import re
import numpy
import random

def controllerToI(new_domain_info_formatting, newGoal, maxPlanLen, new_executer, initialKnowledge):
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
	global currentAssumptions #this conditions are created by the ASP, and include the initialKnowledge. The value of some fluents may have to be assumed by choice.
	global currentDefaults
	global domain_info_formatting
	domain_info_formatting = new_domain_info_formatting
	numberActivities = 1
	numberSteps = 4
	maxPlanLength = maxPlanLen
	executer = new_executer
	goal = newGoal
	goal_correction = 0
	currentDiagnosis = Set()
	currentDefaults = Set()
	currentAssumptions = Set()

	print 'Initial knowledge: ' + str(initialKnowledge)
	preparePreASP_string_lists()
	toi_history = initialKnowledge
	toi_history.append("hpd(select(my_goal), true,0).")

   	currentStep = 1
	finish = False
	while(finish == False):
		defaultsOutput = checkingDefaults()
		inputForPlanning = diagnose(defaultsOutput)
		nextAction = planning(inputForPlanning)

		actionObservations = []
		if(nextAction == 'finish'):
			if check_goal_feedback():
				print '\nCheck goal feedback ' + str(check_goal_feedback())
				toi_history.append('finish')
				finish = True
				break
			else:
				goal_correction += 1
				while(nextAction == 'finish'):
					toi_history.append('obs(my_goal,false,'+str(currentStep)+').')
					print('Wrong assumption - goal not reached !!!!! ')
					defaultsOutput = checkingDefaults()
					inputForPlanning = diagnose(defaultsOutput)
					nextAction = planning(inputForPlanning)

		if(nextAction == None):
	    		toi_history.append("Goal is futile")
			finish = True
			break


		toi_history.append('attempt('+nextAction+','+str(currentStep)+').')
		if(nextAction[0:4] == 'stop'):
			numberActivities += 1
			numberSteps += 1
		elif(nextAction[0:5] == 'start'): pass
		else:
			step_obs = executer.observe(get_fluents_relevant_before_action(nextAction),currentStep)
			executer.executeAction(nextAction)
			step_obs.update(executer.observe(get_fluents_relevant_after_action(nextAction),currentStep+1))
			print '\nAction related observations: ' + ' '.join(step_obs)
			goal_obs = executer.observe(get_fluents_relevant_to_goal(),currentStep+1)
			print 'Goal related observations: ' + ' '.join(goal_obs)
			step_obs.update(goal_obs)
			toi_history = toi_history + [obs+'.' for obs in list(step_obs)]
		currentStep += 1

	if(currentDiagnosis): toi_history + list(currentDiagnosis)
	return (toi_history, numberActivities, goal_correction)

def check_goal_feedback():
	return executer.get_real_values(get_fluents_relevant_to_goal(),currentStep) == goal_as_current_obs_set()

def goal_as_current_obs_set():
	return Set([entry.replace('holds','obs').replace('I',str('-' not in entry).lower()+','+str(currentStep)).replace('-','') for entry in goal.rstrip(' .').split(', ')])

def planning(outputDefaultsAndDiagnosis):
	global numberSteps
	global preASP_toi_split
	global toi_history
	global believes_goal_holds
	nextAction = None
	input = outputDefaultsAndDiagnosis + toi_history +['planning('+str(currentStep)+').']
	current_asp_split = preASP_toi_split[:toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	asp = '\n'.join(current_asp_split)
	f1 = open(domain_info_formatting.asp_ToI_planning_file, 'w')
	f1.write(asp)
	f1.close()
	print '\n' + domain_info_formatting.asp_ToI_planning_file
	answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + domain_info_formatting.asp_ToI_planning_file +' -A ',shell=True)
	while( "intended_action" not in answerSet and "selected_goal_holds" not in answerSet and numberSteps < currentStep + maxPlanLength+3):
		current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
		asp = '\n'.join(current_asp_split)
        	f1 = open(domain_info_formatting.asp_ToI_planning_file, 'w')
		f1.write(asp)
		f1.close()
		('\n' + domain_info_formatting.asp_ToI_planning_file +'Increasing - numberSteps. ASP_ToI_Planning with ' + str(numberSteps) +' number of steps.')
		answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + domain_info_formatting.asp_ToI_planning_file +' -A ',shell=True)
		numberSteps +=1

        possibleAnswers = answerSet.rstrip().split('\n\n')

	chosenAnswer = possibleAnswers[0]
	split_answer = chosenAnswer.strip('}').strip('{').split(', ')
	believes_goal_holds = 'selected_goal_holds' in chosenAnswer
	new_activity_name = None
	for line in split_answer:
		if("intended_action" in line):
			nextAction = line[16:line.rfind(',')]
			if 'start' in nextAction:
				new_activity_name = nextAction[nextAction.find('(')+1:nextAction.find(')')]

	print('Next action with ToI: ' +str(nextAction) +'  at step '+ str(currentStep))
	if new_activity_name:
		activity_info = [line+'.' for line in split_answer if 'activity' in line and '('+new_activity_name+',' in line]
		toi_history = toi_history + activity_info
		print activity_info
	return nextAction



def get_fluents_relevant_after_action(action):
	if 'move' in action: return {action.replace('move','loc')}
	if 'pickup' in action: return {action.replace('pickup','in_hand')}
	if 'put_down' in action: return {action.replace('put_down','in_hand')}
	if 'unlock' in action: return {action.replace('unlock(rob1,','locked(')}


def get_fluents_relevant_before_action(action):
	if 'put_down' in action: return {action.replace('put_down','in_hand')}
	if 'unlock' in action: return {action.replace('unlock(rob1,','locked(')}

def get_fluents_relevant_to_goal():
	return Set([domain_info_formatting.get_fluent(entry) for entry in goal.split(', ') ])

def checkingDefaults():
	global currentDefaults
	preASP_toi_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	preASP_toi_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
	preASP_toi_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
	preASP_toi_split[2] = "#const max_name = " + str(numberActivities) + "."

	checkingDefaultsFlag = "finding_defaults("+str(currentStep)+")."
	checkingDefaultsDisplayLines = ['%%%%%%\ndisplay\n%%%%%%','defined_by_default.']
	current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + toi_history + [checkingDefaultsFlag] + checkingDefaultsDisplayLines
	f1 = open(domain_info_formatting.asp_ToI_defaults_file, 'w')
	f1.write('\n'.join(current_asp_split))
	f1.close()
	print ('\n' + domain_info_formatting.asp_ToI_defaults_file)
	answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + domain_info_formatting.asp_ToI_defaults_file +' -A ',shell=True)
	if answerSet == '' or '{}' in answerSet:
		print 'Current Defaults: '
		return []

	answers = answerSet.rstrip().split('\n\n')
	previousDefaults = currentDefaults
	currentDefaults = Set()

	for a in [answer for answer in answers if answer]:
		answer_defaults = Set(a.strip('{}\n').split(', '))
		if previousDefaults==answer_defaults:
			currentDefaults = answer_defaults
			break
		elif previousDefaults.issubset(answer_defaults):
			currentDefaults = answer_defaults
			break
	if not currentDefaults and answers[0]:
		currentDefaults = Set(answers[0].strip('{}\n').split(', '))
	print 'Current Defaults: ' + '. '.join(currentDefaults)
	return [e+'.' for e in currentDefaults]

def diagnose(defaultsOutput):
	global currentDiagnosis
	global currentAssumptions
	lastDiagnosis = currentDiagnosis
	lastInitialAssumptions = currentAssumptions
	diagnosingFlag = "diagnosing("+str(currentStep)+")."
	diagnosingDisplayLines = ['\n%%%%%%\ndisplay\n%%%%%%','unobserved.','assumed_initial_condition.']

	current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + defaultsOutput + toi_history +[diagnosingFlag] + diagnosingDisplayLines
	asp = '\n'.join(current_asp_split)
        f1 = open(domain_info_formatting.asp_ToI_diagnosing_file, 'w')
	f1.write(asp)
	f1.close()

	# running only diagnosis
	print ('\n' + domain_info_formatting.asp_ToI_diagnosing_file)
	answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + domain_info_formatting.asp_ToI_diagnosing_file +' -A ',shell=True)
	answers = answerSet.rstrip().split('\n\n')

	sorted_answers = []

	for a in answers:
		a = a.strip('{}').split(', ')
		sorted_tuple_a = (Set([v for v in a if 'assumed_initial_condition' in v]) , Set([v for v in a if 'unobserved' in v]))
		sorted_answers.append(sorted_tuple_a)

	chosenAnswer = None

	for sorted_a in sorted_answers:
		if currentAssumptions == sorted_a[0] and currentDiagnosis == sorted_a[1]: chosenAnswer = sorted_a
	if not chosenAnswer:
		for sorted_a in sorted_answers:
			if currentAssumptions == sorted_a[0] and currentDiagnosis.issubset(sorted_a[1]): chosenAnswer = sorted_a
	if not chosenAnswer:
		for a in answers:
			if currentAssumptions.issubset(sorted_a[0]) and currentDiagnosis == sorted_a[1]: chosenAnswer = sorted_a
	if not chosenAnswer:
		for a in answers:
			if currentAssumptions.issubset(sorted_a[0]) and currentDiagnosis.issubset(sorted_a[1]): chosenAnswer = sorted_a
	if not chosenAnswer:
		for a in answers:
			if currentAssumptions.issubset(sorted_a[0]): chosenAnswer = sorted_a
	if not chosenAnswer:
		for a in answers:
			if currentDiagnosis.issubset(sorted_a[1]): chosenAnswer = sorted_a
	if not chosenAnswer: chosenAnswer = sorted_answers[0]

	currentAssumptions = chosenAnswer[0]
	currentDiagnosis = chosenAnswer[1]

	print 'Current assumptions: ' + ', '.join(currentAssumptions)
	print 'Current diagnosis: ' + ', '.join(currentDiagnosis)

	diagnosisAndDefaultsOutput = defaultsOutput + [e + '.' for e in currentAssumptions|currentDiagnosis]
	return diagnosisAndDefaultsOutput


def preparePreASP_string_lists():
	global preASP_toi_split
	global toi_beginning_history_index
	global toi_current_step_index
	reader = open(domain_info_formatting.preASP_ToI_file, 'r')
	preASP_toi = reader.read()
	reader.close()
	preASP_toi_split = preASP_toi.split('\n')
	index_goal = preASP_toi_split.index(domain_info_formatting.goal_marker)
	preASP_toi_split.insert(index_goal+1,  "holds(my_goal,I) :- "+ goal)
	toi_beginning_history_index = preASP_toi_split.index(domain_info_formatting.history_marker)
	toi_current_step_index = preASP_toi_split.index(domain_info_formatting.current_step_marker)
