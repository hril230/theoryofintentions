from datetime import datetime
import subprocess
from realWorld import World
from sets import Set

import re
import numpy
import random
import itertools
def controllerToI(new_domain_info_formatting, newGoal, new_executer, initial_knowledge):
	global executer
	global maxPlanLength
	global numberSteps
	global goal
	global numberActivities
	global currentStep
	global toi_history
	global toi_activities
	global inputForPlanning
	global goal_correction
	global current_diagnosis
	global current_defaults
	global domain_info_formatting
	global other_relevant_information
	global robot_name


	domain_info_formatting = new_domain_info_formatting
	robot_name = list(domain_info_formatting.get_all_constant_subsorts('#robot'))[0]
	numberActivities = 1
	numberSteps = 4
	executer = new_executer
	goal = newGoal
	goal_correction = 0
	current_diagnosis = Set()
	current_defaults = Set()
	defaults_history = []
	diagnosis_history = []
	other_relevant_information = {}
	toi_activities = []

	print 'Initial knowledge: ' + str(initial_knowledge)
	observed = initial_observation(initial_knowledge)
	print 'Initial observation: ' + str(observed)
	preparePreASP_string_lists()
	toi_history = Set(initial_knowledge + [v+'.' for v in observed])
	toi_history.add('hpd(select(my_goal), true,0).')

   	currentStep = 1
	other_relevant_information[currentStep] = []

	finish = False
	while(finish == False):
		defaultsOutput = checkingDefaults()
		inputForPlanning = diagnose(defaultsOutput)
		nextAction = planning(inputForPlanning)

		actionObservations = []
		if(nextAction == 'finish'):
			if check_goal_feedback():
				print '\nCheck goal feedback ' + str(check_goal_feedback())
				toi_history.add('attempt('+nextAction+','+str(currentStep)+').')
				finish = True
				break
			else:
				goal_correction += 1
				while(nextAction == 'finish'):
					toi_history.update(['obs(my_goal,false,' + str(n) + ').' for n in range(currentStep+1)])
					print('Wrong assumption - goal not reached !!!!! ')
					defaultsOutput = checkingDefaults()
					inputForPlanning = diagnose(defaultsOutput)
					nextAction = planning(inputForPlanning)

		if(nextAction == None):
#	    		toi_history.add('Goal is futile')
			finish = True
			break

		toi_history.add('attempt('+nextAction+','+str(currentStep)+').')
		if(nextAction[0:4] == 'stop'):
			numberActivities += 1
			numberSteps += 1
		elif(nextAction[0:5] == 'start'): pass
		else:
			step_obs = executer.observe(robot_name,get_fluents_relevant_before_action(nextAction),currentStep)
			executer.executeAction(nextAction)
			step_obs.update(executer.observe(robot_name,get_fluents_relevant_after_action(nextAction),currentStep+1))
			print 'Observations relevant to action: '
			print step_obs
			goal_obs = executer.observe(robot_name,get_fluents_relevant_to_goal(),currentStep+1)
			print 'Observations relevant to goal:'
			print goal_obs
			step_obs.update(goal_obs)
			toi_history.update([obs+'.' for obs in list(step_obs)])
		currentStep += 1
		other_relevant_information[currentStep] = []


	other_relevant_information_list	= []
	for step in range(currentStep):
		step +=1
		if other_relevant_information[step] != []:
			other_relevant_information_list.append('\nReasoning information from step ' + str(step) + ': ')
			other_relevant_information_list = other_relevant_information_list + other_relevant_information[step]


	return (toi_history, toi_activities, other_relevant_information_list, numberActivities, goal_correction)

def initial_observation(initial_knowledge):
	#the robot controller is going to request to observe all possible fluents and values combinations. Only those
	#fluents that can be observed will be returned by the executer.
	initial_observations_request = Set()
	all_fluents = domain_info_formatting.get_all_function_subsorts('#fluent')
	for a in all_fluents:
		paramters_a = domain_info_formatting.refined_sorts_hierarchy_dic[a]
		all_values_sets = [domain_info_formatting.get_all_constant_subsorts(p) for p in paramters_a]
		values_combinations = list(itertools.product(*all_values_sets))
		grounded_fluents = [a+'('+','.join(c)+')' for c in values_combinations ]
		initial_observations_request.update(grounded_fluents)
	return executer.observe(robot_name,initial_observations_request,0)

def check_goal_feedback():
	return executer.get_real_values(get_fluents_relevant_to_goal(),currentStep) == goal_as_current_obs_set()

def goal_as_current_obs_set():
	return Set([entry.replace('holds','obs').replace('I',str('-' not in entry).lower()+','+str(currentStep)).replace('-','') for entry in goal.rstrip(' .').split(', ')])

def planning(outputDefaultsAndDiagnosis):
	global numberSteps
	global preASP_toi_split
	global toi_history
	global believes_goal_holds
	global toi_activities
	nextAction = None
	input = outputDefaultsAndDiagnosis + get_toi_history_sorted_list() + toi_activities +['planning('+str(currentStep)+').']
	current_asp_split = preASP_toi_split[:toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	asp = '\n'.join(current_asp_split)
	asp_file = domain_info_formatting.asp_ToI_planning_file
	f1 = open(asp_file, 'w')
	f1.write(asp)
	f1.close()
	print '\n' + asp_file
	answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + asp_file +' -A ',shell=True)
	while( 'intended_action' not in answerSet and 'selected_goal_holds' not in answerSet and numberSteps < currentStep + domain_info_formatting.max_number_steps_ToI_planning+3):
		current_asp_split[0] = '#const n = '+str(numberSteps+1)+'. % maximum number of steps.'
		current_asp_split[1] = '#const max_len = '+str(numberSteps)+'. % maximum activity_length of an activity.'
		asp = '\n'.join(current_asp_split)
        	f1 = open(asp_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\n' + asp_file +' - Increasing numberSteps. ASP_ToI_Planning with ' + str(numberSteps) +' number of steps.')
		answerSet = subprocess.check_output('java -jar ' + domain_info_formatting.sparc_path + ' ' + asp_file +' -A ',shell=True)
		numberSteps +=1

        possibleAnswers = answerSet.rstrip().split('\n\n')

	chosenAnswer = possibleAnswers[0]
	split_answer = chosenAnswer.strip('}').strip('{').split(', ')
	believes_goal_holds = 'selected_goal_holds' in chosenAnswer
	new_activity_name = None
	for line in split_answer:
		if('intended_action' in line):
			nextAction = line[16:line.rfind(',')]
			if 'start' in nextAction:
				new_activity_name = nextAction[nextAction.find('(')+1:nextAction.find(')')]

	print('Next action with ToI: ' +str(nextAction) +'  at step '+ str(currentStep))
	if new_activity_name:
		new_activity_info = sort_activity_info([line+'.' for line in split_answer if 'activity' in line and '('+new_activity_name+',' in line])
		toi_activities = toi_activities + new_activity_info
		print 'New activity name: ' + new_activity_name
		print 'Activity actions: ' + ', '.join([v.strip('.') for v in new_activity_info if 'component' in v])
	#raw_input()
	return nextAction

def get_fluents_relevant_after_action(action):
	if 'move' in action: return {action.replace('move','loc')}
	if 'pickup' in action: return {action.replace('pickup','in_hand')}
	if 'put_down' in action: return {action.replace('put_down','in_hand')}
	if 'unlock' in action: return {action.replace('unlock('+robot_name+',','locked(')}

def get_fluents_relevant_before_action(action):
	if 'put_down' in action: return {action.replace('put_down','in_hand')}
	if 'unlock' in action: return {action.replace('unlock('+robot_name+',','locked(')}
	if 'move' in action and ('library' in action or 'kitchen' in action): return {action.replace('move','loc'),'locked(library)'}


def get_fluents_relevant_to_goal():
	return Set([domain_info_formatting.get_fluent(entry) for entry in goal.split(', ') ])

def checkingDefaults():
	global current_defaults
	global other_relevant_information
	preASP_toi_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	preASP_toi_split[0] = '#const n = '+str(numberSteps+1)+'. % maximum number of steps.'
	preASP_toi_split[1] = '#const max_len = '+str(numberSteps)+'. % maximum activity_length of an activity.'
	preASP_toi_split[2] = '#const max_name = ' + str(numberActivities) + '.'
	previous_defaults = current_defaults
	current_defaults = Set()
	checkingDefaultsFlag = 'finding_defaults('+str(currentStep)+').'
	checkingDefaultsDisplayLines = ['%%%%%%\ndisplay\n%%%%%%','defined_by_default.','ab_d1.','ab_d2.']
	current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + get_toi_history_sorted_list()  + toi_activities + [checkingDefaultsFlag] + checkingDefaultsDisplayLines
	asp_file = domain_info_formatting.asp_ToI_defaults_file
	f1 = open(asp_file, 'w')
	f1.write('\n'.join(current_asp_split))
	f1.close()
	print ('\n' + asp_file)
	answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + asp_file +' -A ',shell=True)
	if answerSet == '' or answerSet == '\n':
		diagnosingFlag = 'diagnosing('+str(currentStep)+').'
		current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + get_toi_history_sorted_list() + toi_activities + [checkingDefaultsFlag] +[diagnosingFlag]+ checkingDefaultsDisplayLines + ['unobserved.']
		asp_file = domain_info_formatting.asp_ToI_defaults_file
		f1 = open(asp_file, 'w')
		f1.write('\n'.join(current_asp_split))
		f1.close()
		print ('\nIncluding diagnosis and running: ' + asp_file)
		answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + asp_file +' -A ',shell=True)
		if answerSet == '' or answerSet == '\n':
			print '!!! Something went wrong in ' + asp_file
			raw_input()

	if '{}' in answerSet:
		if current_defaults != previous_defaults: print 'New Current Defaults: '
		return []

	answers = answerSet.rstrip().split('\n\n')

	for a in [answer for answer in answers if answer]:
		answer_defaults = Set([v for v in a.strip('{}\n').split(', ') if 'unobserved' not in v])
		if previous_defaults==answer_defaults:
			current_defaults = answer_defaults
			break
		elif previous_defaults.issubset(answer_defaults):
			current_defaults = answer_defaults
			break

	if not current_defaults and answers[0]:
		current_defaults = Set([v for v in answers[0].strip('{}\n').split(', ') if 'unobserved' not in v])

	if current_defaults != previous_defaults:
		other_relevant_information[currentStep] = other_relevant_information[currentStep] + list(current_defaults)
		print 'New Current Defaults: ' + '. '.join(current_defaults)
	#raw_input()
	return [e+'.' for e in current_defaults]

def get_toi_history_sorted_list():
	global toi_history
	historyInfoSplit = [entry.rstrip(').').split(',') for entry in toi_history]
	historyInfoSplit.sort(key=lambda x:x[0],reverse=True)
	historyInfoSplit.sort(key=lambda x:int(x[-1]))
	return [','.join(entry)+').' for entry in historyInfoSplit]

def sort_activity_info(new_activity_info):
	activity_components_split = [v.split(',') for v in new_activity_info if 'activity_component' in v]
	activity_components_split.sort(key=lambda x:int(x[1]))
	activity_components_sorted = [','.join(v) for v in activity_components_split]
	return [a for a in new_activity_info if not 'component' in a] + activity_components_sorted

def diagnose(defaultsOutput):
	global current_diagnosis
	global other_relevant_information
	last_diagnosis = current_diagnosis
	diagnosingFlag = 'diagnosing('+str(currentStep)+').'
	diagnosingDisplayLines = ['\n%%%%%%\ndisplay\n%%%%%%','unobserved.']

	current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + defaultsOutput + get_toi_history_sorted_list() + toi_activities +[diagnosingFlag]  + diagnosingDisplayLines
	asp = '\n'.join(current_asp_split)
	asp_file = domain_info_formatting.asp_ToI_diagnosing_file
	f1 = open(asp_file, 'w')
	f1.write(asp)
	f1.close()

	# running only diagnosis
	print ('\n' + asp_file)
	answerSet = subprocess.check_output('java -jar '+domain_info_formatting.sparc_path + ' ' + asp_file +' -A ',shell=True)
	if answerSet == '' or answerSet == '\n':
		print '!!! Something went wrong in ' + asp_file
		raw_input()
	answers = answerSet.rstrip().split('\n\n')

	chosenAnswer = None
	# the preferred choice is that in which we keep the current diagnosis
	for a in answers:
		a = a.strip('{}').split(', ')
		if all([d in a for d in current_diagnosis]):
			chosenAnswer = a
			break

	# if we still do not have an answer, then any answer would do.
	if not chosenAnswer: chosenAnswer = answers[0].strip('{}').split(', ')
	current_diagnosis = Set([v for v in chosenAnswer if 'unobserved' in v])



	if current_diagnosis != last_diagnosis:
		other_relevant_information[currentStep] = [v for v in other_relevant_information[currentStep] if all('unobserved' not in e for e in other_relevant_information[currentStep])]
		other_relevant_information[currentStep] = other_relevant_information[currentStep] + list(current_diagnosis)
		print 'Current diagnosis: ' + ', '.join(current_diagnosis)

	diagnosisAndDefaultsOutput = defaultsOutput + [e + '.' for e in current_diagnosis]
	#raw_input()
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
	preASP_toi_split.insert(index_goal+1,  'holds(my_goal,I) :- '+ goal)
	toi_beginning_history_index = preASP_toi_split.index(domain_info_formatting.history_marker)
	toi_current_step_index = preASP_toi_split.index(domain_info_formatting.current_step_marker)
