
from datetime import datetime

from simulation.realWorld import World
#from controllerTraditionalPlanning import ControllerTraditionalPlanning
from controllerToI import ControllerToI
from simulation.executer import Executer
import subprocess
import  csv
import random
import multiprocessing
import time
import global_variables
from simulation.domain_info import DomainInfo
import sys
import csv
import psutil
from decimal import getcontext, Decimal


global trialCount

max_plan_length = 17
maxTimeZooming = 300
timeTakenZooming = 0
def refine_goal_location(book_goal_loc, refined_location_possibilities):
	if book_goal_loc == 'library':
		book_loc_start_index = 0
		book_loc_end_index = global_variables.complexity_level+1
	elif book_goal_loc == 'kitchen':
		book_loc_start_index = global_variables.complexity_level+1
		book_loc_end_index = (global_variables.complexity_level+1)*2
	elif book_goal_loc == 'office1':
		book_loc_start_index = (global_variables.complexity_level+1)*2
		book_loc_end_index = (global_variables.complexity_level+1)*3
	elif book_goal_loc == 'office2':
		book_loc_start_index = (global_variables.complexity_level+1)*3
		book_loc_end_index = (global_variables.complexity_level+1)*4
	else:
		book_loc_start_index = (global_variables.complexity_level+1)*4
		book_loc_end_index = (global_variables.complexity_level+1)*5
	return (refined_location_possibilities[book_loc_start_index:book_loc_end_index])


def runAndWrite(initial_conditions_index, goal, initial_state):
	global_variables.controller_type = 'zooming'
	global timeTakenZooming
	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	my_world = World(initial_state,domain_info)
	executer = Executer(my_world)
	known_world = my_world.getCoarseState()
	print('my_world.getCoarseState() ')
	print(known_world)

	initial_conditions = list(domain_info.coarseStateToAbstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
	results.write('\n\nTrial Number: ' + str(trialCount))
	if global_variables.complexity_level == 1: results.write('\nInitial state: [rob1_loc, book1_loc, book1_in_hand, refined_object_parts_in_hand] = \n ')
	elif global_variables.complexity_level == 2: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book1_in_hand, book2_in_hand, refined_object_parts_in_hand] = \n ')
	elif global_variables.complexity_level == 3: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book3_loc, book1_in_hand, book2_in_hand, book3_in_hand, refined_object_parts_in_hand] = \n')
	elif global_variables.complexity_level == 4: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book3_loc, book4_loc, book1_in_hand, book2_in_hand, book3_in_hand, book4_in_hand, refined_object_parts_in_hand] = \n')
	results.write(str(initial_state))
	results.write('\nGoal: ')
	results.write(goal)
	results.close()
	controllerToI = ControllerToI( domain_info, executer, robot_refined_location, initial_conditions, goal, max_plan_length)
	error = multiprocessing.Event()
	planningTime = multiprocessing.Value('d', 0)
	numAbsPlans = multiprocessing.Value('i', 0)
	numRefPlans = multiprocessing.Value('i', 0)
	numAbsAct = multiprocessing.Value('i', 0)
	numRefAct = multiprocessing.Value('i', 0)
	completeRun = multiprocessing.Value('c','_')
	p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=(error,planningTime, numAbsPlans, numRefPlans, numAbsAct, numRefAct,completeRun))
	timeTakenZooming = 0
	timeout = False
	p1.start()
	startTime = datetime.now()
  	while True:
		if error.is_set():
			timeTakenZooming = (datetime.now()-startTime).total_seconds()
			p1.terminate()
			p1.join()
			print(' !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
			results.write('\n' + global_variables.controller_type + ' ERROR FOUND')
			results.close()
			break
		'''
		if int((datetime.now()-startTime).total_seconds()) > maxTimeZooming and p1.is_alive():
			timeTakenZooming = (datetime.now()-startTime).total_seconds()
			completeRun.value = global_variables.character_code_timeout
			p1.terminate()
			p1.join()
			try:
				subprocess.check_output('killall clingo', shell=True)
			except subprocess.CalledProcessError:
				print('Clingo was not running.')
			print(' !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   TIMEOUT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
			results.write('\n' + global_variables.controller_type + ' TIMEOUT REACHED')
			results.close()
			break
		'''
		if not p1.is_alive(): break
	timeTakenZooming = (datetime.now()-startTime).total_seconds()
	print '$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$      time taken zooming: ' + str(timeTakenZooming) + ' seconds.'
	results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
	results.write('\nTotal trial time with zooming: ' + str(timeTakenZooming) + ' seconds.')
	results.close()
	return [str("{0:.2f}".format(timeTakenZooming)),str("{0:.2f}".format(planningTime.value)),str(numAbsPlans.value),str(numRefPlans.value),str(numAbsAct.value),str(numRefAct.value),str(completeRun.value)]



def runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state):
	global_variables.controller_type = 'non_zooming'
	maxTimeNonZooming = 4*(timeTakenZooming)
	results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
	results.write('\nTime limit for non-zooming: ' + str(maxTimeNonZooming) + ' seconds.')
	results.close()
	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	my_world = World(initial_state,domain_info)
	executer = Executer(my_world)
	known_world = my_world.getCoarseState()
	initial_conditions = list(domain_info.coarseStateToAbstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	controllerToI = ControllerToI( domain_info, executer, robot_refined_location, initial_conditions , goal, max_plan_length)
	error = multiprocessing.Event()
	planningTime = multiprocessing.Value('d', 0)
	numAbsPlans = multiprocessing.Value('i', 0)
	numRefPlans = multiprocessing.Value('i', 0)
	numAbsAct = multiprocessing.Value('i', 0)
	numRefAct = multiprocessing.Value('i', 0)
	completeRun = multiprocessing.Value('c','_')
	p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=(error,planningTime, numAbsPlans, numRefPlans, numAbsAct, numRefAct,completeRun))
	timeTakenNonZooming = 0
	p1.start()
	startTime = datetime.now()
	timeout = False
  	while True:
		if error.is_set():
			p1.terminate()
			p1.join()
			print(' !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
			results.write('\n' + global_variables.controller_type + ' ERROR FOUND')
			results.close()
			break
		'''
		if int((datetime.now()-startTime).total_seconds()) > maxTimeNonZooming and p1.is_alive():
			timeout = True
			p1.terminate()
			p1.join()
			completeRun.value = global_variables.character_code_timeout
			try:
				subprocess.check_output('killall clingo', shell=True)
			except subprocess.CalledProcessError:
				print('Clingo was not running.')
			print(' !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   TIMEOUT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
			results.write('\n' + global_variables.controller_type + ' TIMEOUT REACHED')
			results.close()
			break
		'''
		if not p1.is_alive(): break

	timeTakenNonZooming = (datetime.now()-startTime).total_seconds()
	print '$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$      time taken non zooming: ' + str(timeTakenNonZooming) + ' seconds.'
	results = open('experimental_results_' + str(global_variables.complexity_level) + '.txt', 'a')
	results.write('\nTotal trial time with non zooming: ' + str(timeTakenNonZooming) + ' seconds.')
	results.close()
	return [str("{0:.2f}".format(timeTakenNonZooming)),str("{0:.2f}".format(planningTime.value)),str(numAbsPlans.value),str(numRefPlans.value),str(numAbsAct.value),str(numRefAct.value),str(completeRun.value)]


def createPreASP_AbstractBeliefFile():
	sorts_marker = '%% SORTS GO HERE'
	attributes_marker = '%% ATTRIBUTES GO HERE'
	reader = open(global_variables.file_name_preASP_abstract_domain, 'r')
	my_text = reader.read()
	reader.close()
	my_text_split = my_text.split('\n')
	sorts_index = my_text_split.index(sorts_marker)
	my_text_split[sorts_index] = global_variables.abstract_locations_string[global_variables.complexity_level-1]+'\n'+global_variables.abstract_objects_string[global_variables.complexity_level-1]
	attributes_index = my_text_split.index(attributes_marker)
	my_text_split[attributes_index] = global_variables.abstract_attributes_string[global_variables.complexity_level-1]
	writer = open(global_variables.file_name_preASP_abstract_belief, 'w')
	writer.write('\n'.join(my_text_split))
	writer.close()

def createPreASP_ToIPlanningFile():
	sorts_marker = '%% SORTS GO HERE'
	attributes_marker = '%% ATTRIBUTES GO HERE'
	reader = open(global_variables.file_name_preASP_ToI_domain, 'r')
	my_text = reader.read()
	reader.close()
	my_text_split = my_text.split('\n')
	my_text_split[0] = '#const numSteps = ' + str(global_variables.max_number_steps_ToI_planning[global_variables.complexity_level-1]) +'.'
	my_text_split[1] = '#const max_len = ' + str(global_variables.max_number_steps_ToI_planning[global_variables.complexity_level-1]+1) +'.'
	sorts_index = my_text_split.index(sorts_marker)
	my_text_split[sorts_index] = global_variables.abstract_locations_string[global_variables.complexity_level-1]+'\n'+global_variables.abstract_objects_string[global_variables.complexity_level-1]
	attributes_index = my_text_split.index(attributes_marker)
	my_text_split[attributes_index] = global_variables.abstract_attributes_string[global_variables.complexity_level-1]
	writer = open(global_variables.file_name_preASP_ToI_planning, 'w')
	writer.write('\n'.join(my_text_split))
	writer.close()

def createPreASP_RefinedWorldFile():
	sorts_marker = '%% SORTS GO HERE'
	attributes_marker = '%% ATTRIBUTES GO HERE'
	causal_law_marker = '%% CAUSAL LAWS GO HERE'
	reader = open(global_variables.file_name_preASP_refined_domain, 'r')
	my_text = reader.read()
	reader.close()
	my_text_split = my_text.split('\n')
	sorts_index = my_text_split.index(sorts_marker)
	my_text_split[sorts_index] = global_variables.refined_sorts_string[global_variables.complexity_level-1] + global_variables.sort_of_sorts_string
	attributes_index = my_text_split.index(attributes_marker)
	my_text_split[attributes_index] = global_variables.refined_attributes_string[global_variables.complexity_level-1]
	causal_law_index = my_text_split.index(causal_law_marker)
	my_text_split[causal_law_index] = global_variables.refined_world_causal_law
	exec_condition_index = my_text_split.index(global_variables.old_refined_world_executability_condition)
	my_text_split[exec_condition_index] = global_variables.new_refined_world_executability_condition
	my_text_split.append(global_variables.refined_world_display_string)
	writer = open(global_variables.file_name_preASP_refined_world, 'w')
	writer.write('\n'.join(my_text_split))
	writer.close()

def createPreASP_InferringIndirectObservationsFile():
	sorts_marker = '%% SORTS GO HERE'
	attributes_marker = '%% ATTRIBUTES GO HERE'
	reader = open(global_variables.file_name_preASP_refined_domain, 'r')
	my_text = reader.read()
	reader.close()
	my_text_split = my_text.split('\n')
	#sorts_index = my_text_split.index(sorts_marker)
	#my_text_split[sorts_index] = global_variables.refined_sorts_string[global_variables.complexity_level-1]+global_variables.sort_of_sorts_string
	#attributes_index = my_text_split.index(attributes_marker)
	#my_text_split[attributes_index] = global_variables.refined_attributes_string[global_variables.complexity_level-1]
	my_text_split.append(global_variables.inferring_indirect_observations_display_string)
	#removing exo actions
	for i, line in enumerate(my_text_split):
		if('rob_action' in line): my_text_split[i] = line.replace('rob_action','action')
		if('exo' in line): my_text_split[i] = '\n'
	writer = open(global_variables.file_name_preASP_inferring_indirect_observations, 'w')
	writer.write('\n'.join(my_text_split))
	writer.close()

def createPreASP_RefinedPlanningFile():
	sorts_marker = '%% SORTS GO HERE'
	attributes_marker = '%% ATTRIBUTES GO HERE'
	planning_marker = '%% PLANNING RULES GO HERE'
	testing_marker = '%% TESTING RULES GO HERE'
	reader = open(global_variables.file_name_preASP_refined_domain, 'r')
	my_text = reader.read()
	reader.close()
	my_text_split = my_text.split('\n')
	my_text_split[0] = '#const numSteps = ' + str(global_variables.number_steps_refined_planning[global_variables.complexity_level-1]) +'.'
	sorts_index = my_text_split.index(sorts_marker)
	my_text_split[sorts_index] = global_variables.refined_sorts_string[global_variables.complexity_level-1]+global_variables.sort_of_sorts_string
	attributes_index = my_text_split.index(attributes_marker)
	my_text_split[attributes_index] = global_variables.refined_attributes_string[global_variables.complexity_level-1]
	planning_index = my_text_split.index(planning_marker)
	my_text_split[planning_index] = global_variables.planning_rules_string
	testing_index = my_text_split.index(testing_marker)
	my_text_split[testing_index] = global_variables.testing_rules_string
	my_text_split.append('occurs.')
	my_text = '\n'.join(my_text_split)
	#removing exo actions
	for i, line in enumerate(my_text_split):
		if('rob_action' in line): my_text_split[i] = line.replace('rob_action','action')
		if('exo' in line): my_text_split[i] = '\n'
	writer = open(global_variables.file_name_preASP_refined_planning, 'w')
	writer.write('\n'.join(my_text_split))
	writer.close()

def createConditionsAndRun():
	# set the domain to match the complexity number
	total_refined_location_possibilities = ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'c13', 'c14', 'c15', 'c16', 'c17', 'c18', 'c19', 'c20', 'c21', 'c22', 'c23', 'c24', 'c25']
	location_possibilities_index = (global_variables.complexity_level+1)**2
	refined_location_possibilities = total_refined_location_possibilities[0:location_possibilities_index]
	total_book_choice_possibilities = ['book1', 'book2', 'book3', 'book4']
	book_choice_possibilities = total_book_choice_possibilities[0:global_variables.complexity_level]
	total_book_loc_possibilities = ['library', 'kitchen', 'office1', 'office2', 'storage_cupboard']
	book_loc_possibilities = total_book_loc_possibilities[0:global_variables.complexity_level+1]

	initial_conditions_index = 0
	controlled_run = False

	# set initial conditions
	repeat = True # repeat until initial conditions and a goal have been chosen that means planning has to be done to achieve the goal
	while (repeat):
		#in_hand_possibilities = ['false', 'true']
		in_hand_possibilities = ['false'] # experiments are set up so the robot is not holding anything at the start
		robot_refined_location = sys_random.choice(refined_location_possibilities)
		book1_refined_location = sys_random.choice(refined_location_possibilities)
		book2_refined_location = sys_random.choice(refined_location_possibilities)
		book3_refined_location = sys_random.choice(refined_location_possibilities)
		book4_refined_location = sys_random.choice(refined_location_possibilities)
		if robot_refined_location == book1_refined_location:
			in_handBook1 = sys_random.choice(in_hand_possibilities)
		else: in_handBook1 = 'false'
		if (robot_refined_location == book2_refined_location) and (in_handBook1 == 'false'):
			in_handBook2 = sys_random.choice(in_hand_possibilities)
		else: in_handBook2 = 'false'
		if (robot_refined_location == book3_refined_location) and (in_handBook1 == 'false') and (in_handBook2 == 'false'):
			in_handBook3 = sys_random.choice(in_hand_possibilities)
		else: in_handBook3 = 'false'
		if (robot_refined_location == book4_refined_location) and (in_handBook1 == 'false') and (in_handBook2 == 'false') and (in_handBook3 == 'false'):
			in_handBook4 = sys_random.choice(in_hand_possibilities)
		else: in_handBook4 = 'false'
		if in_handBook1 == 'true':
			in_handBook1Ref1, in_handBook1Ref2, in_handBook1Ref3, in_handBook1Ref4 = 'true', 'false', 'false', 'false'
		else:
			in_handBook1Ref1, in_handBook1Ref2, in_handBook1Ref3, in_handBook1Ref4 = 'false', 'false', 'false', 'false'
		if in_handBook2 == 'true':
			in_handBook2Ref1, in_handBook2Ref2, in_handBook2Ref3, in_handBook2Ref4 = 'true', 'false', 'false', 'false'
		else:
			in_handBook2Ref1, in_handBook2Ref2, in_handBook2Ref3, in_handBook2Ref4 = 'false', 'false', 'false', 'false'
		if in_handBook3 == 'true':
			in_handBook3Ref1, in_handBook3Ref2, in_handBook3Ref3, in_handBook3Ref4 = 'true', 'false', 'false', 'false'
		else:
			in_handBook3Ref1, in_handBook3Ref2, in_handBook3Ref3, in_handBook3Ref4 = 'false', 'false', 'false', 'false'
		if in_handBook4 == 'true':
			in_handBook4Ref1, in_handBook4Ref2, in_handBook4Ref3, in_handBook4Ref4 = 'true', 'false', 'false', 'false'
		else:
			in_handBook4Ref1, in_handBook4Ref2, in_handBook4Ref3, in_handBook4Ref4 = 'false', 'false', 'false', 'false'

		# initial state includes the initial conditions that match the domain of the selected complexity level
		if global_variables.complexity_level == 1: initial_state = [robot_refined_location , book1_refined_location, in_handBook1, in_handBook1Ref1]
		elif global_variables.complexity_level == 2: initial_state = [robot_refined_location , book1_refined_location, book2_refined_location, in_handBook1, in_handBook2, in_handBook1Ref1, in_handBook1Ref2, in_handBook2Ref1, in_handBook2Ref2]
		elif global_variables.complexity_level == 3: initial_state = [robot_refined_location , book1_refined_location, book2_refined_location, book3_refined_location, in_handBook1, in_handBook2, in_handBook3, in_handBook1Ref1, in_handBook1Ref2, in_handBook1Ref3, in_handBook2Ref1, in_handBook2Ref2, in_handBook2Ref3, in_handBook3Ref1, in_handBook3Ref2, in_handBook3Ref3]
		elif global_variables.complexity_level == 4: initial_state = [robot_refined_location , book1_refined_location, book2_refined_location, book3_refined_location, book4_refined_location, in_handBook1, in_handBook2, in_handBook3, in_handBook4, in_handBook1Ref1, in_handBook1Ref2, in_handBook1Ref3, in_handBook1Ref4, in_handBook2Ref1, in_handBook2Ref2, in_handBook2Ref3, in_handBook2Ref4, in_handBook3Ref1, in_handBook3Ref2, in_handBook3Ref3, in_handBook3Ref4, in_handBook4Ref1, in_handBook4Ref2, in_handBook4Ref3, in_handBook4Ref4]

		# chose goal randomly - move one randomly chosen object to one randomly chosen location
		book_choice = sys_random.choice(book_choice_possibilities)
		if book_choice == 'book1': book_refined_location = book1_refined_location
		elif book_choice == 'book2': book_refined_location = book2_refined_location
		elif book_choice == 'book3': book_refined_location = book3_refined_location
		elif book_choice == 'book4': book_refined_location = book4_refined_location
		book_goal_loc = sys_random.choice(book_loc_possibilities)
		goal = 'holds(loc('+book_choice+','+book_goal_loc+'),I).'
		book_loc_refinements = refine_goal_location(book_goal_loc, refined_location_possibilities)
		# make sure the goal won't already be completed at start time
		count = 0
		while (book_refined_location in book_loc_refinements) and (count < 10):
			count = count + 1
			book_goal_loc = sys_random.choice(book_loc_possibilities)
			book_loc_refinements = refine_goal_location(book_goal_loc, refined_location_possibilities)
			goal = 'holds(loc('+book_choice+','+book_goal_loc+'),I).'
		# check that the goal choosing loop completed before it ran out of attempts - it not, need to choose new initial conditions
		if count < 10: repeat = False



	print ('\nComplexity level:')
	print (global_variables.complexity_level)
	print ('Goal:')
	print (goal)
	print ('Initial_state:')
	print (initial_state)
	resultsListZooming = runAndWrite(initial_conditions_index, goal, initial_state)
	resultsListNonZooming = runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state)
	resultsList = [trialCount, str(global_variables.complexity_level)] + resultsListZooming + resultsListNonZooming
	with open('experimental_results_'+ str(global_variables.complexity_level)+'.csv', 'a') as writeFile:
		writer = csv.writer(writeFile)
		writer.writerow(resultsList)
	writeFile.close()


def runGivenInitialState():
	global_variables.complexity_level = 4
	goal = 'holds(loc(book4,storage_cupboard),I).'
	initial_state = ['c13', 'c22', 'c9', 'c21', 'c14', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false']

	print (goal)
	print ('Initial_state:')
	print (initial_state)
	resultsListZooming = runAndWrite(0, goal, initial_state)
	resultsListNonZooming = runAndWriteWithoutZooming(0, goal, initial_state)
	resultsList = [trialCount, str(global_variables.complexity_level)] + resultsListZooming + resultsListNonZooming
	with open('experimental_results_'+ str(global_variables.complexity_level)+'.csv', 'a') as writeFile:
		writer = csv.writer(writeFile)
		writer.writerow(resultsList)
	writeFile.close()

if __name__ == "__main__":
	global trialCount
	singleRunTest = False

	for level in [4,3,2,1]:
		trialCount = 0
		global_variables.init()
		global_variables.complexity_level = level # TODO change this number to change the complexity level
		global_variables.file_name_preASP_ToI_planning = global_variables.ASP_subfolder +'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_ToI_planning.txt' # Used for astract planning and diagnosis with ToI
		global_variables.file_name_preASP_abstract_belief = global_variables.ASP_subfolder + 'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_abstract_belief.txt' # used for updating controller abstract belief
		global_variables.file_name_preASP_refined_planning = global_variables.ASP_subfolder + 'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_refined_planning.txt' # used for zoomed refined planning
		global_variables.file_name_preASP_inferring_indirect_observations = global_variables.ASP_subfolder+ 'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_inferring_indirect_observations.txt' # used for inferring coarse observations
		global_variables.file_name_preASP_refined_world = global_variables.ASP_subfolder + 'pre_ASP_files/complexity_level_'+str(global_variables.complexity_level) + '/preASP_refined_world.txt' # used for creating simulated world

		global_variables.file_name_preASP_abstract_domain = global_variables.ASP_subfolder +'pre_ASP_files/preASP_abstract_domain.txt'
		global_variables.file_name_preASP_ToI_domain = global_variables.ASP_subfolder +'pre_ASP_files/preASP_ToI_domain.txt'
		global_variables.file_name_preASP_refined_domain = global_variables.ASP_subfolder +'pre_ASP_files/preASP_refined_domain.txt'
		createPreASP_AbstractBeliefFile()
	 	createPreASP_RefinedWorldFile()
		createPreASP_InferringIndirectObservationsFile()
		createPreASP_RefinedPlanningFile()
		createPreASP_ToIPlanningFile()

		with open('experimental_results_'+ str(global_variables.complexity_level)+'.csv', 'a') as writeFile:
			writer = csv.writer(writeFile)
			writer.writerow(['Trial Number', 'Complexity Level', 'Run-time zooming', 'Planning Time - zooming', '# Abstract Plans - zooming', '# Refined Plans - zooming', '# Abstract Actions - zooming', '# Refined Actions - zooming', 'Complete Run - zooming', 'Run-time non_zooming', 'Planning Time non_zooming', '# Abstract Plans non_zooming', '# Refined Plans non_zooming', '# Abstract Actions non_zooming', '# Refined Actions non_zooming','Complete Run - non_zooming'])
		writeFile.close()


		sys_random = random.SystemRandom()
		domain_info = DomainInfo(global_variables.complexity_level)
		number_runs = 200
		if(singleRunTest): number_runs = 1
		for x in range (0,number_runs):
			trialCount += 1
			print 'Trial Number: ' + str(trialCount) + '\tComplexity Level: ' + str(global_variables.complexity_level)
			if(singleRunTest): runGivenInitialState()
			else: createConditionsAndRun()
