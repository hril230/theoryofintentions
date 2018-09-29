
from datetime import datetime
from simulation.realWorld import World
from controllerTraditionalPlanning import ControllerTraditionalPlanning
from controllerToI import ControllerToI
from nonZoomingControllerToI import NonZoomingControllerToI
from simulation.executer import Executer
import  csv
import random
import multiprocessing
import time
import global_variables
from simulation.domain_info import DomainInfo
import sys
ASP_subfolder_path = 'simulation/'
results_file_name = "simulation/results/"
sparc_path = "$HOME/work/solverfiles/sparc.jar"
max_plan_length = 17



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




def runAndWrite(initial_conditions_index, trial_number, goal, initial_state):

	#startTime = datetime.now()

	print 'Trials: Initial_conditions_index: ' +str(initial_conditions_index)
	print 'Trials: World refined initial state: ' + str(initial_state)

	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	my_world = World(sparc_path,initial_state,domain_info)
	executer = Executer(my_world)

	known_world = my_world.getCoarseState()
	initial_conditions = list(domain_info.coarseStateToAstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	print 'Trials: World coarse initial state: ' + str(initial_conditions)
	controllerToI = ControllerToI(sparc_path, ASP_subfolder_path, domain_info, executer, robot_refined_location, initial_conditions, goal, max_plan_length)

	#p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=())
	#p1.start()
	#p1.join(1200)
	#if p1.is_alive():
	#	p1.terminate()
	#	p1.join()
	controllerToI.run()

	#endTime = datetime.now()
	#timeTaken = endTime - startTime

	timeTaken = controllerToI.planning_times[0]
	for i in range(1,len(controllerToI.planning_times)): timeTaken = timeTaken + controllerToI.planning_times[i]
	if global_variables.error:
		sys.exit() # for testing purposes only
		timeTaken = 'ERROR: the output from the solver exceeds 100000000 characters.'
	else:
		seconds_taken = timeTaken.total_seconds()
		if not (seconds_taken < 1200): timeTaken = 'TIMEOUT: running the controller took longer than 20 minutes'

	results = open('experimental_results.txt', 'a')
	results.write('\n\nTrial number: ' + str(trial_number))
	if global_variables.complexity_level == 1: results.write('\nInitial state: [rob1_loc, book1_loc, book1_in_hand, refined_object_parts_in_hand] = ')
	elif global_variables.complexity_level == 2: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book1_in_hand, book2_in_hand, refined_object_parts_in_hand] = ')
	elif global_variables.complexity_level == 3: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book3_loc, book1_in_hand, book2_in_hand, book3_in_hand, refined_object_parts_in_hand] = ')
	elif global_variables.complexity_level == 4: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book3_loc, book4_loc, book1_in_hand, book2_in_hand, book3_in_hand, book4_in_hand, refined_object_parts_in_hand] = ')
	results.write(str(initial_state))
	results.write('\nGoal: ')
	results.write(goal)
	for line in controllerToI.lines_to_write: results.write(line)
	results.write('\nTotal time taken with zooming: ')
	results.write(str(timeTaken))
	results.close()

	return timeTaken




def runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state):

	#startTime = datetime.now()

	print 'Trials: Initial_conditions_index: ' +str(initial_conditions_index)
	print 'Trials: World refined initial state: ' + str(initial_state)

	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	my_world = World(sparc_path,initial_state,domain_info)
	executer = Executer(my_world)

	known_world = my_world.getCoarseState()
	initial_conditions = list(domain_info.coarseStateToAstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	print 'Trials: World coarse initial state: ' + str(initial_conditions)
	controllerToI = NonZoomingControllerToI(sparc_path, ASP_subfolder_path, domain_info, executer, robot_refined_location, initial_conditions , goal, max_plan_length)

	#p2 = multiprocessing.Process(target=controllerToI.run, name="Func", args=())
	#p2.start()
	#p2.join(1200)
	#if p2.is_alive():
	#	p2.terminate()
	#	p2.join()
	controllerToI.run()

	#endTime = datetime.now()
	#timeTaken = endTime - startTime
	timeTaken = controllerToI.planning_times[0]
	for i in range(1,len(controllerToI.planning_times)): timeTaken = timeTaken + controllerToI.planning_times[i]

	if global_variables.error: timeTaken = 'ERROR: the output from the solver exceeds 100000000 characters.'
	else:
		seconds_taken = timeTaken.total_seconds()
		if not (seconds_taken < 1200): timeTaken = 'TIMEOUT: running the non-zooming controller took longer than 20 minutes'

	results = open('experimental_results.txt', 'a')
	for line in controllerToI.lines_to_write: results.write(line)
	results.write('\nTotal time taken without zooming: ')
	results.write(str(timeTaken))
	results.close()

	return timeTaken




def createConditionsAndRun(trial_number):

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
	        in_hand_possibilities = ['false', 'true']
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

	# run experiment
	zoom_time = runAndWrite(initial_conditions_index, trial_number, goal, initial_state)
	non_zoom_time = runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state)



if __name__ == "__main__":
	global_variables.init()
	global_variables.complexity_level = 2 # TODO change this number to change the complexity level
	sys_random = random.SystemRandom()
	domain_info = DomainInfo(global_variables.complexity_level)
	number_runs = 50
	for x in range (0,number_runs): createConditionsAndRun(x+1)
