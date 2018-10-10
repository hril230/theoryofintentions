
from datetime import datetime

from simulation.realWorld import World
#from controllerTraditionalPlanning import ControllerTraditionalPlanning
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
import csv
ASP_subfolder_path = 'simulation/'
results_file_name = "simulation/results/"
sparc_path = "$HOME/work/solverfiles/sparc.jar"
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




def runAndWrite(initial_conditions_index, trial_number, goal, initial_state):
	global timeTakenZooming
	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	my_world = World(sparc_path,initial_state,domain_info)
	executer = Executer(my_world)

	known_world = my_world.getCoarseState()
	initial_conditions = list(domain_info.coarseStateToAstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	results = open('experimental_results.txt', 'a')
	results.write('\n\nTrial number: ' + str(trial_number))
	if global_variables.complexity_level == 1: results.write('\nInitial state: [rob1_loc, book1_loc, book1_in_hand, refined_object_parts_in_hand] = ')
	elif global_variables.complexity_level == 2: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book1_in_hand, book2_in_hand, refined_object_parts_in_hand] = ')
	elif global_variables.complexity_level == 3: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book3_loc, book1_in_hand, book2_in_hand, book3_in_hand, refined_object_parts_in_hand] = ')
	elif global_variables.complexity_level == 4: results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book3_loc, book4_loc, book1_in_hand, book2_in_hand, book3_in_hand, book4_in_hand, refined_object_parts_in_hand] = ')
	results.write(str(initial_state))
	results.write('\nGoal: ')
	results.write(goal)
	results.close()

	controllerToI = ControllerToI(sparc_path, ASP_subfolder_path, domain_info, executer, robot_refined_location, initial_conditions, goal, max_plan_length)
	#controllerToI.run()
	p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=())
	p1.start()
	start = datetime.now()
	p1.join(maxTimeZooming)
	if p1.is_alive():
		results = open('experimental_results.txt', 'a')
		results.write('\nTIMEOUT: running the zooming controller took longer than 5 minutes')
		results.close()
		timeTakenZooming = 0
		p1.terminate()
		p1.join()

	else:
		endTime = datetime.now()
		timeTakenZooming = (endTime - start).seconds
		results = open('experimental_results.txt', 'a')
		results.write('\nTotal trial time with zooming: ' + str(timeTakenZooming) + ' seconds.')
		results.close()

	print('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n')
	print '$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$      time taken zooming: ' + str(timeTakenZooming) + ' seconds.'



def runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state):
	maxTimeNoZooming = 2*(timeTakenZooming)
	results = open('experimental_results.txt', 'a')
	results.write('\nTime limit for non-zooming: ' + str(maxTimeNoZooming) + ' seconds.')
	results.close()

	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	my_world = World(sparc_path,initial_state,domain_info)
	executer = Executer(my_world)

	known_world = my_world.getCoarseState()
	initial_conditions = list(domain_info.coarseStateToAstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	controllerToI = NonZoomingControllerToI(sparc_path, ASP_subfolder_path, domain_info, executer, robot_refined_location, initial_conditions , goal, max_plan_length)
	p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=())
	p1.start()
	start = datetime.now()
	p1.join(maxTimeNoZooming)
	if p1.is_alive():
		results = open('experimental_results.txt', 'a')
		results.write('\nTIMEOUT: running the non-zooming controller took longer than '+str(maxTimeNoZooming) + ' seconds.')
		results.close()
		p1.terminate()
		p1.join()
	else:
		endTime = datetime.now()
		timeTaken = (endTime - start).seconds
		results = open('experimental_results.txt', 'a')
		results.write('\nTotal trial time with no zooming: ' + str(timeTaken) + ' seconds.')
		results.close()





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


	goal = 'holds(loc(book3,kitchen),I).'
	initial_state = ['c20', 'c25', 'c17', 'c16', 'c1', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false', 'false']
	print ('\nComplexity level:')
	print (global_variables.complexity_level)
	print ('Goal:')
	print (goal)
	print ('Initial_state:')
	print (initial_state)
	runAndWrite(initial_conditions_index, trial_number, goal, initial_state)
	#runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state)



if __name__ == "__main__":
	global_variables.init()
	global_variables.complexity_level = 4 # TODO change this number to change the complexity level
	sys_random = random.SystemRandom()
	domain_info = DomainInfo(global_variables.complexity_level)
	number_runs = 1
	for x in range (0,number_runs): createConditionsAndRun(x+1)
