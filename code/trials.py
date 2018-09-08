
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
from simulation.domain_info import DomainInfo
ASP_subfolder_path = 'simulation/'
results_file_name = "simulation/results/"

sparc_path = "$HOME/work/solverfiles/sparc.jar"

max_plan_length = 17

textfile = None
csvfile = None
writer = None
initial_state = []

coarse_locations = DomainInfo.CoarseLocations
coarse_locations_as_cells = DomainInfo.CoarseLocationsAsCells
boolean = ['true', 'false']

sys_random = random.SystemRandom()


run_count = 0

def runAndWrite(initial_conditions_index, trial_number, goal, initial_state):

	startTime = datetime.now()

	print 'Trials: Initial_conditions_index: ' +str(initial_conditions_index)
	print 'Trials: World refined initial state: ' + str(initial_state)

	domain_info = DomainInfo()

	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	random_seed = run_count
	my_world = World(sparc_path,initial_state,random_seed,domain_info)
	executer = Executer(my_world)

	known_world = my_world.getCoarseState()
	initial_conditions = list(domain_info.coarseStateToAstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	print 'Trials: World coarse initial state: ' + str(initial_conditions)
	controllerToI = ControllerToI(sparc_path, ASP_subfolder_path, domain_info, executer, robot_refined_location, initial_conditions, goal, max_plan_length)

	p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=())
	p1.start()
	p1.join(600)
	if p1.is_alive():
		p1.terminate()
		p1.join()

	endTime = datetime.now()
	timeTaken = endTime - startTime

	results = open('experimental_results.txt', 'a')
	results.write('\n\nTrial number: ' + str(trial_number))
	results.write('\nInitial state: [rob1_loc, book1_loc, book2_loc, book1_in_hand, book2_in_hand] = ')
	results.write(str(initial_state))
	results.write('\nGoal: ')
	results.write(goal)
	results.write('\nPlan: ')
	results.write(controllerToI.abstract_action_plan)
	results.write('\nTime taken with zooming: ')
	if timeTaken.total_seconds() > 600: results.write('More than ten minutes')
	else: results.write(str(timeTaken))
	results.close()

	return timeTaken




def runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state):

	startTime = datetime.now()

	print 'Trials: Initial_conditions_index: ' +str(initial_conditions_index)
	print 'Trials: World refined initial state: ' + str(initial_state)

	domain_info = DomainInfo()

	indexes_relevant_goal = domain_info.getIndexesRelevantToGoal(goal)
	random_seed = run_count
	my_world = World(sparc_path,initial_state,random_seed,domain_info)
	executer = Executer(my_world)

	known_world = my_world.getCoarseState()
	initial_conditions = list(domain_info.coarseStateToAstractHoldsSet(known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()

	print 'Trials: World coarse initial state: ' + str(initial_conditions)
	controllerToI = NonZoomingControllerToI(sparc_path, ASP_subfolder_path, domain_info, executer, robot_refined_location, initial_conditions , goal, max_plan_length)

	p2 = multiprocessing.Process(target=controllerToI.run, name="Func", args=())
	p2.start()
	p2.join(600)
	if p2.is_alive():
		p2.terminate()
		p2.join()

	endTime = datetime.now()
	timeTaken = endTime - startTime

	results = open('experimental_results.txt', 'a')
	results.write('\nTime taken without zooming: ')
	if timeTaken.total_seconds() > 600: results.write('More than ten minutes')
	else: results.write(str(timeTaken))
	results.close()

	return timeTaken




def createConditionsAndRun(trial_number):
	initial_conditions_index = 0
	controlled_run = False

	# set initial conditions
        refined_location_possibilities = ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9']
        refined_in_hand_possibilities = ['false', 'true']
	robot_refined_location = sys_random.choice(refined_location_possibilities)
	book1_refined_location = sys_random.choice(refined_location_possibilities)
	book2_refined_location = sys_random.choice(refined_location_possibilities)
	#book3_refined_location = 'c5'
        if robot_refined_location == book1_refined_location:
            refined_in_handBook1 = sys_random.choice(refined_in_hand_possibilities)
	else: refined_in_handBook1 = 'false'
        if robot_refined_location == book2_refined_location:
            refined_in_handBook2 = sys_random.choice(refined_in_hand_possibilities)
	else: refined_in_handBook2 = 'false'
	#refined_in_handBook3 = 'false'
	refined_in_handBook1Ref1, refined_in_handBook1Ref2 = refined_in_handBook1, refined_in_handBook1
	refined_in_handBook2Ref1, refined_in_handBook2Ref2 = refined_in_handBook2, refined_in_handBook2
	#refined_in_handBook3Ref1, refined_in_handBook3Ref2, refined_in_handBook3Ref3 = 'false', 'false', 'false'
	initial_state = [robot_refined_location , book1_refined_location, book2_refined_location, refined_in_handBook1, refined_in_handBook2, refined_in_handBook1Ref1, refined_in_handBook1Ref2, refined_in_handBook2Ref1, refined_in_handBook2Ref2]

	# choose goal randomly
	book_loc_possibilities = ['library', 'kitchen', 'office1']
	book_in_hand_possibilities = ['holds', '-holds', '']
	book1_loc = sys_random.choice(book_loc_possibilities)
	book1_in_hand = sys_random.choice(book_in_hand_possibilities)
	book2_loc = sys_random.choice(book_loc_possibilities)
	book2_in_hand = sys_random.choice(book_in_hand_possibilities)
	goal = 'holds(loc(book1,'+book1_loc+'),I), holds(loc(book2,'+book2_loc+'),I)'
	if not book1_in_hand == '': goal = goal + ', '+book1_in_hand+'(in_hand(rob1,book1),I)'
	if book2_in_hand == '': goal = goal + '.'
	elif book1_in_hand == 'holds': goal = goal + ', -holds(in_hand(rob1,book2),I).'
	else: goal = goal + ', '+book2_in_hand+'(in_hand(rob1,book2),I).'
	print ('\nGoal:')
	print (goal)
	print ('Initial_state:')
	print (initial_state)

	# run experiment
	zoom_time = runAndWrite(initial_conditions_index, trial_number, goal, initial_state)
	non_zoom_time = runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state)

	zoom_seconds = zoom_time.total_seconds()
	non_zoom_seconds = non_zoom_time.total_seconds()
	results = open('experimental_results.txt', 'a')
	results.write('\nRatio: ')
	if zoom_seconds > 600: results.write('N/A - zooming timed out')
	elif non_zoom_seconds > 600: results.write('N/A - non zooming timed out')
	else: results.write(str(float(non_zoom_seconds)/float(zoom_seconds)))
	results.close()



if __name__ == "__main__":
	number_runs = 50
	for x in range (0,number_runs): createConditionsAndRun(x+1)
