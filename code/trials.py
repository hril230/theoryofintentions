
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

	#p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=())
	#p1.start()
	#p1.join(1200)
	#if p1.is_alive():
	#	p1.terminate()
	#	p1.join()
	controllerToI.run()

	endTime = datetime.now()
	timeTaken = endTime - startTime
	if global_variables.error:
		sys.exit() # for testing
		#timeTaken = 'ERROR: the output from the solver exceeds 100000000 characters.'
	else:
		seconds_taken = timeTaken.total_seconds()
		if not (seconds_taken < 1200): timeTaken = 'TIMEOUT: running the controller took longer than 20 minutes'

	results = open('experimental_results.txt', 'a')
	results.write('\n\nTrial number: ' + str(trial_number))
	results.write('\nInitial state: [rob1_loc, book1_loc, book1_in_hand, refined_object_parts_in_hand] = ')
	results.write(str(initial_state))
	results.write('\nGoal: ')
	results.write(goal)
	results.write('\nPlan: ')
	results.write(controllerToI.abstract_action_plan)
	results.write('\nTime taken with zooming: ')
	results.write(str(timeTaken))
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
	p2.join(1200)
	if p2.is_alive():
		p2.terminate()
		p2.join()
	#controllerToI.run()

	endTime = datetime.now()
	timeTaken = endTime - startTime
	if global_variables.error: timeTaken = 'ERROR: the output from the solver exceeds 100000000 characters.'
	else:
		seconds_taken = timeTaken.total_seconds()
		if not (seconds_taken < 1200): timeTaken = 'TIMEOUT: running the non-zooming controller took longer than 20 minutes'

	results = open('experimental_results.txt', 'a')
	results.write('\nTime taken without zooming: ')
	results.write(str(timeTaken))
	results.close()

	return timeTaken




def createConditionsAndRun(trial_number):
	initial_conditions_index = 0
	controlled_run = False

	# set initial conditions
        refined_location_possibilities = ['c1', 'c2', 'c3', 'c4']#, 'c5', 'c6', 'c7', 'c8', 'c9']#, 'c10', 'c11', 'c12', 'c13', 'c14', 'c15', 'c16']
        in_hand_possibilities = ['false', 'true']
	robot_refined_location = sys_random.choice(refined_location_possibilities)
	book1_refined_location = sys_random.choice(refined_location_possibilities)
	#book2_refined_location = sys_random.choice(refined_location_possibilities)
	#book3_refined_location = sys_random.choice(refined_location_possibilities)
	#book4_refined_location = sys_random.choice(refined_location_possibilities)
        if robot_refined_location == book1_refined_location:
            in_handBook1 = sys_random.choice(in_hand_possibilities)
	else: in_handBook1 = 'false'
        #if (robot_refined_location == book2_refined_location) and (in_handBook1 == 'false'):
        #    in_handBook2 = sys_random.choice(in_hand_possibilities)
	#else: in_handBook2 = 'false'
        #if (robot_refined_location == book3_refined_location) and (in_handBook1 == 'false') and (in_handBook2 == 'false'):
        #    in_handBook3 = sys_random.choice(in_hand_possibilities)
	#else: in_handBook3 = 'false'
        #if (robot_refined_location == book4_refined_location) and (in_handBook1 == 'false') and (in_handBook2 == 'false') and (in_handBook3 == 'false'):
        #    in_handBook4 = sys_random.choice(in_hand_possibilities)
	#else: in_handBook4 = 'false'
	if in_handBook1 == 'true': in_handBook1Ref1, in_handBook1Ref2, in_handBook1Ref3 = 'true', 'false', 'false'
	else: in_handBook1Ref1, in_handBook1Ref2, in_handBook1Ref3 = 'false', 'false', 'false'
	#if in_handBook2 == 'true': in_handBook2Ref1, in_handBook2Ref2, in_handBook2Ref3 = 'true', 'false', 'false'
	#else: in_handBook2Ref1, in_handBook2Ref2, in_handBook2Ref3 = 'false', 'false', 'false'
	#if in_handBook3 == 'true': in_handBook3Ref1, in_handBook3Ref2, in_handBook3Ref3 = 'true', 'false', 'false'
	#else: in_handBook3Ref1, in_handBook3Ref2, in_handBook3Ref3 = 'false', 'false', 'false'
	#if in_handBook4 == 'true': in_handBook4Ref1, in_handBook4Ref2, in_handBook4Ref3, in_handBook4Ref4 = 'true', 'false', 'false', 'false'
	#else: in_handBook4Ref1, in_handBook4Ref2, in_handBook4Ref3, in_handBook4Ref4 = 'false', 'false', 'false', 'false'
	initial_state = [robot_refined_location , book1_refined_location, in_handBook1, in_handBook1Ref1]


	# choose goal randomly - old version
        '''
	book_loc_possibilities = ['library', 'kitchen', 'office1', 'office2', 'storage_cupboard']
	book_in_hand_possibilities = ['holds', '-holds', '']
	book1_loc = sys_random.choice(book_loc_possibilities)
	book1_in_hand = sys_random.choice(book_in_hand_possibilities)
	book2_loc = sys_random.choice(book_loc_possibilities)
	book2_in_hand = sys_random.choice(book_in_hand_possibilities)
	book3_loc = sys_random.choice(book_loc_possibilities)
	book3_in_hand = sys_random.choice(book_in_hand_possibilities)
	book4_loc = sys_random.choice(book_loc_possibilities)
	book4_in_hand = sys_random.choice(book_in_hand_possibilities)
	goal = 'holds(loc(book1,'+book1_loc+'),I), holds(loc(book2,'+book2_loc+'),I), holds(loc(book3,'+book3_loc+'),I), holds(loc(book4,'+book4_loc+'),I)'
	if not book1_in_hand == '': goal = goal + ', '+book1_in_hand+'(in_hand(rob1,book1),I)'
	if book1_in_hand == 'holds': goal = goal + ', -holds(in_hand(rob1,book2),I)'
	elif not book2_in_hand == '': goal = goal + ', '+book2_in_hand+'(in_hand(rob1,book2),I)'
	if book1_in_hand == 'holds' or book2_in_hand == 'holds': goal = goal + ', -holds(in_hand(rob1,book3),I)'
	elif not book3_in_hand == '': goal = goal + ', '+book3_in_hand+'(in_hand(rob1,book3),I)'
	if book4_in_hand == '': goal = goal + '.'
	elif book1_in_hand == 'holds' or book2_in_hand == 'holds' or book3_in_hand == 'holds': goal = goal + ', -holds(in_hand(rob1,book4),I).'
	else: goal = goal + ', '+book4_in_hand+'(in_hand(rob1,book4),I).'
        '''

        # chose goal randomly - move one randomly chosen object to one randomly chosen location
        book_choice_possibilities = ['book1']#, 'book2']#, 'book3']#, 'book4']
	book_loc_possibilities = ['library', 'kitchen']#, 'office1']#, 'office2']#, 'storage_cupboard']
        book_choice = sys_random.choice(book_choice_possibilities)
        book_loc = sys_random.choice(book_loc_possibilities)
        goal = 'holds(loc('+book_choice+','+book_loc+'),I).'

	print ('\nGoal:')
	print (goal)
	print ('Initial_state:')
	print (initial_state)

	# run experiment
	zoom_time = runAndWrite(initial_conditions_index, trial_number, goal, initial_state)
	non_zoom_time = runAndWriteWithoutZooming(initial_conditions_index, goal, initial_state)

	if isinstance(zoom_time,datetime) and isinstance(non_zoom_time,datetime):
		zoom_seconds = zoom_time.total_seconds()
		non_zoom_seconds = non_zoom_time.total_seconds()
		results = open('experimental_results.txt', 'a')
		results.write('\nRatio: ')
		results.write(str(float(non_zoom_seconds)/float(zoom_seconds)))
		results.close()



if __name__ == "__main__":
	global_variables.init()
	number_runs = 50
	for x in range (0,number_runs): createConditionsAndRun(x+1)
