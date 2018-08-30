
from datetime import datetime
from simulation.realWorld import World
from controllerTraditionalPlanning import ControllerTraditionalPlanning
from controllerToI import ControllerToI
from nonZoomingControllerToI import NonZoomingControllerToI
from simulation.executer import Executer
import  csv
import random
from simulation.domain_info import DomainInfo
ASP_subfolder_path = 'simulation/'
results_file_name = "simulation/results/"

sparc_path = "$HOME/work/solverfiles/sparc.jar"

goal = "holds(loc(book1,kitchen),I)."

max_plan_length = 17

textfile = None
csvfile = None
writer = None
initial_state = []

coarse_locations = DomainInfo.CoarseLocations
coarse_locations_as_cells = DomainInfo.CoarseLocationsAsCells
boolean = ['true', 'false']


run_count = 0

def runAndWrite(initial_conditions_index):

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
	controllerToI = ControllerToI(sparc_path, ASP_subfolder_path, domain_info, executer, robot_refined_location, initial_conditions , goal, max_plan_length)

	history_toi, numberPlans_toi, goal_correction_toi = controllerToI.run()

	endTime = datetime.now()
	timeTaken = endTime - startTime

	results = open('experimental_results.txt', 'w')
	results.write('Initial state: [rob1_loc, book1_loc, book2_loc, book1_in_hand, book2_in_hand] = ')
	results.write(str(initial_state))
	results.write('\nGoal: ')
	results.write(goal)
	results.write('\nTime taken with zooming: ')
	results.write(str(timeTaken))




def runAndWriteWithoutZooming(initial_conditions_index):

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

	history_toi, numberPlans_toi, goal_correction_toi = controllerToI.run()

	endTime = datetime.now()
	timeTaken = endTime - startTime

	results = open('experimental_results.txt', 'a')
	results.write('\nTime taken without zooming: ')
	results.write(str(timeTaken))




def createConditionsAndRun():
	global initial_state
	initial_conditions_index = 0
	controlled_run = False

	# set initial conditions
	robot_refined_location = 'c4'
	book1_refined_location = 'c2'
	refined_in_handBook1 = 'false'
	refined_in_handBook1Ref1 = 'false'
	initial_state = [robot_refined_location , book1_refined_location, refined_in_handBook1, refined_in_handBook1Ref1]

	# run experiment
	#runAndWrite(initial_conditions_index)
	runAndWriteWithoutZooming(initial_conditions_index)


	#Cases when rob1 is holding book1 (16 possible combinations)
	#for robot_coarse_location_as_cells in coarse_locations_as_cells:
	#	robot_refined_location = random.choice(robot_coarse_location_as_cells)
	#	for book2_coarse_location_as_cells in coarse_locations_as_cells:
	#		book2_refined_location = random.choice(book2_coarse_location_as_cells)
	#		initial_conditions_index +=1
	#		if(controlled_run == True and initial_conditions_index != controlled_run_conditions): continue
	#		book1_refined_location = robot_refined_location
	#		refined_in_handBook1 = 'true'
	#		refined_in_handBook2 = 'false'
	#		initial_state = [robot_refined_location , book1_refined_location , book2_refined_location, refined_in_handBook1, refined_in_handBook2]
	#		runAndWrite(initial_conditions_index)

	#Cases when rob1 is holding book2 (16 possible combinations)
	#for robot_coarse_location_as_cells in coarse_locations_as_cells:
	#	robot_refined_location = random.choice(robot_coarse_location_as_cells)
	#	for book1_coarse_location_as_cells in coarse_locations_as_cells:
	#		book1_refined_location = random.choice(book1_coarse_location_as_cells)
	#		initial_conditions_index +=1
	#		if(controlled_run == True and initial_conditions_index != controlled_run_conditions): continue
	#		book2_refined_location = robot_refined_location
	#		refined_in_handBook2 = 'true'
	#		refined_in_handBook1 = 'false'
	#		initial_state = [robot_refined_location , book1_refined_location , book2_refined_location, refined_in_handBook1, refined_in_handBook2]
	#		runAndWrite(initial_conditions_index)

	#Cases when rob1 is not holding book1 and book2 (64 possible combinations)
	#for robot_coarse_location_as_cells in coarse_locations_as_cells:
	#	robot_refined_location = random.choice(robot_coarse_location_as_cells)
	#	for book1_coarse_location_as_cells in coarse_locations_as_cells:
	#		book1_refined_location = random.choice(book1_coarse_location_as_cells)
	#		for book2_coarse_location_as_cells in coarse_locations_as_cells:
	#			book2_refined_location = random.choice(book2_coarse_location_as_cells)
	#			initial_conditions_index +=1
	#			if(controlled_run == True and initial_conditions_index != controlled_run_conditions): continue
	#			refined_in_handBook1 = 'false'
	#			refined_in_handBook2 = 'false'
	#			initial_state = [robot_refined_location , book1_refined_location , book2_refined_location, refined_in_handBook1, refined_in_handBook2]
	#			runAndWrite(initial_conditions_index)



if __name__ == "__main__":
	number_runs = 1
	for x in range (0,number_runs): createConditionsAndRun()
