from datetime import datetime

from simulation.realWorld import World
from controllerToI import ControllerToI
from simulation.executer import Executer
from sets import Set
import subprocess
import  csv
import random
import multiprocessing
import time
import global_variables
from simulation.domain_info import DomainInfo
import sys
import csv
from decimal import getcontext, Decimal
from pprint import pprint
from collections import OrderedDict

global trialCount
global maxRunningTime

def runAndWrite( goal, dic_initial_state):
	print 'Trial Number: ' + str(trialCount)
	print 'Complexity Level: ' + str(global_variables.complexity_level)
	print 'Controler type: ' + str(global_variables.controller_type).upper()
	print 'Goal: ' + goal
	print ('Initial_state:')
	pprint(dic_initial_state.items())
	my_world = World(dic_initial_state,domain_info)
	executer = Executer(my_world)
	dic_known_world = my_world.getAbstractState()
	initial_history = list(domain_info.dic_abstractStateToAbstractHoldsSet(dic_known_world,0))
	robot_refined_location = my_world.getRobotRefinedLocation()
	results = open(global_variables.txt_results_file, 'a')
	results.write('\nTrial Number: ' + str(trialCount))
	results.write('\nComplexity Level: ' + str(global_variables.complexity_level))
	results.write('\nControler type: ' + str(global_variables.controller_type).upper())
	results.write('\nGoal: ' + goal)
	results.write('\nInitial World State: '+ str(dic_initial_state))
	results.close()

	controllerToI = ControllerToI( domain_info, executer, robot_refined_location, initial_history , goal)
	error = multiprocessing.Event()
	refinedPlanningTime = multiprocessing.Value('d', 0)
	abstractPlanningTime = multiprocessing.Value('d', 0)
	inferObsTime = multiprocessing.Value('d', 0)
	diagnosingTime = multiprocessing.Value('d', 0)
	execTime = multiprocessing.Value('d', 0)
	numAbsPlans = multiprocessing.Value('d', 0)
	numRefPlans = multiprocessing.Value('d', 0)
	numAbsAct = multiprocessing.Value('d', 0)
	numRefAct = multiprocessing.Value('d', 0)
	completeRun = multiprocessing.Value('c','_')
	p1 = multiprocessing.Process(target=controllerToI.run, name="Func", args=(error, refinedPlanningTime, abstractPlanningTime, inferObsTime, diagnosingTime, execTime, numAbsPlans, numRefPlans, numAbsAct, numRefAct,completeRun))
	timeTaken = 0
	p1.start()
	startTime = datetime.now()
	timeout = False
  	while True:
		if error.is_set():
			print refinedPlanningTime.value
			print abstractPlanningTime.value
			print inferObsTime.value
			print diagnosingTime.value
			print execTime.value
			print numAbsPlans.value
			print numAbsAct.value
			print numRefPlans.value
			print numRefAct.value
			print completeRun.value
			print '\n\n'
			try:
				subprocess.check_output('killall clingo', shell=True)
				print('Clingo was killed')
			except:
				print('Clingo was not running.')
			print(' !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			results = open(global_variables.txt_results_file, 'a')
			results.write('\n' + global_variables.controller_type + ' ERROR FOUND')
			results.close()
			break
		if not p1.is_alive(): break

		'''
		if (datetime.now()-startTime).seconds > maxRunningTime and p1.is_alive():
			timeout = True
			completeRun.value = global_variables.character_code_timeout
			p1.terminate()
			p1.join()
			try:
				subprocess.check_output('killall clingo', shell=True)
			except subprocess.CalledProcessError:
				print('Clingo was not running.')
			results = open(global_variables.txt_results_file, 'a')
			results.write('\nTIMEOUT: running the zooming controller took ' + str(maxRunningTime) + ' seconds.')
			results.close()
			print '$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$: TIMEOUT at '+ str((datetime.now()-startTime).total_seconds())+' seconds!!!!!!!!!!'
		'''
	timeTaken = (datetime.now()-startTime).total_seconds()
	print '$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$: Total run time: ' + str(timeTaken) + ' seconds.'
	results = open(global_variables.txt_results_file, 'a')
	results.write('\nTotal trial time: ' + str(timeTaken) + ' seconds.')
	results.write('\nTime Refined planning: '+ str(refinedPlanningTime.value))
	results.write('\nTime Abstract planning: '+ str(abstractPlanningTime.value))
	results.write('\nTime Diagnosing: '+ str(diagnosingTime.value))
	results.write('\nTime Inferring coarse resolution obs: '+str(inferObsTime.value))
	results.write('\nTime Executing: ' + str(execTime.value))
	results.write('\nNumber of abstract plans: '  + str(numAbsPlans.value))
	results.write('\nNumber of abstract actions: ' + str(numAbsAct.value))
	results.write('\nNumber of refined plans: ' + str(numRefPlans.value))
	results.write('\nNumber of refined actions: ' + str(numRefAct.value))
	results.write('\nComplete run code: ' + str(completeRun.value))
	results.write('\n----------------------------------------------------------------------------\n\n\n')
	results.close()

	return [timeTaken,refinedPlanningTime.value,abstractPlanningTime.value,diagnosingTime.value, inferObsTime.value, numAbsPlans.value,numAbsAct.value,numRefPlans.value,numRefAct.value,completeRun.value]


def create_pre_ASP_abstract_belief_file():
	## read
	reader = open(domain_info.file_name_preASP_abstract_domain, 'r')
	pre_ASP_lines = reader.read().split('\n')
	reader.close()
	## add abstract constants and attributes for the appropriate complexity (given by domain_info already)
	pre_ASP_lines[pre_ASP_lines.index(domain_info.refined_constants_marker)] = '\n'.join(domain_info.abstract_constants_lines)
	pre_ASP_lines[pre_ASP_lines.index(domain_info.attributes_marker)] = '\n'.join(domain_info.abstract_attributes_lines)
	## write
	writer = open(domain_info.file_name_preASP_abstract_belief, 'w')
	writer.write('\n'.join(pre_ASP_lines))
	writer.close()

def create_pre_ASP_ToI_planning_file():
	## read
	reader = open(domain_info.file_name_preASP_ToI_domain, 'r')
	pre_ASP_lines = reader.read().split('\n')
	reader.close()
	## add abstract constants and attributes for the appropriate complexity (given by domain_info already)
	pre_ASP_lines[0] = '#const numSteps = ' + str(domain_info.max_number_steps_ToI_planning) +'.'
	pre_ASP_lines[1] = '#const max_len = ' + str(domain_info.max_number_steps_ToI_planning+1) +'.'
	pre_ASP_lines[pre_ASP_lines.index(domain_info.refined_constants_marker)] = '\n'.join(domain_info.abstract_constants_lines)
	pre_ASP_lines[pre_ASP_lines.index(domain_info.attributes_marker)] = '\n'.join(domain_info.abstract_attributes_lines)
	## write
	writer = open(domain_info.file_name_preASP_ToI_planning, 'w')
	writer.write('\n'.join(pre_ASP_lines))
	writer.close()



def runPairwiseControllersAndWrite(dic_initial_state, goal):
	global_variables.controller_type = 'zooming'
	resultsListZooming = runAndWrite(goal, dic_initial_state)

	if global_variables.complexity_level <= 4: #run both zooming and non zooming
		global_variables.controller_type = 'non_zooming'
		resultsListNonZooming = runAndWrite(goal, dic_initial_state)
	else: resultsListNonZooming = ['']*len(resultsListZooming)

	resultsList = [trialCount, str(global_variables.complexity_level)]
	for i in range(len(resultsListZooming)-1):
		if(i < 5 or i == 31):
			try: resultsList.append("{0:.2f}".format(resultsListZooming[i]))
			except: resultsList.append("")
			try: resultsList.append("{0:.2f}".format(resultsListNonZooming[i]))
			except: resultsList.append("")
		else:
			try: resultsList.append(int(resultsListZooming[i]))
			except: resultsList.append("")
			try: resultsList.append(int(resultsListNonZooming[i]))
			except: resultsList.append("")
		if resultsListNonZooming[-1] == "C" and resultsListZooming[-1] == "C":
			try: resultsList.append("{0:.2f}".format(resultsListNonZooming[i]/resultsListZooming[i]))
			except ZeroDivisionError: resultsList.append(float("inf"))
		else: resultsList.append("")
	resultsList.append(resultsListZooming[-1])
	resultsList.append(resultsListNonZooming[-1])

	with open(global_variables.csv_results_file, 'a') as writeFile:
		writer = csv.writer(writeFile)
		writer.writerow(resultsList)
	writeFile.close()

def createInitialConditionsAndGoal():
	dic_initial_state = OrderedDict()
	refined_objects = list(domain_info.refined_signature_dic['#object'])
	refined_locations = list(domain_info.refined_signature_dic['#place'])
	abstract_objects = list(domain_info.refined_signature_dic['#coarse_object'])
	for book in abstract_objects:
		chosen_location = random.choice(refined_locations)
		for ref_book in refined_objects:
			if book in ref_book: dic_initial_state['loc('+ref_book] = chosen_location
	if(random.uniform(0, 1) < 0.2):
		ref_object_holding = random.choice(refined_objects)
		dic_initial_state['in_hand(rob1'] = ref_object_holding
		dic_initial_state['loc(rob1'] = dic_initial_state['loc('+ref_object_holding]
	else: dic_initial_state['loc(rob1'] = random.choice(refined_locations)

	#creating gol
	book_choice = random.choice(abstract_objects)
	locations = list(domain_info.refined_signature_dic['#coarse_place'])
	chosen_book_location = dic_initial_state['loc(ref1_'+book_choice]
	chosen_book_abstract_location = domain_info.components_dic[chosen_book_location]
	locations.remove(chosen_book_abstract_location)
	location_choice = random.choice(locations)
	goal = 'holds(loc('+book_choice+','+location_choice+'),I).'

	return dic_initial_state, goal



def runGivenInitialState():
	global_variables.complexity_level = 4
	goal = 'holds(loc(book3,office2),I).'
	dic_initial_state =  OrderedDict([('loc(ref4_book1', 'c10'), ('loc(ref2_book1', 'c10'), ('loc(ref3_book1', 'c10'), ('loc(ref1_book1', 'c10'), ('loc(ref3_book2', 'c16'), ('loc(ref2_book2', 'c16'), ('loc(ref4_book2', 'c16'), ('loc(ref1_book2', 'c16'), ('loc(ref2_book3', 'c5'), ('loc(ref3_book3', 'c5'), ('loc(ref4_book3', 'c5'), ('loc(ref1_book3', 'c5'), ('loc(ref3_book4', 'c9'), ('loc(ref2_book4', 'c9'), ('loc(ref1_book4', 'c9'), ('loc(ref4_book4', 'c9'), ('loc(rob1', 'c19')])
	runPairwiseControllersAndWrite(dic_initial_state, goal)


if __name__ == "__main__":
	global trialCount
	global maxRunningTime
	maxRunningTime = 300
	trialCount = 0
	singleRunTest = False
	number_runs = 200
	for level in [8,7,31,9]:
		trialCount = 0
		global_variables.init()
		global_variables.complexity_level = level # TODO change this number to change the complexity level
		global_variables.csv_results_file = 'simulation/results/experimental_results_' +  str(global_variables.complexity_level) +  '.csv'
		global_variables.txt_results_file = 'simulation/results/experimental_results_' +  str(global_variables.complexity_level) +  '.txt'
		domain_info = DomainInfo()
		## The next two files would normally be created already in other versions, but in this case we need to create them now as the constants of the domain change with each complexity level. These functions will use
		## the complexity level (initialised above) and the paths of the files with the pre ASP info, initialised in DomainInfo().
		create_pre_ASP_abstract_belief_file()
		create_pre_ASP_ToI_planning_file()
		with open(global_variables.csv_results_file, 'a') as writeFile:
			writer = csv.writer(writeFile)
			writer.writerow(['Trial Number', 'Complexity Level', 'Run-time zooming','Run-time non_zooming','Run-time Ratio',  'Refined Planning Time - zooming','Refined Planning Time - non_zooming','Refined Planning Time - Ratio', 'Abstract Planning Time - zooming','Abstract Planning Time - non_zooming','Abstract Planning Time - Ratio','Diagnosing - zooming', 'Diagnosing - non_zooming', 'Diagnosing - Ratio', 'Inferring -zooming', ' Inferring - non_zooming', 'Inferring - Ratio', '# Abstract Plans - zooming', '# Abstract Plans - non_zooming','# Abstract Plans - Ratio','# Abstract Actions - zooming','# Abstract Actions - non_zooming','# Abstract Actions - Ratio','# Refined Plans - zooming','# Refined Plans - non_zooming','# Refined Plans - Ratio',  '# Refined Actions - zooming','# Refined Actions - non_zooming' ,'# Refined Actions - Ratio','Complete Run - zooming','Complete Run - non_zooming'])
		remaining_runs = number_runs - trialCount
		if(singleRunTest): remaining_runs = 1
		for x in range (0,remaining_runs):
			trialCount += 1
			if(singleRunTest): runGivenInitialState()
			else:
				dic_initial_state, goal = createInitialConditionsAndGoal()
				runPairwiseControllersAndWrite(dic_initial_state, goal)
