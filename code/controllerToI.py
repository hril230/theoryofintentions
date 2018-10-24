from datetime import datetime
from sets import Set
import subprocess
import re
import numpy as np
import random
import sys
import global_variables
from itertools import groupby
import time
import multiprocessing
import inspect
import shutil
from decimal import Decimal

class ControllerToI():
	def __init__(self,  domain_info, executer, refined_location, initial_conditions , goal, max_plan_length):
		self.lines_to_write = []
		self.acting_times = []
		self.testing_times = []
		self.last_abstract_planning_time = None
		self.abstract_action_plan = ''
		self.error = multiprocessing.Event()

		self.goal = goal
		self.executer = executer
		self.domain_info = domain_info
		self.max_plan_length = max_plan_length



		## used for paths and names of the ASP files created
		self.asp_ToI_planning_file = global_variables.ASP_subfolder+'ASP_files/ToI_Planning.sp'
		self.asp_ToI_diagnosing_file = global_variables.ASP_subfolder + 'ASP_files/ToI_Diagnosis.sp'
		self.asp_zoomed_domain_file = global_variables.ASP_subfolder + 'ASP_files/Zoomed_Planning.sp'
		self.asp_non_zoomed_domain_file = global_variables.ASP_subfolder + 'ASP_files/Non_Zoomed_Planning.sp'
		self.asp_abstract_belief_file = global_variables.ASP_subfolder + 'ASP_files/Abstract_Belief.sp'
		self.asp_inferring_indirect_observations_file = global_variables.ASP_subfolder + 'ASP_files/Inferring_Indirect_Observations.sp'

		# holds the markers in the text files.
		self.goal_marker = '%% GOAL GOES HERE'
		self.current_step_marker = '%% CURRENT STEP GOES HERE'
		self.history_marker = '%% HISTORY GOES HERE'

		# variables relevant for the ASP_ToI planning and diagnosis
		self.number_toi_steps = 4 #initial total number of steps the controller assumes it will need for planning
		self.number_activities = 1 #keeping record of the number of activitites the ASP needs
		self.goal_correction = 0 #keeping record of the number of times the goal has been assumed to be true but it was False
		self.current_diagnosis = '' #keeping the latest diagnosis
		self.input_for_planning = [] #holds the input necessary to get the next intended action in the ASP_TOI_Planning
		self.current_step = 1 # holds the current step of the controllerToI, which is the same as the ASP_TOI_Planning

		self.preparePreASPStringLists()
		self.setInitialBelief(self.filteredPlainHistory(initial_conditions))
		self.history_ToI_diagnosis = initial_conditions #holds the history input for ASP_ToI_Diagnosis
		self.refined_location = refined_location

		print ('ControllerToI ' + global_variables.controller_type + ' - initial coarse belief: ' + str(self.belief))
		print ('Controller ToI - initial refined location: ' + str(self.refined_location))


	def run(self,error, final_planning_time, numAbsPlans, numRefPlans, numAbsAct, numRefAct,completeRun):
		self.completeRun = completeRun
		self.final_planning_time = final_planning_time
		self.planning_time = datetime.now()-datetime.now()
		self.error = error
		self.believes_goal_holds = False
		self.history_ToI_diagnosis.append("hpd(select(my_goal), true,0).")

		if(global_variables.controller_type == 'zooming'): self.lines_to_write.append('\nZOOMING CONTROLLER')
		else: self.lines_to_write.append('\nNON - ZOOMING CONTROLLER')


		self.diagnose()
		finish = False # flag that breaks the loop that calls the ASP_ToI_Planning when finish == True
		while(finish == False and not self.error.is_set()):
			abstract_action = self.runToIPlanning(self.input_for_planning)
			if(abstract_action == 'finish'):
				#check if the assuption that the goal has been reached is true.
				if(self.executer.getGoalFeedback() == True):
					self.history_ToI_diagnosis.append('finish')
					print('ControllerToI ' + global_variables.controller_type + ' : \t Belief: ' + str(self.belief))
					print('ControllerToI ' + global_variables.controller_type + ' : \t Feedback from the workd: Belief is True')
					print('ControllerToI ' + global_variables.controller_type + ' : \t Finish')
					self.completeRun.value = global_variables.character_code_complete_run
					finish = True
					break
				# if it is false, count the correction and diagnose again
				else:
					self.goal_correction += 1
					while(abstract_action == 'finish'):
						print('ControllerToI ' + global_variables.controller_type + ' : \t Belief: ' + str(self.belief))
						print('ControllerToI ' + global_variables.controller_type + ' : \t Feedback from the workd: Belief is False')
						self.history_ToI_diagnosis.append('obs(my_goal,false,'+str(self.current_step)+').')
						self.diagnose()
						abstract_action = self.runToIPlanning(self.input_for_planning)
			if(abstract_action == None):
				self.history_ToI_diagnosis.append("Goal is futile")
				print ('ControllerToI ' + global_variables.controller_type + ' : \t Goal is futile ')
				finish = True
				break

			# if action is 'stop' it is because an unexpected action has happened and it needs to replan, so the number_activities should increase
			# to pass this as an input to the ASP_ToI_Planning
			if(abstract_action[0:4] == 'stop'):
				self.number_activities += 1
				self.number_toi_steps += 1
			elif(abstract_action[0:5] == 'start'):
				numAbsPlans.value += 1
				self.lines_to_write.append('\nAbstract action plan: ')
				self.lines_to_write.append(self.abstract_action_plan)
				self.lines_to_write.append('\nTime taken to plan abstract action plan: ' + str(self.last_abstract_planning_time))
			else:
				print ('\nControllerToI - ref location and coarse belief: ' +self.refined_location + ', '+ str(self.belief))
				print ('\nControllerToI ' + global_variables.controller_type + ' - next abstract action: ' + abstract_action)
				numAbsAct.value += numAbsAct.value
				need_refined_plan = True
				refined_plan_step = 0
				refined_history = self.refine(abstract_action, self.refined_location) #holds all the history of the refined plan created wtih refined domain in form of 'hpd' and 'obs'
				direct_observations_history = Set() #holds only the direct observations made during the execution of refined plan in form of 'holds(directly_observed...)'
				initial_refined_location = self.refined_location #holds the refined location before the refined plan is executed. It is used as initial
																 #condition for the ASP that infers coarse observations from the refined direct observations
				while(need_refined_plan and not self.error.is_set()):

					startTime = datetime.now() # record the time taken to plan the refined plan
					refined_actions = self.runRefinedPlanning(abstract_action,refined_plan_step)
					numRefPlans.value += 1
					endTime = datetime.now()
					timeTaken = endTime - startTime
					need_refined_plan = False
					'''
					if (refined_actions == ['']) or (refined_actions == None):
						print ('ControllerToI ' + global_variables.controller_type + ' : \t\t No refined plan ')
						lineno = self.lineno()
						print 'Error set at line:  '+str(lineno)
						self.final_planning_time.value = self.planning_time.total_seconds()
						self.completeRun = global_variables.character_code_inconsistency
						self.error.set()
					'''
					refined_actions.sort(key=self.getStep)
					print ('ControllerToI ' + global_variables.controller_type + ' - refined plan: ' + str(refined_actions[refined_plan_step:]))
					# store refined plan
					self.lines_to_write.append('\nAbstract action: ' + str(abstract_action))
					self.lines_to_write.append('\nRefined plan: ' + str(refined_actions))
					self.lines_to_write.append('\nTime taken to create refined plan: ' + str(timeTaken))
					self.planning_time += timeTaken

					last_action_needs_testing = False
					last_action = ''
					for i in range(len(refined_actions)):
						numRefAct.value += 1
						refined_occurence = refined_actions[i]
						refined_action = refined_occurence[refined_occurence.find('(')+1 : refined_occurence.rfind(',')]
						occurrence_step = int(refined_occurence[refined_occurence.rfind(',')+1:refined_occurence.rfind(')')])
						if occurrence_step != refined_plan_step: continue
						print ('ControllerToI ' + global_variables.controller_type + ' - Refined action: ' + str(refined_action) + ' at refined step: ' + str(occurrence_step))
						if 'test' in refined_action:
							startTime = datetime.now()
							action_test_result, action_direct_observation = self.executer.test(refined_action)
							direct_observations_refined_step = Set([action_direct_observation])
							other_direct_observations = []
							if last_action_needs_testing:
								if action_test_result==True : refined_history.add('hpd(' + last_action + ',' + str(refined_plan_step - 1) +').')
								last_action_needs_testing = False
								if 'put_down' in last_action:
									other_direct_observations = self.test_in_hand_all_refined_parts(last_action)
							endTime = datetime.now()
							totalTime = endTime - startTime
							self.testing_times.append(totalTime)
							refined_history.add('hpd(' + refined_action + ',' + str(refined_plan_step) +').')
							refined_plan_step += 1
							if(action_test_result==True and 'loc(rob1' in refined_action):
								self.refined_location = action_direct_observation.split(',')[2][:-1]
								print ('ControllerToI ' + global_variables.controller_type + ' - Refined location: '+self.refined_location)
							goal_direct_observations = self.getObservationsRelevantGoal()
							print 'goal direct observations '
							print goal_direct_observations
							direct_observations_refined_step = direct_observations_refined_step.union(goal_direct_observations).union(other_direct_observations)
							direct_observations_history = direct_observations_history.union(self.prepareRefinedObservations(direct_observations_refined_step,refined_plan_step))
							obs_refined_step = Set()
							for direct_observations in direct_observations_refined_step:
								obs_refined_step.add(self.domain_info.directObservationToRefinedObs(direct_observations, refined_plan_step))
							refined_history = refined_history.union(obs_refined_step)


							if(action_test_result == False):
								print ('ControllerToI ' + global_variables.controller_type + ' - need another refined plan ')
								need_refined_plan = True
								self.addObsRefinedDomain(list(refined_history),refined_plan_step+6)
								break
						else:
							if('put_down' in refined_action):
								holdingBook = ''
								if self.belief[self.domain_info.In_handBook1_index]: holdingBook = 'book1'
								elif self.belief[self.domain_info.In_handBook2_index]: holdingBook = 'book2'
								elif self.belief[self.domain_info.In_handBook3_index]: holdingBook = 'book3'
								elif self.belief[self.domain_info.In_handBook4_index]: holdingBook = 'book4'
								if holdingBook != '': refined_history.add('holds(in_hand(rob1,ref1_'+holdingBook+'),'+str(refined_plan_step)+').')
							previous_belief = self.belief[:]
							startTime = datetime.now()
							self.executer.executeAction(refined_action)
							endTime = datetime.now()
							self.acting_times.append(endTime-startTime)
							self.belief = previous_belief
							last_action = refined_action
							last_action_needs_testing = True

							refined_plan_step += 1

				refined_history_list = list(refined_history)
				refined_history_list.sort(key=self.getStep)
				direct_observations_history_list = list(direct_observations_history)
				direct_observations_history_list.sort(key=self.getStep)
				history_for_inferring_indirect_observations = refined_history_list + direct_observations_history_list
				if global_variables.controller_type == 'zooming':
					abstract_step_obs = list(self.infer_abstract_obs_from_refined_observations_zooming(list(history_for_inferring_indirect_observations), refined_plan_step))
				elif global_variables.controller_type == 'non_zooming':
					abstract_step_obs = list(self.infer_abstract_obs_from_refined_observations_non_zooming(list(history_for_inferring_indirect_observations), refined_plan_step))

				print ('ControllerToI ' + global_variables.controller_type + ' - Abstract action ' +abstract_action+' has finished at step ' + str(self.current_step))
				print ('ControllerToI ' + global_variables.controller_type + ' - Abstract obs: ' + ': ' + str(abstract_step_obs))
				self.update_belief(abstract_action, abstract_step_obs)
				self.history_ToI_diagnosis = self.history_ToI_diagnosis + abstract_step_obs
			self.history_ToI_diagnosis.append('attempt('+abstract_action+','+str(self.current_step)+').')
			self.current_step += 1
			self.diagnose()
		if(self.current_diagnosis != ''): self.history_ToI_diagnosis.append(self.current_diagnosis)


		final_planning_time.value = self.planning_time.total_seconds()

		results = open('experimental_results_'+ str(global_variables.complexity_level)+'.txt', 'a')
		for line in self.lines_to_write: results.write(line)
		results.write('\nTotal time taken planning with zooming: ')
		results.write(str(self.planning_time))
		results.close()


		# write results to text file
		timeTakenActing = self.acting_times[0]
		for i in range(1,len(self.acting_times)): timeTakenActing = timeTakenActing + self.acting_times[i]
		results = open('experimental_results_'+ str(global_variables.complexity_level)+'.txt', 'a')
		results.write('\nTotal time taken acting: ')
		results.write(str(timeTakenActing))
		results.close()

		# write results to text file
		timeTakenTesting = self.testing_times[0]
		for i in range(1,len(self.testing_times)): timeTakenTesting = timeTakenTesting + self.testing_times[i]
		results = open('experimental_results_'+ str(global_variables.complexity_level)+'.txt', 'a')
		results.write('\nTotal time taken testing: ')
		results.write(str(timeTakenTesting))
		results.close()


 	def test_in_hand_all_refined_parts(self, put_down_action):
		observations = Set()
		object = put_down_action[put_down_action.rfind('_')+1:-1]
		if(global_variables.complexity_level > 1 ):
			result, observation = self.executer.test('test(rob1,in_hand(rob1,ref2_'+object+'),true).')
			observations.add(observation)
		if(global_variables.complexity_level > 2 ):
			result, observation = self.executer.test('test(rob1,in_hand(rob1,ref3_'+object+'),true).')
			observations.add(observation)
		if(global_variables.complexity_level > 3 ):
			result, observation = self.executer.test('test(rob1,in_hand(rob1,ref4_'+object+'),true).')
			observations.add(observation)
		return observations

	def getStep(self,myString):
		partOfString = myString[myString.rfind(',')+1:]
		myStep = map(int, re.findall('\d+', partOfString))[0]
		return int(myStep)


	def prepareRefinedObservations(self,observations,step):
		if observations == Set(['']) or not observations: return observations
		updated_observations = Set()
		for observation in observations:
			updated_observations.add(observation[:observation.rfind(',')+1] + str(step) + ').')
		return updated_observations

	def getObservationsRelevantGoal(self):
		startTime = datetime.now()
		observations = Set()
		objectsRelevantToGoal = [o for o in global_variables.abstract_objects_list[global_variables.complexity_level-1] if o in self.goal]
		#print ' objects relevant to goal '
		#print objectsRelevantToGoal
		for object in objectsRelevantToGoal:
			if 'loc(' + object in self.goal:
				result, observation = self.executer.test('test(rob1,loc(ref1_'+object+',' + self.refined_location+ '),true)')
				observations.add(observation)
			if 'in_hand(rob1,' + object in self.goal:
				result, observation = self.executer.test('test(rob1,in_hand(rob1,ref1_'+object+'),true)')
				observations.add(observation)
		endTime = datetime.now()
		totalTime = endTime - startTime
		self.testing_times.append(totalTime)
		return observations

	def find_relevant_sorts_and_attributes(self,refined_history):

		all_coarse_objects =  global_variables.abstract_objects_list[global_variables.complexity_level-1][:]
		relevant_coarse_objects = []
		for object in all_coarse_objects:
			if object in ('\n').join(refined_history): relevant_coarse_objects.append(object)
		#print 'relevant coarse objects'
		#print relevant_coarse_objects
		all_refined_objects = global_variables.refined_objects_list[global_variables.complexity_level-1][:]
		relevant_refined_objects = []
		for coarse_object in relevant_coarse_objects:
			for refined_object in all_refined_objects:
				if coarse_object in refined_object: relevant_refined_objects.append(refined_object)

		#print 'relevant refine objects'
		#print relevant_refined_objects

		all_refined_places = global_variables.refined_places_list[global_variables.complexity_level-1][:]
		refined_places_in_history = []
		for refined_place in all_refined_places:
			if refined_place+')' in ('\n').join(refined_history): refined_places_in_history.append(refined_place)

		#print 'refined places in history'
		#print refined_places_in_history

		all_coarse_places = global_variables.abstract_locations_list[global_variables.complexity_level-1][:]
		relevant_coarse_places = []
		for coarse_place in all_coarse_places:
			if coarse_place in ('\n').join(refined_history): relevant_coarse_places.append(coarse_place)
		for refined_place in refined_places_in_history:
			for component in global_variables.refined_locations_components_list[global_variables.complexity_level-1]:
				if refined_place in component:
					coarse_place = component[component.find(',')+1:-1]
					if coarse_place not in relevant_coarse_places: relevant_coarse_places.append(coarse_place)
		#print ' relevant coarse places'
		#print relevant_coarse_places
		relevant_refined_places = []
		for coarse_location in relevant_coarse_places:
			for component in global_variables.refined_locations_components_list[global_variables.complexity_level-1]:
				if coarse_location in component:
					refined_place = component[component.find('(')+1:component.find(',')]
					if refined_place not in relevant_refined_places: relevant_refined_places.append(refined_place)

		#print ' relevant refined places'
		#print relevant_refined_places

		all_refined_attributes_list = (global_variables.refined_attributes_string[global_variables.complexity_level-1]).split('\n')
		irrelevant_coarse_places = np.setdiff1d(all_coarse_places,relevant_coarse_places)
		irrelevant_refined_places = np.setdiff1d(all_refined_places,relevant_refined_places)
		irrelevant_coarse_objects = np.setdiff1d(all_coarse_objects, relevant_coarse_objects)

		#print 'relevant coarse places ' + str(relevant_coarse_places)
		#print 'irrelevant coarse palces ' + str(irrelevant_coarse_places)
		#print 'irrelevant refined places ' +str(irrelevant_refined_places)
		#print 'irrelevant coarse objects ' + str(irrelevant_coarse_objects)

		relevant_refined_attributes = all_refined_attributes_list[:]
		for attribute in all_refined_attributes_list:
			#print('attribute ') + attribute
			for place in irrelevant_coarse_places:
				#print(' irrelevant place ') + place
				if place in attribute and attribute in relevant_refined_attributes:
					relevant_refined_attributes.remove(attribute)
					#print( ' in attribute - removed')
					break
			for object in irrelevant_coarse_objects:
				#print( ' irrelevant coarse object ') + object
				if object in attribute and attribute in relevant_refined_attributes:
					relevant_refined_attributes.remove(attribute)
					#print( ' in attribute - removed')
					break
			for refined_place in irrelevant_refined_places:
				#print ( ' irrelevant refined palces ')+ refined_place
				if refined_place+')' in attribute and attribute in relevant_refined_attributes:
					relevant_refined_attributes.remove(attribute)
					#print( ' in attribute - removed')
					break
				if refined_place+',' in attribute and attribute in relevant_refined_attributes:
					relevant_refined_attributes.remove(attribute)
					#print( ' in attribute - removed')
					break


		coarse_places_string = ''
		if relevant_refined_places != []: coarse_places_string = '#coarse_place = {'+','.join(relevant_coarse_places)+'}.'
		coarse_object_string = ''
		coarse_thing_string = '#coarse_thing = #robot.'
		if relevant_coarse_objects != []:
			coarse_object_string = '#coarse_object = {'+','.join(relevant_coarse_objects)+'}.'
			coarse_thing_string = '#coarse_thing = #coarse_object + #robot.'
		robot_string = '#robot = {rob1}.'
		thing_string = '#thing = #robot.'
		objects_string = ''
		if relevant_refined_objects != []:
			objects_string = '#object = {'+','.join(relevant_refined_objects)+'}.'
			thing_string = '#thing = #object + #robot.'

		places_string = ''
		if relevant_refined_places != []: places_string = '#place = {'+','.join(relevant_refined_places)+'}.'
		sorts = '\n'.join([coarse_places_string,coarse_object_string,objects_string,places_string,robot_string,coarse_thing_string,thing_string])



		attributes = '\n'.join(relevant_refined_attributes)

		return sorts, attributes




	def infer_abstract_obs_from_refined_observations_zooming(self,refined_history, step):
		#print 'refine history: '
		#print '\n'.join(refined_history)
		history_index = self.preASP_inferring_indirect_observations_split.index(self.history_marker)
		new_preASP_inferring_indirect_observations_split = self.preASP_inferring_indirect_observations_split[:history_index +1] + refined_history +self.preASP_inferring_indirect_observations_split[history_index +1:]

		mySorts, myAttributes = self.find_relevant_sorts_and_attributes(refined_history)


		sorts_index = new_preASP_inferring_indirect_observations_split.index('%% SORTS GO HERE')
		new_preASP_inferring_indirect_observations_split[sorts_index] = mySorts
		attributes_index = new_preASP_inferring_indirect_observations_split.index('%% ATTRIBUTES GO HERE')
		new_preASP_inferring_indirect_observations_split[attributes_index] = myAttributes

		new_preASP_inferring_indirect_observations_split[0]= '#const numSteps = ' + str(step) +'.'
		asp = '\n'.join(new_preASP_inferring_indirect_observations_split)

		f1 = open(self.asp_inferring_indirect_observations_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI ' + global_variables.controller_type + ' : Inferring indirect coarse obs from refined history ')
		print self.asp_inferring_indirect_observations_file
		startTime = datetime.now()

		answer_set = subprocess.check_output('java -jar '+global_variables.sparc_path + ' ' + self.asp_inferring_indirect_observations_file +' -A ', shell=True)
		if answer_set == '' or answer_set =='\n':
			lineno = self.lineno()
			print 'Error set at line:  '+str(lineno)
			self.final_planning_time.value = self.planning_time.total_seconds()
			if answer_set == '': self.completeRun.value = global_variables.character_code_too_many_answers
			elif answer_set == '\n':  self.completeRun.value = global_variables.character_code_inconsistency
			self.error.set()
		timeTaken = datetime.now() - startTime
		self.planning_time += timeTaken
		observations = ((answer_set.rstrip().split('\n\n'))[0]).strip('{').strip('}').split(', ')
		return self.domain_info.indirectObservationsToObsSet(observations,self.current_step+1)


	def infer_abstract_obs_from_refined_observations_non_zooming(self,refined_history, step):
		history_index = self.preASP_inferring_indirect_observations_split.index(self.history_marker)
		new_preASP_inferring_indirect_observations_split = self.preASP_inferring_indirect_observations_split[:history_index +1] + refined_history +self.preASP_inferring_indirect_observations_split[history_index +1:]
		new_preASP_inferring_indirect_observations_split[0]= '#const numSteps = ' + str(step) +'.'

		sorts_marker = '%% SORTS GO HERE'
		attributes_marker = '%% ATTRIBUTES GO HERE'
		sorts_index = new_preASP_inferring_indirect_observations_split.index(sorts_marker)
		new_preASP_inferring_indirect_observations_split[sorts_index] = global_variables.refined_sorts_string[global_variables.complexity_level-1]+global_variables.sort_of_sorts_string
		attributes_index = new_preASP_inferring_indirect_observations_split.index(attributes_marker)
		new_preASP_inferring_indirect_observations_split[attributes_index] = global_variables.refined_attributes_string[global_variables.complexity_level-1]


		asp = '\n'.join(new_preASP_inferring_indirect_observations_split)

		f1 = open(self.asp_inferring_indirect_observations_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI ' + global_variables.controller_type + ' : Inferring indirect coarse obs from refined history ')
		print self.asp_inferring_indirect_observations_file
		startTime = datetime.now()
		answer_set = subprocess.check_output('java -jar '+global_variables.sparc_path + ' ' + self.asp_inferring_indirect_observations_file +' -A ', shell=True)
		if answer_set == '' or answer_set =='\n':
			lineno = self.lineno()
			print 'Error set at line:  '+str(lineno)
			if answer_set == '': self.completeRun.value = global_variables.character_code_too_many_answers
			elif answer_set == '\n': self.completeRun.value = global_variables.character_code_inconsistency
			self.final_planning_time.value = self.planning_time.total_seconds()
			self.error.set()
		timeTaken = datetime.now() - startTime
		self.planning_time += timeTaken
		observations = ((answer_set.rstrip().split('\n\n'))[0]).strip('{').strip('}').split(', ')
		return self.domain_info.indirectObservationsToObsSet(observations,self.current_step+1)




	def addObsRefinedDomain(self,obsList,max_steps):
		self.refined_planning_list = self.refined_planning_list[:self.zoomed_obs_index]+obsList+self.refined_planning_list[self.zoomed_obs_index:]
		self.refined_planning_list[0] = '#const numSteps = ' + str(max_steps) +'.'
		asp = '\n'.join(self.refined_planning_list)
		if global_variables.controller_type == 'zooming': f1 = open(self.asp_zoomed_domain_file, 'w')
		elif global_variables.controller_type == 'non_zooming': f1 = open(self.asp_non_zoomed_domain_file, 'w')
		f1.write(asp)
		f1.close()


	###########################################################################################
	### this function runs the zoomed asp held in the file zoomed_domain_file.
	### This file is updated with the refined history through the function addObsRefinedDomain
	###########################################################################################
	def runRefinedPlanning(self, abstract_action,refined_plan_step):
		myRefinedFile = ''
		if(global_variables.controller_type == 'zooming'): myRefinedFile = self.asp_zoomed_domain_file
		elif(global_variables.controller_type == 'non_zooming'): myRefinedFile = self.asp_non_zoomed_domain_file
		answer_set = ''

		print myRefinedFile
		answer_set = subprocess.check_output('java -jar '+global_variables.sparc_path + ' ' + myRefinedFile +' -A',shell=True)
		if answer_set == '':
			lineno = self.lineno()
			print 'Too many answers: '+str(lineno)
			self.final_planning_time.value = self.planning_time.total_seconds()
			self.completeRun.value = global_variables.character_code_too_many_answers
			self.error.set()

		chosenAnswer = ''
		numStepsPlanning = refined_plan_step+1 #starting one step ahead of the plan we have so far, if it has already startes
		if refined_plan_step == 0:
			numStepsPlanning = global_variables.number_steps_refined_planning[global_variables.complexity_level-1]
		while answer_set == '\n' and numStepsPlanning < global_variables.max_number_steps_refined_planning[global_variables.complexity_level -1]:
			numStepsPlanning +=1
			reader = open(myRefinedFile, 'r')
			my_text = reader.read()
			reader.close()
			my_text_split = my_text.split('\n')
			my_text_split[0] = '#const numSteps = '+ str(numStepsPlanning) +'.'
			writer = open(myRefinedFile, 'w')
			writer.write('\n'.join(my_text_split))
			writer.close()
			#print numStepsPlanning
			#print myRefinedFile
			answer_set = ''


			print myRefinedFile
			answer_set = subprocess.check_output('java -jar '+global_variables.sparc_path + ' ' + myRefinedFile +' -A',shell=True)
			if answer_set == '':
				lineno = self.lineno()
				print 'Too many answers: '+str(lineno)
				self.final_planning_time.value = self.planning_time.total_seconds()
				self.completeRun.value = global_variables.character_code_too_many_answers
				self.error.set()
		if answer_set == '\n' and maxStepsPlanning >= global_variables.max_number_steps_refined_planning[global_variables.complexity_level -1]:
			lineno = self.lineno()
			print 'Error set at line:  '+str(lineno)
			self.final_planning_time.value = self.planning_time.total_seconds()
			self.completeRun.value = global_variables.character_code_inconsistency
			self.error.set()
		answer_set_split = answer_set.rstrip().split('\n\n')
		if 'put_down' in abstract_action:
			for answer in answer_set_split:
				if 'put_down' in answer: return answer.strip('{').strip('}').split(', ')
		return answer_set_split[0].strip('{').strip('}').split(', ')

	def filteredPlainHistory(self,this_list):
		return [a for a in this_list if 'select' not in a and 'start' not in a and 'stop' not in a]

	def getIndexesRelevantToGoal(self):
		return Set([self.domain_info.LocationBook1_index, self.domain_info.In_handBook1_index, self.domain_info.LocationBook2_index, self.domain_info.In_handBook2_index, self.domain_info.LocationBook3_index, self.domain_info.In_handBook3_index])

	def runToIPlanning(self,input):
		startTime = datetime.now() # record the time taken to plan the abstract plan
		abstract_action = None
		current_asp_split = self.preASP_ToI_split[:self.index_beginning_history_ToI +1] + input + self.preASP_ToI_split[self.index_beginning_history_ToI +1:]
		current_asp_split[self.ToI_current_step_index +1] = 'current_step('+str(self.current_step)+').'
		current_asp_split[0] = "#const numSteps = "+str(self.number_toi_steps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(self.number_toi_steps)+". % maximum activity_length of an activity."
		current_asp_split[2] = "#const max_name = " + str(self.number_activities) + "."
		asp = '\n'.join(current_asp_split)
		f1 = open(self.asp_ToI_planning_file, 'w')
		f1.write(asp)
		f1.close()
		print self.asp_ToI_planning_file + ' \t\t Finding next intended action'
		answer_set = subprocess.check_output('java -jar '+global_variables.sparc_path + ' ' + self.asp_ToI_planning_file +' -A ',shell=True)
		if answer_set == '':
			lineno = self.lineno()
			print 'Error set at line:  '+str(lineno)
			self.final_planning_time.value = self.planning_time.total_seconds()
			self.completeRun.value = global_variables.character_code_too_many_answers
			self.error.set()

		max_step = int(self.current_step + self.max_plan_length+3)
		step_num = int(self.number_toi_steps)
		if (step_num < max_step): max_reached = True
		else: max_reached = False
		while( not ("intended_action" in answer_set) and not ("selected_goal_holds" in answer_set) and max_reached):
			current_asp_split[0] = "#const numSteps = "+str(self.number_toi_steps+1)+". % maximum number of steps."
			current_asp_split[1] = "#const max_len = "+str(self.number_toi_steps)+". % maximum activity_length of an activity."
			asp = '\n'.join(current_asp_split)
			f1 = open(self.asp_ToI_planning_file, 'w')
			f1.write(asp)
			f1.close()
			print self.asp_ToI_planning_file
			answer_set = subprocess.check_output('java -jar '+global_variables.sparc_path + ' ' + self.asp_ToI_planning_file +' -A ',shell=True)
			if answer_set == '':
				lineno = self.lineno()
				print 'Error set at line:  '+str(lineno)
				self.final_planning_time.value = self.planning_time.total_seconds()
				self.completeRun.value = global_variables.character_code_too_many_answers
				self.error.set()
			self.number_toi_steps +=1
			step_num = int(self.number_toi_steps)
			if (step_num < max_step): max_reached = True
			else: max_reached = False
		possibleAnswers = answer_set.rstrip().split('\n\n')
		chosenAnswer = possibleAnswers[0]
		print ('\nAbstract action plan:')
		print (chosenAnswer)
		if self.abstract_action_plan == '': self.abstract_action_plan = chosenAnswer
		endTime = datetime.now()
		self.last_abstract_planning_time = endTime - startTime
		self.planning_time += self.last_abstract_planning_time
		split_answer = chosenAnswer.strip('}').strip('{').split(', ')
		self.history_ToI_diagnosis = []
		self.believes_goal_holds = False
		for line in split_answer:
			if("intended_action" in line):
				abstract_action = line[16:line.rfind(',')]
			elif("selected_goal_holds" in line):
				self.believes_goal_holds = True
			else:
				self.history_ToI_diagnosis.append(line + '.')
		return abstract_action

	def diagnose(self):
		startTime = datetime.now()
		self.input_for_planning = []
		possibleDiagnosis = []
		input = list(self.history_ToI_diagnosis)
		input.append("explaining("+str(self.current_step)+").")
		current_asp_split = self.preASP_ToI_split[: self.index_beginning_history_ToI +1] + input +self.preASP_ToI_split[self.index_beginning_history_ToI +1:]
		current_asp_split[self.ToI_current_step_index +1] = 'current_step('+str(self.current_step)+').'
		current_asp_split[0] = "#const numSteps = "+str(self.number_toi_steps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(self.number_toi_steps)+". % maximum activity_length of an activity."
		current_asp_split[2] = "#const max_name = " + str(self.number_activities) + "."
		asp = '\n'.join(current_asp_split)
		f1 = open(self.asp_ToI_diagnosing_file, 'w')
		f1.write(asp)
		f1.close()
		print self.asp_ToI_diagnosing_file
		answer_set = subprocess.check_output('java -jar '+global_variables.sparc_path + ' ' + self.asp_ToI_diagnosing_file +' -A ',shell=True)
		if answer_set == '':
			lineno = self.lineno()
			print 'Error set at line:  '+str(lineno)
			self.final_planning_time.value = self.planning_time.total_seconds()
			self.completeRun.value = global_variables.character_code_too_many_answers
			self.error.set()
		answers = answer_set.rstrip().split('\n\n')
		endTime = datetime.now()
		timeTaken = endTime -  startTime

		if self.current_diagnosis in answer_set:
			for a in answers:
				if(self.current_diagnosis in a): chosenAnswer = a
		else:
			chosenAnswer = answers[0]
		split_diagnosis = chosenAnswer.strip('}').strip('{').split(', ')
		for line in split_diagnosis:
			if("number_unobserved" in line):
				newLine =line.replace("number_unobserved","explanation")
				self.input_for_planning.append(newLine + '.')
			elif("unobserved" in line):
				newLine = line.replace("unobserved", "occurs") + '.'
				self.input_for_planning.append(newLine)
				if(self.current_diagnosis != line):
					self.update_belief(line[line.find('(')+1:line.rfind(',')],[])
					self.current_diagnosis = line
					print ('ControllerToI - new diagnosis: ' + self.current_diagnosis)
			elif("selected_goal_holds" in line): pass
			elif(line == ""): pass
			else:
				self.input_for_planning.append(line + '.')

		self.lines_to_write.append('\nDiagnosis ' + str(self.current_diagnosis))
		self.lines_to_write.append('\nTime taken to diagnose : ' + str(timeTaken))
		self.planning_time += timeTaken
		return


	def preparePreASPStringLists(self):
		#preparing preASP_ToI_split and self.index_beginning_history_ToI
		reader = open(global_variables.file_name_preASP_ToI_planning, 'r')
		preASP_ToI = reader.read()
		reader.close()
		self.preASP_ToI_split = preASP_ToI.split('\n')
		index_goal = self.preASP_ToI_split.index(self.goal_marker)
		self.preASP_ToI_split.insert(index_goal+1, "holds(my_goal,I) :- "+ self.goal)
		self.index_beginning_history_ToI = self.preASP_ToI_split.index(self.history_marker)
		self.ToI_current_step_index = self.preASP_ToI_split.index(self.current_step_marker)

		reader = open(global_variables.file_name_preASP_abstract_belief, 'r')
		preASP_domain = reader.read()
		reader.close()
		self.preASP_abstract_domain_split = preASP_domain.split('\n')
		self.index_history_abstract_domain = self.preASP_abstract_domain_split.index(self.history_marker)

		reader = open(global_variables.file_name_preASP_inferring_indirect_observations,'r')
		preASP_inferring_indirect_observations = reader.read()
		reader.close()
		self.preASP_inferring_indirect_observations_split = preASP_inferring_indirect_observations.split('\n')

	def update_belief(self, action, obsList):
		if('start' in action or 'stop' in action): return
		possible_last_action = 'hpd(' +action+', 0).'
		obsList = [a[:a.rfind(',')] + ',1).' for a in obsList]
		input = list(self.domain_info.coarseStateToAbstractHoldsSet(self.belief,0)) + [possible_last_action] + obsList
		asp_belief_split = self.preASP_abstract_domain_split[:self.index_history_abstract_domain] + input + self.preASP_abstract_domain_split[self.index_history_abstract_domain+1:]
		asp_belief_split[0] = "#const numSteps = 1."
		asp = '\n'.join(asp_belief_split)
		f1 = open(self.asp_abstract_belief_file, 'w')
		f1.write(asp)
		f1.close()
		print self.asp_abstract_belief_file
		answer_set = subprocess.check_output('java -jar '+ global_variables.sparc_path + ' ' + self.asp_abstract_belief_file +' -A',shell=True)
		if answer_set == '\n':
			lineno = self.lineno()
			print 'Error set at line:  '+str(lineno)
			self.final_planning_time.value = self.planning_time.total_seconds()
			self.completeRun.value = global_variables.character_code_inconsistency
			self.error.set()
		answer_set = answer_set.rstrip().strip('{').strip('}')
		if 'holds' in answer_set: self.belief = self.domain_info.abstractAnswerToCoarseState(answer_set)
		print ('ControllerToI ' + global_variables.controller_type + ' : updated belief: ' + str(self.belief))


	def setInitialBelief(self,input):
		asp_belief_split = self.preASP_abstract_domain_split[:self.index_history_abstract_domain] + input + self.preASP_abstract_domain_split[self.index_history_abstract_domain+1:]
		asp_belief_split[0] = "#const numSteps = "+ str(self.current_step) + "."
		asp = '\n'.join(asp_belief_split)
		f1 = open(self.asp_abstract_belief_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI ' + global_variables.controller_type + ' : Setting initial belief')
		print self.asp_abstract_belief_file
		answer_set = subprocess.check_output('java -jar '+ global_variables.sparc_path + ' ' + self.asp_abstract_belief_file +' -A',shell=True)
		if answer_set == '' or answer_set == '\n':
			lineno = self.lineno()
			print 'Error set at line:  '+str(lineno)
			self.final_planning_time.value = self.planning_time.total_seconds()
			if answer_set == '': self.completeRun.value = global_variables.character_code_too_many_answers
			elif answer_set == '\n': self.completeRun.value = global_variables.character_code_inconsistency
			self.error.set()
		answer_set = answer_set.rstrip().strip('{').strip('}')
		self.belief = self.domain_info.abstractAnswerToCoarseState(answer_set)



	# this function uses the preASP_refined_Domain.txt file and SPARC to get a refined action plan
	def refine(self, action, refined_location):
		initial_state = Set()
		final_state = Set()
		coarse_location = self.belief[self.domain_info.LocationRobot_index]
		# use action and history to figure out the transition (initial_state, action, final_state)
		# the location of the robot is relevant for move transitions
		action_object = action[action.find(',')+1:-1]
		if 'move' in action:
			initial_state.add('coarse_loc(rob1,' + coarse_location + ')')
			final_state.add('coarse_loc(rob1,' + action_object + ')')
		elif 'pickup' in action:
			initial_state.add('-coarse_in_hand(rob1,' + action_object + ')')
			final_state.add('coarse_in_hand(rob1,' + action_object + ')')
			initial_state.add('coarse_loc(rob1,' + coarse_location + ')')
			initial_state.add('coarse_loc('+ action_object+','+ coarse_location+')')
		elif 'put_down' in action:
			initial_state.add('coarse_in_hand(rob1,' + action_object + ')')
			initial_state.add('in_hand(rob1,ref1_'+ action_object +')')
			final_state.add('-coarse_in_hand(rob1,' + action_object + ')')

		formatted_initial_state = Set()
		if(global_variables.controller_type == 'zooming'):
			formatted_initial_state = self.zoom(list(initial_state), action, list(final_state))
		elif(global_variables.controller_type == 'non_zooming'):
			formatted_initial_state = self.non_zoom(list(final_state))
		return formatted_initial_state




	# this function writes a zoomed ASP file
	def zoom(self,initial_state, action, final_state):
		# use complexity level to determine what is included in each sort
		total_coarse_places_list = ['library', 'kitchen', 'office1', 'office2', 'storage_cupboard']
		coarse_places_list = total_coarse_places_list[0:global_variables.complexity_level+1]
		total_coarse_objects_list = ['book1', 'book2', 'book3', 'book4']
		coarse_objects_list = total_coarse_objects_list[0:global_variables.complexity_level]
		total_places_list = ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'c13', 'c14', 'c15', 'c16', 'c17', 'c18', 'c19', 'c20', 'c21', 'c22', 'c23', 'c24', 'c25']
		places_list = total_places_list[0:(global_variables.complexity_level+1)**2]
		total_objects_list = ['ref1_book1', 'ref2_book1', 'ref1_book2', 'ref2_book2', 'ref3_book1', 'ref3_book2', 'ref1_book3', 'ref2_book3', 'ref3_book3', 'ref4_book1', 'ref4_book2', 'ref4_book3', 'ref1_book4', 'ref2_book4', 'ref3_book4', 'ref4_book4']
		objects_list = total_objects_list[0:global_variables.complexity_level**2]

		# set up sorts
		coarse_places = Sort('coarse_place', coarse_places_list)
		coarse_objects = Sort('coarse_object', coarse_objects_list)
		places = Sort('place', places_list)
		objects = Sort('object', objects_list)
		coarse_things = Sort('coarse_thing', ['#coarse_object', '#robot'])
		things = Sort('thing', ['#object', '#robot'])
		coarse_components = Sort('coarse_component', ['#coarse_place', '#coarse_object'])
		refined_components = Sort('refined_component', ['#place', '#object'])
		robots = Sort('robot', ['rob1'])
		sorts = [coarse_places, coarse_objects, places, objects, coarse_things, things, coarse_components, refined_components, robots]
		inertial_fluents = ['loc(#thing,#place)', 'in_hand(#robot,#object)']
		defined_fluents = ['coarse_loc(#coarse_thing,#coarse_place)', 'coarse_in_hand(#robot,#coarse_object)']
		actions = ['move(#robot,#place)', 'pickup(#robot,#object)', 'put_down(#robot,#object)']

		# use complexity level to determine which refined components are matched to which coarse-level object constants
		cell_num = 1
		library_components_list = []
		for i in range(global_variables.complexity_level+1):
			library_components_list.append('c'+str(cell_num))
			cell_num = cell_num + 1
		kitchen_components_list = []
		for i in range(global_variables.complexity_level+1):
			kitchen_components_list.append('c'+str(cell_num))
			cell_num = cell_num + 1
		office1_components_list = []
		for i in range(global_variables.complexity_level+1):
			office1_components_list.append('c'+str(cell_num))
			cell_num = cell_num + 1
		office2_components_list = []
		for i in range(global_variables.complexity_level+1):
			office2_components_list.append('c'+str(cell_num))
			cell_num = cell_num + 1
		storage_cupboard_components_list = []
		for i in range(global_variables.complexity_level+1):
			storage_cupboard_components_list.append('c'+str(cell_num))
			cell_num = cell_num + 1
		total_book1_components_list = ['ref1_book1', 'ref2_book1', 'ref3_book1', 'ref4_book1']
		book1_components_list = total_book1_components_list[0:global_variables.complexity_level]
		total_book2_components_list = ['ref1_book2', 'ref2_book2', 'ref3_book2', 'ref4_book2']
		book2_components_list = total_book2_components_list[0:global_variables.complexity_level]
		total_book3_components_list = ['ref1_book3', 'ref2_book3', 'ref3_book3', 'ref4_book3']
		book3_components_list = total_book3_components_list[0:global_variables.complexity_level]
		total_book4_components_list = ['ref1_book4', 'ref2_book4', 'ref3_book4', 'ref4_book4']
		book4_components_list = total_book4_components_list[0:global_variables.complexity_level]

		# match refined components to coarse-level object constants
		library_components = Components('library', library_components_list)
		kitchen_components = Components('kitchen', kitchen_components_list)
		office1_components = Components('office1', office1_components_list)
		office2_components = Components('office2', office2_components_list)
		storage_cupboard_components = Components('storage_cupboard', storage_cupboard_components_list)
		book1_components = Components('book1', book1_components_list)
		book2_components = Components('book2', book2_components_list)
		book3_components = Components('book3', book3_components_list)
		book4_components = Components('book4', book4_components_list)
		total_refinements = [library_components, kitchen_components, book1_components, office1_components, book2_components, office2_components, book3_components, storage_cupboard_components, book4_components]
		refinements = total_refinements[0:global_variables.complexity_level*2+1]

		# initialise relevance lists
		rel_initial_conditions = []
		rel_final_conditions = []
		rel_conditions = []
		rel_obj_consts = []
		rel_sorts = []
		rel_sort_names = ['#coarse_thing', '#thing']
		rel_inertial_fluents = []
		rel_defined_fluents = []
		rel_actions = ['test(#robot,#physical_inertial_fluent,#outcome)']

		# initialise irrelevance lists
		irrelevant_sort_names = ['#coarse_object', '#place', '#object', '#coarse_place']
		irrelevant_obj_consts = coarse_places.constants + coarse_objects.constants + places.constants + objects.constants
		for i in range(len(irrelevant_obj_consts)):
			if irrelevant_obj_consts[i] == 'c1': irrelevant_obj_consts[i] = 'c1,'
			elif irrelevant_obj_consts[i] == 'c2': irrelevant_obj_consts[i] = 'c2,'
		irrelevant_fluents = ['coarse_loc', 'coarse_in_hand', 'loc', 'in_hand']
		irrelevant_actions = ['move', 'pickup', 'put_down']

		# determine which initial conditions are relevant
		for condition in initial_state:
			if not condition in final_state: # conditions that change are relevant
				rel_initial_conditions.append(condition)
				rel_conditions.append(condition)
			elif ('rob1' in condition) and ('loc' in condition): # the robot's location is always relevant for pickup actions
				rel_initial_conditions.append(condition)
				rel_conditions.append(condition)

		# refine initial conditions

		for i in range(len(rel_initial_conditions)):
			if ('loc' in rel_initial_conditions[i]) and ('rob1' in rel_initial_conditions[i]):
				rel_initial_conditions[i] = 'loc(rob1,' + self.refined_location + ')'

		# determine which final conditions are relevant
		for condition in final_state:
			if not condition in initial_state:
				rel_final_conditions.append(condition)
				rel_conditions.append(condition)

		# determine which object constants are relevant
		for condition in rel_conditions:
			for index in range(len(condition)):
				if condition[index] == '(':
					opening_bracket = index
				elif condition[index] == ')':
					closing_bracket = index
			obj_consts = condition[opening_bracket+1:closing_bracket].split(',')
			for const in obj_consts:
				rel_obj_consts.append(const)
				if const in irrelevant_obj_consts:
					irrelevant_obj_consts.remove(const)
		rel_obj_consts = list(set(rel_obj_consts)) # remove duplicates

		# add refined components of relevant object constants
		for const in rel_obj_consts:
			for refinement in refinements:
				if const == refinement.name:
					for refined_const in refinement.components:
						rel_obj_consts.append(refined_const)
						if refined_const == 'c1': refined_const = 'c1,'
						elif refined_const == 'c2': refined_const = 'c2,'
						if refined_const in irrelevant_obj_consts:
							irrelevant_obj_consts.remove(refined_const)

		# sort relevant objects into types
		for sort in sorts:
			for const in sort.constants:
				if const in rel_obj_consts:
					sort.add(const)

		# determine which sorts should be included in the zoomed description
		for sort in sorts:
			if len(sort.rel_constants) != 0:
				rel_sorts.append(sort)
				rel_sort_names.append('#'+sort.name)
				if ('#'+sort.name) in irrelevant_sort_names:
					irrelevant_sort_names.remove('#'+sort.name)

		# add relevant sorts to sorts of sorts (coarse_things, things, coarse_components and refined_components)
		for sort in rel_sorts:
			for sort_of_sorts in sorts:
				if ('#'+sort.name) in sort_of_sorts.constants:
					sort_of_sorts.add('#'+sort.name)

		# determine which inertial fluents are relevant
		for fluent in inertial_fluents:
			fluent_relevant = True
			for index in range(len(fluent)):
				if fluent[index] == '(':
					opening_bracket = index
				elif fluent[index] == ')':
					closing_bracket = index
			fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
			for sort in fluent_sorts:
				if not sort in rel_sort_names:
					fluent_relevant = False
			if fluent_relevant:
				rel_inertial_fluents.append(fluent)
				if fluent[0:opening_bracket] in irrelevant_fluents:
					irrelevant_fluents.remove(fluent[0:opening_bracket])

		# determine which defined fluents are relevant
		for fluent in defined_fluents:
			fluent_relevant = True
			for index in range(len(fluent)):
				if fluent[index] == '(':
					opening_bracket = index
				elif fluent[index] == ')':
					closing_bracket = index
			fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
			for sort in fluent_sorts:
				if not sort in rel_sort_names:
					fluent_relevant = False
			if fluent_relevant:
				rel_defined_fluents.append(fluent)
				if fluent[0:opening_bracket] in irrelevant_fluents:
					irrelevant_fluents.remove(fluent[0:opening_bracket])

		# determine which actions are relevant
		for act in actions:
			action_relevant = True
			for index in range(len(act)):
				if act[index] == '(':
					opening_bracket = index
				elif act[index] == ')':
					closing_bracket = index
			action_sorts = act[opening_bracket+1:closing_bracket].split(',')
			for sort in action_sorts:
				if not sort in rel_sort_names:
					action_relevant = False
			if action_relevant:
				rel_actions.append(act)
				if act[0:opening_bracket] in irrelevant_actions:
					irrelevant_actions.remove(act[0:opening_bracket])


		# determine what the goal of the refined ASP should be
		goal = 'goal(I) :- '
		for condition in rel_final_conditions:
			if '-' in condition:
				condition = condition.replace('-', '')
				goal = goal + '-holds(' + condition + ',I), '
			else:
				goal = goal + 'holds(' + condition + ',I), '
		goal = goal[0:len(goal)-2] + '.'

		# make temporary copy of refined ASP file that can be edited
		original_asp_reader = open(global_variables.file_name_preASP_refined_planning, 'r')
		preASP_planning_split = (original_asp_reader.read()).split('\n')
		original_asp_reader.close()

		self.refined_planning_list = []

		for line in preASP_planning_split:
			if line == '%% GOAL GOES HERE': # put goal in
				self.refined_planning_list.append(goal)
			elif line == '%% HISTORY GOES HERE': # put initial conditions in
				for condition in rel_initial_conditions:
					if '-' in condition:
						condition = condition.replace('-', '')
						self.refined_planning_list.append('-holds(' + condition + ', 0).')
					else:
						self.refined_planning_list.append('holds(' + condition + ', 0).')
			elif ('cannot test in the first step' in line) and ('pickup' in action): # remove this axiom for pickup actions, as the object's location needs to be tested at step zero
				pass
			elif ('#coarse_place =' in line) or ('#coarse_object =' in line) or ('#object =' in line) or ('#place =' in line):
				# add relevant constants to sort
				zoomed_sort = ''
				for sort in sorts:
					if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
						zoomed_sort = '#' + sort.name + ' = {'
						for const in sort.rel_constants:
							zoomed_sort = zoomed_sort + const + ', '
						zoomed_sort = zoomed_sort[0:len(zoomed_sort)-2] + '}.'
						self.refined_planning_list.append(zoomed_sort)
			elif ('#coarse_thing =' in line) or ('#thing =' in line) or ('#refined_component =' in line) or ('#coarse_component =' in line):
				# add relevant sorts
				zoomed_sort = ''
				for sort in sorts:
					if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
						zoomed_sort = '#' + sort.name + ' = '
						for const in sort.rel_constants:
							zoomed_sort = zoomed_sort + const + ' + '
						zoomed_sort = zoomed_sort[0:len(zoomed_sort)-3] + '.'
						self.refined_planning_list.append(zoomed_sort)
			elif '#physical_inertial_fluent =' in line: # add relevant inertial fluents
				inertial_fluent_sort = '#physical_inertial_fluent = '
				for fluent in rel_inertial_fluents:
					inertial_fluent_sort = inertial_fluent_sort + fluent + ' + '
				inertial_fluent_sort = inertial_fluent_sort[0:len(inertial_fluent_sort)-3] + '.'
				self.refined_planning_list.append(inertial_fluent_sort)
			elif '#physical_defined_fluent =' in line: # add relevant defined fluents
				defined_fluent_sort = '#physical_defined_fluent = '
				for fluent in rel_defined_fluents:
					defined_fluent_sort = defined_fluent_sort + fluent + ' + '
				defined_fluent_sort = defined_fluent_sort[0:len(defined_fluent_sort)-3] + '.'
				self.refined_planning_list.append(defined_fluent_sort)
			elif '#action =' in line: # add relevant actions
				action_sort = '#action = '
				for act in rel_actions:
					action_sort = action_sort + act + ' + '
				action_sort = action_sort[0:len(action_sort)-3] + '.'
				self.refined_planning_list.append(action_sort)
			else:
				line_relevant = True

				for sort in irrelevant_sort_names: # don't include predicates with irrelevant sorts
					if sort in line: line_relevant = False
				for const in irrelevant_obj_consts: # don't include attributes with irrelevant object constants
					if const in line: line_relevant = False
				for fluent in irrelevant_fluents: # don't include axioms with irrelevant fluents
					if fluent in line: line_relevant = False
				for act in irrelevant_actions: # don't include axioms with irrelevant actions
					if act in line:	line_relevant = False
				if line_relevant: self.refined_planning_list.append(line)

		self.zoomed_obs_index = self.refined_planning_list.index('%% End of History:') - 2


		asp = '\n'.join(self.refined_planning_list)
		zoomed_asp_writter = open(self.asp_zoomed_domain_file, 'w')
		zoomed_asp_writter.write(asp)
		zoomed_asp_writter.close()
		formatted_initial_state = Set()
		for condition in rel_initial_conditions:
			if '-' in condition: formatted_initial_state.add('-holds('+condition.replace('-','')+',0).')
			else: formatted_initial_state.add('holds('+condition+',0).')
		return formatted_initial_state


	def non_zoom(self, final_state):
		initial_state = self.domain_info.coarseStateToCoarseHoldsSet(self.belief, 0)
		initial_state.add('holds(loc(rob1,'+self.refined_location+'),0).')
		for i, entry in enumerate(final_state):
			final_state[i] = 'holds(' + entry + ', I)'
		reader = open(global_variables.file_name_preASP_refined_planning, 'r')
		refined_planning = reader.read()
		self.refined_planning_list = refined_planning.split('\n')
		goal = 'goal(I) :- ' + (',').join(final_state) + '.'
		goal_index = self.refined_planning_list.index('%% GOAL GOES HERE')
		self.refined_planning_list[goal_index] = goal
		history_index = self.refined_planning_list.index('%% HISTORY GOES HERE')
		self.refined_planning_list = self.refined_planning_list[:history_index] + list(initial_state) + self.refined_planning_list[history_index+1:]
		self.zoomed_obs_index = self.refined_planning_list.index('%% End of History:') - 2
		writer = open(self.asp_non_zoomed_domain_file, 'w')
		writer.write('\n'.join(self.refined_planning_list))
		writer.close()
		return initial_state

	def lineno(self):
		return inspect.currentframe().f_back.f_lineno

# Defines the sorts that may be included in a zoomed description
class Sort():
	def __init__(self, name, constants):
		self.name = name
		self.constants = constants
		self.rel_constants = []
	def add(self, constant):
		self.rel_constants.append(constant)

# Defines the refined components of each coarse object constant
class Components():
	def __init__(self, name, components):
		self.name = name
		self.components = components
