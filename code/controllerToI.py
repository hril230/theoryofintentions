from datetime import datetime
from sets import Set
import subprocess
import re
import numpy
import random
import sys
import global_variables
from itertools import groupby
import time

class ControllerToI():
	def __init__(self, sparc_path, ASP_subfolder, domain_info, executer, refined_location, initial_conditions , goal, max_plan_length):
		self.lines_to_write = []
		self.planning_times = []
		self.abstract_planning_time = None
		self.abstract_action_plan = ''
		self.error = False

		self.goal = goal
		self.sparc_path = sparc_path
		self.executer = executer
		self.domain_info = domain_info
		self.refined_state = self.domain_info.refined_state[:]
		self.max_plan_length = max_plan_length

		## used for creating ASP files
		self.preASP_ToI_file = ASP_subfolder+'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_ToI.txt' # Unsed for planning and diagnosis with ToI
		self.preASP_refined_domain_file = ASP_subfolder + 'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_refined_domain.txt' # used for zooming
		self.preASP_abstract_domain_file = ASP_subfolder + 'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_abstract_domain.txt' # used for updating controller Belief
		self.preASP_refined_domain_no_planning_file = ASP_subfolder+ 'pre_ASP_files/complexity_level_' + str(global_variables.complexity_level) + '/preASP_refined_domain_no_planning.txt' # used for infering coarse observations

		## used for paths and names of the ASP files created
		self.asp_ToI_planning_file = ASP_subfolder+'ASP_files/ASP_ToI_Planning.sp'
		self.asp_ToI_diagnosing_file = ASP_subfolder + 'ASP_files/ASP_ToI_Diagnosis.sp'
		self.zoomed_domain_file = ASP_subfolder + 'ASP_files/Zoomed_Domain.sp'
		self.asp_abstract_belief_file = ASP_subfolder + 'ASP_files/ToI_Abstract_Belief.sp'
		self.asp_infering_coarse_belief_file = ASP_subfolder + 'ASP_files/Infering_Coarse_Belief.sp'

		# holds the markers in the text files.
		self.goal_marker = '%% GOAL GOES HERE'
		self.current_step_marker = '%% CURRENT STEP GOES HERE'
		self.history_marker = '%% HISTORY GOES HERE'

		# variables relevant for the ASP_ToI planning and diagnosis
		self.number_steps = 4 #initial total number of steps the controller assumes it will need for planning
		self.number_activities = 1 #keeping record of the number of activitites the ASP needs
		self.goal_correction = 0 #keeping record of the number of times the goal has been assumed to be true but it was False
		self.current_diagnosis = '' #keeping the latest diagnosis
		self.input_for_planning = [] #holds the input necessary to get the next intended action in the ASP_TOI_Planning
		self.current_step = 1 # holds the current step of the controllerToI, which is the same as the ASP_TOI_Planning

		self.preparePreASPStringLists()
		self.setInitialBelief(self.filteredPlainHistory(initial_conditions))
		self.history_ToI_diagnosis = initial_conditions #holds the history input for ASP_ToI_Diagnosis
		self.refined_location = refined_location

		print ('ControllerToI \t\t initial coarse belief: ' + str(self.belief))
		print ('Controller ToI \t\t initial refined location: ' + str(self.refined_location))


	def run(self):
		self.believes_goal_holds = False
		self.history_ToI_diagnosis.append("hpd(select(my_goal), true,0).")
		self.diagnose()
		finish = False # flag that breaks the loop that calls the ASP_ToI_Planning when finish == True
		while(finish == False and not self.error):
			abstract_action = self.runToIPlanning(self.input_for_planning)
			if(abstract_action == 'finish'):
				#check if the assuption that the goal has been reached is true.
				if(self.executer.getGoalFeedback() == True):
					self.history_ToI_diagnosis.append('finish')
					print('ControllerToI: \t Belief: ' + str(self.belief))
					print('ControllerToI: \t Feedback from the workd: Belief is True')
					print('ControllerToI: \t Finish')
					finish = True
					break
				# if it is false, count the correction and diagnose again
				else:
					self.goal_correction += 1
					while(abstract_action == 'finish'):
						print('ControllerToI: \t Belief: ' + str(self.belief))
						print('ControllerToI: \t Feedback from the workd: Belief is False')
						self.history_ToI_diagnosis.append('obs(my_goal,false,'+str(self.current_step)+').')
						self.diagnose()
						abstract_action = self.runToIPlanning(self.input_for_planning)
			if(abstract_action == None):
				self.history_ToI_diagnosis.append("Goal is futile")
				print ('ControllerToI: \t Goal is futile ')
				finish = True
				break

			# if action is 'stop' it is because an unexpected action has happened and it needs to replan, so the number_activities should increase
			# to pass this as an input to the ASP_ToI_Planning
			if(abstract_action[0:4] == 'stop'):
				self.number_activities += 1
				self.number_steps += 1
			elif(abstract_action[0:5] == 'start'):
				self.lines_to_write.append('\nZooming controller abstract action plan: ')
				self.lines_to_write.append(self.abstract_action_plan)
				self.lines_to_write.append('\nTime taken to plan abstract action plan: ' + str(self.abstract_planning_time))
				self.planning_times.append(self.abstract_planning_time)
			else:
				print ('\nControllerToI \t\t ref location and coarse belief: ' +self.refined_location + ', '+ str(self.belief))
				print ('ControllerToI \t\t next abstract action: ' + abstract_action)
				need_refined_plan = True
				self.refine(abstract_action, self.refined_location)
				refined_plan_step = 0
				zoomed_history = Set() #holds all the history of the refined plan created wtih zoomed refined domain in form of 'hpd' and 'obs'
				refined_observations = Set() #holds only the direct observations made during the execution of refined plan in form of 'holds(directly_observed...)'
				initial_refined_location = self.refined_location #holds the refined location before the refined plan is executed. It is used as initial
																 #condition for the ASP that infers coarse observations from the refined direct observations	
				while(need_refined_plan and not self.error):
					startTime = datetime.now() # record the time taken to plan the refined plan
					refined_occurrences = self.runZoomedDomain()
					endTime = datetime.now()
					timeTaken = endTime - startTime
					need_refined_plan = False
					if (refined_occurrences == ['']) or (refined_occurrences == None):
						print ('ControllerToI \t\t no refined plan ')
						self.error = True
						break
					refined_occurrences.sort(key=self.getStep)
					print ('ControllerToI \t\t refined plan: ' + str(refined_occurrences[refined_plan_step:]))
					# store refined plan
					self.lines_to_write.append('\nAbstract action: ' + str(abstract_action))
					self.lines_to_write.append('\nRefined plan (computed with zooming): ' + str(refined_occurrences))
					self.lines_to_write.append('\nTime taken to create refined plan: ' + str(timeTaken))
					self.planning_times.append(timeTaken)
					for i in range(len(refined_occurrences)):
						refined_occurence = refined_occurrences[i]
						refined_action = refined_occurence[refined_occurence.find('(')+1 : refined_occurence.rfind(',')]
						occurrence_step = int(refined_occurence[refined_occurence.rfind(',')+1:refined_occurence.rfind(')')])
						if occurrence_step != refined_plan_step: continue
						print ('ControllerToI \t\t Refined action: ' + str(refined_action) + ' at refined step: ' + str(occurrence_step))
						if 'test' in refined_action:
							action_test_result, action_observation = self.executer.test(refined_action)
							zoomed_history.add('hpd(' + refined_action + ',' + str(refined_plan_step) +').')
							refined_plan_step += 1
							print ('ControllerToI \t\t test result: ' + str(action_test_result))
							if(action_test_result==True and 'loc(rob1' in refined_action):
								self.refined_location = action_observation.split(',')[2][:-1]
								print ('ControllerToI \t\t refined location: '+self.refined_location)

							refined_observations_step = self.getObservationsRelevantGoal()
							refined_observations_step.add(action_observation)
							refined_observations_step = self.prepareRefinedObservations(refined_observations_step,refined_plan_step)
							refined_observations = refined_observations.union(refined_observations_step)
							zoomed_observation_step = self.domain_info.directObservationToRefinedObs(action_observation, refined_plan_step)
							zoomed_history.add(zoomed_observation_step)
							if(action_test_result == False):
								print ('ControllerToI \t\t need another refined plan ')
								need_refined_plan = True
								self.addObsZoomedDomain(list(zoomed_history),refined_plan_step+6)
								break
						else:
							previous_belief = self.belief[:]
							self.executer.executeAction(refined_action)
							self.belief = previous_belief
							refined_plan_step += 1
				abstract_step_obs = list(self.infer_abstract_obs_from_refined_observations(initial_refined_location,list(refined_observations), refined_plan_step))
				print ('ControllerToI: \t Abstract action ' +abstract_action+' has finished at step ' + str(self.current_step))
				self.refined_state = self.domain_info.refined_state[:]
				print ('ControllerToI \t Abstract obs: ' + ': ' + str(abstract_step_obs))
				self.update_belief(abstract_action, abstract_step_obs)
				self.history_ToI_diagnosis = self.history_ToI_diagnosis + abstract_step_obs
			self.history_ToI_diagnosis.append('attempt('+abstract_action+','+str(self.current_step)+').')
			self.current_step += 1
			self.diagnose()
		if(self.current_diagnosis != ''): self.history_ToI_diagnosis.append(self.current_diagnosis)

		# write results to file
		timeTaken = self.planning_times[0]
		for i in range(1,len(self.planning_times)): timeTaken = timeTaken + self.planning_times[i]
		if self.error: timeTaken = 'ERROR: too many answer sets to process'
		results = open('experimental_results.txt', 'a')
		for line in self.lines_to_write: results.write(line)
		results.write('\nTotal time taken with zooming: ')
		results.write(str(timeTaken))
		results.close()


	def getStep(self,occurrence):
		return int(occurrence[occurrence.rfind(',')+1:-1])

	def prepareRefinedObservations(self,observations,step):
		if observations == Set(['']) or not observations: return observations
		updated_observations = Set()
		for observation in observations:
			updated_observations.add(observation[:observation.rfind(',')+1] + str(step) + ').')
		return updated_observations

	def getObservationsRelevantGoal(self):
		observations = Set()
		result,observation1 = self.executer.test('test(rob1,loc(ref1_book1,' + self.refined_location+ '),true)')
		if 'holds' in observation1: observations.add(observation1)
		if global_variables.complexity_level > 1:
			result,observation2 = self.executer.test('test(rob1,loc(ref1_book2,' + self.refined_location+ '),true)')
			if 'holds' in observation2: observations.add(observation2)
		if global_variables.complexity_level > 2:
			result,observation3 = self.executer.test('test(rob1,loc(ref1_book3,' + self.refined_location+ '),true)')
			if 'holds' in observation3: observations.add(observation3)
		if global_variables.complexity_level > 3:
			result,observation4 = self.executer.test('test(rob1,loc(ref1_book4,' + self.refined_location+ '),true)')
			if 'holds' in observation4: observations.add(observation4)
		return observations

	def infer_abstract_obs_from_refined_observations(self,initial_refined_location,refined_observations_list, step):
		initial_state = list(self.domain_info.refinedStateToRefinedHoldsSet(self.refined_state,0)) + ['holds(loc(rob1,'+initial_refined_location+'),0).']
		history_index = self.preASP_infering_coarse_belief_split.index(self.history_marker)
		new_preASP_infering_coarse_belief_split = self.preASP_infering_coarse_belief_split[:history_index +1] + initial_state + refined_observations_list +self.preASP_infering_coarse_belief_split[history_index +1:]
		new_preASP_infering_coarse_belief_split[0]= '#const numSteps = ' + str(step) +'.'
		asp = '\n'.join(new_preASP_infering_coarse_belief_split)
		f1 = open(self.asp_infering_coarse_belief_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI: Inferring coarse obs from refined history ')
		answer_set = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_infering_coarse_belief_file +' -A ', shell=True)
		if not '{' in answer_set:
			self.error = True
			return
		observations = ((answer_set.rstrip().split('\n\n'))[0]).strip('{').strip('}').split(', ')
		return self.domain_info.indirectObservationsToObsSet(observations,self.current_step+1)

	def getStep(self, observation):
		return int(observation[observation.rfind(',')+1:-1])


	def addObsZoomedDomain(self,obsList,max_steps):
		self.zoomed_asp_list = self.zoomed_asp_list[:self.zoomed_obs_index]+obsList+self.zoomed_asp_list[self.zoomed_obs_index:]
		self.zoomed_asp_list[0] = '#const numSteps = ' + str(max_steps) +'.'
		asp = ''.join(self.zoomed_asp_list)
		f1 = open(self.zoomed_domain_file, 'w')
		f1.write(asp)
		f1.close()

	###########################################################################################
	### this function runs the zoomed asp held in the file zoomed_domain_file.
	### This file is updated with the refined history through the function addObsZoomedDomain
	###########################################################################################
	def runZoomedDomain(self):
		'\nControllerToI: Running zoomed domain to get refined plan '
		answer_set = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.zoomed_domain_file +' -A',shell=True)
		if not '{' in answer_set:
			self.error = True
			return
		plan = ((answer_set.rstrip().split('\n\n'))[0]).strip('{').strip('}').split(', ')
		return plan

	def filteredPlainHistory(self,this_list):
		return [a for a in this_list if 'select' not in a and 'start' not in a and 'stop' not in a]

	def getIndexesRelevantToGoal(self):
		return Set([self.domain_info.LocationBook1_index, self.domain_info.In_handBook1_index, self.domain_info.LocationBook2_index, self.domain_info.In_handBook2_index, self.domain_info.LocationBook3_index, self.domain_info.In_handBook3_index])

	def runToIPlanning(self,input):
		startTime = datetime.now() # record the time taken to plan the abstract plan
		abstract_action = None
		current_asp_split = self.preASP_ToI_split[:self.index_beginning_history_ToI +1] + input + self.preASP_ToI_split[self.index_beginning_history_ToI +1:]
		current_asp_split[self.ToI_current_step_index +1] = 'current_step('+str(self.current_step)+').'
		current_asp_split[0] = "#const numSteps = "+str(self.number_steps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(self.number_steps)+". % maximum activity_length of an activity."
		current_asp_split[2] = "#const max_name = " + str(self.number_activities) + "."
		asp = '\n'.join(current_asp_split)
		f1 = open(self.asp_ToI_planning_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI: \t\t Finding next intended action')
		answerSet = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_ToI_planning_file +' -A ',shell=True)
		max_step = int(self.current_step + self.max_plan_length+3)
		step_num = int(self.number_steps)
		if (step_num < max_step): max_reached = True
		else: max_reached = False
		while( not ("intended_action" in answerSet) and not ("selected_goal_holds" in answerSet) and max_reached):
			current_asp_split[0] = "#const numSteps = "+str(self.number_steps+1)+". % maximum number of steps."
			current_asp_split[1] = "#const max_len = "+str(self.number_steps)+". % maximum activity_length of an activity."
			asp = '\n'.join(current_asp_split)
			f1 = open(self.asp_ToI_planning_file, 'w')
			f1.write(asp)
			f1.close()
			print ('\nControllerToI: Increasing variable number_steps and creating a ToI Plan')
			answerSet = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_ToI_planning_file +' -A ',shell=True)
			self.number_steps +=1
			step_num = int(self.number_steps)
			if (step_num < max_step): max_reached = True
			else: max_reached = False
		possibleAnswers = answerSet.rstrip().split('\n\n')
		chosenAnswer = possibleAnswers[0]
		print ('\nAbstract action plan:')
		print (chosenAnswer)
		if self.abstract_action_plan == '': self.abstract_action_plan = chosenAnswer
		endTime = datetime.now()
		if self.abstract_planning_time == None: self.abstract_planning_time = endTime - startTime
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
		self.input_for_planning = []
		possibleDiagnosis = []
		input = list(self.history_ToI_diagnosis)
		input.append("explaining("+str(self.current_step)+").")
		current_asp_split = self.preASP_ToI_split[: self.index_beginning_history_ToI +1] + input +self.preASP_ToI_split[self.index_beginning_history_ToI +1:]
		current_asp_split[self.ToI_current_step_index +1] = 'current_step('+str(self.current_step)+').'
		current_asp_split[0] = "#const numSteps = "+str(self.number_steps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(self.number_steps)+". % maximum activity_length of an activity."
		current_asp_split[2] = "#const max_name = " + str(self.number_activities) + "."
		asp = '\n'.join(current_asp_split)
		f1 = open(self.asp_ToI_diagnosing_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI: Diagnosing ToI with current obs')
		answerSet = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_ToI_diagnosing_file +' -A ',shell=True)
		answers = answerSet.rstrip().split('\n\n')
		if self.current_diagnosis in answerSet:
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
					print ('controllerToI			new diagnosis: ' + self.current_diagnosis)
			elif("selected_goal_holds" in line): pass
			elif(line == ""): pass
			else:
				self.input_for_planning.append(line + '.')

		return


	def preparePreASPStringLists(self):
		#preparing preASP_ToI_split and self.index_beginning_history_ToI
		reader = open(self.preASP_ToI_file, 'r')
		preASP_ToI = reader.read()
		reader.close()
		self.preASP_ToI_split = preASP_ToI.split('\n')
		index_goal = self.preASP_ToI_split.index(self.goal_marker)
		self.preASP_ToI_split.insert(index_goal+1, "holds(my_goal,I) :- "+ self.goal)
		self.index_beginning_history_ToI = self.preASP_ToI_split.index(self.history_marker)
		self.ToI_current_step_index = self.preASP_ToI_split.index(self.current_step_marker)

		reader = open(self.preASP_abstract_domain_file, 'r')
		preASP_domain = reader.read()
		reader.close()
		self.preASP_abstract_domain_split = preASP_domain.split('\n')
		self.index_history_abstract_domain = self.preASP_abstract_domain_split.index(self.history_marker)

		reader = open(self.preASP_refined_domain_no_planning_file,'r')
		preASP_infering_coarse_belief = reader.read()
		reader.close()
		self.preASP_infering_coarse_belief_split = preASP_infering_coarse_belief.split('\n')

	def update_belief(self, action, obsList):
		if('start' in action or 'stop' in action): return
		possible_last_action = 'hpd(' +action+', 0).'
		obsList = [a[:a.rfind(',')] + ',1).' for a in obsList]
		input = list(self.domain_info.coarseStateToAstractHoldsSet(self.belief,0)) + [possible_last_action] + obsList
		asp_belief_split = self.preASP_abstract_domain_split[:self.index_history_abstract_domain] + input + self.preASP_abstract_domain_split[self.index_history_abstract_domain+1:]
		asp_belief_split[0] = "#const numSteps = 1."
		asp = '\n'.join(asp_belief_split)
		f1 = open(self.asp_abstract_belief_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI: Updating belief')
		output = subprocess.check_output('java -jar '+ self.sparc_path + ' ' + self.asp_abstract_belief_file +' -A',shell=True)
		if not '{' in output:
			self.error = True
			return
		output = output.rstrip().strip('{').strip('}')
		if 'holds' in output: self.belief = self.domain_info.abstractAnswerToCoarseState(output)
		print ('ControllerToI \t\t updated belief: ' + str(self.belief))


	def setInitialBelief(self,input):
		asp_belief_split = self.preASP_abstract_domain_split[:self.index_history_abstract_domain] + input + self.preASP_abstract_domain_split[self.index_history_abstract_domain+1:]
		asp_belief_split[0] = "#const numSteps = "+ str(self.current_step) + "."
		asp = '\n'.join(asp_belief_split)
		f1 = open(self.asp_abstract_belief_file, 'w')
		f1.write(asp)
		f1.close()
		print ('\nControllerToI: Setting initial belief')
		output = subprocess.check_output('java -jar '+ self.sparc_path + ' ' + self.asp_abstract_belief_file +' -A',shell=True)
		output = output.rstrip().strip('{').strip('}')
		self.belief = self.domain_info.abstractAnswerToCoarseState(output)



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
			final_state.add('-coarse_in_hand(rob1,' + action_object + ')')
		self.zoom(list(initial_state), action, list(final_state))


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
			if ('loc' in rel_initial_conditions[i]) and ('book1' in rel_initial_conditions[i]):
				rel_initial_conditions[i] = 'loc(ref1_book1,' + self.domain_info.refined_state[self.domain_info.LocationBook1_index] + ')'
			if ('loc' in rel_initial_conditions[i]) and ('book2' in rel_initial_conditions[i]):
				rel_initial_conditions[i] = 'loc(ref1_book2,' + self.domain_info.refined_state[self.domain_info.LocationBook2_index] + ')'
			if ('loc' in rel_initial_conditions[i]) and ('book3' in rel_initial_conditions[i]):
				rel_initial_conditions[i] = 'loc(ref1_book3,' + self.domain_info.refined_state[self.domain_info.LocationBook3_index] + ')'
			if ('loc' in rel_initial_conditions[i]) and ('book4' in rel_initial_conditions[i]):
				rel_initial_conditions[i] = 'loc(ref1_book4,' + self.domain_info.refined_state[self.domain_info.LocationBook4_index] + ')'
			if ('in_hand' in rel_initial_conditions[i]) and (not '-' in rel_initial_conditions[i]):
				currently_holding = ''
				if(self.domain_info.refined_state[self.domain_info.In_handBook1_Ref1_index] == 'true'):
					currently_holding = 'ref1_book1'
				if global_variables.complexity_level > 1:
					if(self.domain_info.refined_state[self.domain_info.In_handBook1_Ref2_index] == 'true'):
						currently_holding = 'ref2_book1'
					if(self.domain_info.refined_state[self.domain_info.In_handBook2_Ref1_index] == 'true'):
						currently_holding = 'ref1_book2'
					if(self.domain_info.refined_state[self.domain_info.In_handBook2_Ref2_index] == 'true'):
						currently_holding = 'ref2_book2'
				if global_variables.complexity_level > 2:
					if(self.domain_info.refined_state[self.domain_info.In_handBook1_Ref3_index] == 'true'):
						currently_holding = 'ref3_book1'
					if(self.domain_info.refined_state[self.domain_info.In_handBook2_Ref3_index] == 'true'):
						currently_holding = 'ref3_book2'
					if(self.domain_info.refined_state[self.domain_info.In_handBook3_Ref1_index] == 'true'):
						currently_holding = 'ref1_book3'
					if(self.domain_info.refined_state[self.domain_info.In_handBook3_Ref2_index] == 'true'):
						currently_holding = 'ref2_book3'
					if(self.domain_info.refined_state[self.domain_info.In_handBook3_Ref3_index] == 'true'):
						currently_holding = 'ref3_book3'
				if global_variables.complexity_level > 3:
					if(self.domain_info.refined_state[self.domain_info.In_handBook1_Ref4_index] == 'true'):
						currently_holding = 'ref4_book1'
					if(self.domain_info.refined_state[self.domain_info.In_handBook2_Ref4_index] == 'true'):
						currently_holding = 'ref4_book2'
					if(self.domain_info.refined_state[self.domain_info.In_handBook3_Ref4_index] == 'true'):
						currently_holding = 'ref4_book3'
					if(self.domain_info.refined_state[self.domain_info.In_handBook4_Ref1_index] == 'true'):
						currently_holding = 'ref1_book4'
					if(self.domain_info.refined_state[self.domain_info.In_handBook4_Ref2_index] == 'true'):
						currently_holding = 'ref2_book4'
					if(self.domain_info.refined_state[self.domain_info.In_handBook4_Ref3_index] == 'true'):
						currently_holding = 'ref3_book4'
					if(self.domain_info.refined_state[self.domain_info.In_handBook4_Ref4_index] == 'true'):
						currently_holding = 'ref4_book4'
				if(currently_holding != ''): rel_initial_conditions[i] = 'in_hand(rob1,' + currently_holding + ')'

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
		self.goal = 'goal(I) :- '
		for condition in rel_final_conditions:
			if '-' in condition:
				condition = condition.replace('-', '')
				self.goal = self.goal + '-holds(' + condition + ',I), '
			else:
				self.goal = self.goal + 'holds(' + condition + ',I), '
		self.goal = self.goal[0:len(self.goal)-2] + '.\n'

		# make temporary copy of refined ASP file that can be edited
		original_asp_reader = open(self.preASP_refined_domain_file, 'r')
		self.zoomed_asp_list = []
		zoomed_asp_writter = open(self.zoomed_domain_file, 'w')

		for line in original_asp_reader:
			if line == '%% GOAL GOES HERE\n': # put goal in
				self.zoomed_asp_list.append(self.goal)
			elif line == '%% HISTORY GOES HERE\n': # put initial conditions in
				for condition in rel_initial_conditions:
					if '-' in condition:
						condition = condition.replace('-', '')
						self.zoomed_asp_list.append('-holds(' + condition + ', 0).\n')
					else:
						self.zoomed_asp_list.append('holds(' + condition + ', 0).\n')
			elif ('cannot test in the first step' in line) and ('pickup' in action): # remove this axiom for pickup actions, as the object's location needs to be tested at step zero
				pass
			elif 'ZOOM THIS SORT:' in line: # add relevant constants to sort
				zoomed_sort = ''
				for sort in sorts:
					if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
						zoomed_sort = '#' + sort.name + ' = {'
						for const in sort.rel_constants:
							zoomed_sort = zoomed_sort + const + ', '
						zoomed_sort = zoomed_sort[0:len(zoomed_sort)-2] + '}.\n'
						self.zoomed_asp_list.append(zoomed_sort)
			elif 'ZOOM THIS SORT OF SORTS' in line: # add relevant sorts
				zoomed_sort = ''
				for sort in sorts:
					if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
						zoomed_sort = '#' + sort.name + ' = '
						for const in sort.rel_constants:
							zoomed_sort = zoomed_sort + const + ' + '
						zoomed_sort = zoomed_sort[0:len(zoomed_sort)-3] + '.\n'
						self.zoomed_asp_list.append(zoomed_sort)
			elif 'ZOOM INERTIAL FLUENTS' in line: # add relevant inertial fluents
				inertial_fluent_sort = '#physical_inertial_fluent = '
				for fluent in rel_inertial_fluents:
					inertial_fluent_sort = inertial_fluent_sort + fluent + ' + '
				inertial_fluent_sort = inertial_fluent_sort[0:len(inertial_fluent_sort)-3] + '.\n'
				self.zoomed_asp_list.append(inertial_fluent_sort)
			elif 'ZOOM DEFINED FLUENTS' in line: # add relevant defined fluents
				defined_fluent_sort = '#physical_defined_fluent = '
				for fluent in rel_defined_fluents:
					defined_fluent_sort = defined_fluent_sort + fluent + ' + '
				defined_fluent_sort = defined_fluent_sort[0:len(defined_fluent_sort)-3] + '.\n'
				self.zoomed_asp_list.append(defined_fluent_sort)
			elif 'ZOOM ACTIONS' in line: # add relevant actions
				action_sort = '#action = '
				for act in rel_actions:
					action_sort = action_sort + act + ' + '
				action_sort = action_sort[0:len(action_sort)-3] + '.\n'
				self.zoomed_asp_list.append(action_sort)
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
				if line_relevant: self.zoomed_asp_list.append(line)

		self.zoomed_obs_index = self.zoomed_asp_list.index('%% End of History:\n') - 2
		original_asp_reader.close()
		asp = ''.join(self.zoomed_asp_list)
		zoomed_asp_writter.write(asp)
		zoomed_asp_writter.close()

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
