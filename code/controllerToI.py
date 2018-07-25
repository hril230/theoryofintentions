from datetime import datetime
from sets import Set
import subprocess
import re
import numpy
import random


class ControllerToI():
	def __init__(self, sparc_path, ASP_subfolder, domain_info, executer, known_world_state, goal, max_plan_length):
		self.goal = goal
		self.sparc_path = sparc_path
		self.executer = executer
		self.domain_info = domain_info
		self.max_plan_length = max_plan_length

		self.preASP_toi_file = ASP_subfolder+'pre_ASP_files/preASP_ToI.txt'
		self.preASP_refined_domain_file = ASP_subfolder + 'pre_ASP_files/preASP_refined_domain.txt'
		self.preASP_domain_file = ASP_subfolder + 'pre_ASP_files/preASP_Domain.txt'
		self.asp_toi_file = ASP_subfolder+'ASP_files/ASP_ToI.sp'
		self.asp_toi_diagnosing_file = ASP_subfolder + 'ASP_files/ASP_TOI_Diagnosis.sp'
		self.zoomed_domain_file = ASP_subfolder + 'ASP_files/zoomed_domain.sp'
		self.asp_belief_file = ASP_subfolder + 'ASP_files/ToI_belief.sp'

		self.number_steps = 4
		self.number_activities = 1
		self.goal_correction = 0
		self.current_diagnosis = ''
		self.input_for_planning = []
		self.current_step = 1
		self.toi_goal_marker = '%% @_@_@'
		self.toi_beginning_history_marker = '%% #_#_# beginning'
		self.toi_end_history_marker = '%% #_#_# end'
		self.toi_current_step_marker = '%% *_*_*'
		self.domain_history_marker = '%% *_*_*'

		self.believes_goal_holds  = False
		self.preASP_toi_split = []

		first_obs_list = self.domain_info.coarseStateToAstractObsList(known_world_state,0)
		self.toi_history = first_obs_list
		self.belief_history = self.filteredPlainHistory(first_obs_list)
		self.preparePreASPStringLists()
		self.setInitialBelief()



	def run(self):
		self.toi_history.append("hpd(select(my_goal), true,0).")
		self.diagnose()
		finish = False
		while(finish == False):
			next_action = self.runToIPlanning(self.input_for_planning)

			#print(' $$$$$$$$$$$$$$$$$    next action with ToI : ' +str(next_action) +'  at step '+ str(self.current_step))

			if(next_action == 'finish'):
				if(self.executer.getGoalFeedback() == True):
					self.toi_history.append('finish')
					finish = True
					break
				else:
					self.goal_correction += 1
					while(next_action == 'finish'):
						self.toi_history.append('obs(my_goal,false,'+str(self.current_step)+').')
						#print('wrong assumption - goal not reached !!!!! ')
						self.diagnose()
						next_action = self.runToIPlanning(self.input_for_planning)

			if(next_action == None):
		    		self.toi_history.append("Goal is futile")
				finish = True
				break



			action_observations = []
			self.toi_history.append('attempt('+next_action+','+str(self.current_step)+').')

			if(next_action[0:4] == 'stop'):
				self.number_activities += 1
				self.number_steps += 1
			elif(next_action[0:5] == 'start'): pass
			else:
				print('action : '+ str(next_action))
				answer_sets = self.refine(next_action, self.belief_history, self.current_step)
				plans = answer_sets.rstrip().split('\n\n')
				refined_plan = plans[0]
				refined_plan = refined_plan.strip('{').strip('}')
				refined_plan = refined_plan.split(', ')
				for refined_occurence in refined_plan:
					refined_action = refined_occurence[refined_occurence.find('(')+1 : refined_occurence.rfind(',')]
					self.executer.executeAction(refined_action)

			self.current_step += 1
			relevant_observations = action_observations + self.executer.getTheseCoarseObservations(self.getIndexesRelevantToGoal())
			robot_location = self.belief[self.domain_info.LocationRobot_index]
			new_obs_list = list(set(self.domain_info.observations_to_obs(relevant_observations,robot_location,self.current_step)))
			self.toi_history = self.toi_history + new_obs_list
			self.belief_history = self.belief_history + new_obs_list
			self.diagnose()
			self.update_belief(next_action)


		if(self.current_diagnosis != ''): self.toi_history.append(self.current_diagnosis)
		return (self.toi_history, self.number_activities, self.goal_correction)



 	def filteredPlainHistory(self,this_list):
		return [a for a in this_list if 'select' not in a and 'start' not in a and 'stop' not in a]

	def getIndexesRelevantToGoal(self):
		return [self.domain_info.LocationBook1_index, self.domain_info.LocationBook2_index, self.domain_info.In_handBook1_index, self.domain_info.In_handBook2_index]

	def runToIPlanning(self,input):
		next_action = None
		#print('running ToI planning ')


		current_asp_split = self.preASP_toi_split[:self.toi_beginning_history_index +1] + input + self.preASP_toi_split[self.toi_beginning_history_index +1:]
		current_asp_split[self.toi_current_step_index +1] = 'current_step('+str(self.current_step)+').'
		current_asp_split[0] = "#const numSteps = "+str(self.number_steps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(self.number_steps)+". % maximum activity_length of an activity."
		current_asp_split[2] = "#const max_name = " + str(self.number_activities) + "."
		asp = '\n'.join(current_asp_split)
	        f1 = open(self.asp_toi_file, 'w')
		f1.write(asp)
		f1.close()

		answerSet = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_toi_file +' -A ',shell=True)
		while( "intended_action" not in answerSet and "selected_goal_holds" not in answerSet and self.number_steps < self.current_step + self.max_plan_length+3):
			current_asp_split[0] = "#const numSteps = "+str(self.number_steps+1)+". % maximum number of steps."
			current_asp_split[1] = "#const max_len = "+str(self.number_steps)+". % maximum activity_length of an activity."
			asp = '\n'.join(current_asp_split)
			f1 = open(self.asp_toi_file, 'w')
			f1.write(asp)
			f1.close()
			#print('Looking for next action (ToI) - number_steps ' + str(number_steps))
			answerSet = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_toi_file +' -A ',shell=True)
			self.number_steps +=1

	        possibleAnswers = answerSet.rstrip().split('\n\n')

		chosenAnswer = possibleAnswers[0]
		split_answer = chosenAnswer.strip('}').strip('{').split(', ')
		self.toi_history = []
		self.believes_goal_holds = False
		for line in split_answer:
			if("intended_action" in line):
				next_action = line[16:line.rfind(',')]
			#elif("number_unobserved" in line): continue
			elif("selected_goal_holds" in line):
				self.believes_goal_holds = True
			else:
				self.toi_history.append(line + '.')
		return next_action



	def diagnose(self):
		self.input_for_planning = []
		possibleDiagnosis = []
		input = list(self.toi_history)
		input.append("explaining("+str(self.current_step)+").")
		current_asp_split = self.preASP_toi_split[: self.toi_beginning_history_index +1] + input +self.preASP_toi_split[self.toi_beginning_history_index +1:]
		current_asp_split[self.toi_current_step_index +1] = 'current_step('+str(self.current_step)+').'
		current_asp_split[0] = "#const numSteps = "+str(self.number_steps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(self.number_steps)+". % maximum activity_length of an activity."
		current_asp_split[2] = "#const max_name = " + str(self.number_activities) + "."


		asp = '\n'.join(current_asp_split)
		f1 = open(self.asp_toi_diagnosing_file, 'w')
		f1.write(asp)
		f1.close()

		# running only diagnosis
		answerSet = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_toi_diagnosing_file +' -A ',shell=True)
		answers = answerSet.rstrip().split('\n\n')
		print "diagnosis"


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
				self.current_diagnosis = line
			elif("selected_goal_holds" in line): pass
			elif(line == ""): pass
			else:
				self.input_for_planning.append(line + '.')
		return


	def preparePreASPStringLists(self):
		#preparing preASP_toi_split and self.toi_beginning_history_index
		reader = open(self.preASP_toi_file, 'r')
		preASP_toi = reader.read()
		reader.close()
		self.preASP_toi_split = preASP_toi.split('\n')

	   	index_goal = self.preASP_toi_split.index(self.toi_goal_marker)
		self.preASP_toi_split.insert(index_goal+1,  "holds(my_goal,I) :- "+ self.goal)
		self.toi_beginning_history_index = self.preASP_toi_split.index(self.toi_beginning_history_marker)
		self.toi_current_step_index = self.preASP_toi_split.index(self.toi_current_step_marker)

		reader = open(self.preASP_domain_file, 'r')
		preASP_domain = reader.read()
		reader.close()
		self.preASP_domain_split = preASP_domain.split('\n')
		#history_index_domainasp : integer that holds the line where the history needs to be added in the asp domain file
		self.history_index_domain_asp = self.preASP_domain_split.index(self.domain_history_marker)

	def update_belief(self, action):
		if('start' in action or 'stop' in action): return
		possible_last_action = 'hpd(' +action+', '+ str(self.current_step-1) + ').'
		input = self.belief_history + [possible_last_action]
		if(self.current_diagnosis != ''):
			input.append(self.current_diagnosis.replace('unobserved','hpd')+'.')

		asp_belief_split = self.preASP_domain_split[:self.history_index_domain_asp] + input + self.preASP_domain_split[self.history_index_domain_asp+1:]
		asp_belief_split[0] = "#const numSteps = "+ str(self.current_step) + "."
		asp = '\n'.join(asp_belief_split)
		f1 = open(self.asp_belief_file, 'w')
		f1.write(asp)
		f1.close()
		print('Next, checking belief-observations consistency ')
		output = subprocess.check_output('java -jar '+ self.sparc_path + ' ' + self.asp_belief_file +' -A',shell=True)
		output = output.rstrip().strip('{').strip('}')

		if 'holds' in output:
			self.belief_history.append(possible_last_action)
			self.belief = self.domain_info.abstractAnswerToCoarseState(output)
		print 'updated belief'
		print self.belief

	def setInitialBelief(self):
		input = self.belief_history
		asp_belief_split = self.preASP_domain_split[:self.history_index_domain_asp] + input + self.preASP_domain_split[self.history_index_domain_asp+1:]
		asp_belief_split[0] = "#const numSteps = "+ str(self.current_step) + "."
		asp = '\n'.join(asp_belief_split)
		f1 = open(self.asp_belief_file, 'w')
		f1.write(asp)
		f1.close()
		output = subprocess.check_output('java -jar '+ self.sparc_path + ' ' + self.asp_belief_file +' -A',shell=True)
		output = output.rstrip().strip('{').strip('}')
		self.belief = self.domain_info.abstractAnswerToCoarseState(output)
		print 'initial belief: ' + str(self.belief)



	# this function uses the preASP_refined_Domain.txt file and SPARC to get a refined action plan
	def refine(self,action, history, current_step):
		initial_state = []
		final_state = []
		robot_coarse_location = self.belief[self.domain_info.LocationRobot_index]
	    # use action and history to figure out the transition (initial_state, action, final_state)
	    # the location of the robot is relevant for move transitions
		action_object = action[action.find(',')+1:-1]
		if 'move' in action:
			initial_state = ['coarse_loc(rob1,' + robot_coarse_location + ')']
			final_state = ['coarse_loc(rob1,' + action_object + ')']

		elif 'pickup' in action:
			initial_state = ['-coarse_in_hand(rob1,' + action_object + ')']
			final_state = ['coarse_in_hand(rob1,' + action_object + ')']
			initial_state.append('coarse_loc(rob1,' + robot_coarse_location + ')')
			initial_state.append('coarse_loc('+ action_object+','+ robot_coarse_location+')')
	    # the in_hand status of the object is relevant for put_down transitions
		elif 'put_down' in action:
			initial_state = ['coarse_in_hand(rob1,' + action_object + ')']
			final_state = ['-coarse_in_hand(rob1,' + action_object + ')']

	    # edit refined_asp to get temporary zoomed_asp file
		self.zoom(initial_state, action, final_state)


	    # get refined answer set
		refined_answer = subprocess.check_output('java -jar '+self.sparc_path +' '+ self.zoomed_domain_file + ' -A ',shell=True)
		if refined_answer == "" : raw_input()

	    # stop running code if refined answer set is empty
		if refined_answer == '\n':
			raw_input('No refined answer set, press enter if you wish to continue.\n')

	    # refined_plan = parse answer set
		refined_plan = refined_answer
		return refined_plan

	# this action writes a zoomed ASP file
	def zoom(self,initial_state, action, final_state):
	    # EDIT these lists to change the domain
		coarse_places = Sort('coarse_place', ['library', 'kitchen', 'office1', 'office2', 'unknown'])
		coarse_objects = Sort('coarse_object', ['book1', 'book2'])
		places = Sort('place', ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'c13', 'c14', 'c15', 'c16', 'unknown_cell'])
		objects = Sort('object', ['ref_book1', 'ref_book2'])
		coarse_things = Sort('coarse_thing', ['#coarse_object', '#robot'])
		things = Sort('thing', ['#object', '#robot'])
		coarse_components = Sort('coarse_component', ['#coarse_place', '#coarse_object'])
		refined_components = Sort('refined_component', ['#place', '#object'])
		robots = Sort('robot', ['rob1'])
		sorts = [coarse_places, coarse_objects, places, objects, coarse_things, things, coarse_components, refined_components, robots]
		inertial_fluents = ['loc(#thing,#place)', 'in_hand(#robot,#object)']
		defined_fluents = ['coarse_loc(#coarse_thing,#coarse_place)', 'coarse_in_hand(#robot,#coarse_object)']
		actions = ['move(#robot,#place)', 'pickup(#robot,#object)', 'put_down(#robot,#object)']

	    # EDIT these instantiations of the Components class to change which refined objects are associated with which coarse ones
		library_components = Components('library', ['c1', 'c2', 'c3', 'c4'])
		kitchen_components = Components('kitchen', ['c5', 'c6', 'c7', 'c8'])
		office1_components = Components('office1', ['c9', 'c10', 'c11', 'c12'])
		office2_components = Components('office2', ['c13', 'c14', 'c15', 'c16'])
		unknown_components = Components('unknown', ['unknown_cell'])
		book1_components = Components('book1', ['ref_book1'])
		book2_components = Components('book2', ['ref_book2'])
		refinements = [library_components, kitchen_components, office1_components, office2_components, book1_components, book2_components, unknown_components]

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

	    # initialise irrelevance lists - EDIT to include new objects or zones or cells
		irrelevant_sort_names = ['#coarse_place', '#coarse_object', '#place', '#object']
		irrelevant_obj_consts = ['library', 'kitchen', 'office1', 'office2', 'unknown', 'book1', 'book2', 'c1,', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12','c13', 'c14', 'c15', 'c16' 'ref_boo1', 'ref_book2']
		irrelevant_fluents = ['coarse_loc', 'coarse_in_hand', 'loc', 'in_hand']
		irrelevant_actions = ['move', 'pickup', 'put_down']

	    # determine which initial conditions are relevant
		for condition in initial_state:
			if not condition in final_state: # conditions that change are relevant
				rel_initial_conditions.append(condition)
				rel_conditions.append(condition)
			elif ('rob1' in condition) and ('loc' in condition) and ('pickup' in action): # the robot's location is always relevant for pickup actions
				rel_initial_conditions.append(condition)
				rel_conditions.append(condition)


	    # refine initial conditions
		for i in range(len(rel_initial_conditions)):
			if ('loc' in rel_initial_conditions[i]) and ('rob1' in rel_initial_conditions[i]):
				rel_initial_conditions[i] = 'loc(rob1,' + self.executer.getMyRefinedLocation() + ')'
			if ('in_hand' in rel_initial_conditions[i]) and (not '-' in rel_initial_conditions[i]):
				currently_holding = ''
				if(self.belief[self.domain_info.In_handBook1_index] == 'true'): currently_holding = 'ref_book1'
				elif(self.belief[self.domain_info.In_handBook2_index] == 'true'): currently_holding = 'ref_book2'
				if(currently_holding != ''):  rel_initial_conditions[i] = 'in_hand(rob1,' + currently_holding + ')'

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
						if refined_const == 'c1':
							refined_const = 'c1,'
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
		original_asp = open(self.preASP_refined_domain_file, 'r')
		zoomed_asp = open(self.zoomed_domain_file, 'w')

		for line in original_asp:
			if line == '%% GOAL GOES HERE\n': # put goal in
				zoomed_asp.write(self.goal)
			elif line == '%% INITIAL STATE GOES HERE\n': # put initial conditions in
				for condition in rel_initial_conditions:
					if '-' in condition:
						condition = condition.replace('-', '')
						zoomed_asp.write('-holds(' + condition + ', 0).\n')
					else:
						zoomed_asp.write('holds(' + condition + ', 0).\n')
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
						zoomed_asp.write(zoomed_sort)
			elif 'ZOOM THIS SORT OF SORTS' in line: # add relevant sorts
				zoomed_sort = ''
				for sort in sorts:
					if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
						zoomed_sort = '#' + sort.name + ' = '
						for const in sort.rel_constants:
							zoomed_sort = zoomed_sort + const + ' + '
						zoomed_sort = zoomed_sort[0:len(zoomed_sort)-3] + '.\n'
						zoomed_asp.write(zoomed_sort)
			elif 'ZOOM INERTIAL FLUENTS' in line: # add relevant inertial fluents
				inertial_fluent_sort = '#physical_inertial_fluent = '
				for fluent in rel_inertial_fluents:
					inertial_fluent_sort = inertial_fluent_sort + fluent + ' + '
				inertial_fluent_sort = inertial_fluent_sort[0:len(inertial_fluent_sort)-3] + '.\n'
				zoomed_asp.write(inertial_fluent_sort)
			elif 'ZOOM DEFINED FLUENTS' in line: # add relevant defined fluents
				defined_fluent_sort = '#physical_defined_fluent = '
				for fluent in rel_defined_fluents:
					defined_fluent_sort = defined_fluent_sort + fluent + ' + '
				defined_fluent_sort = defined_fluent_sort[0:len(defined_fluent_sort)-3] + '.\n'
				zoomed_asp.write(defined_fluent_sort)
			elif 'ZOOM ACTIONS' in line: # add relevant actions
				action_sort = '#action = '
				for act in rel_actions:
					action_sort = action_sort + act + ' + '
				action_sort = action_sort[0:len(action_sort)-3] + '.\n'
				zoomed_asp.write(action_sort)
			else:
				line_relevant = True
				for sort in irrelevant_sort_names: # don't include predicates with irrelevant sorts
					if sort in line:
						line_relevant = False
				for const in irrelevant_obj_consts: # don't include attributes with irrelevant object constants
					if const in line:
						line_relevant = False
				for fluent in irrelevant_fluents: # don't include axioms with irrelevant fluents
					if fluent in line:
						line_relevant = False
				for act in irrelevant_actions: # don't include axioms with irrelevant actions
					if act in line:
						line_relevant = False
				if line_relevant:
					zoomed_asp.write(line)
		original_asp.close()
		zoomed_asp.close()

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
