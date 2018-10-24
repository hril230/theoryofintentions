#from sets import Set
import subprocess
import random
import sys
history_marker = '%% HISTORY GOES HERE'
display_marker = 'display'
asp_Refined_World_file = 'simulation/ASP_files/ASP_Refined_World.sp'
import global_variables


class World(object):
	def __init__(self,world_initial_state, new_domain_info):
		self.domain_info = new_domain_info
		preASP_refined_world_file = 'simulation/pre_ASP_files/complexity_level_' + str(self.domain_info.complexity_level) + '/preASP_refined_world.txt'
		reader = open(preASP_refined_world_file, 'r')
		pre_asp = reader.read()
		reader.close()
		self.pre_asp_split = pre_asp.split('\n')
		self.history_marker_index = self.pre_asp_split.index(history_marker) + 1
		self.display_marker_index = self.pre_asp_split.index(display_marker) + 2
		self.exo_action_happened = True
		self.history = []
		self.executionTimeUnits = 0
		self.executedSteps = 0
		obs_list = list(self.domain_info.refinedStateToRefinedHoldsSet(world_initial_state,0))
		output = self.__runASPDomain(obs_list)
		self.__updateStateFromAnswer(output)


	def __updateStateFromAnswer(self,answer):
		self.RefinedState = self.domain_info.refinedAnswerToRefinedState(answer)
		self.CoarseState = self.domain_info.refinedAnswerToCoarseState(answer)


	def __getExecutionTimeUnits(self,action):
		if 'move' in action: return 1
		elif 'pickup' in action or 'put_down' in action: return 3
		elif 'test' in action: return 1

		return 0

	def __getRandomExoAction(self,action):
		print ('EXO ACTION HAPPENING!!')
		exo_action = ''

		if(random.random()<0.08): return exo_action
		choice = random.choice(['ref1_book1','ref1_book2'])
		if(choice == 'ref1_book1'  and (self.RefinedState[self.domain_info.In_handBook1_index] == 'true' or action == 'pickup(rob1,ref1_book1)')): choice = 'ref1_book2'
		elif(choice == 'ref1_book2'  and (self.RefinedState[self.domain_info.In_handBook2_index] == 'true' or action == 'pickup(rob1,ref1_book2)')): choice = 'ref1_book1'
		if(choice == 'ref1_book1'):
			allRefinedLocations = list(self.domain_info.RefinedLocations)
			currentRefinedLocation = self.RefinedState[self.domain_info.LocationBook1_index]
			allRefinedLocations.remove(currentRefinedLocation)
			newRefinedLocation = random.choice(allRefinedLocations)
			exo_action =  'exo_move(ref1_book1,' +newRefinedLocation+ ')'
		elif(choice == 'ref1_book2'):
			allRefinedLocations = list(self.domain_info.RefinedLocations)
			currentRefinedLocation = self.RefinedState[self.domain_info.LocationBook2_index]
			allRefinedLocations.remove(currentRefinedLocation)
			newRefinedLocation = random.choice(allRefinedLocations)
			exo_action =  'exo_move(ref1_book2,' +newRefinedLocation+ ')'
		return exo_action

	def __del__(self):
		print('realWorld - deleting world  #############################################################################################################\n\n\n\n ')

	def executeAction(self,action):
		if 'test' in action: return self.testFluent(action)
		happened = False
		input = list(self.domain_info.refinedStateToRefinedHoldsSet(self.RefinedState,0)) + ['hpd('+ action +',0).']
		#print('refined action happening: ' + action)
		answer = self.__runASPDomain(input)
		self.executionTimeUnits += self.__getExecutionTimeUnits(action)
		self.executedSteps += 1
		if(answer == '\n'):
			print ('                nothing happned in real world ')
			self.history.append(action + "realWorld -  (FAILED) ")
		else:
			happened = True
			self.__updateStateFromAnswer(answer)
			self.history.append(action)
		direct_observation = self.__getDirectObservation(answer)
		return direct_observation

	def __getDirectObservation(self,answer):
		answer = answer.rstrip().strip('{').strip('}')
		for entry in answer.split(', '):
			if 'directly_observed' in entry:
				return entry
		return ''

	def testFluent(self,testedFluent):
		observed = ''
		if('in_hand' in testedFluent):
			observed = 'holds('+(testedFluent[:testedFluent.rfind(',')+1]).replace('test','directly_observed')
			if('book1' in testedFluent):
				if('ref1' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook1_Ref1_index] +'),1).'
				elif('ref2' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook1_Ref2_index] +'),1).'
				elif('ref3' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook1_Ref3_index] +'),1).'
				elif('ref4' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook1_Ref4_index] +'),1).'
			elif('book2' in testedFluent):
				if('ref1' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook2_Ref1_index] +'),1).'
				elif('ref2' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook2_Ref2_index] +'),1).'
				elif('ref3' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook2_Ref3_index] +'),1).'
				elif('ref4' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook2_Ref4_index] +'),1).'
			elif('book3' in testedFluent):
				if('ref1' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook3_Ref1_index] +'),1).'
				elif('ref2' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook3_Ref2_index] +'),1).'
				elif('ref3' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook3_Ref3_index] +'),1).'
				elif('ref4' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook3_Ref4_index] +'),1).'
			elif('book4' in testedFluent):
				if('ref1' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook4_Ref1_index] +'),1).'
				elif('ref2' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook4_Ref2_index] +'),1).'
				elif('ref3' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook4_Ref3_index] +'),1).'
				elif('ref4' in testedFluent): observed = observed + self.RefinedState[self.domain_info.In_handBook4_Ref4_index] +'),1).'
		if('loc' in testedFluent):
			observed = 'holds(' + (testedFluent[:testedFluent.rfind(',')+1]).replace('test', 'directly_observed')
			if('book1' in testedFluent):
			 	if self.RefinedState[self.domain_info.LocationBook1_index] == self.RefinedState[self.domain_info.LocationRobot_index]:
					 observed = observed + 'true),1).'
				else: observed = observed + 'false),1).'
			elif('book2' in testedFluent):
			 	if self.RefinedState[self.domain_info.LocationBook2_index] == self.RefinedState[self.domain_info.LocationRobot_index]:
				 	observed = observed + 'true),1).'
				else: observed = observed + 'false),1).'
			elif('book3' in testedFluent):
			 	if self.RefinedState[self.domain_info.LocationBook3_index] == self.RefinedState[self.domain_info.LocationRobot_index]:
					 observed = observed + 'true),1).'
				else: observed = observed + 'false),1).'
			elif('book4' in testedFluent):
			 	if self.RefinedState[self.domain_info.LocationBook4_index] == self.RefinedState[self.domain_info.LocationRobot_index]:
					 observed = observed + 'true),1).'
				else: observed = observed + 'false),1).'
			else:
				observed = observed[:observed.rfind('rob1,')] + 'rob1,' + self.RefinedState[self.domain_info.LocationRobot_index] +'),true),1).'
		return observed




	def __runASPDomain(self,input):
		asp_split = self.pre_asp_split[:self.history_marker_index] + input + self.pre_asp_split[self.history_marker_index:]
		asp = '\n'.join(asp_split)
		f1 = open(asp_Refined_World_file, 'w')
		f1.write(asp)
		f1.close()
		print (asp_Refined_World_file)
		answer = subprocess.check_output('java -jar '+ global_variables.sparc_path + ' ' +asp_Refined_World_file+' -A',shell=True)
		answer = answer.decode().strip('{')
		answer = answer.strip('}')
		return answer

	def achievedGoal(self): # TODO edit here to change goal
		#if(self.CoarseState[self.domain_info.LocationBook1_index] != 'kitchen'): return 'false'
		#elif(self.CoarseState[self.domain_info.LocationBook2_index] != 'library'): return 'false'
		#elif(self.CoarseState[self.domain_info.LocationBook3_index] != 'office2'): return 'false'
		#elif(self.CoarseState[self.domain_info.In_handBook1_index] == 'true'): return 'false'
		#elif(self.CoarseState[self.domain_info.In_handBook2_index] == 'false'): return 'false'
		#elif(self.CoarseState[self.domain_info.In_handBook3_index] == 'true'): return 'false'
		#else: return 'true'
		return 'true'

	def getGoalFeedback(self):
		if(self.achievedGoal() == 'true'): return True
		return False

	def getHistory(self):
		return self.history

	def getExecutionTimeUnits(self):
		return self.executionTimeUnits

	def getCoarseState(self):
		return self.CoarseState

	def getRobotRefinedLocation(self):
		return self.RefinedState[self.domain_info.LocationRobot_index]
