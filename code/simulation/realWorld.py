from sets import Set
import subprocess
import random
import sys
preASP_refined_world_file = 'simulation/pre_ASP_files/preASP_refined_world.txt'
asp_World_file = 'simulation/ASP_files/ASP_World.sp'
asp_Refined_World_file = 'simulation/ASP_files/ASP_Refined_World.sp'
history_marker = '%% HISTORY GOES HERE'
display_marker = 'display'


class World(object):
	def __init__(self,thisPath,world_initial_state, this_seed, new_domain_info):
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
		self.sparcPath = thisPath
		self.domain_info = new_domain_info
		random.seed(this_seed)
		obs_list = list(self.domain_info.refinedStateToRefinedHoldsSet(world_initial_state,0))
		print('updating world state from initial state in ASP: ')
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
		happened = False
		input = list(self.domain_info.refinedStateToRefinedHoldsSet(self.RefinedState,0)) + ['hpd('+ action +',0).']
		answer = self.__runASPDomain(input)
		self.executionTimeUnits += self.__getExecutionTimeUnits(action)
		self.executedSteps += 1
		if(answer == '\n'):
			print '                nothing happned in real world '
			self.history.append(action + "realWorld -  (FAILED) ")
		else:
			happened = True
			self.__updateStateFromAnswer(answer)
			self.history.append(action)
	 	return self.__getDirectObservation(answer)

	def __getDirectObservation(self,answer):
		answer = answer.rstrip().strip('{').strip('}')
		for entry in answer.split(', '):
			if 'directly_observed' in entry: return entry
		return ''

	def __runASPDomain(self,input):
		asp_split = self.pre_asp_split[:self.history_marker_index] + input + self.pre_asp_split[self.history_marker_index:]
		asp = '\n'.join(asp_split)
		f1 = open(asp_Refined_World_file, 'w')
		f1.write(asp)
		f1.close()
		print '\nRealWorld: updating state'
		answer = subprocess.check_output('java -jar '+ self.sparcPath + ' ' +asp_Refined_World_file+' -A',shell=True)
		return answer.rstrip().strip('{').strip('}')

	def achievedGoal(self): # TODO edit here to change goal
		if(self.CoarseState[self.domain_info.LocationBook1_index] != 'kitchen'): return 'false'
		#if(self.CoarseState[self.domain_info.LocationBook2_index] != 'library'): return 'false'
		#elif(self.CoarseState[self.domain_info.In_handBook1_index] == 'true'): return 'false'
		#elif(self.CoarseState[self.domain_info.In_handBook2_index] == 'true'): return 'false'
		else: return 'true'

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
