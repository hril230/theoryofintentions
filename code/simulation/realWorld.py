from sets import Set
import subprocess
import random
preASP_refined_world_file = 'simulation/pre_ASP_files/preASP_refined_world.txt'
asp_World_file = 'simulation/ASP_files/ASP_World.sp'
asp_Refined_World_file = 'simulation/ASP_files/ASP_Refined_World.sp'
history_marker = '%% *_*_*'
display_marker = 'display'


class World(object):


	def __init__(self,thisPath,world_initial_state, this_seed, new_domain_info):
		reader = open(preASP_refined_world_file, 'r')
		pre_asp = reader.read()
		reader.close()
		self.pre_asp_split = pre_asp.split('\n')
		self.history_marker_index = self.pre_asp_split.index(history_marker) + 1
		self.display_marker_index = self.pre_asp_split.index(display_marker) + 2
		self.exo_action_happened = False
		self.history = []
		self.executionTimeUnits = 0
		self.executedSteps = 0
		self.sparcPath = thisPath
		self.domain_info = new_domain_info
		random.seed(this_seed)
		output = self.__runASPDomain(self.domain_info.refinedStateToRefinedObsList(world_initial_state,0))
		output = output.rstrip().strip('{').strip('}')
		self.__updateStateFromAnswer(output)


	def __updateStateFromAnswer(self,answer):
		self.RefinedState = self.domain_info.refinedAnswerToRefinedState(answer)
		self.CoarseState = self.domain_info.refinedAnswerToCoarseState(answer)

	def __getActionObservations(self,action,happened):
		relevant_indexes= Set([])
		if(action[0:4] == 'move'):
			relevant_indexes.add(self.domain_info.LocationRobot_index)
		if(action == 'pickup(rob1,book1)' or action == '+put_down(rob1,book1)'):
			relevant_indexes.add(self.domain_info.In_handBook1_index)
			relevant_indexes.add(self.domain_info.LocationBook1_index)

		if(action == 'pickup(rob1,book2)' or action == '+put_down(rob1,book2)'):
			relevant_indexes.add(self.domain_info.In_handBook2_index)
			relevant_indexes.add(self.domain_info.LocationBook2_index)

		return self.getTheseCoarseObservations(relevant_indexes)

	def __getExecutionTimeUnits(self,action):
		print 'execution time units for action: ' + action
		if(action[0:4] == 'move'):
			return 3
		if(action[0] == 'p'):
			return 1

	def __getRandomExoAction(self,action):
		exo_action = ''

		if(random.random()<0.08): return exo_action
		choice = random.choice(['book1','book2'])
		if(choice == 'book1'  and (self.RefinedState[self.domain_info.In_handBook1_index] == 'true' or action == 'pickup(rob1,book1)')): choice = 'book2'
		elif(choice == 'book2'  and (self.RefinedState[self.domain_info.In_handBook2_index] == 'true' or action == 'pickup(rob1,book2)')): choice = 'book1'
		if(choice == 'book1'):
			allRefinedLocations = list(self.domain_info.RefinedLocations)
			currentRefinedLocation = self.RefinedState[self.domain_info.LocationBook1_index]
			print ' book 1 location before exo move: ' + currentRefinedLocation
			allRefinedLocations.remove(currentRefinedLocation)
			newRefinedLocation = random.choice(allRefinedLocations)
			exo_action =  'exo_move(book1,' +newRefinedLocation+ ')'
		elif(choice == 'book2'):
			allRefinedLocations = list(self.domain_info.RefinedLocations)
			currentRefinedLocation = self.RefinedState[self.domain_info.LocationBook2_index]
			print ' book 2 location before exo move: ' + currentRefinedLocation
			allRefinedLocations.remove(currentRefinedLocation)
			newRefinedLocation = random.choice(allRefinedLocations)
			exo_action =  'exo_move(book2,' +newRefinedLocation+ ')'
		return exo_action


	def __del__(self):
		print('deleting world ')

	def executeAction(self,action):
		print 'execution actoin: ' + action
		happened = False
		exo_action = ''
		input = self.domain_info.refinedStateToRefinedObsList(self.RefinedState,0) + ['hpd('+ action +',0).']
		if(self.exo_action_happened == False):
			exo_action = self.__getRandomExoAction(action)
			if(exo_action != ''): input = input + ['hpd('+ exo_action +',0).']
		answer = self.__runASPDomain(input)
		self.executionTimeUnits += self.__getExecutionTimeUnits(action)
		self.executedSteps += 1

		print ' \n\n\n Answer:'
		print answer
		print ' \n\n\n'

		if(answer == '\n'):
			happened = False
			self.history.append(action + " (FAILED) ")
		else:
			happened = True
			self.__updateStateFromAnswer(answer)
			self.history.append(action)
			if(exo_action != ''):
				self.history.append(exo_action)
				self.exo_action_happened = True
				print('%%%%%%%%%     exo_action happened in realWorld.py :  '+exo_action)
		return (self.__getActionObservations(action, happened))

	def __runASPDomain(self,input):
		asp_split = self.pre_asp_split[0:self.history_marker_index] + input + self.pre_asp_split[self.history_marker_index:]
		asp = '\n'.join(asp_split)
		f1 = open(asp_Refined_World_file, 'w')
		f1.write(asp)
		f1.close()
		answer = subprocess.check_output('java -jar '+ self.sparcPath + ' ' +asp_Refined_World_file+' -A',shell=True)

		return answer


	def getRefinedState(self):
		return self.RefinedState
	def getCoarseState(self):
		return self.CoarseState
	def getRobotsRefinedLocation(self):
		return self.RefinedState[self.domain_info.LocationRobot_index]
	def getRobotsCoarseLocation(self):
		return self.CoarseState[self.domain_info.LocationRobot_index]

	def getTheseCoarseObservations(self,indexes):
		observableValues = list(self.CoarseState)
		robotLocation = self.CoarseState[self.domain_info.LocationRobot_index]
		observations = []
		if(self.CoarseState[self.domain_info.LocationBook1_index] != robotLocation):
			observableValues[self.domain_info.LocationBook1_index] = 'unknown'
		if(self.CoarseState[self.domain_info.LocationBook2_index] != robotLocation):
			observableValues[self.domain_info.LocationBook2_index] = 'unknown'
		for index in indexes:
			observations.append([index,observableValues[index]])
		return observations



	def achievedGoal(self):
		if(self.State[self.domain_info.LocationBook1_index] != 'library'): return 'false'
		elif(self.State[self.domain_info.LocationBook2_index] != 'library'): return 'false'
		elif(self.State[self.domain_info.In_handBook1_index] == 'true'): return 'false'
		elif(self.State[self.domain_info.In_handBook2_index] == 'true'): return 'false'
		else: return 'true'

	def getGoalFeedback(self):
		if(self.achievedGoal() == 'true'): return True
		return False

	def getHistory(self):
		return self.history

	def getExecutionTimeUnits(self):
		return self.executionTimeUnits

	def getExecutedSteps(self):
		return self.executedSteps
