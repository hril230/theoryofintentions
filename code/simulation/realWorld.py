''' This is a simulation of the real world would be, with observations as return and action as input
The RealValues is going to be kept in self.RealValues list. The indexes that belong to different
fluents are as follows:
	LibraryLocked_index = 0
	LocationRobot_index = 1
	LocationBook1_index = 2
	LocationBook2_index = 3
	In_handBook1_index = 4
	In_handBook2_index = 5
'''

from sets import Set
import subprocess
import random
preASP_domain_file = 'simulation/pre_ASP_files/preASP_Domain.txt'
asp_World_file = 'simulation/ASP_files/ASP_World.sp'
history_marker = '%% *_*_*'
display_marker = 'display'


class World(object):


	def __init__(self,thisPath,initialConditionsWorld, scenario, this_seed, new_domain_info):

		self.RealValues = list(initialConditionsWorld)
		reader = open(preASP_domain_file, 'r')
		pre_asp = reader.read()
		reader.close()
		self.pre_asp_split = pre_asp.split('\n')
		self.history_marker_index = self.pre_asp_split.index(history_marker) + 1
		self.display_marker_index = self.pre_asp_split.index(display_marker) + 2
		self.exo_action_happened = False
		self.scenario = scenario
		self.history = []
		self.executionTimeUnits = 0
		self.executedSteps = 0
		self.sparcPath = thisPath
		self.domain_info = new_domain_info
		random.seed(this_seed)

	def __updateRealValues(self,answerSet):
		self.RealValues[0] = 'false'
		self.RealValues[4] = 'false'
		self.RealValues[5] = 'false'
		newRealValues = answerSet.strip('\n')
		newRealValues_split = newRealValues.strip('}').strip('{').split(', ')
		for fluent in newRealValues_split:
			fluent = fluent[6:fluent.rfind(',')]
			if(fluent[0:7] == 'locked('):
				self.RealValues[0] = 'true'
			elif(fluent[0:4] == 'loc('):
				fluent = fluent[4:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[0] == 'rob1'): self.RealValues[1] = split_fluent[1]
				elif(split_fluent[0] == 'book1'): self.RealValues[2] = split_fluent[1]
				elif(split_fluent[0] == 'book2'): self.RealValues[3] = split_fluent[1]
			elif(fluent[0:8] == 'in_hand('):
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'book1'): self.RealValues[4] = 'true'
				if(split_fluent[1] == 'book2'): self.RealValues[5] = 'true'



	def __getActionObservations(self,action,happened):
		relevant_indexes= Set([])
		if(action[0:4] == 'move'):
			relevant_indexes.add(self.domain_info.LocationRobot_index)
			if(not happened):
				relevant_indexes.add(self.domain_info.LibraryLocked_index)

		if(action == 'unlock(rob1,library)'):
			relevant_indexes.add(self.domain_info.LibraryLocked_index)

		if(action == 'pickup(rob1,book1)' or action == '+put_down(rob1,book1)'):
			relevant_indexes.add(self.domain_info.In_handBook1_index)
			relevant_indexes.add(self.domain_info.LocationBook1_index)

		if(action == 'pickup(rob1,book2)' or action == '+put_down(rob1,book2)'):
			relevant_indexes.add(self.domain_info.In_handBook2_index)
			relevant_indexes.add(self.domain_info.LocationBook2_index)

		return self.getTheseObservations(relevant_indexes)

	def __getRealValues_as_obsList(self,step):
		obsList = []
		if(self.RealValues[self.domain_info.LibraryLocked_index] != 'unknown'):
			obsList.append('obs(locked(library),'+self.RealValues[self.domain_info.LibraryLocked_index]+','+str(step)+').')
		if(self.RealValues[self.domain_info.LocationRobot_index] != 'unknown'):
			obsList.append('obs(loc(rob1,'+str(self.RealValues[self.domain_info.LocationRobot_index])+'),true,'+str(step)+').')
		if(self.RealValues[self.domain_info.LocationBook1_index] != 'unknown'):
			obsList.append('obs(loc(book1,'+str(self.RealValues[self.domain_info.LocationBook1_index])+'),true,'+str(step)+').')
		if(self.RealValues[self.domain_info.LocationBook2_index] != 'unknown'):
			obsList.append('obs(loc(book2,'+str(self.RealValues[self.domain_info.LocationBook2_index])+'),true,'+str(step)+').')
		if(self.RealValues[self.domain_info.In_handBook1_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book1),'+self.RealValues[self.domain_info.In_handBook1_index]+','+str(step)+').')
		if(self.RealValues[self.domain_info.In_handBook2_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book2),'+self.RealValues[self.domain_info.In_handBook2_index]+','+str(step)+').')
		return obsList

	def __getexecutionTimeUnits(self,action):
		if(action[0:4] == 'move'):
			return 3
		if(action[0] == 'p'):
			return 1
		if(action[0:6] == 'unlock'):
			return 1

	def __get_scenario_exo_action(self,action):
		scenario = self.scenario
		exo_action = ''

 		#case scenario 1 - just planning, everything goes fine, no unexpected changes in the self.domain_info.


		# case scenario 2 Unexpected achievement of the goal
		if(scenario == 2):
			if(action[0:8] == 'put_down' and self.RealValues[self.domain_info.LibraryLocked_index] == 'false'):
				if(self.RealValues[self.domain_info.LocationBook1_index] != 'library'):
					exo_action = 'exo_move(book1,library)'
				elif(self.RealValues[self.domain_info.LocationBook2_index] != 'library'):
					exo_action = 'exo_move(book2,library)'


		if(scenario == 3):
			robotDestination = ""
			if(action[0:4] == 'move'): robotDestination = action[10:-1]
			elif(action[0] == 'p'): robotDestination = self.RealValues[self.domain_info.LocationRobot_index]
			else:  return exo_action

			choice = random.choice(['book1','book2'])
			if(choice == 'book1'  and (self.RealValues[self.domain_info.In_handBook1_index] == 'true' or action == 'pickup(rob1,book1)')): choice = 'book2'
			elif(choice == 'book2'  and (self.RealValues[self.domain_info.In_handBook2_index] == 'true' or action == 'pickup(rob1,book2)')): choice = 'book1'

			if(choice == 'book1'):
				currentBookLocation = self.RealValues[self.domain_info.LocationBook1_index]
				if(currentBookLocation == robotDestination and currentBookLocation != 'library'):
					allLocations = list(self.domain_info.DomainLocations)
					allLocations.remove(currentBookLocation)
					newLocation = random.choice(allLocations)
					if(self.RealValues[self.domain_info.LibraryLocked_index] == 'false' or newLocation != 'library'):
						return( 'exo_move(book1,' +newLocation+ ')')

			elif(choice == 'book2'):
				currentBookLocation = self.RealValues[self.domain_info.LocationBook2_index]
				if(currentBookLocation == robotDestination and currentBookLocation != 'library'):
					allLocations = list(DomianInfo.DomainLocations)
					allLocations.remove(currentBookLocation)
					newLocation = random.choice(allLocations)
					if(self.RealValues[self.domain_info.LibraryLocked_index] == 'false' or newLocation != 'library'):
						return( 'exo_move(book2,' +newLocation+ ')')


		if(scenario == 4):
			destination = ""
			if(action[0:4] == 'move'): destination = action[10:-1]
			elif(action[0] == 'p'): destination = self.RealValues[self.domain_info.LocationRobot_index]
			else:  return exo_action

			choice = random.choice(['book1','book2'])
			if(choice == 'book1'  and self.RealValues[self.domain_info.In_handBook1_index] == 'true'): choice = 'book2'
			elif(choice == 'book2'  and self.RealValues[self.domain_info.In_handBook2_index] == 'true'): choice = 'book1'

			if(choice == 'book1'):
				if(self.RealValues[self.domain_info.LocationBook1_index] != destination and destination != 'library'):
					if(self.RealValues[self.domain_info.LibraryLocked_index] == 'false' or self.RealValues[self.domain_info.LocationBook1_index] != 'library'):
						exo_action =  'exo_move(book1,' +destination+ ')'

			elif(choice == 'book2'):
				if(self.RealValues[self.domain_info.LocationBook2_index] != destination and destination != 'library'):
					if(self.RealValues[self.domain_info.LibraryLocked_index] == 'false' or self.RealValues[self.domain_info.LocationBook2_index] != 'library'):
						exo_action =  'exo_move(book2,' +destination+ ')'




		# case scenario 5 - Failure to achieve goal, diagnosis, and re-planning
		if(scenario == 5):
			if (self.RealValues[self.domain_info.LocationBook1_index] == 'library'  and self.RealValues[self.domain_info.LocationBook2_index] == 'library' and self.RealValues[self.domain_info.LibraryLocked_index] == 'false'):
				if(self.RealValues[self.domain_info.In_handBook1_index] == 'true'):
					exo_action = 'exo_move(book2,kitchen)'
				elif(self.RealValues[self.domain_info.In_handBook2_index] == 'true'):
					exo_action = 'exo_move(book1,kitchen)'


		# case scenario 6 - unexpected failure to execute
		if(scenario == 6):
			if self.RealValues[self.domain_info.LibraryLocked_index] == 'false':
				exo_action = 'exo_lock(library)'


		if(scenario == 'random'):
			if(random.random()<0.08): return exo_action
			choice = random.choice(['library','book1','book2'])
			if(choice == 'library' and self.RealValues[self.domain_info.LibraryLocked_index] == 'false'): return 'exo_lock(library)'
			elif(choice == 'book1'  and (self.RealValues[self.domain_info.In_handBook1_index] == 'true' or action == 'pickup(rob1,book1)')): choice = 'book2'
			elif(choice == 'book2'  and (self.RealValues[self.domain_info.In_handBook2_index] == 'true' or action == 'pickup(rob1,book2)')): choice = 'book1'
			if(choice == 'book1'):
				allLocations = list(self.domain_info.DomainLocations)
				currentLocation = self.RealValues[self.domain_info.LocationBook1_index]
				allLocations.remove(currentLocation)
				newLocation = random.choice(allLocations)
				if((newLocation != 'library' and currentLocation != 'library') or self.RealValues[self.domain_info.LibraryLocked_index] == 'false'):
					exo_action =  'exo_move(book1,' +newLocation+ ')'
			elif(choice == 'book2'):
				allLocations = list(self.domain_info.DomainLocations)
				currentLocation = self.RealValues[self.domain_info.LocationBook2_index]
				allLocations.remove(currentLocation)
				newLocation = random.choice(allLocations)
				if((newLocation != 'library' and currentLocation != 'library') or self.RealValues[self.domain_info.LibraryLocked_index] == 'false'):
					exo_action =  'exo_move(book2,' +newLocation+ ')'
		return exo_action


	def __del__(self):
		print('deleting world ')

	def executeAction(self,action):
		happened = False
		exo_action = ''
		input = self.__getRealValues_as_obsList(0) + ['hpd('+ action +',0).']
		if(self.exo_action_happened == False):
			exo_action = self.__get_scenario_exo_action(action)
			if(exo_action != ''): input = input + ['hpd('+ exo_action +',0).']
		asp_split = self.pre_asp_split[0:self.history_marker_index] + input + self.pre_asp_split[self.history_marker_index:]
		asp = '\n'.join(asp_split)
		f1 = open(asp_World_file, 'w')
		f1.write(asp)
		f1.close()
		answer = subprocess.check_output('java -jar '+ self.sparcPath + ' ' +asp_World_file+' -A',shell=True)
		self.executionTimeUnits += self.__getexecutionTimeUnits(action)
		self.executedSteps += 1

		if(answer == '\n'):
			happened = False
			self.history.append(action + " (FAILED) ")
		else:
			happened = True
			self.__updateRealValues(answer)
			self.history.append(action)
			if(exo_action != ''):
				self.history.append(exo_action)
				self.exo_action_happened = True
				print('%%%%%%%%%     exo_action happened in realWorld.py :  '+exo_action)

		return (self.__getActionObservations(action, happened))

	def getRealValues(self):
		observations =[]
		for index, val in enumerate(self.RealValues):
			observations.append([index,val])
		return observations

	def getTheseObservations(self,indexes):
		observableValues = list(self.RealValues)
		robotLocation = self.RealValues[self.domain_info.LocationRobot_index]
		observations = []
		if(robotLocation != 'library' and robotLocation != 'kitchen'):
			observableValues[self.domain_info.LibraryLocked_index] = 'unknown'
		if(self.RealValues[self.domain_info.LocationBook1_index] != robotLocation):
			observableValues[self.domain_info.LocationBook1_index] = 'unknown'
		if(self.RealValues[self.domain_info.LocationBook2_index] != robotLocation):
			observableValues[self.domain_info.LocationBook2_index] = 'unknown'
		for index in indexes:
			observations.append([index,observableValues[index]])
		return observations

	def getRobotLocation(self):
		return self.RealValues[self.domain_info.LocationRobot_index]

	def achievedGoal(self):
		if(self.RealValues[self.domain_info.LocationBook1_index] != 'library'): return 'false'
		elif(self.RealValues[self.domain_info.LocationBook2_index] != 'library'): return 'false'
		elif(self.RealValues[self.domain_info.In_handBook1_index] == 'true'): return 'false'
		elif(self.RealValues[self.domain_info.In_handBook2_index] == 'true'): return 'false'
		else: return 'true'

	def getGoalFeedback(self):
		if(self.achievedGoal() == 'true'): return True
		return False

	def get_exo_action_happened(self):
		return self.exo_action_happened

	def getHistory(self):
		return self.history

	def getExecutionTimeUnits(self):
		return self.executionTimeUnits

	def getExecutedSteps(self):
		return self.executedSteps
