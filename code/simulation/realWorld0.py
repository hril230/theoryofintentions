from sets import Set
import subprocess
import random
preASP_refined_domain_file = 'simulation/pre_ASP_files/preASP_refined_world.txt'
asp_World_file = 'simulation/ASP_files/refiend_world.sp'
history_marker = '%% *_*_*'
display_marker = 'display'


class World(object):


	def __init__(self,thisPath,initialConditionsWorld, scenario, this_seed, new_domain_info):

		self.RealValues = list(initialConditionsWorld)
		reader = open(preASP_refined_domain_file, 'r')
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
		print 'initial real values'
		print self.RealValues

	def __updateRealValues(self,answerSet):
		self.RealValues[self.domain_info.In_handBook1_index] =  'false'
		self.RealValues[self.domain_info.In_handBook2_index] =  'false'
		newRealValues = answerSet.strip('\n')
		newRealValues_split = newRealValues.strip('}').strip('{').split(', ')
		for fluent in newRealValues_split:
			fluent = fluent[6:fluent.rfind(',')]
			if(fluent[0:4] == 'loc('):
				fluent = fluent[4:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[0] == 'rob1'): self.RealValues[self.domain_info.LocationRobot_index] = split_fluent[1]
				elif(split_fluent[0] == 'book1'): self.RealValues[self.domain_info.LocationBook1_index] = split_fluent[1]
				elif(split_fluent[0] == 'book2'): self.RealValues[self.domain_info.LocationBook2_index] = split_fluent[1]
			elif(fluent[0:8] == 'in_hand('):
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'book1'): self.RealValues[self.domain_info.In_handBook1_index] = 'true'
				if(split_fluent[1] == 'book2'): self.RealValues[self.domain_info.In_handBook2_index] = 'true'
		print 'realValues'
		print self.RealValues

	def getRefinedLocation(self):
		if self.RealValues[self.domain_info.LocationRobot_index] == 'library': return 'c1'
		elif self.RealValues[self.domain_info.LocationRobot_index] == 'kitchen': return 'c5'
		elif self.RealValues[self.domain_info.LocationRobot_index] == 'office1': return 'c9'
		elif self.RealValues[self.domain_info.LocationRobot_index] == 'office2': return 'c13'

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

		return self.getTheseObservations(relevant_indexes)

	def __getRealValues_as_obsList(self,step):
		obsList = []
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

	def __get_scenario_exo_action(self,action):
		scenario = self.scenario
		exo_action = ''

		if(scenario == 'random'):
			if(random.random()<0.08): return exo_action
			choice = random.choice(['library','book1','book2'])
			if(choice == 'book1'  and (self.RealValues[self.domain_info.In_handBook1_index] == 'true' or action == 'pickup(rob1,book1)')): choice = 'book2'
			elif(choice == 'book2'  and (self.RealValues[self.domain_info.In_handBook2_index] == 'true' or action == 'pickup(rob1,book2)')): choice = 'book1'
			if(choice == 'book1'):
				allLocations = list(self.domain_info.DomainLocations)
				currentLocation = self.RealValues[self.domain_info.LocationBook1_index]
				allLocations.remove(currentLocation)
				newLocation = random.choice(allLocations)
				exo_action =  'exo_move(book1,' +newLocation+ ')'
			elif(choice == 'book2'):
				allLocations = list(self.domain_info.DomainLocations)
				currentLocation = self.RealValues[self.domain_info.LocationBook2_index]
				allLocations.remove(currentLocation)
				newLocation = random.choice(allLocations)
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
