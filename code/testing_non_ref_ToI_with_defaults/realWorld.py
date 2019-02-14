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
''' This is a simulation of the real world would be, with observations as return and action as input
The real values are going to be kept in a dictionary with key of the form 'loc(rob1,' and value of the form 'library'
If the dictionary holds the key 'locked(library' the value for this key is None. This system to keep
the state of the world works only if our ASP's are working with fluents that have two parameters at most.
'''
from sets import Set
import subprocess
import random


class World(object):
	LibraryLocked_index = 0
	LocationRobot_index = 1
	LocationBook1_index = 2
	LocationBook2_index = 3
	In_handBook1_index = 4
	In_handBook2_index = 5
	DomainLocations = ['office1', 'office2' ,'kitchen','library']
	def __init__(self,new_domain_info_formatting,initialConditionsWorld, this_seed):
		self.domain_info_formatting = new_domain_info_formatting
		self.dic_WorldState = self._listToDictionary(initialConditionsWorld)
		self.RealValues = list(initialConditionsWorld)
		reader = open(self.domain_info_formatting.preASP_domain_file, 'r')
    		pre_asp = reader.read()
    		reader.close()
    		self.pre_asp_split = pre_asp.split('\n')
		self.history_marker_index = self.pre_asp_split.index(self.domain_info_formatting.history_marker) + 1
		self.exo_action_happened = True
		self.history = []
		self.executionTimeUnits = 0
		self.executedSteps = 0
		random.seed(this_seed)

	def _listToDictionary(self, stateList):
		dictionary = {}
		for a in stateList:
			if stateList.index(a)==self.LibraryLocked_index: dictionary['locked(library'] = None
			if stateList.index(a)==self.LocationRobot_index: dictionary['loc(rob'] = a
			if stateList.index(a)==self.LocationBook1_index: dictionary['loc(book1'] = a
			if stateList.index(a)==self.LocationBook2_index: dictionary['loc(book2'] = a
			if stateList.index(a)==self.In_handBook1_index and a=='true': dictionary['in_hand(rob1'] = 'book1'
			if stateList.index(a)==self.In_handBook2_index and a=='true': dictionary['in_hand(rob1'] = 'book2'
		#print [','.join([v for v in [a,b] if v])+')' for a,b in dictionary.items()]
		return dictionary

	def __updateDicWorldState(self,answer_split):
		self.dic_WorldState = {}
		my_fluents = [entry[entry.find('(')+1:entry.find(')')+1] for entry in answer_split if 'holds' in entry]
		my_fluents_splitted = [v.replace(')','').split(',') for v in my_fluents]
		for split_fluent in my_fluents_splitted:
			## if my fluent had a comma in it, for example 'loc(rob1,kitchen)', then the splitted version is a list with two elements,
			## and  the dictionary will have a key that corresponds to the first element of the list 'loc(rob1', with value
			## that corresponds to the second part of the fluent, i.e. 'kitchen' (the brackets are removed for formatting )
			if len(split_fluent)>1:
				print 'this is my v '+ str(split_fluent)
				self.dic_WorldState[split_fluent[0]] = split_fluent[1]
			## if my fluent did not have a comma, for example 'locked(library)', then the splitted version is a list wit only one element,
			## so the dictionary will have a key 'locked(library' with corresonding value None.
			else: self.dic_WorldState[split_fluent[0]] = None

	def __updateRealValues(self,answer_split):
		self.RealValues[0] = 'false'
		self.RealValues[4] = 'false'
		self.RealValues[5] = 'false'
		for fluent in answer_split:
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
		print self.RealValues


	def __getActionObservations(self,action,happened):
		relevant_indexes= Set([])
		if(action[0:4] == 'move'):
			relevant_indexes.add(World.LocationRobot_index)
			if(not happened):
				relevant_indexes.add(World.LibraryLocked_index)

		if(action == 'unlock(rob1,library)'):
			relevant_indexes.add(World.LibraryLocked_index)

		if(action == 'pickup(rob1,book1)' or action == '+put_down(rob1,book1)'):
			relevant_indexes.add(World.In_handBook1_index)
			relevant_indexes.add(World.LocationBook1_index)

		if(action == 'pickup(rob1,book2)' or action == '+put_down(rob1,book2)'):
			relevant_indexes.add(World.In_handBook2_index)
			relevant_indexes.add(World.LocationBook2_index)

		return self.getTheseObservations(relevant_indexes)

	def __getRealValues_as_obsList(self,step):
		obsList = []
		if(self.RealValues[World.LibraryLocked_index] != 'unknown'):
			obsList.append('obs(locked(library),'+self.RealValues[World.LibraryLocked_index]+','+str(step)+').')
		if(self.RealValues[World.LocationRobot_index] != 'unknown'):
			obsList.append('obs(loc(rob1,'+str(self.RealValues[World.LocationRobot_index])+'),true,'+str(step)+').')
		if(self.RealValues[World.LocationBook1_index] != 'unknown'):
			obsList.append('obs(loc(book1,'+str(self.RealValues[World.LocationBook1_index])+'),true,'+str(step)+').')
		if(self.RealValues[World.LocationBook2_index] != 'unknown'):
			obsList.append('obs(loc(book2,'+str(self.RealValues[World.LocationBook2_index])+'),true,'+str(step)+').')
		if(self.RealValues[World.In_handBook1_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book1),'+self.RealValues[World.In_handBook1_index]+','+str(step)+').')
		if(self.RealValues[World.In_handBook2_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book2),'+self.RealValues[World.In_handBook2_index]+','+str(step)+').')
		return obsList

	def __getexecutionTimeUnits(self,action):
		if(action[0:4] == 'move'):
			return 3
		if(action[0] == 'p'):
			return 1
		if(action[0:6] == 'unlock'):
			return 1

	def __get_exo_action(self,action):
		exo_action = ''
		if(random.random()<0.08): return exo_action
		choice = random.choice(['library','book1','book2'])
		if(choice == 'library' and self.RealValues[World.LibraryLocked_index] == 'false'): return 'exo_lock(library)'
		elif(choice == 'book1'  and (self.RealValues[World.In_handBook1_index] == 'true' or action == 'pickup(rob1,book1)')): choice = 'book2'
		elif(choice == 'book2'  and (self.RealValues[World.In_handBook2_index] == 'true' or action == 'pickup(rob1,book2)')): choice = 'book1'
		if(choice == 'book1'):
			allLocations = list(self.DomainLocations)
			currentLocation = self.RealValues[World.LocationBook1_index]
			allLocations.remove(currentLocation)
			newLocation = random.choice(allLocations)
			if((newLocation != 'library' and currentLocation != 'library') or self.RealValues[World.LibraryLocked_index] == 'false'):
				exo_action =  'exo_move(book1,' +newLocation+ ')'
		elif(choice == 'book2'):
			allLocations = list(self.DomainLocations)
			currentLocation = self.RealValues[World.LocationBook2_index]
			allLocations.remove(currentLocation)
			newLocation = random.choice(allLocations)
			if((newLocation != 'library' and currentLocation != 'library') or self.RealValues[World.LibraryLocked_index] == 'false'):
				exo_action =  'exo_move(book2,' +newLocation+ ')'
		return exo_action


	def __del__(self):
        	print('deleting world ')

	def __runASPDomain(self,input):
		asp_split = self.pre_asp_split[:self.history_marker_index] + input + self.pre_asp_split[self.history_marker_index:]
		asp = '\n'.join(asp_split)
		f1 = open(self.domain_info_formatting.asp_World_file, 'w')
		f1.write(asp)
		f1.close()
		print (self.domain_info_formatting.asp_World_file)
		answer = subprocess.check_output('java -jar '+ self.domain_info_formatting.sparc_path + ' ' +self.domain_info_formatting.asp_World_file+' -A',shell=True)
		answer_split = answer.strip('{}\n').split(', ')
		return answer_split

	def executeAction(self,action):
		happened = False
		exo_action = ''
		input = self.__getRealValues_as_obsList(0) + ['hpd('+ action +',0).']
		if(self.exo_action_happened == False):
			exo_action = self.__get_exo_action(action)
			if(exo_action != ''): input = input + ['hpd('+ exo_action +',0).']
		asp_split = self.pre_asp_split[0:self.history_marker_index] + input + self.pre_asp_split[self.history_marker_index:]
		asp = '\n'.join(asp_split)
		f1 = open(self.domain_info_formatting.asp_World_file, 'w')
		f1.write(asp)
		f1.close()
		answer_split = self.__runASPDomain(input)
		self.executionTimeUnits += self.__getexecutionTimeUnits(action)
		self.executedSteps += 1
		if(answer_split and 'occurs('+ action +',0)' in answer_split):
			happened = True
			self.__updateRealValues(answer_split)
			self.__updateDicWorldState(answer_split)
			self.history.append(action)
			if(exo_action != ''):
				self.history.append(exo_action)
				self.exo_action_happened = True
 				print('%%%%%%%%%     exo_action happened in realWorld.py :  '+exo_action)
		else:
			happened = False
			self.history.append(action + " (FAILED) ")
		return (self.__getActionObservations(action, happened))

	def getRealValues(self):
		observations =[]
		for index, val in enumerate(self.RealValues):
			observations.append([index,val])
		return observations

	def getTheseObservations(self,indexes):
		observableValues = list(self.RealValues)
		robotLocation = self.RealValues[World.LocationRobot_index]
		observations = []
		if(robotLocation != 'library' and robotLocation != 'kitchen'):
			observableValues[World.LibraryLocked_index] = 'unknown'
		if(self.RealValues[World.LocationBook1_index] != robotLocation):
			observableValues[World.LocationBook1_index] = 'unknown'
		if(self.RealValues[World.LocationBook2_index] != robotLocation):
			observableValues[World.LocationBook2_index] = 'unknown'
		for index in indexes:
			observations.append([index,observableValues[index]])
		return observations

	def getRobotLocation(self):
		return self.RealValues[World.LocationRobot_index]

	def achievedGoal(self):
		if(self.RealValues[World.LocationBook1_index] != 'library'): return 'false'
		elif(self.RealValues[World.LocationBook2_index] != 'library'): return 'false'
		elif(self.RealValues[World.In_handBook1_index] == 'true'): return 'false'
		elif(self.RealValues[World.In_handBook2_index] == 'true'): return 'false'
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
