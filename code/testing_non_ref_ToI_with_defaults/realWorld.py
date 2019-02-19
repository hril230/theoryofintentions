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
	def __init__(self,new_domain_info_formatting,dic_initial_conditions, this_seed):
		print '\n-------------------------------------------------------------------------------------------------\n'
		print 'World initial conditions: ' + str(dic_initial_conditions)
		self.domain_info_formatting = new_domain_info_formatting
		self.dic_WorldState = dic_initial_conditions
		reader = open(self.domain_info_formatting.preASP_domain_file, 'r')
    		pre_asp = reader.read()
    		reader.close()
    		self.pre_asp_split = pre_asp.split('\n')
		self.history_marker_index = self.pre_asp_split.index(self.domain_info_formatting.history_marker) + 1
		self.exo_action_happened = True
		self.history = []
		self.executionTimeUnits = 0
		self.executedSteps = 0
		self.domain_locations = self.domain_info_formatting.get_all_basic_subsorts('#room')
		random.seed(this_seed)


	def __answer_to_dicWorldState(self,answer_split):
		self.dic_WorldState = {}
		my_fluents = [entry[entry.find('(')+1:entry.find(')')+1] for entry in answer_split if 'holds' in entry]
		my_fluents_splitted = [v.replace(')','').split(',') for v in my_fluents]
		for split_fluent in my_fluents_splitted:
			## if my fluent had a comma in it, for example 'loc(rob1,kitchen)', then the splitted version is a list with two elements,
			## and  the dictionary will have a key that corresponds to the first element of the list 'loc(rob1', with value
			## that corresponds to the second part of the fluent, i.e. 'kitchen' (the brackets are removed for formatting )
			if len(split_fluent)>1:
				self.dic_WorldState[split_fluent[0]] = split_fluent[1]
			## if my fluent did not have a comma, for example 'locked(library)', then the splitted version is a list wit only one element,
			## so the dictionary will have a key 'locked(library' with corresonding value None.
			else: self.dic_WorldState[split_fluent[0]] = None



	def __getActionObservations(self,action,happened):
		relevant_indexes= Set([])
		if(action[0:4] == 'move'):
			relevant_indexes.add(World.LocationRobot_index)
			if(not happened):
				relevant_indexes.add(World.LibraryLocked_index)

		if(action == 'unlock(rob1,library)'):
			relevant_indexes.add(World.LibraryLocked_index)

		if(action == 'pickup(rob1,book1)' or action == '+put_down(rob1,book1)'):
			relevant_indexes.add('in_hand(rob1')
			relevant_indexes.add('loc(book1')

		if(action == 'pickup(rob1,book2)' or action == '+put_down(rob1,book2)'):
			relevant_indexes.add('in_hand(rob1')
			relevant_indexes.add('loc(book2')

		return self.getTheseObservations(relevant_indexes)


	def __getexecutionTimeUnits(self,action):
		if(action[0:4] == 'move'):
			return 3
		if(action[0] == 'p'):
			return 1
		if(action[0:6] == 'unlock'):
			return 1

	def __get_exo_action(self,action):
		exo_action = ''
		'''
		if(random.random()<0.08): return exo_action
		choice = random.choice(['library','book1','book2'])
		if(choice == 'library' and self.dic_WorldState['locked(library'] == 'false'): return 'exo_lock(library)'
		elif(choice == 'book1'  and (self.dic_WorldState['in_hand(book1'] == 'true' or action == 'pickup(rob1,book1)')): choice = 'book2'
		elif(choice == 'book2'  and (self.dic_WorldState['in_hand(book2'] == 'true' or action == 'pickup(rob1,book2)')): choice = 'book1'
		if(choice == 'book1'):
			allLocations = list(self.domain_locations)
			currentLocation = self.dic_WorldState['loc(book1']
			allLocations.remove(currentLocation)
			newLocation = random.choice(allLocations)
			if((newLocation != 'library' and currentLocation != 'library') or self.dic_WorldState['locked(library'] == 'false'):
				exo_action =  'exo_move(book1,' +newLocation+ ')'
		elif(choice == 'book2'):
			allLocations = list(self.domain_locations)
			currentLocation = self.dic_WorldState['loc(book2']
			allLocations.remove(currentLocation)
			newLocation = random.choice(allLocations)
			if((newLocation != 'library' and currentLocation != 'library') or self.dic_WorldState['locked(library'] == 'false'):
				exo_action =  'exo_move(book2,' +newLocation+ ')'
		'''
		return exo_action



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
		input = self.domain_info_formatting.dic_state_to_obs_list(self.dic_WorldState,0) + ['hpd('+ action +',0).']

		if(self.exo_action_happened == False):
			exo_action = self.__get_exo_action(action)
			if(exo_action != ''): input = input + ['hpd('+ exo_action +',0).']
		answer_split = self.__runASPDomain(input)
		self.executionTimeUnits += self.__getexecutionTimeUnits(action)
		self.executedSteps += 1
		if(answer_split and 'occurs('+ action +',0)' in answer_split):
			happened = True
			self.__answer_to_dicWorldState(answer_split)
			self.history.append(action)
			if(exo_action != ''):
				self.history.append(exo_action)
				self.exo_action_happened = True
 				print('%%%%%%%%%     exo_action happened in realWorld.py :  '+exo_action)
		else:
			happened = False
			self.history.append(action + " (FAILED) ")


	def get_exo_action_happened(self):
		return self.exo_action_happened

	def getHistory(self):
		return self.history

	def getExecutionTimeUnits(self):
		return self.executionTimeUnits

	def getExecutedSteps(self):
		return self.executedSteps

	def robotObserves(self,fluent_to_observe):
		fluent_to_observe = fluent_to_observe.strip(')').split(',')
		new_fluent = [fluent_to_observe[0]]
		fluent_boolean = None
		if 'in_hand(' in fluent_to_observe[0]:
			if 'in_hand(rob1' in self.dic_WorldState:
				new_fluent.append(self.dic_WorldState['in_hand(rob1'])
				fluent_boolean = True
		 	else:
				new_fluent.append(fluent_to_observe[1])
			 	fluent_boolean = False
		if 'loc(rob1' == fluent_to_observe[0]:
			new_fluent.append(self.dic_WorldState['loc(rob1'])
			fluent_boolean = True
		if 'loc(' in fluent_to_observe[0] and 'rob' not in fluent_to_observe[0]:
			new_fluent.append(self.dic_WorldState['loc(rob1'])
			fluent_boolean = self.dic_WorldState[fluent_to_observe[0]] == self.dic_WorldState['loc(rob1']
		if 'locked(' in fluent_to_observe[0] and (self.dic_WorldState['loc(rob1']=='library' or self.dic_WorldState['loc(rob1']=='kitchen'):
			'locked is in the fluent, yes'
			fluent_boolean = fluent_to_observe[0] in self.dic_WorldState
		if fluent_boolean!=None:
			return ','.join(new_fluent)+')', str(fluent_boolean).lower()

	def getRealValue(self,fluent_to_observe):
		fluent_to_observe = fluent_to_observe.strip(')').split(',')
		new_fluent = [fluent_to_observe[0]]
		fluent_boolean = None
		if 'in_hand(' in fluent_to_observe[0]:
			if 'in_hand(rob1' in self.dic_WorldState:
				new_fluent.append(self.dic_WorldState['in_hand(rob1'])
				fluent_boolean = True
		 	else:
				new_fluent.append(fluent_to_observe[1])
			 	fluent_boolean = False
		if 'loc(' in fluent_to_observe[0]:
			new_fluent.append(self.dic_WorldState['loc(rob1'])
			fluent_boolean = True
		if 'locked(' in fluent_to_observe[0]:
			fluent_boolean = fluent_to_observe[0] in self.dic_WorldState
		if fluent_boolean!=None:
			return ','.join(new_fluent)+')', str(fluent_boolean).lower()
