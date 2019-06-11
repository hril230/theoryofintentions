## This class simulates a world at refined level and the interactions between the exectuer and the world. For the
## representation of the states of the world and the changes in states we use an ASP called Real_World.sp
## In this world there are exogeneous actions that happen randomly

# The state of this world is held in a dictionary called dic_RefinedState were the elements of the dictionary
# are of the form:
#'coarse_loc(rob1': 'office2',
#'coarse_loc(book2': 'office2',
#'in_hand(rob1': 'ref3_book1',
#'coarse_in_hand(rob1': 'book1',
#'loc(ref3_book2': 'c14'
from pprint import pprint
import subprocess
import random
history_marker = '%% HISTORY GOES HERE'
display_marker = 'display'
asp_Refined_World_file = 'simulation/ASP_files/Real_World.sp'


class World(object):
	def __init__(self,dic_initial_state, new_domain_info):
		self.domain_info = new_domain_info
		preASP_real_world_file = 'simulation/pre_ASP_files/preASP_real_world.txt'
		reader = open(preASP_real_world_file, 'r')
		pre_asp = reader.read()
		reader.close()
		self.pre_asp_split = pre_asp.split('\n')
		self.history_marker_index = self.pre_asp_split.index(history_marker) + 1
		self.display_marker_index = self.pre_asp_split.index(display_marker) + 2
		self.exo_action_happened = True
		self.history = []
		self.executionTimeUnits = 0
		self.executedSteps = 0
		obs_list = list(self.domain_info.dic_refinedStateToRefinedHoldsSet(dic_initial_state,0))
		answer_list = self.__runASPDomain(obs_list)
		self.dic_RefinedState = self.domain_info.dic_answerToState(answer_list)


	def __getExecutionTimeUnits(self,action):
		if 'move' in action: return 1
		elif 'pickup' in action or 'put_down' in action: return 3
		elif 'test' in action: return 1
		return 0

	def __getRandomExoAction(self,action):
		print ('EXO ACTION HAPPENING!!')
		exo_action = ''
		if(random.random()<0): return ''
		object_choice = random.choice(self.domain_info.refined_sorts_hierarchy_dic['#coarse_object'])
		if(object_choice in action and 'pickup' in action): return ''
		if 'coarse_in_hand(rob1' + object_choice in self.dic_RefinedState: return ''
		location_choice = random.chocie(self.domain_info.refined_sorts_hierarchy_dic['#coarse_object'])
		if self.dic_RefinedState['loc(ref1_'+object_choice] == location_choice: return ''
		return 'hpd(exo_move(ref1_'+object_choice+','+location_choice+'),0).'

	def executeAction(self,action):
		happened = False
		input = list(self.domain_info.dic_refinedStateToRefinedHoldsSet(self.dic_RefinedState,0))
		input.append('hpd('+ action +',0).')
		#input.append(self.__getRandomExoAction(action))
		#print('refined action happening: ' + action)
		answer_list = self.__runASPDomain(input)
		self.executionTimeUnits += self.__getExecutionTimeUnits(action)
		self.executedSteps += 1
		if(answer_list and 'occurs('+ action +',0)' in answer_list):
			happened = True
			self.dic_RefinedState = self.domain_info.dic_answerToState(answer_list)
			self.history.append(action)
		else:
			print ('                nothing happned in real world ')
			self.history.append(action + "realWorld -  (FAILED) ")



	def __getDirectObservation(self,answer):
		answer = answer.rstrip().strip('{').strip('}')
		for entry in answer.split(', '):
			if 'directly_observed' in entry:
				return entry
		return ''

	# it takes a fluent_string as input and returns a boolean that represents if this fluent has or not been directly observed.
	def directlyObserve(self,fluent_to_observe):
		fluent_to_observe_split = fluent_to_observe[:-1].split(',')
		# If robot is testing the location of something rather than itself, it will only return True if the object
		# is in its same location, or false if it is not.
		if fluent_to_observe_split[0] in self.dic_RefinedState:
			if 'loc' in fluent_to_observe_split[0] and not 'rob1' in fluent_to_observe_split[0]:
					return self.dic_RefinedState[fluent_to_observe_split[0]] == self.dic_RefinedState['loc(rob1']
			return self.dic_RefinedState[fluent_to_observe_split[0]] == fluent_to_observe_split[1]
		return False




	def __runASPDomain(self,input):
		asp_split = self.pre_asp_split[:self.history_marker_index] + input + self.pre_asp_split[self.history_marker_index:]
		asp = '\n'.join(asp_split)
		f1 = open(asp_Refined_World_file, 'w')
		f1.write(asp)
		f1.close()
		print (asp_Refined_World_file)
		answer = subprocess.check_output('java -jar '+ self.domain_info.sparc_path + ' ' +asp_Refined_World_file+' -A',shell=True)
		answer_split = ((answer.rstrip().split('\n\n'))[0]).strip('{}\n').split(', ')
		return answer_split


	def isGoalReached(self,abstract_goal):
		abstract_goal_string_split = abstract_goal.replace(' ','').split(',holds')
		coarse_fluents_list = ['coarse_'+entry[entry.find('(')+1:entry.find(')')+1] for entry in abstract_goal_string_split]
		for fluent in coarse_fluents_list:
			fluent_split = fluent.replace(')','').split(',')
			if fluent_split[0] not in self.dic_RefinedState or self.dic_RefinedState[fluent_split[0]] != fluent_split[1]:
				return False
		return True

	def getHistory(self):
		return self.history

	def getExecutionTimeUnits(self):
		return self.executionTimeUnits

	def getAbstractState(self):
		asbtract_state_dic = {}
		for key,value in self.dic_RefinedState.iteritems():
			if 'coarse' in key:  asbtract_state_dic[key.replace('coarse_','')] = value
		return asbtract_state_dic

	def getRobotRefinedLocation(self):
		return self.dic_RefinedState['loc(rob1']
