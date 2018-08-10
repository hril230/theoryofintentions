from sets import Set
class DomainInfo():

	CoarseLocations = ['library','kitchen','office1', 'office2']
	CoarseLocationsAsCells = [['c1','c2','c3','c4'],['c5','c6','c7','c8'],['c9','c10','c11','c12'],['c13','c14','c15','c16']]
	RefinedLocations = ['c1','c2','c3','c4','c5','c6','c7','c8','c9','c10','c11','c12','c13','c14','c15','c16']
	def __init__(self):
		self.LocationRobot_index = 0
		self.LocationBook1_index = 1
		self.LocationBook2_index = 2
		self.In_handBook1_index = 3
		self.In_handBook2_index = 4

	def observations_to_obs_set(self, observations, robotLocation, step):
		obsSet = Set()
		for observation in observations:
			if (observation[0] == self.LocationRobot_index and observation[1] != 'unknown'):
				obsSet.add('obs(loc(rob1,'+str(observation[1])+ '),true,'+ str(step) +').')
			if (observation[0] == self.LocationBook1_index):
				if(observation[1] != 'unknown'):
					obsSet.add('obs(loc(book1,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsSet.add('obs(loc(book1,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.LocationBook2_index):
				if(observation[1] != 'unknown'):
					obsSet.add('obs(loc(book2,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsSet.add('obs(loc(book2,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.In_handBook1_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,book1),' + observation[1]+ ','+ str(step) +').')
			if (observation[0] == self.In_handBook2_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,book2),' + observation[1]+ ','+ str(step) +').')
		return obsSet

	def abstractAnswerToCoarseState(self,answer):
		answer = answer.rstrip().strip('{').strip('}')
		state = ['unknown'] * 5
		for holds in answer.split(', '):
			if holds[0] == '-':
				fluent = holds[7:holds.rfind(',')]
				if(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): state[self.In_handBook1_index] = 'false'
					if(split_fluent[1] == 'book2'): state[self.In_handBook2_index] = 'false'
			else:
				fluent = holds[6:holds.rfind(',')]
				if(fluent[0:4] == 'loc('):
					fluent = fluent[4:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): state[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'book1'):state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'book2'): state[self.LocationBook2_index] = split_fluent[1]
				elif(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): state[self.In_handBook1_index] = 'true'
					if(split_fluent[1] == 'book2'): state[self.In_handBook2_index] = 'true'
		return state

	def refinedAnswerToRefinedState(self,answer):
		answer = answer.rstrip().strip('{').strip('}')
		state = ['unknown'] * 5
		for holds in answer.split(', '):
			if holds[0] == '-':
				fluent = holds[7:holds.rfind(',')]
				if(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'ref_book1'): state[self.In_handBook1_index] = 'false'
					if(split_fluent[1] == 'ref_book2'): state[self.In_handBook2_index] = 'false'
			else:
				fluent = holds[6:holds.rfind(',')]
 				if(fluent[0:4] == 'loc('):
					fluent = fluent[4:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): state[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref_book1'):state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref_book2'): state[self.LocationBook2_index] = split_fluent[1]
				elif(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'ref_book1'): state[self.In_handBook1_index] = 'true'
					if(split_fluent[1] == 'ref_book2'): state[self.In_handBook2_index] = 'true'
		return state

	def refinedAnswerToCoarseState(self,answer):
		state = ['unknown'] * 5
		for holds in answer.split(', '):
			if holds[0] == '-':
				fluent = holds[7:holds.rfind(',')]
				if(fluent[0:15] == 'coarse_in_hand('):
					fluent = fluent[15:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): state[self.In_handBook1_index] = 'false'
					if(split_fluent[1] == 'book2'): state[self.In_handBook2_index] = 'false'
			else:
				fluent = holds[6:holds.rfind(',')]
				if(fluent[0:11] == 'coarse_loc('):
					fluent = fluent[11:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): state[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'book1'):state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'book2'): state[self.LocationBook2_index] = split_fluent[1]
				elif(fluent[0:15] == 'coarse_in_hand('):
					fluent = fluent[15:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): state[self.In_handBook1_index] = 'true'
					if(split_fluent[1] == 'book2'): state[self.In_handBook2_index] = 'true'
		return state

	def coarseStateToAstractObsSet(self,state,step):
		obsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			obsSet.add('obs(loc(rob1,'+str(state[self.LocationRobot_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			obsSet.add('obs(loc(book1,'+str(state[self.LocationBook1_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			obsSet.add('obs(loc(book2,'+str(state[self.LocationBook2_index])+'),true,'+str(step)+').')
		if(state[self.In_handBook1_index] != 'unknown'):
			obsSet.add('obs(in_hand(rob1,book1),'+state[self.In_handBook1_index]+','+str(step)+').')
		if(state[self.In_handBook2_index] != 'unknown'):
			obsSet.add('obs(in_hand(rob1,book2),'+state[self.In_handBook2_index]+','+str(step)+').')
		return obsSet

	def coarseStateToCoarseObsSet(self,state,step):
		obsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			obsSet.add('obs(coarse_loc(rob1,'+str(state[self.LocationRobot_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			obsSet.add('obs(coarse_loc(book1,'+str(state[self.LocationBook1_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			obsSet.add('obs(coarse_loc(book2,'+str(state[self.LocationBook2_index])+'),true,'+str(step)+').')
		if(state[self.In_handBook1_index] != 'unknown'):
			obsSet.add('obs(coarse_in_hand(rob1,book1),'+state[self.In_handBook1_index]+','+str(step)+').')
		if(state[self.In_handBook2_index] != 'unknown'):
			obsSet.add('obs(coarse_in_hand(rob1,book2),'+state[self.In_handBook2_index]+','+str(step)+').')
		return obsSet

	def refinedStateToRefinedObsSet(self,state,step):
		obsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			obsSet.add('obs(loc(rob1,'+str(state[self.LocationRobot_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			obsSet.add('obs(loc(ref_book1,'+str(state[self.LocationBook1_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			obsSet.add('obs(loc(ref_book2,'+str(state[self.LocationBook2_index])+'),true,'+str(step)+').')
		if(state[self.In_handBook1_index] != 'unknown'):
			obsSet.add('obs(in_hand(rob1,ref_book1),'+state[self.In_handBook1_index]+','+str(step)+').')
		if(state[self.In_handBook2_index] != 'unknown'):
			obsSet.add('obs(in_hand(rob1,ref_book2),'+state[self.In_handBook2_index]+','+str(step)+').')
		return obsSet


	def indirectObservationsToObsSet(self,indirectObservationsSet,step):
		newSet = set()
		for a in indirectObservationsSet:
			a = a.replace('indirect_observation_at_step','obs').replace('coarse_','')
			a = a[:a.rfind(',')+1] + str(step) +').'
			newSet.add(a)
		return newSet

	def getIndexesRelevantToGoal(self,goal):
		indexes = Set()
		goal = goal.rstrip()
		conditions = goal.split(', ')
		for condition in conditions:
			fluent = condition[condition.find('(') + 1: condition.rfind(',')]
			if 'in_hand' in fluent:
				if 'book1' in fluent: indexes.add(self.In_handBook1_index)
				elif 'book2' in fluent: indexes.add(self.In_handBook2_index)
			elif 'loc' in fluent:
				if 'rob1' in fluent: indexes.add(self.LocationRobot_index)
				elif 'book1' in fluent: indexes.add(self.LocationBook1_index)
				elif 'book2' in fluent: indexes.add(self.LocationBook2_index)
		return indexes

	def getObsFromAnswerObservations(self,answer):
		obsSet = Set()
		answer_split = answer.split(', ')
		for entry in answer_split:
			if('observed') in entry:
				obs_step = entry[entry.rfind(',')+1:entry.rfind(')')]
				entry =  entry[entry.find('(')+1:entry.rfind(',')]
				fluent = entry[entry.find(',')+1:entry.rfind(',')]
				value = entry[entry.rfind(',')+1:-1]
				obs = 'obs('+(',').join([fluent,value,obs_step])+')'
				obsSet.add(obs)
		return obsSet

	def getActionIndexes(self,action):
		relevant_indexes= Set()
		if('move' in action):
			relevant_indexes.add(self.LocationRobot_index)
		elif('pickup' in action or 'put_down' in action):
			if('book1' in action):
				relevant_indexes.add(self.In_handBook1_index)
				relevant_indexes.add(self.LocationBook1_index)
			if('book2' in action):
				relevant_indexes.add(self.In_handBook2_index)
				relevant_indexes.add(self.LocationBook2_index)
		return relevant_indexes

	def getFluentIndex(self,fluent):
		if 'loc' in fluent:
			if 'rob' in fluent: return self.LocationRobot_index
			if 'book1' in fluent: return self.LocationBook1_index
			if 'book2' in fluent: return self.LocationBook2_index
		if 'in_hand' in fluent:
			if 'book1' in fluent: return self.In_handBook1_index
			if 'book2' in fluent: return self.In_handBook2_index
