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

	def observations_to_obs(self, observations, robotLocation, step):
		obsList = []
		for observation in observations:
			if (observation[0] == self.LocationRobot_index and observation[1] != 'unknown'):
				obsList.append('obs(loc(rob1,'+str(observation[1])+ '),true,'+ str(step) +').')
			if (observation[0] == self.LocationBook1_index):
				if(observation[1] != 'unknown'):
					obsList.append('obs(loc(book1,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsList.append('obs(loc(book1,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.LocationBook2_index):
				if(observation[1] != 'unknown'):
					obsList.append('obs(loc(book2,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsList.append('obs(loc(book2,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.In_handBook1_index and observation[1] != 'unknown'):
				obsList.append('obs(in_hand(rob1,book1),' + observation[1]+ ','+ str(step) +').')
			if (observation[0] == self.In_handBook2_index and observation[1] != 'unknown'):
				obsList.append('obs(in_hand(rob1,book2),' + observation[1]+ ','+ str(step) +').')
		return obsList

	def abstractAnswerToCoarseState(self,answer):
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

	def coarseStateToAstractObsList(self,state,step):
		obsList = []
		if(state[self.LocationRobot_index] != 'unknown'):
			obsList.append('obs(loc(rob1,'+str(state[self.LocationRobot_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			obsList.append('obs(loc(book1,'+str(state[self.LocationBook1_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			obsList.append('obs(loc(book2,'+str(state[self.LocationBook2_index])+'),true,'+str(step)+').')
		if(state[self.In_handBook1_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book1),'+state[self.In_handBook1_index]+','+str(step)+').')
		if(state[self.In_handBook2_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,book2),'+state[self.In_handBook2_index]+','+str(step)+').')
		return obsList

	def refinedStateToRefinedObsList(self,state,step):
		obsList = []
		if(state[self.LocationRobot_index] != 'unknown'):
			obsList.append('obs(loc(rob1,'+str(state[self.LocationRobot_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			obsList.append('obs(loc(ref_book1,'+str(state[self.LocationBook1_index])+'),true,'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			obsList.append('obs(loc(ref_book2,'+str(state[self.LocationBook2_index])+'),true,'+str(step)+').')
		if(state[self.In_handBook1_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,ref_book1),'+state[self.In_handBook1_index]+','+str(step)+').')
		if(state[self.In_handBook2_index] != 'unknown'):
			obsList.append('obs(in_hand(rob1,ref_book2),'+state[self.In_handBook2_index]+','+str(step)+').')
		return obsList

	def getIndexesRelevantToGoal(self,goal):
		indexes = Set([])
		goal = goal.rstrip()
		conditions = goal.split(', ')
		for condition in conditions:
			print 'condition ' +  condition
			fluent = condition[condition.find('(') + 1: condition.rfind(',')]
			print 'fluent ' + fluent
			if 'in_hand' in fluent:
				if 'book1' in fluent: indexes.add(self.In_handBook1_index)
				elif 'book2' in fluent: indexes.add(self.In_handBook2_index)
			elif 'loc' in fluent:
				if 'rob1' in fluent: indexes.add(self.LocationRobot_index)
				elif 'book1' in fluent: indexes.add(self.LocationBook1_index)
				elif 'book2' in fluent: indexes.add(self.LocationBook2_index)
		return indexes
