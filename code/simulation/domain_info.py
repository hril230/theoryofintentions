from sets import Set
class DomainInfo():

	CoarseLocations = ['library', 'kitchen', 'office1', 'office2', 'storage_cupboard']
	CoarseLocationsAsCells = [['c1', 'c2', 'c3', 'c4', 'c5'], ['c6', 'c7', 'c8', 'c9', 'c10'], ['c11', 'c12', 'c13', 'c14', 'c15'], ['c16', 'c17', 'c18', 'c19', 'c20'], ['c21', 'c22', 'c23', 'c24', 'c25']]
	def __init__(self):
		self.LocationRobot_index = 0
		self.LocationBook1_index = 1
		self.LocationBook2_index = 2
		self.LocationBook3_index = 3
		self.LocationBook4_index = 4
		self.In_handBook1_index = 5
		self.In_handBook2_index = 6
		self.In_handBook3_index = 7
		self.In_handBook4_index = 8
		self.In_handBook1_Ref1_index = 9
		self.In_handBook1_Ref2_index = 10
		self.In_handBook1_Ref3_index = 11
		self.In_handBook1_Ref4_index = 12
		self.In_handBook2_Ref1_index = 13
		self.In_handBook2_Ref2_index = 14
		self.In_handBook2_Ref3_index = 15
		self.In_handBook2_Ref4_index = 16
		self.In_handBook3_Ref1_index = 17
		self.In_handBook3_Ref2_index = 18
		self.In_handBook3_Ref3_index = 19
		self.In_handBook3_Ref4_index = 20
		self.In_handBook4_Ref1_index = 21
		self.In_handBook4_Ref2_index = 22
		self.In_handBook4_Ref3_index = 23
		self.In_handBook4_Ref4_index = 24
		self.num_coarse_indexes = 9
		self.num_refined_indexes = 25
		self.coarse_state = ['false'] * self.num_coarse_indexes
		self.refined_state = ['false'] * self.num_refined_indexes

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
			if (observation[0] == self.LocationBook3_index):
				if(observation[1] != 'unknown'):
					obsSet.add('obs(loc(book3,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsSet.add('obs(loc(book3,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.LocationBook4_index):
				if(observation[1] != 'unknown'):
					obsSet.add('obs(loc(book4,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsSet.add('obs(loc(book4,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.In_handBook1_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,book1),' + observation[1]+ ','+ str(step) +').')
			if (observation[0] == self.In_handBook2_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,book2),' + observation[1]+ ','+ str(step) +').')
			if (observation[0] == self.In_handBook3_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,book3),' + observation[1]+ ','+ str(step) +').')
			if (observation[0] == self.In_handBook4_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,book4),' + observation[1]+ ','+ str(step) +').')
		return obsSet

	def abstractAnswerToCoarseState(self,answer):
		answer = answer.rstrip().strip('{').strip('}')
		for holds in answer.split(', '):
			if holds[0] == '-':
				fluent = holds[7:holds.rfind(',')]
				if(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): self.coarse_state[self.In_handBook1_index] = 'false'
					if(split_fluent[1] == 'book2'): self.coarse_state[self.In_handBook2_index] = 'false'
					if(split_fluent[1] == 'book3'): self.coarse_state[self.In_handBook3_index] = 'false'
					if(split_fluent[1] == 'book4'): self.coarse_state[self.In_handBook4_index] = 'false'
			else:
				fluent = holds[6:holds.rfind(',')]
				if(fluent[0:4] == 'loc('):
					fluent = fluent[4:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): self.coarse_state[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'book1'): self.coarse_state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'book2'): self.coarse_state[self.LocationBook2_index] = split_fluent[1]
					elif(split_fluent[0] == 'book3'): self.coarse_state[self.LocationBook3_index] = split_fluent[1]
					elif(split_fluent[0] == 'book4'): self.coarse_state[self.LocationBook4_index] = split_fluent[1]
				elif(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): self.coarse_state[self.In_handBook1_index] = 'true'
					if(split_fluent[1] == 'book2'): self.coarse_state[self.In_handBook2_index] = 'true'
					if(split_fluent[1] == 'book3'): self.coarse_state[self.In_handBook3_index] = 'true'
					if(split_fluent[1] == 'book4'): self.coarse_state[self.In_handBook4_index] = 'true'
		return self.coarse_state

	def refinedAnswerToRefinedState(self,answer):
		answer = answer.split('}')
		answer = answer[0]
		answer = answer.rstrip().strip('{').strip('}')
		new_refined_state = self.refined_state[:]
		new_refined_state[self.In_handBook1_Ref1_index] = 'false'
		new_refined_state[self.In_handBook1_Ref2_index] = 'false'
		new_refined_state[self.In_handBook1_Ref3_index] = 'false'
		new_refined_state[self.In_handBook1_Ref4_index] = 'false'
		new_refined_state[self.In_handBook2_Ref1_index] = 'false'
		new_refined_state[self.In_handBook2_Ref2_index] = 'false'
		new_refined_state[self.In_handBook2_Ref3_index] = 'false'
		new_refined_state[self.In_handBook2_Ref4_index] = 'false'
		new_refined_state[self.In_handBook3_Ref1_index] = 'false'
		new_refined_state[self.In_handBook3_Ref2_index] = 'false'
		new_refined_state[self.In_handBook3_Ref3_index] = 'false'
		new_refined_state[self.In_handBook3_Ref4_index] = 'false'
		new_refined_state[self.In_handBook4_Ref1_index] = 'false'
		new_refined_state[self.In_handBook4_Ref2_index] = 'false'
		new_refined_state[self.In_handBook4_Ref3_index] = 'false'
		new_refined_state[self.In_handBook4_Ref4_index] = 'false'
		for holds in answer.split(', '):
			if not (holds[0] == '-'):
				fluent = holds[6:holds.rfind(',')]
 				if(fluent[0:4] == 'loc('):
					fluent = fluent[4:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): new_refined_state[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref1_book1'): new_refined_state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref2_book1'): new_refined_state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref3_book1'): new_refined_state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref4_book1'): new_refined_state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref1_book2'): new_refined_state[self.LocationBook2_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref2_book2'): new_refined_state[self.LocationBook2_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref3_book2'): new_refined_state[self.LocationBook2_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref4_book2'): new_refined_state[self.LocationBook2_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref1_book3'): new_refined_state[self.LocationBook3_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref2_book3'): new_refined_state[self.LocationBook3_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref3_book3'): new_refined_state[self.LocationBook3_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref4_book3'): new_refined_state[self.LocationBook3_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref1_book4'): new_refined_state[self.LocationBook4_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref2_book4'): new_refined_state[self.LocationBook4_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref3_book4'): new_refined_state[self.LocationBook4_index] = split_fluent[1]
					elif(split_fluent[0] == 'ref4_book4'): new_refined_state[self.LocationBook4_index] = split_fluent[1]
				elif(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'ref1_book1'): new_refined_state[self.In_handBook1_Ref1_index] = 'true'
					if(split_fluent[1] == 'ref2_book1'): new_refined_state[self.In_handBook1_Ref2_index] = 'true'
					if(split_fluent[1] == 'ref3_book1'): new_refined_state[self.In_handBook1_Ref3_index] = 'true'
					if(split_fluent[1] == 'ref4_book1'): new_refined_state[self.In_handBook1_Ref4_index] = 'true'
					if(split_fluent[1] == 'ref1_book2'): new_refined_state[self.In_handBook2_Ref1_index] = 'true'
					if(split_fluent[1] == 'ref2_book2'): new_refined_state[self.In_handBook2_Ref2_index] = 'true'
					if(split_fluent[1] == 'ref3_book2'): new_refined_state[self.In_handBook2_Ref3_index] = 'true'
					if(split_fluent[1] == 'ref4_book2'): new_refined_state[self.In_handBook2_Ref4_index] = 'true'
					if(split_fluent[1] == 'ref1_book3'): new_refined_state[self.In_handBook3_Ref1_index] = 'true'
					if(split_fluent[1] == 'ref2_book3'): new_refined_state[self.In_handBook3_Ref2_index] = 'true'
					if(split_fluent[1] == 'ref3_book3'): new_refined_state[self.In_handBook3_Ref3_index] = 'true'
					if(split_fluent[1] == 'ref4_book3'): new_refined_state[self.In_handBook3_Ref4_index] = 'true'
					if(split_fluent[1] == 'ref1_book4'): new_refined_state[self.In_handBook4_Ref1_index] = 'true'
					if(split_fluent[1] == 'ref2_book4'): new_refined_state[self.In_handBook4_Ref2_index] = 'true'
					if(split_fluent[1] == 'ref3_book4'): new_refined_state[self.In_handBook4_Ref3_index] = 'true'
					if(split_fluent[1] == 'ref4_book4'): new_refined_state[self.In_handBook4_Ref4_index] = 'true'
		if (new_refined_state[self.In_handBook1_Ref1_index] == 'true') or (new_refined_state[self.In_handBook1_Ref2_index] == 'true') or (new_refined_state[self.In_handBook1_Ref3_index] == 'true'):
			new_refined_state[self.In_handBook1_index] = 'true'
		else: new_refined_state[self.In_handBook1_index] = 'false'
		if (new_refined_state[self.In_handBook2_Ref1_index] == 'true') or (new_refined_state[self.In_handBook2_Ref2_index] == 'true') or (new_refined_state[self.In_handBook2_Ref3_index] == 'true'):
			new_refined_state[self.In_handBook2_index] = 'true'
		else: new_refined_state[self.In_handBook2_index] = 'false'
		if (new_refined_state[self.In_handBook3_Ref1_index] == 'true') or (new_refined_state[self.In_handBook3_Ref2_index] == 'true') or (new_refined_state[self.In_handBook3_Ref3_index] == 'true'):
			new_refined_state[self.In_handBook3_index] = 'true'
		else: new_refined_state[self.In_handBook3_index] = 'false'
		if (new_refined_state[self.In_handBook4_Ref1_index] == 'true') or (new_refined_state[self.In_handBook4_Ref2_index] == 'true') or (new_refined_state[self.In_handBook4_Ref3_index] == 'true') or (new_refined_state[self.In_handBook4_Ref4_index] == 'true'):
			new_refined_state[self.In_handBook4_index] = 'true'
		else: new_refined_state[self.In_handBook4_index] = 'false'
		self.refined_state = new_refined_state
		return new_refined_state

	def refinedAnswerToCoarseState(self,answer):
		for holds in answer.split(', '):
			if holds[0] == '-':
				fluent = holds[7:holds.rfind(',')]
				if(fluent[0:15] == 'coarse_in_hand('):
					fluent = fluent[15:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): self.coarse_state[self.In_handBook1_index] = 'false'
					if(split_fluent[1] == 'book2'): self.coarse_state[self.In_handBook2_index] = 'false'
					if(split_fluent[1] == 'book3'): self.coarse_state[self.In_handBook3_index] = 'false'
					if(split_fluent[1] == 'book4'): self.coarse_state[self.In_handBook4_index] = 'false'
			else:
				fluent = holds[6:holds.rfind(',')]
				if(fluent[0:11] == 'coarse_loc('):
					fluent = fluent[11:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): self.coarse_state[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'book1'): self.coarse_state[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'book2'): self.coarse_state[self.LocationBook2_index] = split_fluent[1]
					elif(split_fluent[0] == 'book3'): self.coarse_state[self.LocationBook3_index] = split_fluent[1]
					elif(split_fluent[0] == 'book4'): self.coarse_state[self.LocationBook4_index] = split_fluent[1]
				elif(fluent[0:15] == 'coarse_in_hand('):
					fluent = fluent[15:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): self.coarse_state[self.In_handBook1_index] = 'true'
					if(split_fluent[1] == 'book2'): self.coarse_state[self.In_handBook2_index] = 'true'
					if(split_fluent[1] == 'book3'): self.coarse_state[self.In_handBook3_index] = 'true'
					if(split_fluent[1] == 'book4'): self.coarse_state[self.In_handBook4_index] = 'true'
		return self.coarse_state


	def coarseStateToAstractHoldsSet(self,state,step):
		holdsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			holdsSet.add('holds(loc(rob1,'+str(state[self.LocationRobot_index])+'),'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			holdsSet.add('holds(loc(book1,'+str(state[self.LocationBook1_index])+'),'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			holdsSet.add('holds(loc(book2,'+str(state[self.LocationBook2_index])+'),'+str(step)+').')
		if(state[self.LocationBook3_index] != 'unknown'):
			holdsSet.add('holds(loc(book3,'+str(state[self.LocationBook3_index])+'),'+str(step)+').')
		if(state[self.LocationBook4_index] != 'unknown'):
			holdsSet.add('holds(loc(book4,'+str(state[self.LocationBook4_index])+'),'+str(step)+').')
		if(state[self.In_handBook1_index] == 'true'):
			holdsSet.add('holds(in_hand(rob1,book1),'+str(step)+').')
		elif(state[self.In_handBook1_index] == 'false'):
			holdsSet.add('-holds(in_hand(rob1,book1),'+str(step)+').')
		if(state[self.In_handBook2_index] == 'true'):
			holdsSet.add('holds(in_hand(rob1,book2),'+str(step)+').')
		elif(state[self.In_handBook2_index] == 'false'):
			holdsSet.add('-holds(in_hand(rob1,book2),'+str(step)+').')
		if(state[self.In_handBook3_index] == 'true'):
			holdsSet.add('holds(in_hand(rob1,book3),'+str(step)+').')
		elif(state[self.In_handBook3_index] == 'false'):
			holdsSet.add('-holds(in_hand(rob1,book3),'+str(step)+').')
		if(state[self.In_handBook4_index] == 'true'):
			holdsSet.add('holds(in_hand(rob1,book4),'+str(step)+').')
		elif(state[self.In_handBook4_index] == 'false'):
			holdsSet.add('-holds(in_hand(rob1,book4),'+str(step)+').')
		return holdsSet


	def coarseStateToCoarseHoldsSet(self,state,step):
		holdsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(rob1,'+str(state[self.LocationRobot_index])+'),'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(book1,'+str(state[self.LocationBook1_index])+'),'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(book2,'+str(state[self.LocationBook2_index])+'),'+str(step)+').')
		if(state[self.LocationBook3_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(book3,'+str(state[self.LocationBook3_index])+'),'+str(step)+').')
		if(state[self.LocationBook4_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(book4,'+str(state[self.LocationBook4_index])+'),'+str(step)+').')
		if(state[self.In_handBook1_index] == 'true'):
			holdsSet.add('holds(coarse_in_hand(rob1,book1),'+str(step)+').')
		if(state[self.In_handBook1_index] == 'false'):
			holdsSet.add('-holds(coarse_in_hand(rob1,book1),'+str(step)+').')
		if(state[self.In_handBook2_index] == 'true'):
			holdsSet.add('holds(coarse_in_hand(rob1,book2),'+str(step)+').')
		if(state[self.In_handBook2_index] == 'false'):
			holdsSet.add('-holds(coarse_in_hand(rob1,book2),'+str(step)+').')
		if(state[self.In_handBook3_index] == 'true'):
			holdsSet.add('holds(coarse_in_hand(rob1,book3),'+str(step)+').')
		if(state[self.In_handBook3_index] == 'false'):
			holdsSet.add('-holds(coarse_in_hand(rob1,book3),'+str(step)+').')
		if(state[self.In_handBook4_index] == 'true'):
			holdsSet.add('holds(coarse_in_hand(rob1,book4),'+str(step)+').')
		if(state[self.In_handBook4_index] == 'false'):
			holdsSet.add('-holds(coarse_in_hand(rob1,book4),'+str(step)+').')
		return holdsSet


	def refinedStateToRefinedHoldsSet(self,state,step):
		holdsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			holdsSet.add('holds(loc(rob1,'+str(state[self.LocationRobot_index])+'),'+str(step)+').')
		if(state[self.LocationBook1_index] != 'unknown'):
			holdsSet.add('holds(loc(ref1_book1,'+str(state[self.LocationBook1_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref2_book1,'+str(state[self.LocationBook1_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref3_book1,'+str(state[self.LocationBook1_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref4_book1,'+str(state[self.LocationBook1_index])+'),'+str(step)+').')
		if(state[self.LocationBook2_index] != 'unknown'):
			holdsSet.add('holds(loc(ref1_book2,'+str(state[self.LocationBook2_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref2_book2,'+str(state[self.LocationBook2_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref3_book2,'+str(state[self.LocationBook2_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref4_book2,'+str(state[self.LocationBook2_index])+'),'+str(step)+').')
		if(state[self.LocationBook3_index] != 'unknown'):
			holdsSet.add('holds(loc(ref1_book3,'+str(state[self.LocationBook3_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref2_book3,'+str(state[self.LocationBook3_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref3_book3,'+str(state[self.LocationBook3_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref4_book3,'+str(state[self.LocationBook3_index])+'),'+str(step)+').')
		if(state[self.LocationBook4_index] != 'unknown'):
			holdsSet.add('holds(loc(ref1_book4,'+str(state[self.LocationBook4_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref2_book4,'+str(state[self.LocationBook4_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref3_book4,'+str(state[self.LocationBook4_index])+'),'+str(step)+').')
			holdsSet.add('holds(loc(ref4_book4,'+str(state[self.LocationBook4_index])+'),'+str(step)+').')
		if(state[self.In_handBook1_Ref1_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref1_book1),'+str(step)+').')
		if(state[self.In_handBook1_Ref2_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref2_book1),'+str(step)+').')
		if(state[self.In_handBook1_Ref3_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref3_book1),'+str(step)+').')
		if(state[self.In_handBook1_Ref4_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref4_book1),'+str(step)+').')
		if(state[self.In_handBook1_Ref1_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref1_book1),'+str(step)+').')
		if(state[self.In_handBook1_Ref2_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref2_book1),'+str(step)+').')
		if(state[self.In_handBook1_Ref3_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref3_book1),'+str(step)+').')
		if(state[self.In_handBook1_Ref4_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref4_book1),'+str(step)+').')
		if(state[self.In_handBook2_Ref1_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref1_book2),'+str(step)+').')
		if(state[self.In_handBook2_Ref2_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref2_book2),'+str(step)+').')
		if(state[self.In_handBook2_Ref3_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref3_book2),'+str(step)+').')
		if(state[self.In_handBook2_Ref4_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref4_book2),'+str(step)+').')
		if(state[self.In_handBook2_Ref1_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref1_book2),'+str(step)+').')
		if(state[self.In_handBook2_Ref2_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref2_book2),'+str(step)+').')
		if(state[self.In_handBook2_Ref3_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref3_book2),'+str(step)+').')
		if(state[self.In_handBook2_Ref4_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref4_book2),'+str(step)+').')
		if(state[self.In_handBook3_Ref1_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref1_book3),'+str(step)+').')
		if(state[self.In_handBook3_Ref2_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref2_book3),'+str(step)+').')
		if(state[self.In_handBook3_Ref3_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref3_book3),'+str(step)+').')
		if(state[self.In_handBook3_Ref4_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref4_book3),'+str(step)+').')
		if(state[self.In_handBook3_Ref1_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref1_book3),'+str(step)+').')
		if(state[self.In_handBook3_Ref2_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref2_book3),'+str(step)+').')
		if(state[self.In_handBook3_Ref3_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref3_book3),'+str(step)+').')
		if(state[self.In_handBook3_Ref4_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref4_book3),'+str(step)+').')
		if(state[self.In_handBook4_Ref1_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref1_book4),'+str(step)+').')
		if(state[self.In_handBook4_Ref2_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref2_book4),'+str(step)+').')
		if(state[self.In_handBook4_Ref3_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref3_book4),'+str(step)+').')
		if(state[self.In_handBook4_Ref4_index] == 'true'): holdsSet.add('holds(in_hand(rob1,ref4_book4),'+str(step)+').')
		if(state[self.In_handBook4_Ref1_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref1_book4),'+str(step)+').')
		if(state[self.In_handBook4_Ref2_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref2_book4),'+str(step)+').')
		if(state[self.In_handBook4_Ref3_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref3_book4),'+str(step)+').')
		if(state[self.In_handBook4_Ref4_index] == 'false'): holdsSet.add('-holds(in_hand(rob1,ref4_book4),'+str(step)+').')
		return holdsSet

	def indirectObservationsToObsSet(self,indirectObservationsSet,step):
		newSet = set()
		for a in indirectObservationsSet:
			a = a.replace('holds(indirectly_observed(rob1,', 'obs(').replace('coarse_','')
			a = a[:a.rfind('),')] + ','+str(step) +').'
			newSet.add(a)
		return newSet

	def directObservationToRefinedObs(self,directObservation,step):
		directObservation = directObservation.replace('holds(directly_observed(rob1,','obs(')
		directObservation = directObservation[:directObservation.rfind('),')]+','+str(step)+').\n'
		return directObservation

	def getIndexesRelevantToGoal(self,goal):
		indexes = Set()
		goal = goal.rstrip()
		conditions = goal.split(', ')
		for condition in conditions:
			fluent = condition[condition.find('(') + 1: condition.rfind(',')]
			if 'in_hand' in fluent:
				if 'book1' in fluent: indexes.add(self.In_handBook1_index)
				elif 'book2' in fluent: indexes.add(self.In_handBook2_index)
				elif 'book3' in fluent: indexes.add(self.In_handBook3_index)
				elif 'book4' in fluent: indexes.add(self.In_handBook4_index)
			elif 'loc' in fluent:
				if 'rob1' in fluent: indexes.add(self.LocationRobot_index)
				elif 'book1' in fluent: indexes.add(self.LocationBook1_index)
				elif 'book2' in fluent: indexes.add(self.LocationBook2_index)
				elif 'book3' in fluent: indexes.add(self.LocationBook3_index)
				elif 'book4' in fluent: indexes.add(self.LocationBook4_index)
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
			if('book3' in action):
				relevant_indexes.add(self.In_handBook3_index)
				relevant_indexes.add(self.LocationBook3_index)
			if('book4' in action):
				relevant_indexes.add(self.In_handBook4_index)
				relevant_indexes.add(self.LocationBook4_index)
		return relevant_indexes

	def getFluentIndex(self,fluent):
		if 'loc' in fluent:
			if 'rob' in fluent: return self.LocationRobot_index
			if 'book1' in fluent: return self.LocationBook1_index
			if 'book2' in fluent: return self.LocationBook2_index
			if 'book3' in fluent: return self.LocationBook3_index
			if 'book4' in fluent: return self.LocationBook4_index
		if 'in_hand' in fluent:
			if 'book1' in fluent: return self.In_handBook1_index
			if 'book2' in fluent: return self.In_handBook2_index
			if 'book3' in fluent: return self.In_handBook3_index
			if 'book4' in fluent: return self.In_handBook4_index
