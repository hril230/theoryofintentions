from sets import Set
class DomainInfo():

	CoarseLocations = ['zoneR','zoneG','zoneY']
	CoarseLocationsAsCells = [['c1','c2','c3','c4'],['c5','c6','c7','c8'],['c9','c10','c11','c12']]
	RefinedLocations = ['c1','c2','c3','c4','c5','c6','c7','c8','c9','c10','c11','c12']

	def __init__(self):
		self.LocationRobot_index = 0
		self.LocationGreenBox_index = 1
		self.LocationBlueBox_index = 2
		self.In_handGreenBox_index = 3
		self.In_handBlueBox_index = 4

	def coarseStateToAstractObsSet(self,state,step):
		obsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			obsSet.add('obs(loc(rob1,'+str(state[self.LocationRobot_index])+'),true,'+str(step)+').')
		if(state[self.LocationGreenBox_index] != 'unknown'):
			obsSet.add('obs(loc(green_box,'+str(state[self.LocationGreenBox_index])+'),true,'+str(step)+').')
		if(state[self.LocationBlueBox_index] != 'unknown'):
			obsSet.add('obs(loc(blue_box,'+str(state[self.LocationBlueBox_index])+'),true,'+str(step)+').')
		if(state[self.In_handGreenBox_index] != 'unknown'):
			obsSet.add('obs(in_hand(rob1,green_box),'+state[self.In_handGreenBox_index]+','+str(step)+').')
		if(state[self.In_handBlueBox_index] != 'unknown'):
			obsSet.add('obs(in_hand(rob1,blue_box),'+state[self.In_handBlueBox_index]+','+str(step)+').')
		return obsSet

	def abstractAnswerToCoarseState(self,answers):
                answerlist = answers.split('}')
                answer = answerlist[0]
		answer = answer.rstrip().strip('{').strip('}')
		state = ['unknown'] * 5
		for holds in answer.split(', '):
			if holds[0] == '-':
				fluent = holds[7:holds.rfind(',')]
				if(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'green_box'): state[self.In_handGreenBox_index] = 'false'
					if(split_fluent[1] == 'blue_box'): state[self.In_handBlueBox_index] = 'false'
			else:
				fluent = holds[6:holds.rfind(',')]
				if(fluent[0:4] == 'loc('):
					fluent = fluent[4:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): state[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'green_box'):state[self.LocationGreenBox_index] = split_fluent[1]
					elif(split_fluent[0] == 'blue_box'): state[self.LocationBlueBox_index] = split_fluent[1]
				elif(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'green_box'): state[self.In_handGreenBox_index] = 'true'
					if(split_fluent[1] == 'blue_box'): state[self.In_handBlueBox_index] = 'true'
		return state


	def getActionIndexes(self,action):
		relevant_indexes= Set()
		if('move' in action):
			relevant_indexes.add(self.LocationRobot_index)
		elif('pickup' in action or 'put_down' in action):
			if('green_box' in action):
				relevant_indexes.add(self.In_handGreenBox_index)
				relevant_indexes.add(self.LocationGreenBox_index)
			if('blue_box' in action):
				relevant_indexes.add(self.In_handBlueBox_index)
				relevant_indexes.add(self.LocationBlueBox_index)
		return relevant_indexes

	def observations_to_obs_set(self, observations, robotLocation, step):
		obsSet = Set()
		for observation in observations:
			if (observation[0] == self.LocationRobot_index and observation[1] != 'unknown'):
				obsSet.add('obs(loc(rob1,'+str(observation[1])+ '),true,'+ str(step) +').')
			if (observation[0] == self.LocationGreenBox_index):
				if(observation[1] != 'unknown'):
					obsSet.add('obs(loc(green_box,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsSet.add('obs(loc(green_box,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.LocationBlueBox_index):
				if(observation[1] != 'unknown'):
					obsSet.add('obs(loc(blue_box,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsSet.add('obs(loc(blue_box,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == self.In_handGreenBox_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,green_box),' + observation[1]+ ','+ str(step) +').')
			if (observation[0] == self.In_handBlueBox_index and observation[1] != 'unknown'):
				obsSet.add('obs(in_hand(rob1,blue_box),' + observation[1]+ ','+ str(step) +').')
		return obsSet

	def coarseStateToAstractHoldsSet(self,state,step):
		holdsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			holdsSet.add('holds(loc(rob1,'+str(state[self.LocationRobot_index])+'),'+str(step)+').')
		if(state[self.LocationGreenBox_index] != 'unknown'):
			holdsSet.add('holds(loc(green_box,'+str(state[self.LocationGreenBox_index])+'),'+str(step)+').')
		if(state[self.LocationBlueBox_index] != 'unknown'):
			holdsSet.add('holds(loc(blue_box,'+str(state[self.LocationBlueBox_index])+'),'+str(step)+').')
		if(state[self.In_handGreenBox_index] == 'true'):
			holdsSet.add('holds(in_hand(rob1,green_box),'+str(step)+').')
		elif(state[self.In_handGreenBox_index] == 'false'):
			holdsSet.add('-holds(in_hand(rob1,green_box),'+str(step)+').')
		if(state[self.In_handBlueBox_index] == 'true'):
			holdsSet.add('holds(in_hand(rob1,blue_box),'+str(step)+').')
		elif(state[self.In_handBlueBox_index] == 'false'):
			holdsSet.add('-holds(in_hand(rob1,blue_box),'+str(step)+').')
		return holdsSet

	def directObservationToRefinedObs(self,directObservation,step):
		directObservation = directObservation.replace('holds(directly_observed(rob1,','obs(')
		directObservation = directObservation[:directObservation.rfind('),')]+','+str(step)+').\n'
		return directObservation

	def coarseStateToCoarseHoldsSet(self,state,step):
		holdsSet = set([])
		if(state[self.LocationRobot_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(rob1,'+str(state[self.LocationRobot_index])+'),'+str(step)+').')
		if(state[self.LocationGreenBox_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(green_box,'+str(state[self.LocationGreenBox_index])+'),'+str(step)+').')
		if(state[self.LocationBlueBox_index] != 'unknown'):
			holdsSet.add('holds(coarse_loc(blue_box,'+str(state[self.LocationBlueBox_index])+'),'+str(step)+').')
		if(state[self.In_handGreenBox_index] == 'true'):
			holdsSet.add('holds(coarse_in_hand(rob1,green_box),'+str(step)+').')
		if(state[self.In_handGreenBox_index] == 'false'):
			holdsSet.add('-holds(coarse_in_hand(rob1,green_box),'+str(step)+').')
		if(state[self.In_handBlueBox_index] == 'true'):
			holdsSet.add('holds(coarse_in_hand(rob1,blue_box),'+str(step)+').')
		if(state[self.In_handBlueBox_index] == 'false'):
			holdsSet.add('-holds(coarse_in_hand(rob1,blue_box),'+str(step)+').')
		return holdsSet

	def indirectObservationsToObsSet(self,indirectObservationsSet,step):
		newSet = set()
		for a in indirectObservationsSet:
			#a = a.replace('indirect_observation_at_step','obs').replace('coarse_','')
			a = a.replace('holds(indirectly_observed(rob1,', 'obs(').replace('coarse_','')
			a = a[:a.rfind('),')] + ','+str(step) +').'
			newSet.add(a)
		return newSet
