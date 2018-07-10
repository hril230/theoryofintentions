class DomainInfo():

	DomainLocations = ['library','kitchen','office1', 'office2']

	def __init__(self):
		self.LibraryLocked_index = 0
		self.LocationRobot_index = 1
		self.LocationBook1_index = 2
		self.LocationBook2_index = 3
		self.In_handBook1_index = 4
		self.In_handBook2_index = 5

	def observations_to_obsList(self,observations, robotLocation, step):
		obsList = []
		for observation in observations:
			if (observation[0] == self.LibraryLocked_index and observation[1] != 'unknown'):
				obsList.append('obs(locked(library),'+ observation[1] + ',' + str(step) +').')
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

	def getBelief_fromAnswer(self,answer):
		belief = ['unknown'] * 6
		for holds in answer.split(', '):
			if holds[0] == '-':
				fluent = holds[7:holds.rfind(',')]
				if(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): belief[self.In_handBook1_index] = 'false'
					if(split_fluent[1] == 'book2'): belief[self.In_handBook2_index] = 'false'
				if(fluent[0:7] == 'locked('):
					belief[self.LibraryLocked_index] = 'false'
			else:
				fluent = holds[6:holds.rfind(',')]
				if(fluent[0:7] == 'locked('):
					belief[self.LibraryLocked_index] = 'true'
				elif(fluent[0:4] == 'loc('):
					fluent = fluent[4:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[0] == 'rob1'): belief[self.LocationRobot_index] = split_fluent[1]
					elif(split_fluent[0] == 'book1'):belief[self.LocationBook1_index] = split_fluent[1]
					elif(split_fluent[0] == 'book2'): belief[self.LocationBook2_index] = split_fluent[1]
				elif(fluent[0:8] == 'in_hand('):
					fluent = fluent[8:-1]
					split_fluent = fluent.split(',')
					if(split_fluent[1] == 'book1'): belief[self.In_handBook1_index] = 'true'
					if(split_fluent[1] == 'book2'): belief[self.In_handBook2_index] = 'true'
		return belief

	def getBelief_as_obsList(self,belief,step):
		obsList = []
		if(belief[self.LibraryLocked_index] != 'unknown'): obsList.append('obs(locked(library),'+ belief[self.LibraryLocked_index] + ','+str(step)+').')
		if(belief[self.LocationRobot_index] != 'unknown'): obsList.append('obs(loc(rob1,' +str(belief[self.LocationRobot_index])+ '),true,'+str(step)+').')
		if(belief[self.LocationBook1_index] != 'unknown'): obsList.append('obs(loc(book1,' +str(belief[self.LocationBook1_index])+ '),true,'+str(step)+').')
		if(belief[self.LocationBook2_index] != 'unknown'): obsList.append('obs(loc(book2,' +str(belief[self.LocationBook2_index])+ '),true,'+str(step)+').')
		if(belief[self.In_handBook1_index] != 'unknown'): obsList.append('obs(in_hand(rob1,book1),' + belief[self.In_handBook1_index]+ ','+str(step)+').')
		if(belief[self.In_handBook2_index] != 'unknown'): obsList.append('obs(in_hand(rob1,book2),' + belief[self.In_handBook2_index]+ ','+str(step)+').')
		return obsList
