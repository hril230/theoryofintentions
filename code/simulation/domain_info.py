class DomainInfo():
	LibraryLocked_index = 0
	LocationRobot_index = 1
	LocationBook1_index = 2
	LocationBook2_index = 3
	In_handBook1_index = 4
	In_handBook2_index = 5
	DomainLocations = ['office1', 'office2' ,'kitchen','library']
	#following the order of indexes in World class variables
	ObsStringList = ['obs(locked(library),','obs(loc(rob1,','obs(loc(book1,','obs(loc(book2,','obs(in_hand(rob1,book1),','obs(in_hand(rob1,book2),']
	def __init__(self,name):
		self.name = name

	def observations_to_obsList(self,observations, robotLocation, step):
		obsList = []
		for observation in observations:
			if (observation[0] == DomainInfo.LibraryLocked_index and observation[1] != 'unknown'):
				obsList.append('obs(locked(library),'+ observation[1] + ',' + str(step) +').')
			if (observation[0] == DomainInfo.LocationRobot_index and observation[1] != 'unknown'):
				obsList.append('obs(loc(rob1,'+str(observation[1])+ '),true,'+ str(step) +').')
			if (observation[0] == DomainInfo.LocationBook1_index):
				if(observation[1] != 'unknown'):
					obsList.append('obs(loc(book1,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsList.append('obs(loc(book1,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == DomainInfo.LocationBook2_index):
				if(observation[1] != 'unknown'):
					obsList.append('obs(loc(book2,' +str(observation[1])+ '),true,'+ str(step) +').')
				else:
					obsList.append('obs(loc(book2,' +str(robotLocation)+ '),false,'+ str(step) +').')
			if (observation[0] == DomainInfo.In_handBook1_index and observation[1] != 'unknown'):
				obsList.append('obs(in_hand(rob1,book1),' + observation[1]+ ','+ str(step) +').')
			if (observation[0] == DomainInfo.In_handBook2_index and observation[1] != 'unknown'):
				obsList.append('obs(in_hand(rob1,book2),' + observation[1]+ ','+ str(step) +').')
		return obsList
