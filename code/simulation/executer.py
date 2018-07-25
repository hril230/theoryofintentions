class Executer(object):
	def __init__(self,thisWorld):
		self.world = thisWorld

	def executeAction(self, action):
		return self.world.executeAction(action)

	def getTheseCoarseObservations(self,indexes):
		return self.world.getTheseCoarseObservations(indexes)

	def getState(self):
		return self.world.getState()

	def getMyRefinedLocation(self):
		return self.world.getRobotsRefinedLocation()

	def getGoalFeedback(self):
		return self.world.getGoalFeedback()

	def getExecutedSteps(self):
		return self.world.getExecutedSteps()

	def __del__(self):
		print('deleting executer ')
