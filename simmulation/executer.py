class Executer(object):

	def __init__(self,thisWorld):
		self.world = thisWorld
	
	def executeAction(self, action):
		return self.world.executeAction(action)

	def getTheseObservations(self,indexes):
		return self.world.getTheseObservations(indexes)

	def getRealValues(self):
		
		return self.world.getRealValues()
	
	def getRobotLocation(self):
		return self.world.getRobotLocation()

	def getGoalFeedback(self):
		return self.world.getGoalFeedback()

	def getExecutedSteps(self):
		return self.world.getExecutedSteps()

	def __del__(self):
        	print('deleting executer ')