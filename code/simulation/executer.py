class Executer(object):
	def __init__(self,thisWorld):
		self.world = thisWorld

	def executeAction(self, action):
		self.world.executeAction(action)


    ######################################################
    # funcitons with a test_action as parameterself.
	# it returns two values: a boolean with the test results
	# and a tuple with the fluent to be tested and the observed
	# value of this fluent
	######################################################
	def test(self,test):
		fluent_test = test[test.find(',')+1:test.rfind(',')]
		value_test = test[test.rfind(',')+1:-1]
		test_result = True
		direct_observation = self.world.executeAction(test)
		if fluent_test not in direct_observation:
			if value_test == 'false': real_value = 'true'
			elif value_test == 'true': real_value = 'false'
			test_result = False
		else: real_value = value_test
		return test_result,  direct_observation

	def getTheseCoarseObservations(self,indexes):
		return self.world.getTheseCoarseObservations(indexes)

	def getGoalFeedback(self):
		return self.world.getGoalFeedback()

	def getExecutedSteps(self):
		return self.world.getExecutedSteps()

	def __del__(self):
		print('deleting executer ')
