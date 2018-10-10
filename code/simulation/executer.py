class Executer(object):
	def __init__(self,thisWorld):
		self.world = thisWorld

	def executeAction(self, action):
		self.world.executeAction(action)


    ######################################################
    # funcitons with a test_action as parameter for example
	# 'test(rob1,loc(ref_book1,c11),true)'
	# it returns two values: a boolean with the test results
	# and a holds statement about the tested fluent.
	# False, 'holds(directly_observed(rob1,loc(ref_book1,c11),false),1)'
	######################################################
	def test(self,test_action):
		tested_fluent = test_action[test_action.find(',')+1:test_action.rfind(',')]
		tested_value = test_action[test_action.rfind(',')+1:-1]
		test_result = None
		direct_observation = self.world.executeAction(test_action)
		short_observation = direct_observation[:direct_observation.rfind(',')]
		real_value = short_observation[short_observation.rfind(',')+1:-1]
		if tested_value != real_value: test_result = False
		else: test_result = True
		if 'loc(rob1' in test_action and test_result == False: raw_input('robot location test result is false ')
		return test_result,  direct_observation


	def getGoalFeedback(self):
		return self.world.getGoalFeedback()


	def __del__(self):
		print('deleting executer ')
