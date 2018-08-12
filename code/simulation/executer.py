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
	def test(self,test):
		fluent_test = test[test.find(',')+1:test.rfind(',')]
		value_test = test[test.rfind(',')+1:-1]
		test_result = True
		direct_observation = self.world.executeAction(test)
		if 'holds' not in direct_observation:
			if value_test == 'false': real_value = 'true'
			elif value_test == 'true': real_value = 'false'
			test_result = False
			direct_observation = 'holds(directly_observed(rob1,'+fluent_test+','+real_value+'),1)'
		else: real_value = value_test
		if 'loc(rob1' in test and test_result == False: raw_input('robot location test result is false ')
		return test_result,  direct_observation


	def getGoalFeedback(self):
		return self.world.getGoalFeedback()


	def __del__(self):
		print('deleting executer ')
