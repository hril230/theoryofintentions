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
		fluent_to_observe = test_action[test_action.find(',')+1:test_action.rfind(',')]
		value_to_check = test_action[test_action.rfind(',')+1:-1]
		observed_value = self.world.directlyObserve(fluent_to_observe)
		directly_observed_string = 'directly_observed(rob1,'+fluent_to_observe+','+str(observed_value).lower()+')'
		return value_to_check == str(observed_value).lower(), directly_observed_string


	def isGoalReached(self,goal):
		return self.world.isGoalReached(goal)
 
