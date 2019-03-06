from sets import Set
class Executer(object):

	def __init__(self,thisWorld):
		self.world = thisWorld

	def executeAction(self, action):
		self.world.executeAction(action)

	def getRobotLocation(self):
		return self.world.getRobotLocation()

	def get_real_values(self,fluents, step):
		obs_set = Set()
		if not fluents: return obs_set
		for fluent in fluents:
			new_fluent, boolean = self.world.getRealValue(fluent)
			if new_fluent: obs_set.add('obs('+new_fluent+','+boolean+','+ str(step) +')')
		return obs_set

	def observe(self,robot_name,fluents, step):
		obs_set = Set()
		if not fluents: return obs_set
	 	observations = self.world.robotObserves(fluents)
		obs_set.update(Set([entry[:entry.rfind(',')+1].replace('directly_observed('+robot_name+',','obs(') + str(step) +')' for entry in observations]))
		return obs_set
