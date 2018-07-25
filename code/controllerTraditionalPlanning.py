'''
This program represnets the Controller of the agent that uses traidtional planning.
It needs three arguments:
1. The goal to be achieved (in terms of the fluents of the domain description).
2. The length of the longest possible plan of the domain.
3. An instance of the Executer that will execute the plan and obtain observations from the World.

Once the controller has run the simmulation, it will return:
1. The history of his actions and observations (as he believes them).
2. All the plans that has created during the run.
3. A boolean specifiying if it has been rectified for having produced, during diagnosis, the false assumption that the goal has been reached when it was not.


The initial coniditions of the knowledge representation of the domain will be observed and given by the executer. In this simmulations, we create it so that the executer can observe the state of the whole domain, and therefore the knowledge of the initial conditions is alwasy complete and True.
'''
from datetime import datetime
import subprocess
import re
import random

class ControllerTraditionalPlanning():
	def __init__(self, sparc_path, ASP_subfolder, domain_info, executer, known_world_state, newGoal, max_plan_length):
		self.planning_goal_marker = '%% @_@_@'
		self.planning_history_marker = '%% #_#_#'
		self.belief_history_marker = '%% *_*_*'

		self.domain_info = domain_info
		self.preASP_planning_file = subfolder_path + 'pre_ASP_files/preASP_Traditional.txt'
		self.preASP_domain_file = subfolder_path + 'pre_ASP_files/preASP_Domain.txt'
		self.asp_planning_file = subfolder_path + 'ASP_files/ASP_Traditional.sp'
		self.asp_belief_file = subfolder_path + 'ASP_files/ASP_Believe.sp'
		self.asp_diagnosing_file = subfolder_path + 'ASP_files/ASP_Diagnosis.sp'

		self.allPlans = []
		self.currentPlan = []
		self.currentStep = 0
		self.goal_correction = 0
		self.max_plan_length = maxPlanLen
		self.executer = executer
		self.goal = goal
		self.sparc_path = thisPath
		initialConditions = list(self.executer.getState())
		self.history = self.domain_info.observations_to_obs(initialConditions,self.executer.getMyCoarseLocation(),0)
		self.preparePreASP_string_lists()

	def run(self):
		needNewPlan = True
		while(needNewPlan == True):
			self.diagnose()
			self.updateBeliefWithDiagnosis()
			self.currentPlan = self.findPlan()
			needNewPlan = False
			if(len(self.currentPlan) > 1 and self.currentPlan[1] == "Goal holds" and self.executer.getGoalFeedback() == False):
				self.goal_correction += 1
				while(self.possibleDiagnosis and self.currentPlan[1] == "Goal holds"):
					self.updateBeliefWithDiagnosis()
					self.currentPlan = self.findPlan()
			if(self.currentPlan[0] == "No Plan"): break
			for action in self.currentPlan:
				print('Next action (traditional planning) : ' + str(action))
				relevantObservations = self.executer.executeAction(action)
				happened = self.update_belief(action,relevantObservations)
				if(happened == True):
					self.history.append('hpd('+action+','+str(self.currentStep)+').')
					self.currentStep += 1
					self.history = self.history + self.domain_info.observations_to_obs(relevantObservations,self.executer.getMyCoarseLocation(), self.currentStep)
					self.possibleDiagnosis = []
				else:
					print('Inconsistent observations, action did not happen, need to call planner')
					self.history = self.history + self.domain_info.observations_to_obs(relevantObservations,self.executer.getMyCoarseLocation(), self.currentStep)
					needNewPlan = True
					break
		print('%%%%%%%%%%%%%%  Finish Plan Traditional %%%%%%%%%%%%%%%% ')
		return (self.history, self.allPlans, self.goal_correction)


	def diagnose(self):
		asp_diagnosing_split = self.preASP_belief_split[:self.history_index_domain_asp] + self.history + self.preASP_belief_split[self.history_index_domain_asp+1:]
		asp_diagnosing_split[0] = "#const numSteps = "+str(self.currentStep)+"."
		asp = '\n'.join(asp_diagnosing_split)
		f1 = open(self.asp_diagnosing_file, 'w')
		f1.write(asp)
		f1.close()
		output = subprocess.check_output('java -jar '+self.sparc_path + ' ' + self.asp_diagnosing_file +' -A',shell=True)
		output_split = output.rstrip().split('\n\n')
		self.possibleDiagnosis = [a.strip('}').strip('{').split(', ') for a in output_split]


	def updateBeliefWithDiagnosis(self):
		chosenDiagnosis = self.possibleDiagnosis.pop()
		fluents = []
		for item in chosenDiagnosis:
			if('holds' in item): fluents.append(item)
			elif(item[0:4] == 'diag'):
				item = item.replace('diag', 'hpd')
				commaIndex = item.rfind(',')
				item = item[0:commaIndex+1] + 'true,'+item[commaIndex+1:]
		self.belief = self.domain_info.get_coarse_belief(', '.join(fluents))


	def findPlan(self):
		newPlan = []
		numSteps = 4 #the min number of steps of the first ASP to run.

		asp_planning_split = self.preASP_planning_split[:self.history_index_planning_asp] + self.domain_info.coarseStateToAstractObsList(self.belief,0) + self.preASP_planning_split[self.history_index_planning_asp+1:]

		answerSet = '\n'
		while(answerSet == '\n' and numSteps < self.max_plan_length):
			asp_planning_split[0] = '#const numSteps = '+ str(numSteps) + '.'
			asp_planning = '\n'.join(asp_planning_split)
	        	f1 = open(self.asp_planning_file, 'w')
			f1.write(asp_planning)
			f1.close()
			print('Looking for next plan (Trad) - numberSteps ' + str(numSteps))
		        answerSet = subprocess.check_output('java -jar '+self.sparc_path + ' ' +self.asp_planning_file+' -A ',shell=True)
			numSteps +=1

		if(answerSet == "\n"):
			newPlan = ["No Plan","Futile Goal/Inconsistent"]
		elif("{}" in answerSet):
			newPlan = ["No Plan","Goal holds"]
		elif(answerSet == ""):
			newPlan = ["No Plan","Unable to process"]
		else:
			plans = answerSet.rstrip().split('\n\n')
	    		plans = [plan for plan in plans if plan!='' and plan!='{}']
			#chosenPlan = random.choice(plans)
			chosenPlan = plans[0]
			plan_in_actions = chosenPlan.strip('}').strip('{').split(', ')
			splitActionsList = [action[:-1].split('),') for action in plan_in_actions]
			splitActionsList.sort(key=lambda x:int(x[1]))
	 		plan_in_actions = ['),'.join(action)+ ')' for action in splitActionsList]
			for a in plan_in_actions:
				newPlan.append(a[7:a.rfind(',')])
		self.allPlans.append(newPlan)
		return newPlan


	def update_belief(self,action, observations):
		input = self.domain_info.get_state_as_obsList(self.belief,0) + self.domain_info.observations_to_obs(observations,self.executer.getMyCoarseLocation(),1) + ['hpd('+ action +',0).']
		asp_belief_split = self.preASP_belief_split[:self.history_index_domain_asp] + input + self.preASP_belief_split[self.history_index_domain_asp+1:]
		asp = '\n'.join(asp_belief_split)
		f1 = open(self.asp_belief_file, 'w')
		f1.write(asp)
		f1.close()
		print('Next, checking belief-observations consistency ')
		output = subprocess.check_output('java -jar '+ self.sparc_path + ' ' + self.asp_belief_file +' -A',shell=True)
		output = output.strip('}').strip('{')
		if 'holds' in output:
			self.belief = self.domain_info.get_coarse_belief(output)
			return True
		else: return False


	def preparePreASP_string_lists(self):
		#preparing preASP_planning_split and history_index_planning_asp
		reader = open(self.preASP_planning_file, 'r')
		self.preASP_planning = reader.read()
		reader.close()
		self.preASP_planning_split = self.preASP_planning.split('\n')
		self.preASP_planning_split[0] = "#const numSteps = "+ str(self.max_plan_length)+"."
		index_goal = self.preASP_planning_split.index(self.planning_goal_marker)
		goal_formatted = "goal(I) :- "+ self.goal
		self.preASP_planning_split.insert(index_goal+1,  goal_formatted)
		self.history_index_planning_asp = self.preASP_planning_split.index(self.planning_history_marker)

		#preparing preASP_belief_split and history_index_domain_asp
		reader = open(self.preASP_domain_file, 'r')
		preASP_belief = reader.read()
		reader.close()
		self.preASP_belief_split = preASP_belief.split('\n')
		self.history_index_domain_asp = self.preASP_belief_split.index(self.belief_history_marker)




	def getBelief(self):
		return self.belief
