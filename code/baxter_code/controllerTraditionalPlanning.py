
from datetime import datetime
import subprocess
import re
import baxter_controller as baxter
import global_variables


goal = ''
maxPlanLength = 0
history = []
currentStep = 0
belief = ['unknown']*7

allPlans = []  
currentPlan = [] # given by asp 
diagnosis = []


planning_goal_marker = '%% @_@_@'
planning_history_marker = '%% #_#_#'
preASP_planning_file = 'preASP_Traditional.txt'
preASP_planning_split = []
history_index_planning_asp = 0
belief_history_marker = '%% *_*_*'
asp_belief_file = 'ASP_Believe.sp'
asp_diagnosing_file = 'ASP_Diagnosis.sp'
preASP_domain_file = 'preASP_Domain.txt'
preASP_belief_split = []
history_index_domain_asp = 0

def controllerTraditionalPlanning(newGoal, maxPlanLen, initialConditions, objects, zones, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding):
	global belief
	global maxPlanLength
	global goal
	global allPlans
	global currentStep
	global history

	maxPlanLength = maxPlanLen
	goal = newGoal	

	history = history + observations_to_obsList(initialConditions,0)

	preparePreASP_string_lists()
	
	executionTime = 0
	
	needNewPlan = True
	while(needNewPlan == True):
		diagnoseAndUpdate()
		currentPlan = findPlan()
                print '\nAction Plan:'
                print currentPlan
                print '\n'
		needNewPlan = False
                if(len(currentPlan) > 1 and currentPlan[1] == "Goal holds" and baxter.getGoalFeedback() == False):
                    while(currentPlan[1] == "Goal holds"):
                        updateBeliefWithDiagnosis()
                        currentPlan = findPlan()
		if(currentPlan[0] == "No Plan"):
			break
		for current_step in range(len(currentPlan)):
                        action = currentPlan[current_step]
			print('\n\nNext action (traditional planning) : ' + str(action))
                        answer_sets = baxter.refine(action, history, current_step, 'traditional', current_refined_location, currently_holding)
                        refined_plans = answer_sets.split('\n')
                        refined_plan = refined_plans[0]
                        print 'refined action plan: '
                        print refined_plan
                        current_refined_location, current_location, currently_holding, relevantObservations, time, happened, objects = baxter.execute_refined_plan(refined_plan, objects, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding)
			executionTime += time
			if(happened == True): 
				history.append('hpd('+action+','+str(currentStep)+').')
				currentStep += 1
                                global_variables.current_location = current_location
				history = history + observations_to_obsList(relevantObservations, currentStep)	
			else: 
				print('Inconsistent observations, action did not happen, need to call planner')
				currentStep += 1
                                global_variables.current_location = current_location
				history = history + observations_to_obsList(relevantObservations, currentStep)	
				needNewPlan = True
				break	
	return (history, allPlans, executionTime) 



# this function will find  minimal Plan AND update the current belief with inferred observations.

def diagnoseAndUpdate():
	global history
	global currentStep
	global history_index_domain_asp
	global preASP_belief_split
	global diagnosis

	diagnosis = []
	asp_diagnosing_split = preASP_belief_split[:history_index_domain_asp] + history + preASP_belief_split[history_index_domain_asp+1:]
	asp_diagnosing_split[0] = "#const numSteps = "+str(currentStep)+"."
	asp = '\n'.join(asp_diagnosing_split)
	f1 = open(asp_diagnosing_file, 'w')
	f1.write(asp) 
	f1.close()
	output = subprocess.check_output('java -jar sparc.jar '+ asp_diagnosing_file +' -A',shell=True)
        outputs = output.split('}')
        firstOutput = outputs[0]
	firstOutput = firstOutput.strip('}').strip('{').split(', ')
	fluents = []
	for item in firstOutput:
	    if('holds' in item): fluents.append(item)
	    elif(item[0:4] == 'diag'): 
	        item = item.replace('diag', 'hpd')
	        commaIndex = item.rfind(',') 
	        item = item[0:commaIndex+1] + 'true,'+item[commaIndex+1:]	
	        diagnosis.append(item)
        print 'diagnosis:'
        print diagnosis
        if diagnosis != []:
            history.append(diagnosis[0])
        answer = (', '.join(fluents))
	updateBelief_fromAnswer(answer)
	




def findPlan():
	global currentStep
	global currentPlan
	global preASP_planning_split
	global allPlans
	
	newPlan = []

	asp_planning_split = preASP_planning_split[:history_index_planning_asp] + getBelief_as_obsList(0) + preASP_planning_split[history_index_planning_asp+1:]
	asp_planning = '\n'.join(asp_planning_split)
        f1 = open('ASP_Traditional.sp', 'w')
	f1.write(asp_planning) 
	f1.close()
        answerSet = subprocess.check_output('java -jar sparc.jar ASP_Traditional.sp -A', shell=True)
	if(answerSet == "\n"):
		newPlan = ["No Plan","Futile Goal/Inconsistent"]
	elif("{}" in answerSet):
		newPlan = ["No Plan","Goal holds"]
	elif(answerSet == ""):
		newPlan = ["No Plan","Unable to process"]
	else:
		plans = answerSet.strip('\n').split('\n') 
    		plans = [plan for plan in plans if plan!='' and plan!='{}']
		firstPlan = plans[0]
		plan_in_actions = firstPlan.strip('}').strip('{').split(', ')
		splitActionsList = [action[:-1].split('),') for action in plan_in_actions]
		splitActionsList.sort(key=lambda x:int(x[1]))
 		plan_in_actions = ['),'.join(action)+ ')' for action in splitActionsList]
		for a in plan_in_actions:
			newPlan.append(a[7:a.rfind(',')])
	allPlans.append(newPlan)

	return newPlan


def updateBelief_fromAction(action, observations):
	global history_index_domain_asp
	global preASP_belief_split

	input = getBelief_as_obsList(0) + observations_to_obsList(observations,1) + ['hpd('+ action +',0).']
	asp_belief_split = preASP_belief_split[:history_index_domain_asp] + input + preASP_belief_split[history_index_domain_asp+1:]
	asp = '\n'.join(asp_belief_split)
	f1 = open(asp_belief_file, 'w')
	f1.write(asp) 
	f1.close()
	print('Next, checking belief-observations consistency ')
	output = subprocess.check_output('java -jar sparc.jar '+ asp_belief_file +' -A',shell=True)
	output = output.strip('}').strip('{')

	if 'holds' in output:
		updateBelief_fromAnswer(output)
		return True
	else: return False

def preparePreASP_string_lists():
    	global preASP_planning_split 
    	global preASP_belief_split 
	global history_index_domain_asp
	global history_index_planning_asp
	global goal

	#preparing preASP_planning_split and history_index_planning_asp
    	reader = open(preASP_planning_file, 'r')
    	preASP_planning = reader.read()
    	reader.close()
	preASP_planning_split = preASP_planning.split('\n')
	preASP_planning_split[0] = "#const numSteps = "+ str(maxPlanLength)+"."

   	index_goal = preASP_planning_split.index(planning_goal_marker)
    	goal_formatted = "goal(I) :- "+ goal
    	preASP_planning_split.insert(index_goal+1,  goal_formatted)
    	history_index_planning_asp = preASP_planning_split.index(planning_history_marker)
	
	#preparing preASP_belief_split and history_index_domain_asp
    	reader = open(preASP_domain_file, 'r')
    	preASP_belief = reader.read()
    	reader.close()
	preASP_belief_split = preASP_belief.split('\n')
    	history_index_domain_asp = preASP_belief_split.index(belief_history_marker)

def updateBelief_fromAnswer(answer):
	global belief 
	belief = ['unknown'] * 7
	for holds in answer.split(', '):
            try:
		if holds[0] == '-':
			fluent = holds[7:holds.rfind(',')]
			if(fluent[0:8] == 'in_hand('): 
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'yellow_box'): belief[3] = 'false'
				if(split_fluent[1] == 'blue_box'): belief[4] = 'false'
				if(split_fluent[1] == 'green_box'): belief[6] = 'false'
			
		else:
			fluent = holds[6:holds.rfind(',')]
			if(fluent[0:4] == 'loc('):
				fluent = fluent[4:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[0] == 'rob1'): belief[0] = split_fluent[1]
				elif(split_fluent[0] == 'yellow_box'): belief[1] = split_fluent[1]
				elif(split_fluent[0] == 'blue_box'): belief[2] = split_fluent[1]
				elif(split_fluent[0] == 'green_box'): belief[5] = split_fluent[1]
			elif(fluent[0:8] == 'in_hand('): 
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'yellow_box'): belief[3] = 'true'
				if(split_fluent[1] == 'blue_box'): belief[4] = 'true'
				if(split_fluent[1] == 'green_box'): belief[6] = 'true'
            except IndexError:
                print 'holds[0] doesnt exist'

def getBelief(index):
	return belief[index]


def getBelief_as_obsList(step):
	obsList = []
	global belief
	obsList.append('obs(loc(rob1,' +str(belief[0])+ '),true,'+str(step)+').')
	if(belief[1] != 'unknown'): obsList.append('obs(loc(yellow_box,' +str(belief[1])+ '),true,'+str(step)+').')
	if(belief[2] != 'unknown'): obsList.append('obs(loc(blue_box,' +str(belief[2])+ '),true,'+str(step)+').')
	if(belief[3] != 'unknown'): obsList.append('obs(in_hand(rob1,yellow_box),' + belief[3]+ ','+str(step)+').')
	if(belief[4] != 'unknown'): obsList.append('obs(in_hand(rob1,blue_box),' + belief[4]+ ','+str(step)+').')
	if(belief[5] != 'unknown'): obsList.append('obs(loc(green_box,' +str(belief[5])+ '),true,'+str(step)+').')
	if(belief[6] != 'unknown'): obsList.append('obs(in_hand(rob1,green_box),' + belief[6]+ ','+str(step)+').')
	return obsList



def observations_to_obsList(observations, step):

	obsList = []
	for observation in observations:
		if (observation[0] == 0):
			obsList.append('obs(loc(rob1,'+str(observation[1])+ '),true,'+ str(step) +').')
		if (observation[0] == 1): 
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(yellow_box,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(yellow_box,' + global_variables.current_location + '),false,'+ str(step) +').')
		if (observation[0] == 2): 
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(blue_box,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(blue_box,' + global_variables.current_location + '),false,'+ str(step) +').')
		if (observation[0] == 5): 
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(green_box,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(green_box,' + global_variables.current_location + '),false,'+ str(step) +').')
		if (observation[0] == 3 and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,yellow_box),' + observation[1]+ ','+ str(step) +').')
		if (observation[0] == 4 and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,blue_box),' + observation[1]+ ','+ str(step) +').')
		if (observation[0] == 6 and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,green_box),' + observation[1]+ ','+ str(step) +').')

	return obsList


def updateBeliefWithDiagnosis():
	chosenDiagnosis = possibleDiagnosis.pop()
	fluents = []
	for item in chosenDiagnosis:
		if('holds' in item): fluents.append(item)
		elif(item[0:4] == 'diag'): 
			item = item.replace('diag', 'hpd')
			commaIndex = item.rfind(',') 
			item = item[0:commaIndex+1] + 'true,'+item[commaIndex+1:]	
        updateBelief_fromAnswer(', '.join(fluents))
