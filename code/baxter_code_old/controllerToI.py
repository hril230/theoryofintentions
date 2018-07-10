
from datetime import datetime
import subprocess
from sets import Set
import re
import numpy
import baxter_controller as baxter
import global_variables


goal = None
maxPlanLength = None
numberSteps = None
toi_history = None
currentStep = None
toi_belief = None
numberActivities = None
scenario = None

toi_goal_marker = '%% @_@_@'
toi_beginning_history_marker = '%% #_#_# beginning'
toi_end_history_marker = '%% #_#_# end'
toi_current_step_marker = '%% *_*_*'
preASP_toi_file = 'preASP_ToI.txt'
preASP_toi_split = None
toi_beginning_history_index = None
toi_current_step_index = None



toi_belief_history_marker = '%% *_*_*'
asp_toi_belief_file = 'ASP_TOI_Believe.sp'
asp_toi_diagnosing_file = 'ASP_TOI_Diagnosis.sp'
preASP_domain_file = 'preASP_Domain.txt'
preASP_toi_belief_split = None
domain_asp_history_index = None

abandoned = None

possibleDiagnosis = None

def controllerToI(newGoal, maxPlanLen, initialConditions, objects, zones, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding):
    global toi_belief
    global maxPlanLength
    global numberSteps
    global goal
    global goal_correction
    global numberActivities
    global currentStep
    global toi_history
    global abandoned

    numberActivities = 1
    toi_belief = ['unknown']*7
    numberSteps = 4
    maxPlanLength = maxPlanLen
    goal = newGoal
    goal_correction = 0	
    abandoned = False

    preparePreASP_string_lists()	
    toi_history = observations_to_obsList(initialConditions,0,current_location)
    toi_history.append("hpd(select(my_goal), true,0).")

    currentStep = 1

    inputForPlanning = diagnoseAndUpdateBelief()
    finish = False

    while(finish == False):
        nextAction = runToIPlanning(inputForPlanning)
        print '\n\nNext coarse-level action: ' + nextAction

        if(nextAction == None):
            toi_history.append("Goal is futile")
            finish = True
            break

        elif("selected_goal_holds" in nextAction):
            toi_history.append("Goal holds")
            finish = True
            break

        elif(nextAction == 'finish'): 
            if(baxter.getGoalFeedback() == True): 
                toi_history.append('finish')
                finish = True
                break
            else:
                goal_correction += 1
                while(nextAction == 'finish'):
                    toi_history.append('obs(my_goal,false,'+str(currentStep)+').')
                    print('wrong assumption - goal not reached !!!!! ')
                    diagnose()
                    nextAction = runToIPlanning(inputForPlanning)

        toi_history.append('attempt('+nextAction+','+str(currentStep)+').')
        if(nextAction[0:5] == 'start' or nextAction[0:4] == 'stop'):
            toi_history.append('hpd('+nextAction+',true,'+ str(currentStep) + ').')
            currentStep += 1
            if(nextAction[0:4] == 'stop'): 
                numberActivities += 1 
                numberSteps += 1

        else:
            answer_sets = baxter.refine(nextAction, toi_history, currentStep, 'ToI', current_refined_location, currently_holding)
            refined_plans = answer_sets.split('\n')
            refined_plan = refined_plans[0]
            print 'refined action plan: '
            print refined_plan
            current_refined_location, current_location, currently_holding, relevantObservations, time, happened, objects = baxter.execute_refined_plan(refined_plan, objects, cells, arm, limb_object, gripper, current_location, current_refined_location, currently_holding)

            if(happened == True): 
                toi_history.append('hpd('+nextAction+',true,'+str(currentStep)+').')

            else: 
                print('!!!!!!!!!!!!!!  action did not happen')
                toi_history.append('hpd('+nextAction+',false,'+str(currentStep)+').')

            # test goal fluents after each action
            relevantObservations = baxter.test_goal(newGoal, relevantObservations, current_location)
				
            currentStep += 1
            toi_history = toi_history + list(set(observations_to_obsList(relevantObservations,currentStep,current_location)))
	
        inputForPlanning = diagnoseAndUpdateBelief()

    return (toi_history, numberActivities)



# this function will find  minimal Plan AND update the current toi_belief with inferred observations.

def wasAbandoned():
	return abandoned

def getIndexesRelevantToGoal():
	return [1, 2, 3, 4]

def runToIPlanning(input):	
	global numberSteps
	global preASP_toi_split
	global toi_history
	global maxPlanLength
	global currentStep
	nextAction = None

	current_asp_split = preASP_toi_split[:toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	current_asp_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
	current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
	current_asp_split[2] = "#const max_name = " + str(numberActivities) + "."
	asp = '\n'.join(current_asp_split)
        f1 = open('ASP_ToI.sp', 'w')
	f1.write(asp) 
	f1.close()

	answerSet = subprocess.check_output('java -jar sparc.jar ASP_ToI.sp -A ',shell=True)
        print '\nAction Plan:'
        print answerSet
        print '\n'
	while( "intended_action" not in answerSet and "selected_goal_holds" not in answerSet and numberSteps < currentStep + maxPlanLength+3):	
		current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
		asp = '\n'.join(current_asp_split)
        	f1 = open('ASP_ToI.sp', 'w')
		f1.write(asp) 
		f1.close()
		answerSet = subprocess.check_output('java -jar sparc.jar ASP_ToI.sp -A ',shell=True)
		numberSteps +=1
	
        split_sets = answerSet.split('}')
	firstAnswer = split_sets[0]
	split_answer = firstAnswer[1:].split(', ') 
	toi_history = []
	for line in split_answer:	
		if("number_unobserved" in line): continue	
		elif("intended_action" in line):      		
			nextAction = line[16:line.rfind(',')] 
		else:
			toi_history.append(line + '.')	

	return nextAction



def diagnoseAndUpdateBelief():
	input = list(toi_history)
	input.append("explaining("+str(currentStep)+").")
	current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	current_asp_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
	current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
	current_asp_split[2] = "#const max_name = " + str(numberActivities) + "."
        current_asp_split.append("holds(in_hand(rob1,B),"+str(currentStep)+").")
        current_asp_split.append("-holds(in_hand(rob1,B),"+str(currentStep)+").")
        current_asp_split.append("holds(loc(A,B),"+str(currentStep)+").")

	asp = '\n'.join(current_asp_split)
        f1 = open(asp_toi_diagnosing_file, 'w')
	f1.write(asp) 
	f1.close()

	# running only diagnosis
	diagnosis = subprocess.check_output('java -jar sparc.jar '+ asp_toi_diagnosing_file +' -A ',shell=True)

	#take only first set
       	split_diagnosis = diagnosis.split('}')
	firstDiagnosis = split_diagnosis[0]
	split_diagnosis = firstDiagnosis[1:].split(', ') 
	output = []
        fluents = []
	for line in split_diagnosis: 
		if("number_unobserved" in line): 
			newLine =line.replace("number_unobserved","explanation") 
			output.append(newLine + '.') 
  		elif("unobserved" in line):
			newLine = line.replace("unobserved", "occurs")
			output.append(newLine + '.')
                elif('holds' in line):
                        fluents.append(line)
		elif(line != ""):
			output.append(line + '.')
        
        update_toi_belief_fromAnswer(', '.join(fluents))

        print 'diagnosis:'
        print output

	return output



def inferInitialBelief():
	global toi_history
	global domain_asp_history_index
	global preASP_toi_belief_split

	asp_diagnosing_split = preASP_toi_belief_split[:domain_asp_history_index] + toi_history + preASP_toi_belief_split[domain_asp_history_index+1:]
	asp_diagnosing_split[0] = "#const numSteps = 0."
	asp = '\n'.join(asp_diagnosing_split)
	f1 = open(asp_toi_diagnosing_file, 'w')
	f1.write(asp) 
	f1.close()
	output = subprocess.check_output('java -jar sparc.jar '+ asp_toi_diagnosing_file +' -A',shell=True)
	output = output.strip('\n').split('\n') 
	firstOutput = output[0].strip('}').strip('{')
	update_toi_belief_fromAnswer(firstOutput)


def checkConsistency(action, observations, current_location):
	global domain_asp_history_index
	global preASP_toi_belief_split

	input = get_toi_belief_as_obsList(0) + observations_to_obsList(observations,1,current_location) + ['hpd('+ action +',0).']
	asp_toi_belief_split = preASP_toi_belief_split[:domain_asp_history_index] + input + preASP_toi_belief_split[domain_asp_history_index+1:]
	asp = '\n'.join(asp_toi_belief_split)
	f1 = open(asp_toi_belief_file, 'w')
	f1.write(asp) 
	f1.close()
	output = subprocess.check_output('java -jar sparc.jar '+ asp_toi_belief_file +' -A',shell=True)
	if output == '\n': return False
	return True

def preparePreASP_string_lists():
	global toi_goal_marker
	global toi_beginning_history_marker
	global toi_current_step_marker
	global preASP_toi_file
	global goal

	global preASP_toi_split
	global toi_beginning_history_index
	global toi_current_step_index

	global toi_belief_history_marker
	global preASP_domain_file

    	global preASP_toi_belief_split
	global domain_asp_history_index



	#preparing preASP_toi_split and toi_beginning_history_index
    	reader = open(preASP_toi_file, 'r')
    	preASP_toi = reader.read()
    	reader.close()
	preASP_toi_split = preASP_toi.split('\n')

   	index_goal = preASP_toi_split.index(toi_goal_marker)
    	preASP_toi_split.insert(index_goal+1,  "holds(my_goal,I) :- "+ goal)
    	toi_beginning_history_index = preASP_toi_split.index(toi_beginning_history_marker)
	toi_current_step_index = preASP_toi_split.index(toi_current_step_marker)


	#preparing preASP_toi_belief_split and domain_asp_history_index
    	reader = open(preASP_domain_file, 'r')
    	preASP_toi_belief = reader.read()
    	reader.close()
	preASP_toi_belief_split = preASP_toi_belief.split('\n')
    	domain_asp_history_index = preASP_toi_belief_split.index(toi_belief_history_marker)

	

def update_toi_belief_fromAnswer(answer):
	global toi_belief 
	toi_belief = ['unknown'] * 7
	for holds in answer.split(', '):
		if holds[0] == '-':
			fluent = holds[7:holds.rfind(',')]
			if(fluent[0:8] == 'in_hand('): 
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'yellow_box'): toi_belief[3] = 'false'
				if(split_fluent[1] == 'blue_box'): toi_belief[4] = 'false'
				if(split_fluent[1] == 'green_box'): toi_belief[6] = 'false'
			
		else:
			fluent = holds[6:holds.rfind(',')]
			if(fluent[0:4] == 'loc('):
				fluent = fluent[4:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[0] == 'rob1'): toi_belief[0] = split_fluent[1]
				elif(split_fluent[0] == 'yellow_box'): toi_belief[1] = split_fluent[1]
				elif(split_fluent[0] == 'blue_box'): toi_belief[2] = split_fluent[1]
				elif(split_fluent[0] == 'green_box'): toi_belief[5] = split_fluent[1]
			elif(fluent[0:8] == 'in_hand('): 
				fluent = fluent[8:-1]
				split_fluent = fluent.split(',')
				if(split_fluent[1] == 'yellow_box'): toi_belief[3] = 'true'
				if(split_fluent[1] == 'blue_box'): toi_belief[4] = 'true'
				if(split_fluent[1] == 'green_box'): toi_belief[6] = 'true'

def get_toi_belief(index):
	return toi_belief[index]


def get_toi_belief_as_obsList(step):
	obsList = []
	global toi_belief		
	obsList.append('obs(loc(rob1,' +str(toi_belief[0])+ '),true,'+str(step)+').')
	if(toi_belief[1] != 'unknown'): obsList.append('obs(loc(yellow_box,' +str(toi_belief[1])+ '),true,'+str(step)+').')
	if(toi_belief[2] != 'unknown'): obsList.append('obs(loc(blue_box,' +str(toi_belief[2])+ '),true,'+str(step)+').')
	if(toi_belief[3] != 'unknown'): obsList.append('obs(in_hand(rob1,yellow_box),' + toi_belief[3]+ ','+str(step)+').')
	if(toi_belief[4] != 'unknown'): obsList.append('obs(in_hand(rob1,blue_box),' + toi_belief[4]+ ','+str(step)+').')
	if(toi_belief[5] != 'unknown'): obsList.append('obs(loc(green_box,' +str(toi_belief[5])+ '),true,'+str(step)+').')
	if(toi_belief[6] != 'unknown'): obsList.append('obs(in_hand(rob1,green_box),' + toi_belief[6]+ ','+str(step)+').')
	return obsList



def observations_to_obsList(observations, step, current_location):

	obsList = []
	for observation in observations:
		if (observation[0] == 0):
			obsList.append('obs(loc(rob1,'+str(observation[1])+ '),true,'+ str(step) +').')
		if (observation[0] == 1):
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(yellow_box,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(yellow_box,' + current_location+ '),false,'+ str(step) +').')
		if (observation[0] == 2): 
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(blue_box,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(blue_box,' + current_location + '),false,'+ str(step) +').')
		if (observation[0] == 5): 
			if(observation[1] != 'unknown'):
				obsList.append('obs(loc(green_box,' +str(observation[1])+ '),true,'+ str(step) +').')
			else:
				obsList.append('obs(loc(green_box,' + current_location + '),false,'+ str(step) +').')
		if (observation[0] == 3 and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,yellow_box),' + observation[1]+ ','+ str(step) +').')
		if (observation[0] == 4 and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,blue_box),' + observation[1]+ ','+ str(step) +').')
		if (observation[0] == 6 and observation[1] != 'unknown'):
			obsList.append('obs(in_hand(rob1,green_box),' + observation[1]+ ','+ str(step) +').')

	return obsList
	


