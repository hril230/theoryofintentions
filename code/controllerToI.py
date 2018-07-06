
from datetime import datetime
import subprocess
from simulation.domain_info import DomainInfo
from sets import Set

import re
import numpy
import random

goal = None  # given when initialised
maxPlanLength = None  #given when initialised
numberSteps = None
toi_history = None  #sarting with initial conditions taken from world and adding occurrence
currentStep = None
numberActivities = None
inputForPlanning = None
believes_goal_holds = False
currentDiagnosis = None

toi_goal_marker = '%% @_@_@'
toi_beginning_history_marker = '%% #_#_# beginning'
toi_end_history_marker = '%% #_#_# end'
toi_current_step_marker = '%% *_*_*'

preASP_toi_split = None
toi_beginning_history_index = None
toi_current_step_index = None

preASP_toi_file = None
asp_toi_file = None
asp_toi_diagnosing_file = None
preASP_refined_domain_file = None
zoomed_domain_file = None

executer = None
goal_correction = None



def controllerToI(thisPath, newGoal, maxPlanLen, new_executer,new_domain_info,files_path):
	global executer
	global maxPlanLength
	global numberSteps
	global goal
	global numberActivities
	global currentStep
	global toi_history
	global inputForPlanning
	global goal_correction
	global currentDiagnosis
	global sparcPath

	global current_location
	global current_refined_location
	global currently_holding
	global preASP_toi_file
	global asp_toi_file
	global asp_toi_diagnosing_file
	global preASP_refined_domain_file
	global zoomed_domain_file

	preASP_toi_file = files_path+'preASP_ToI.txt'
	asp_toi_file = files_path+'ASP_ToI.sp'
	asp_toi_diagnosing_file = files_path + 'ASP_TOI_Diagnosis.sp'
	preASP_refined_domain_file = files_path + 'preASP_refined_domain.txt'
	zoomed_domain_file = files_path + 'zoomed_domain.sp'
	current_location = 'unknown'
	current_refined_location = 'unknown_cell'
	currently_holding = 'nothing'


	sparcPath = thisPath
	numberActivities = 1
	numberSteps = 4
	maxPlanLength = maxPlanLen
	executer = new_executer
	my_domain_info = new_domain_info
	goal = newGoal
	goal_correction = 0
	initialConditions = list(executer.getRealValues())
	currentDiagnosis = ''
	inputForPlanning = []

	preparePreASP_string_lists()
	toi_history = my_domain_info.observations_to_obsList(initialConditions,executer.getRobotLocation(),0)
	toi_history.append("hpd(select(my_goal), true,0).")

   	currentStep = 1
	diagnose()
    	finish = False
	while(finish == False):
		nextAction = runToIPlanning(inputForPlanning)

		#print(' $$$$$$$$$$$$$$$$$    next action with ToI : ' +str(nextAction) +'  at step '+ str(currentStep))

		if(nextAction == 'finish'):
			if(executer.getGoalFeedback() == True):
				toi_history.append('finish')
				finish = True
				break
			else:
				goal_correction += 1
				while(nextAction == 'finish'):
					toi_history.append('obs(my_goal,false,'+str(currentStep)+').')
					#print('wrong assumption - goal not reached !!!!! ')
					diagnose()
					nextAction = runToIPlanning(inputForPlanning)

		if(nextAction == None):
	    		toi_history.append("Goal is futile")
			finish = True
			break



		actionObservations = []
		toi_history.append('attempt('+nextAction+','+str(currentStep)+').')

		if(nextAction[0:4] == 'stop'):
			numberActivities += 1
			numberSteps += 1
		elif(nextAction[0:5] == 'start'): pass
		else:
			print('action : '+ str(nextAction))
			actionObservations = executer.executeAction(nextAction)
			answer_sets = refine(nextAction, toi_history, currentStep, 'ToI', current_refined_location, currently_holding)

		currentStep += 1
		relevantObservations = actionObservations + executer.getTheseObservations(getIndexesRelevantToGoal())
		robotLocation = executer.getRobotLocation()
		toi_history = toi_history + list(set(my_domain_info.observations_to_obsList(relevantObservations,robotLocation,currentStep)))
		diagnose()

	if(currentDiagnosis != ''): toi_history.append(currentDiagnosis)
	return (toi_history, numberActivities, goal_correction)




def getIndexesRelevantToGoal():
	return [DomainInfo.LocationBook1_index, DomainInfo.LocationBook2_index, DomainInfo.In_handBook1_index, DomainInfo.In_handBook2_index]

def runToIPlanning(input):
	global numberSteps
	global preASP_toi_split
	global toi_history
	global maxPlanLength
	global currentStep
	global believes_goal_holds
	nextAction = None
	#print('running ToI planning ')


	current_asp_split = preASP_toi_split[:toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	current_asp_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
	current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
	current_asp_split[2] = "#const max_name = " + str(numberActivities) + "."
	asp = '\n'.join(current_asp_split)
        f1 = open(asp_toi_file, 'w')
	f1.write(asp)
	f1.close()

	answerSet = subprocess.check_output('java -jar '+sparcPath + ' ' + asp_toi_file +' -A ',shell=True)
	while( "intended_action" not in answerSet and "selected_goal_holds" not in answerSet and numberSteps < currentStep + maxPlanLength+3):
		current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
		current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
		asp = '\n'.join(current_asp_split)
		f1 = open(asp_toi_file, 'w')
		f1.write(asp)
		f1.close()
		#print('Looking for next action (ToI) - numberSteps ' + str(numberSteps))
		answerSet = subprocess.check_output('java -jar '+sparcPath + ' ' + asp_toi_file +' -A ',shell=True)
		numberSteps +=1

        possibleAnswers = answerSet.rstrip().split('\n\n')

	chosenAnswer = possibleAnswers[0]
	split_answer = chosenAnswer.strip('}').strip('{').split(', ')
	toi_history = []
	believes_goal_holds = False
	for line in split_answer:
		if("intended_action" in line):
			nextAction = line[16:line.rfind(',')]
		#elif("number_unobserved" in line): continue
		elif("selected_goal_holds" in line):
			believes_goal_holds = True
		else:
			toi_history.append(line + '.')
	return nextAction



def diagnose():

	global inputForPlanning
	global currentDiagnosis

	inputForPlanning = []
	possibleDiagnosis = []
	input = list(toi_history)
	input.append("explaining("+str(currentStep)+").")
	current_asp_split = preASP_toi_split[: toi_beginning_history_index +1] + input + preASP_toi_split[toi_beginning_history_index +1:]
	current_asp_split[toi_current_step_index +1] = 'current_step('+str(currentStep)+').'
	current_asp_split[0] = "#const n = "+str(numberSteps+1)+". % maximum number of steps."
	current_asp_split[1] = "#const max_len = "+str(numberSteps)+". % maximum activity_length of an activity."
	current_asp_split[2] = "#const max_name = " + str(numberActivities) + "."


	asp = '\n'.join(current_asp_split)
	f1 = open(asp_toi_diagnosing_file, 'w')
	f1.write(asp)
	f1.close()

	# running only diagnosis
	answerSet = subprocess.check_output('java -jar '+sparcPath + ' ' + asp_toi_diagnosing_file +' -A ',shell=True)
       	answers = answerSet.rstrip().split('\n\n')


	if currentDiagnosis in answerSet:
		for a in answers:
			if(currentDiagnosis in a): chosenAnswer = a
	else:
		chosenAnswer = answers[0]

 	split_diagnosis = chosenAnswer.strip('}').strip('{').split(', ')
	for line in split_diagnosis:
		if("number_unobserved" in line):
			newLine =line.replace("number_unobserved","explanation")
			inputForPlanning.append(newLine + '.')
		elif("unobserved" in line):
			newLine = line.replace("unobserved", "occurs") + '.'
			inputForPlanning.append(newLine)
			currentDiagnosis = line
		elif("selected_goal_holds" in line): pass
		elif(line == ""): pass
		else:
			inputForPlanning.append(line + '.')


	#print('current diagnosis: '+str(currentDiagnosis))
	return


def preparePreASP_string_lists():
	global toi_goal_marker
	global toi_beginning_history_marker
	global toi_current_step_marker
	global preASP_toi_file
	global goal
	global preASP_toi_split
	global toi_beginning_history_index
	global toi_current_step_index


	#preparing preASP_toi_split and toi_beginning_history_index
	reader = open(preASP_toi_file, 'r')
	preASP_toi = reader.read()
	reader.close()
	preASP_toi_split = preASP_toi.split('\n')

   	index_goal = preASP_toi_split.index(toi_goal_marker)
	preASP_toi_split.insert(index_goal+1,  "holds(my_goal,I) :- "+ goal)
	toi_beginning_history_index = preASP_toi_split.index(toi_beginning_history_marker)
	toi_current_step_index = preASP_toi_split.index(toi_current_step_marker)




# this function uses the preASP_refined_Domain.txt file and SPARC to get a refined action plan
def refine(action, history, current_step, version, current_refined_location, currently_holding):

    # convert ToI history into traditional format so that they can be processed in the same way
    history_formatted = []
    if version == 'ToI':
        for i in range(len(history)):
            if ('hpd' in history[i]) and ('true' in history[i]):
                if ('0)' in history[i]) or ('1)' in history[i]) or ('2)' in history[i]) or ('3)' in history[i]) or ('4)' in history[i]) or ('5)' in history[i]) or ('6)' in history[i]) or ('7)' in history[i]) or ('8)' in history[i]) or ('9)' in history[i]):
                    history_formatted.append(history[i][0:len(history[i])-8] + history[i][len(history[i])-3:len(history[i])])
                else:
                    history_formatted.append(history[i][0:len(history[i])-9] + history[i][len(history[i])-4:len(history[i])])
            elif not 'hpd' in history[i]:
                history_formatted.append(history[i])
    else: history_formatted = history
    initial_state = []
    final_state = []
    # use action and history to figure out the transition (initial_state, action, final_state)
    # the location of the robot is relevant for move transitions

    print "history_formatted"
    print history_formatted
    print "action"
    print action

    if 'move' in action:
        # get the initial state from step zero
        for observation in history_formatted:
            if ('obs' in observation) and ('rob1' in observation) and ('loc' in observation):
                initial_state = ['coarse_loc(rob1,' + observation[13:len(observation)-10] + ')']

        # if actions have happened since step zero, initial state may need to be updated
        for observation in history_formatted:
            if ('hpd' in observation) and ('move' in observation) and (not 'exo' in observation):
                if ('0)' in observation) or ('1)' in observation) or ('2)' in observation) or ('3)' in observation) or ('4)' in observation) or ('5)' in observation) or ('6)' in observation) or ('7)' in observation) or ('8)' in observation) or ('9)' in observation):
                    initial_state = ['coarse_loc(rob1,' + observation[14:len(observation)-5] + ')']
                else:
                    initial_state = ['coarse_loc(rob1,' + observation[14:len(observation)-6] + ')']
        final_state = ['coarse_loc(rob1,' + action[10:len(action)-1] + ')']

    # the location of the robot and object, and the in_hand status of the object, are relevant for pickup transitions
    elif 'pickup' in action:
        obj = action[12:len(action)-1]
        # include the in_hand status of the object
        initial_state = ['-coarse_in_hand(rob1,' + obj + ')']
        final_state = ['coarse_in_hand(rob1,' + obj + ')']
        # include the location of the robot
        for observation in history_formatted:
            if ('obs' in observation) and ('rob1' in observation) and ('loc' in observation):
                rob_loc = 'coarse_loc(rob1,' + observation[13:len(observation)-10] + ')'
        for observation in history_formatted:
            if ('hpd' in observation) and ('move' in observation):
                rob_loc = 'coarse_loc(rob1,' + observation[14:len(observation)-5] + ')'
        # include the location of the object
        for observation in history_formatted:
            if ('obs' in observation) and (obj in observation) and ('loc' in observation):
                obj_loc = 'coarse_loc(' + obj + ',' + observation[9+len(obj):len(observation)-10] + ')'
            if ('exo' in observation) and (obj in observation):
                obj_loc = 'coarse_loc(' + obj + ',' + observation[14+len(obj):len(observation)-9] + ')'
        initial_state.append(rob_loc)
        initial_state.append(obj_loc)
        final_state.append('coarse_loc(rob1,' + obj_loc[12+len(obj):len(obj_loc)-1] + ')')
        final_state.append(obj_loc)

    # the in_hand status of the object is relevant for put_down transitions
    elif 'put_down' in action:
        initial_state = ['coarse_in_hand(rob1,' + action[14:len(action)-1] + ')']
        final_state = ['-coarse_in_hand(rob1,' + action[14:len(action)-1] + ')']

    # edit refined_asp to get temporary zoomed_asp file
    zoom(initial_state, action, final_state, current_refined_location, currently_holding)


    # get refined answer set
    refined_answer = subprocess.check_output('java -jar '+sparcPath + ' zoomed_domain.sp -A ',shell=True)
    if refined_answer == "" : raw_input()



    # stop running code if refined answer set is empty
    if refined_answer == '\n':
		raw_input('No refined answer set, press enter if you wish to continue.\n')

    # refined_plan = parse answer set
    refined_plan = refined_answer
    return refined_plan

# this action writes a zoomed ASP file
def zoom(initial_state, action, final_state, current_refined_location, currently_holding):

    # clean up typos
    for i in range(len(initial_state)):
        if '))' in initial_state[i]:
            initial_state[i] = initial_state[i][0:len(initial_state[i])-1]
    for i in range(len(final_state)):
        if '))' in final_state[i]:
            final_state[i] = final_state[i][0:len(final_state[i])-1]

    # EDIT these lists to change the domain
    coarse_places = Sort('coarse_place', ['zoneR', 'zoneG', 'zoneY', 'unknown'])
    coarse_objects = Sort('coarse_object', ['yellow_box', 'blue_box', 'green_box'])
    places = Sort('place', ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'unknown_cell'])
    objects = Sort('object', ['yellow_box_top', 'blue_box_top', 'green_box_top'])
    coarse_things = Sort('coarse_thing', ['#coarse_object', '#robot'])
    things = Sort('thing', ['#object', '#robot'])
    coarse_components = Sort('coarse_component', ['#coarse_place', '#coarse_object'])
    refined_components = Sort('refined_component', ['#place', '#object'])
    robots = Sort('robot', ['rob1'])
    sorts = [coarse_places, coarse_objects, places, objects, coarse_things, things, coarse_components, refined_components, robots]
    inertial_fluents = ['loc(#thing,#place)', 'in_hand(#robot,#object)']
    defined_fluents = ['coarse_loc(#coarse_thing,#coarse_place)', 'coarse_in_hand(#robot,#coarse_object)']
    actions = ['move(#robot,#place)', 'pickup(#robot,#object)', 'put_down(#robot,#object)']

    # EDIT these instantiations of the Components class to change which refined objects are associated with which coarse ones
    zoneR_components = Components('zoneR', ['c1', 'c2', 'c3', 'c4'])
    zoneG_components = Components('zoneG', ['c5', 'c6', 'c7', 'c8'])
    zoneY_components = Components('zoneY', ['c9', 'c10', 'c11', 'c12'])
    unknown_components = Components('unknown', ['unknown_cell'])
    yellow_box_components = Components('yellow_box', ['yellow_box_top'])
    blue_box_components = Components('blue_box', ['blue_box_top'])
    green_box_components = Components('green_box', ['green_box_top'])
    refinements = [zoneR_components, zoneG_components, zoneY_components, yellow_box_components, blue_box_components, green_box_components, unknown_components]

    # initialise relevance lists
    rel_initial_conditions = []
    rel_final_conditions = []
    rel_conditions = []
    rel_obj_consts = []
    rel_sorts = []
    rel_sort_names = ['#coarse_thing', '#thing']
    rel_inertial_fluents = []
    rel_defined_fluents = []
    rel_actions = ['test(#robot,#physical_inertial_fluent,#outcome)']

    # initialise irrelevance lists - EDIT to include new objects or zones or cells
    irrelevant_sort_names = ['#coarse_place', '#coarse_object', '#place', '#object']
    irrelevant_obj_consts = ['zoneR', 'zoneG', 'zoneY', 'unknown', 'yellow_box', 'blue_box', 'green_box', 'c1,', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'yellow_box_top', 'blue_box_top', 'green_box_top']
    irrelevant_fluents = ['coarse_loc', 'coarse_in_hand', 'loc', 'in_hand']
    irrelevant_actions = ['move', 'pickup', 'put_down']

    # determine which initial conditions are relevant
    for condition in initial_state:
        if not condition in final_state: # conditions that change are relevant
            rel_initial_conditions.append(condition)
            rel_conditions.append(condition)
        elif ('rob1' in condition) and ('loc' in condition) and ('pickup' in action): # the robot's location is relevant for pickup actions
            rel_initial_conditions.append(condition)
            rel_conditions.append(condition)

    # refine initial conditions
    for i in range(len(rel_initial_conditions)):
        if ('loc' in rel_initial_conditions[i]) and ('rob1' in rel_initial_conditions[i]):
            rel_initial_conditions[i] = 'loc(rob1,' + current_refined_location + ')'
        if ('in_hand' in rel_initial_conditions[i]) and (not '-' in rel_initial_conditions[i]):
            rel_initial_conditions[i] = 'in_hand(rob1,' + currently_holding + ')'

    # determine which final conditions are relevant
    for condition in final_state:
        if not condition in initial_state:
            rel_final_conditions.append(condition)
            rel_conditions.append(condition)

    # determine which object constants are relevant
    for condition in rel_conditions:
        for index in range(len(condition)):
            if condition[index] == '(':
                opening_bracket = index
            elif condition[index] == ')':
                closing_bracket = index
        obj_consts = condition[opening_bracket+1:closing_bracket].split(',')
        for const in obj_consts:
            rel_obj_consts.append(const)
            if const in irrelevant_obj_consts:
                irrelevant_obj_consts.remove(const)
    rel_obj_consts = list(set(rel_obj_consts)) # remove duplicates

    # add refined components of relevant object constants
    for const in rel_obj_consts:
        for refinement in refinements:
            if const == refinement.name:
                for refined_const in refinement.components:
                    rel_obj_consts.append(refined_const)
                    if refined_const == 'c1':
                        refined_const = 'c1,'
                    if refined_const in irrelevant_obj_consts:
                        irrelevant_obj_consts.remove(refined_const)

    # sort relevant objects into types
    for sort in sorts:
        for const in sort.constants:
            if const in rel_obj_consts:
                sort.add(const)

    # determine which sorts should be included in the zoomed description
    for sort in sorts:
        if len(sort.rel_constants) != 0:
            rel_sorts.append(sort)
            rel_sort_names.append('#'+sort.name)
            if ('#'+sort.name) in irrelevant_sort_names:
                irrelevant_sort_names.remove('#'+sort.name)

    # add relevant sorts to sorts of sorts (coarse_things, things, coarse_components and refined_components)
    for sort in rel_sorts:
        for sort_of_sorts in sorts:
            if ('#'+sort.name) in sort_of_sorts.constants:
                sort_of_sorts.add('#'+sort.name)

    # determine which inertial fluents are relevant
    for fluent in inertial_fluents:
        fluent_relevant = True
        for index in range(len(fluent)):
            if fluent[index] == '(':
                opening_bracket = index
            elif fluent[index] == ')':
                closing_bracket = index
        fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
        for sort in fluent_sorts:
            if not sort in rel_sort_names:
                fluent_relevant = False
        if fluent_relevant:
            rel_inertial_fluents.append(fluent)
            if fluent[0:opening_bracket] in irrelevant_fluents:
                irrelevant_fluents.remove(fluent[0:opening_bracket])

    # determine which defined fluents are relevant
    for fluent in defined_fluents:
        fluent_relevant = True
        for index in range(len(fluent)):
            if fluent[index] == '(':
                opening_bracket = index
            elif fluent[index] == ')':
                closing_bracket = index
        fluent_sorts = fluent[opening_bracket+1:closing_bracket].split(',')
        for sort in fluent_sorts:
            if not sort in rel_sort_names:
                fluent_relevant = False
        if fluent_relevant:
            rel_defined_fluents.append(fluent)
            if fluent[0:opening_bracket] in irrelevant_fluents:
                irrelevant_fluents.remove(fluent[0:opening_bracket])

    # determine which actions are relevant
    for act in actions:
        action_relevant = True
        for index in range(len(act)):
            if act[index] == '(':
                opening_bracket = index
            elif act[index] == ')':
                closing_bracket = index
        action_sorts = act[opening_bracket+1:closing_bracket].split(',')
        for sort in action_sorts:
            if not sort in rel_sort_names:
                action_relevant = False
        if action_relevant:
            rel_actions.append(act)
            if act[0:opening_bracket] in irrelevant_actions:
                irrelevant_actions.remove(act[0:opening_bracket])

    # determine what the goal of the refined ASP should be
    goal = 'goal(I) :- '
    for condition in rel_final_conditions:
        if '-' in condition:
            condition = condition.replace('-', '')
            goal = goal + '-holds(' + condition + ',I), '
        else:
            goal = goal + 'holds(' + condition + ',I), '
    goal = goal[0:len(goal)-2] + '.\n'

    # make temporary copy of refined ASP file that can be edited
    original_asp = open(preASP_refined_domain_file, 'r')
    zoomed_asp = open(zoomed_domain_file, 'w')

    for line in original_asp:
        if line == '%% GOAL GOES HERE\n': # put goal in
            zoomed_asp.write(goal)
        elif line == '%% INITIAL STATE GOES HERE\n': # put initial conditions in
            for condition in rel_initial_conditions:
                if '-' in condition:
                    condition = condition.replace('-', '')
                    zoomed_asp.write('-holds(' + condition + ', 0).\n')
                else:
                    zoomed_asp.write('holds(' + condition + ', 0).\n')
        elif ('cannot test in the first step' in line) and ('pickup' in action): # remove this axiom for pickup actions, as the object's location needs to be tested at step zero
            pass
        elif 'ZOOM THIS SORT:' in line: # add relevant constants to sort
            zoomed_sort = ''
            for sort in sorts:
                if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
                    zoomed_sort = '#' + sort.name + ' = {'
                    for const in sort.rel_constants:
                        zoomed_sort = zoomed_sort + const + ', '
                    zoomed_sort = zoomed_sort[0:len(zoomed_sort)-2] + '}.\n'
                    zoomed_asp.write(zoomed_sort)
        elif 'ZOOM THIS SORT OF SORTS' in line: # add relevant sorts
            zoomed_sort = ''
            for sort in sorts:
                if (('#'+sort.name+' = ') in line) and (len(sort.rel_constants) != 0):
                    zoomed_sort = '#' + sort.name + ' = '
                    for const in sort.rel_constants:
                        zoomed_sort = zoomed_sort + const + ' + '
                    zoomed_sort = zoomed_sort[0:len(zoomed_sort)-3] + '.\n'
                    zoomed_asp.write(zoomed_sort)
        elif 'ZOOM INERTIAL FLUENTS' in line: # add relevant inertial fluents
            inertial_fluent_sort = '#physical_inertial_fluent = '
            for fluent in rel_inertial_fluents:
                inertial_fluent_sort = inertial_fluent_sort + fluent + ' + '
            inertial_fluent_sort = inertial_fluent_sort[0:len(inertial_fluent_sort)-3] + '.\n'
            zoomed_asp.write(inertial_fluent_sort)
        elif 'ZOOM DEFINED FLUENTS' in line: # add relevant defined fluents
            defined_fluent_sort = '#physical_defined_fluent = '
            for fluent in rel_defined_fluents:
                defined_fluent_sort = defined_fluent_sort + fluent + ' + '
            defined_fluent_sort = defined_fluent_sort[0:len(defined_fluent_sort)-3] + '.\n'
            zoomed_asp.write(defined_fluent_sort)
        elif 'ZOOM ACTIONS' in line: # add relevant actions
            action_sort = '#action = '
            for act in rel_actions:
                action_sort = action_sort + act + ' + '
            action_sort = action_sort[0:len(action_sort)-3] + '.\n'
            zoomed_asp.write(action_sort)
        else:
            line_relevant = True
            for sort in irrelevant_sort_names: # don't include predicates with irrelevant sorts
                if sort in line:
                    line_relevant = False
            for const in irrelevant_obj_consts: # don't include attributes with irrelevant object constants
                if const in line:
                    line_relevant = False
            for fluent in irrelevant_fluents: # don't include axioms with irrelevant fluents
                if fluent in line:
                    line_relevant = False
            for act in irrelevant_actions: # don't include axioms with irrelevant actions
                if act in line:
                    line_relevant = False
            if line_relevant:
                zoomed_asp.write(line)
    original_asp.close()
    zoomed_asp.close()

# Defines the sorts that may be included in a zoomed description
class Sort():
    def __init__(self, name, constants):
        self.name = name
        self.constants = constants
        self.rel_constants = []

    def add(self, constant):
        self.rel_constants.append(constant)
# Defines the refined components of each coarse object constant
class Components():
    def __init__(self, name, components):
        self.name = name
        self.components = components
