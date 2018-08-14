#const numSteps = 5. % maximum number of steps.
#const max_len = 4. % maximum activity_length of an activity.
#const max_name = 1.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This ASP is used to plan with ToI and to Diagnose with ToI.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#step = 0..numSteps.
#integer = 0..numSteps.

#room = {library, kitchen, office1, office2}.
#robot = {rob1}.
#book = {book1, book2}.
#object = #book.
#thing = #object + #robot.
#positive_index = 1..max_len.
#index = #positive_index + {neg1, 0}.
#activity_name = 1..max_name.
#boolean = {true, false}.


#physical_inertial_fluent = loc(#thing, #room) + in_hand(#robot, #object).
#possible_goal = {my_goal}.
#physical_defined_fluent = #possible_goal.

#physical_fluent = #physical_inertial_fluent + #physical_defined_fluent.

#physical_agent_action = move(#robot,#room) + pickup(#robot,#object) + put_down(#robot,#object).
#physical_exogenous_action = exo_move(#object, #room).

#mental_agent_action = start(#activity_name) + stop(#activity_name).

#mental_exogenous_action = select(#possible_goal) + abandon(#possible_goal).
#exogenous_action = #mental_exogenous_action + #physical_exogenous_action.
#agent_action = #physical_agent_action + #mental_agent_action + {finish}.
#action = #agent_action + #exogenous_action.

#mental_defined_fluent = active_activity(#activity_name) + in_progress_activity(#activity_name) +
 in_progress_goal(#possible_goal) + next_action(#activity_name, #action).
#defined_fluent = #mental_defined_fluent + #physical_defined_fluent.

#mental_inertial_fluent = active_goal(#possible_goal) + current_action_index(#activity_name, #index) + next_available_name(#activity_name).
#inertial_fluent = #physical_inertial_fluent + #mental_inertial_fluent.

#fluent = #inertial_fluent + #defined_fluent.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% static relation
next_to(#room,#room).
activity_goal(#activity_name,#possible_goal).
activity_component(#activity_name,#index,#physical_agent_action).
activity_length(#activity_name,#index).


holds(#fluent,#step).
occurs(#action,#step).


%% used in history
obs(#fluent, #boolean, #step).
hpd(#action, #boolean, #step).
attempt(#action,#step).
impossible(#action, #step).
current_step(#step).
observed_result(#action, #step).
unobserved(#physical_exogenous_action, #step).
number_unobserved(#integer,#step).
explanation(#integer,#step).
explaining(#step).

%% three different situations determined by history
no_goal_for_activity(#activity_name, #step).
active_goal_activity(#activity_name, #step).
no_activity_for_goal(#possible_goal, #step).


%% used to create a new plan
active_goal_or_activity(#step).
some_action_occurred(#step).
intended_action(#agent_action, #step).
projected_success(#activity_name,#step).
has_intention(#step).

candidate(#activity_name,#step).
has_component(#activity_name,#index).
equal_activities(#activity_name,#activity_name).
equal_components(#activity_name,#activity_name).
different_component(#activity_name,#activity_name).

futile_activity(#activity_name,#step).

selected_goal_holds(#possible_goal,#step).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inertial axiom + CWA %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CWA for Defined Fluents (Thesis: 2.12)
-holds(F,I) :- #defined_fluent(F),
               not holds(F,I).

%%  General inertia axioms... (Thesis: 2.15)
holds(F,I+1) :- #inertial_fluent(F),
                holds(F,I),
                not -holds(F,I+1).
-holds(F,I+1) :- #inertial_fluent(F),
                 -holds(F,I),
                 not holds(F,I+1).

%%  CWA for Actions... (Thesis: 2.16)
-occurs(A,I) :- not occurs(A,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Physical Causal Laws %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(loc(R, L), I+1) :- occurs(move(R, L), I).
holds(in_hand(R,O),I+1) :- occurs(pickup(R,O), I).
-holds(in_hand(R,O),I+1) :- occurs(put_down(R,O), I).
holds(loc(O,L),I+1) :- occurs(exo_move(O,L),I).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Physical State Constraints %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
next_to(L1,L2) :- next_to(L2,L1).
-holds(loc(T, L2), I) :- holds(loc(T, L1), I), L1!=L2.
holds(loc(O,L), I) :- holds(loc(R,L), I) , holds(in_hand(R,O),I).
-holds(in_hand(R, O2), I) :- holds(in_hand(R, O1), I), O1!=O2.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Physical Executability Condition %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
-occurs(A,I) :- impossible(A,I).

%% (Desc: 10 - 14)
impossible(move(R,L),I) :- holds(loc(R,L),I).
impossible(move(R,L2),I) :- holds(loc(R,L1),I), -next_to(L1,L2).


impossible(put_down(R,O), I) :- -holds(in_hand(R,O), I).
impossible(pickup(R,O1), I) :- holds(in_hand(R,O2), I).
impossible(pickup(R,O), I) :- holds(loc(R,L), I), not holds(loc(O,L), I).
impossible(exo_move(O,L),I) :- holds(loc(O,L),I).
impossible(exo_move(O,L),I) :- holds(in_hand(R,L),I).

%%%%%%%%%%%%%%
%% Defaults %%
%%%%%%%%%%%%%%
holds(loc(O,library),0) :- #book(O), not -holds(loc(O,library),0).
holds(loc(O,office1),0) :- #book(O), -holds(loc(O,library),0), not -holds(loc(O,office1),0).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Theory of Intention %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (1)
holds(current_action_index(AN,0), I+1) :- occurs(start(AN),I).
holds(current_action_index(AN,neg1), I+1) :- occurs(stop(AN),I).


%% (2)
holds(active_goal(G),I+1) :- occurs(select(G),I), not holds(G,I).
-holds(active_goal(G),I+1) :- occurs(abandon(G),I).

%% (3)
holds(current_action_index(AN,K+1),I+1) :- occurs(PAA,I),
				holds(next_action(AN,PAA),I),
				holds(current_action_index(AN,K),I),
				activity_component(AN,K+1,PAA),
				#physical_agent_action(PAA).

%% (4)
holds(next_available_name(AN+1),I+1) :- holds(next_available_name(AN),I), occurs(start(AN),I).

%% (5)
-holds(current_action_index(AN,K1),I) :- holds(current_action_index(AN,K2),I),
					K1 != K2.

%% (6)
holds(active_activity(AN),I) :- -holds(current_action_index(AN,neg1),I).

%% (7)
-holds(active_goal(G),I) :- holds(G,I).

%% (8)
holds(in_progress_activity(AN),I) :- holds(active_activity(AN),I),
			holds(active_goal(G),I),
			activity_goal(AN,G).
holds(in_progress_goal(G),I) :- holds(active_activity(AN),I),
			holds(active_goal(G),I),
			activity_goal(AN,G).

%% (9)
holds(next_action(AN,PAA),I) :- holds(current_action_index(AN,K),I),
				activity_component(AN,K+1,PAA),
				holds(in_progress_activity(AN),I),
				#physical_agent_action(PAA).

%% (10)
-holds(next_available_name(AN),I) :- holds(next_available_name(AN1), I), AN != AN1.






%% (11) definition of my_goal: Included a the end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Executability Conditions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (12)
impossible(start(AN), I) :- holds(active_activity(AN), I).
impossible(stop(AN), I) :- -holds(active_activity(AN), I).

%% (13)
impossible(PAA,I) :- occurs(MAA,I), #physical_agent_action(PAA), #mental_agent_action(MAA).
impossible(MAA,I) :- occurs(PAA,I), #physical_agent_action(PAA), #mental_agent_action(MAA).
impossible(MAA1,I) :- occurs(MAA2,I), #mental_agent_action(MAA1), #mental_agent_action(MAA2), MAA1 != MAA2.

%% (14) %impossible(wait,I) :- occurs(A1,I),  #physical_agent_action(A1).
%impossible(PAA,I) :- occurs(finish,I),  #physical_agent_action(PAA).
%impossible(MAA,I) :- occurs(finish,I),  #mental_agent_action(MAA).


%% (15)
impossible(select(G), I) :- holds(active_goal(G), I).
impossible(abandon(G), I) :- -holds(active_goal(G), I).

%% (16)
impossible(PAA,I) :- occurs(MEA,I), #mental_exogenous_action(MEA), #physical_agent_action(PAA).
impossible(MEA,I) :- occurs(PAA,I), #mental_exogenous_action(MEA), #physical_agent_action(PAA).

impossible(PEA,I) :- occurs(MEA,I), #mental_exogenous_action(MEA), #physical_exogenous_action(PEA).
impossible(MEA,I) :- occurs(PEA,I), #mental_exogenous_action(MEA), #physical_exogenous_action(PEA).

impossible(MAA,I) :- occurs(MEA,I), #mental_exogenous_action(MEA), #mental_agent_action(MAA).
impossible(MEA,I) :- occurs(MAA,I), #mental_exogenous_action(MEA), #mental_agent_action(MAA).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Automatic Behaviour %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% History records and initial state rules %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  (17)
holds(F, 0) :- obs(F, true, 0).
-holds(F, 0) :- obs(F, false, 0).

%% (18)
:-current_step(I1), I <= I1, obs(F, false, I), holds(F,I).
:-current_step(I1), I <= I1, obs(F, true, I), -holds(F,I).

%% (19)
occurs(A,I) :- hpd(A, true, I), current_step(I1), I<I1.
-occurs(A,I) :- hpd(A, false, I), current_step(I1), I<I1.

% (20)
occurs(AA,I) :- current_step(I1),
   		I<I1,
   		attempt(AA,I),
   		not impossible(AA,I),
		#agent_action(AA).

:- current_step(I1),
	I<I1,
  	occurs(AA,I),
 	not attempt(AA,I),
  	#agent_action(AA).

%% (21)
impossible(select(G),I) :- current_step(I1),
			I<I1,
   			occurs(select(G1),I),
			G1 != G.
impossible(select(G),I) :- current_step(I1),
			I<I1,
   			holds(active_activity(AN),I).
impossible(select(G),I) :- current_step(I1),
			I<I1,
   			holds(active_goal(G1),I).

%  (22)
holds(current_action_index(AN,neg1),0).
-holds(active_goal(G),0).
holds(next_available_name(1),0).


% (24)
:- current_step(I1),
	I<=I1,
	occurs(select(G),I),
	not hpd(select(G),true,I).

:- current_step(I1),
	I<=I1,
	occurs(abandon(G),I),
	not hpd(abandon(G),true,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Diagnosys Generation %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (25)
occurs(PEA,I1) :+ current_step(I),
		explaining(I),
		I1<I,
		#physical_exogenous_action(PEA).
%%(26)
unobserved(PEA,I1) :- current_step(I),
		explaining(I),
		I1<I,
		occurs(PEA,I1),
		not hpd(PEA,true,I1),
		#physical_exogenous_action(PEA).

%%(27)
number_unobserved(N,I) :- current_step(I),
			explaining(I),
			N = #count{EX:unobserved(EX,IX)}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rules for determining intended action %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (29)
different_component(AN,AN1) :- activity_component(AN,K,AA),
	activity_component(AN1,K,AA1),
	AA != AA1.
equal_components(AN,AN1) :- activity_length(AN,L),
			activity_length(AN1,L),
			not different_component(AN,AN1).

equal_activities(AN,AN1) :- activity_goal(AN,G),
			activity_goal(AN1,G),
			equal_components(AN,AN1).

:- equal_activities(AN,AN1), AN != AN1.




%  (30)
no_activity_for_goal(G,I) :- current_step(I),
			explanation(N, I),
			holds(active_goal(G),I),
                        -holds(in_progress_goal(G),I).

%% (31)
no_goal_for_activity(AN,I) :- current_step(I),
			explanation(N, I),
			holds(active_activity(AN),I),
			activity_goal(AN,G),
			-holds(active_goal(G),I).

%% (32)
active_goal_activity(AN,I) :- current_step(I),
			explanation(N, I),
			holds(in_progress_activity(AN),I).


% (22)
intended_action(finish,I) :- current_step(I),
			explanation(N, I),
			no_goal_for_activity(AN,I).


%%%%%% Finding intended action for active_goal_activity
%% (34)
occurs(AA,I1) :- current_step(I),
		explanation(N, I),
		I <= I1,
		active_goal_activity(AN,I),
		holds(in_progress_activity(AN),I1),
		holds(next_action(AN,AA),I1),
		not impossible(AA,I1),
		#agent_action(AA).

% (35)
projected_success(AN,I) :- current_step(I),
			explanation(N, I),
			I < I1,
		     	holds(active_activity(AN),I1),
		     	activity_goal(AN,G),
 			holds(G,I1).
% (36)
-projected_success(AN,I) :-  current_step(I),
			explanation(N, I),
			not projected_success(AN,I).


%% (37)
intended_action(AA,I) :- current_step(I),
			explanation(N, I),
                        active_goal_activity(AN,I),
			holds(next_action(AN,AA),I),
			projected_success(AN,I),
			#agent_action(AA).

%% (38)
:- current_step(I),
	explanation(N, I),
	active_goal_activity(AN,I),
	-projected_success(AN,I),
	not futile_activity(AN,I).

%% (39)
futile_activity(AN,I) :+ current_step(I),
	explanation(N, I),
        active_goal_activity(AN,I),
	-projected_success(AN,I).

%% (40)
intended_action(stop(AN),I) :- current_step(I),
	explanation(N, I),
	active_goal_activity(AN,I),
	futile_activity(AN,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Creating a new activity by specifying its goal, activity_components and activity_length.%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (41)
candidate(AN,I) :-  current_step(I),
		explanation(N, I),
		no_activity_for_goal(G,I),
		holds(next_available_name(AN),I).


%% (42)
activity_goal(AN,G) :-  current_step(I),
		explanation(N, I),
            	no_activity_for_goal(G,I),
		candidate(AN,I).

%%  (43)
impossible(start(AN),I) :-  current_step(I),
			explanation(N, I),
                        no_activity_for_goal(G,I),
                        activity_goal(AN,G),
			occurs(start(AN1),I),
	 		AN != AN1.

%%  (44)
occurs(start(AN),I) :- current_step(I),
			explanation(N, I),
         	     	no_activity_for_goal(G,I),
			candidate(AN,I),
         	     	activity_goal(AN,G),
			not impossible(start(AN),I).


%% (48)
some_action_occurred(I1) :-  current_step(I),
			explanation(N, I),
			I <= I1,
			occurs(A,I1).


%% (49) original
occurs(PAA,I1) :+ current_step(I),
		explanation(N, I),
                no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		I < I1,
		some_action_occurred(I1-1),
		#physical_agent_action(PAA).



% (50)
activity_component(AN,I1-I,PAA) :- current_step(I),
		explanation(N, I),
		I < I1,
		no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		occurs(PAA,I1),
		#physical_agent_action(PAA).

% (51)
:- current_step(I),
	explanation(N, I),
	no_activity_for_goal(G,I),
	candidate(AN,I),
	activity_component(AN,K,PAA1),
	activity_component(AN,K,PAA2),
	PAA1 != PAA2,
	#physical_agent_action(PAA1),
	#physical_agent_action(PAA2).

% (52)
has_component(AN,K) :- current_step(I),
		explanation(N, I),
                no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		activity_component(AN,K,C).

% (53)
activity_length(AN,K) :- current_step(I),
		explanation(N, I),
                no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		has_component(AN,K),
                not has_component(AN,K+1).

% (54)
intended_action(start(AN),I) :- current_step(I),
		explanation(N, I),
		no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		projected_success(AN,I).



%% latest addition
selected_goal_holds(G,I) :- current_step(I),
		holds(G,I),
		occurs(select(G),I1),
		I1 <= I.



intended_action(finish,I) :- current_step(I),
			explanation(N,I),
			selected_goal_holds(G,I).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Engine %%%%%%%%%%%%%%%%
% (55)
has_intention(I) :- intended_action(A,I).
:- current_step(I),
	explanation(N, I),
	0<I,
	not has_intention(I).



%%%%%%%%%%%%%%%%%
%%Attributes:
%%%%%%%%%%%%%%%%%%%
next_to(library, kitchen).
next_to(office2,office1).
next_to(office1,kitchen).
-next_to(L1,L2) :- not next_to(L1,L2).


-holds(in_hand(R,O),0) :- not holds(in_hand(R,O),0).

%%%%%%%%%
%% Goal:
%%%%%%%%%
%% GOAL GOES HERE
holds(my_goal,I) :- holds(loc(book1,library),I), holds(loc(book2,library),I), -holds(in_hand(rob1,book1),I), -holds(in_hand(rob1,book2),I) .



%%%%%%%%%%%%%%%%%
%% Current Step:
%%%%%%%%%%%%%%%%%
%% CURRENT STEP GOES HERE
current_step(3).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial State and history:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HISTORY GOES HERE
hpd(select(my_goal),true,0).
attempt(start(1),1).
activity_goal(1,my_goal).
activity_component(1,1,put_down(rob1,book1)).
activity_length(1,1).
holds(loc(book1,library),0).
holds(loc(book2,library),0).
holds(loc(rob1,library),0).
holds(in_hand(rob1,book1),0).
-holds(in_hand(rob1,book2),0).
,3).
attempt(put_down(rob1,book1),2).
explaining(3).





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obs.
hpd.
attempt.
activity_goal.
activity_component.
futile_activity.
activity_length.
intended_action.
number_unobserved.
selected_goal_holds.
unobserved.
holds(loc(A,B),0).
holds(in_hand(R,B),0).
-holds(in_hand(R,B),0).
