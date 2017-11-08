% This is the version of Intentional Agent I am working together with Heather. It consists of an environment of three rooms next_to in line to each other (library, kitchen, office).
% and a robot that moves from room to room. The library has a lock on it, and it can be locked or unlocked. There are two objects, book1 and book2 that my robot can pick up and put down. The robot can only hold one thing at a time. The doorway between the library and the kitchen may be locked or unlocked. My robot can unlock it if it is locked.
% There are two exogenous actions one that unlocks the doorway between library and kitchen, and the other that moves the book to a different room.



#const n = 15. % maximum number of steps.
#const max_name = 2.
#const max_len = 13. % maximum activity_length of an activity.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#step = 0..n.
#integer = 0..n.

#secure_room = {library}.
#room = #secure_room + {kitchen, office}.
#robot = {rob1}.
#book = {book1, book2}.
#object = #book.
#thing = #object + #robot.
#positive_index = 1..max_len.
#index = #positive_index + {neg1, 0}.
#activity_name = 1..max_name.
#boolean = {true, false}.


#physical_inertial_fluent = loc(#thing, #room) + in_hand(#robot, #object) + locked(#secure_room).
#possible_goal = {my_goal}.
#physical_defined_fluent = #possible_goal.

#physical_fluent = #physical_inertial_fluent + #physical_defined_fluent.

#physical_agent_action = move(#robot,#room) + pickup(#robot,#object) + put_down(#robot,#object) + unlock(#robot,#secure_room).
#physical_exogenous_action = exo_move(#object, #room) + exo_lock(#secure_room).

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
immediate_child_activity(#activity_name, #activity_name).
immediate_child_goal(#possible_goal, #possible_goal).
descendant(#activity_name, #activity_name).
minor_activity(#activity_name).
minor_goal(#possible_goal).


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
futile_goal(#possible_goal,#step).
planned_action(#agent_action, #step).


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
-holds(locked(L),I+1) :- occurs(unlock(R,L),I).
holds(locked(L),I+1) :- occurs(exo_lock(L),I).
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
impossible(move(R,L1),I) :- holds(locked(L1),I).
impossible(move(R,L2),I) :- holds(loc(R,L1),I), holds(locked(L1),I).
impossible(unlock(R,L), I) :- -holds(locked(L),I).
impossible(unlock(R,L1),I) :- holds(loc(R,L2),I), -next_to(L1,L2), L1 != L2.
impossible(put_down(R,O), I) :- -holds(in_hand(R,O), I).
impossible(pickup(R1,O), I) :- holds(in_hand(R2,O), I).
impossible(pickup(R,O), I) :- holds(loc(R,L1), I), holds(loc(O,L2), I) , L1 != L2.
impossible(exo_move(O,L),I) :- holds(loc(O,L),I).
impossible(exo_move(O,L),I) :- holds(locked(L),I).
impossible(exo_move(O,L2),I) :- holds(loc(O,L1),I), holds(locked(L1),I).
impossible(exo_move(O,L),I) :- holds(in_hand(O,L),I).
impossible(exo_lock(L),I) :- holds(locked(L),I).

%%%%%%%%%%%%%%
%% Defaults %%
%%%%%%%%%%%%%%
holds(loc(O,library),0) :- #book(O), not -holds(loc(O,library),0).
holds(loc(O,office),0) :- #book(O), -holds(loc(O,library),0), not -holds(loc(O,office),0).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Theory of Intention %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (1)
holds(current_action_index(AN,0), I+1) :- occurs(start(AN),I).
holds(current_action_index(AN,neg1), I+1) :- occurs(stop(AN),I).


%% (2)
holds(active_goal(G),I+1) :- occurs(select(G),I).
-holds(active_goal(G),I+1) :- occurs(abandon(G),I).

%% (3)
holds(current_action_index(AN,K+1),I+1) :- occurs(PAA,I),
				holds(next_action(AN,PAA),I),
				holds(current_action_index(AN,K),I),
				activity_component(AN,K+1,PAA),
				#physical_agent_action(PAA).

%% (4)
holds(next_available_name(AN+1),I+1) :- holds(next_available_name(AN),I),
        occurs(start(AN),I).

%% (5)
holds(current_action_index(AN, K + 1), I+1) :- holds(current_action_index(AN, K), I),
        activity_component(AN, K + 1, AN1),
        holds(next_action(AN, stop(AN1)), I),
        occurs(stop(AN1), I).

%% (6)
holds(current_action_index(AN1, neg1), I+1) :- descendant(AN1, AN),
        occurs(stop(AN), I).

%% (7)
-holds(current_action_index(AN,K1),I) :- holds(current_action_index(AN,K2),I),
					K1 != K2.

%% (8)
holds(active_activity(AN),I) :- -holds(current_action_index(AN,neg1),I).

%% (9)
-holds(active_goal(G),I) :- holds(G,I).

%% (10)
holds(in_progress_activity(AN),I) :- holds(active_activity(AN),I),
			holds(active_goal(G),I),
			activity_goal(AN,G).
holds(in_progress_goal(G),I) :- holds(active_activity(AN),I),
			holds(active_goal(G),I),
			activity_goal(AN,G).

%% (11)
holds(next_action(AN,PAA),I) :- holds(current_action_index(AN,K),I),
				activity_component(AN,K+1,PAA),
				holds(in_progress_activity(AN),I),
				#physical_agent_action(PAA).

%% (12)
-holds(next_available_name(AN),I) :- holds(next_available_name(AN1), I), AN != AN1.

%% (13) definition of my_goal: Included a the end.

%% (14)
immediate_child_activity(AN1, AN) :- activity_component(AN, K + 1, AN1),
        holds(current_action_index(AN, K), I).

%% (15)
immediate_child_goal(G1, G) :- immediate_child_activity(AN1, AN),
        activity_goal(AN, G),
        activity_goal(AN1, G1).

%% (16)
descendant(AN1, AN) :- immediate_child_activity(AN1, AN).

%% (17)
descendant(AN1, AN) :- descendant(AN1, AN),
        descendant(AN2, AN1).

%% (18)
minor_activity(AN1) :- immediate_child_activity(AN1, AN).

%% (19)
minor_goal(G1) :- immediate_child_goal(G1, G).

%% (20)
holds(active_goal(G1), I) :- immediate_child_goal(G1, G),
        holds(active_goal(G), I),
        activity_goal(AN1, G1),
        -holds(G1, I),
        holds(current_action_index(AN1, neg1), I).

%% (21)
-holds(active_goal(G1), I) :- immediate_child_goal(G1, G),
        -holds(active_goal(G), I).

%% (22)
-holds(active_goal(G1), I) :- immediate_child_goal(G1, G),
        holds(active_goal(G), I),
        activity_goal(AN1, G1),
        holds(current_action_index(AN1, K), I),
        activity_length(AN1, K).

%% (23)
holds(next_action(AN, start(AN1)), I) :- holds(current_action_index(AN, K), I),
        activity_component(AN, K + 1, AN1),
        holds(in_progress_activity(AN), I),
        -holds(active_activity(AN1), I).

%% (24)
holds(next_action(AN, AA), I) :- holds(current_action_index(AN, K), I),
        activity_component(AN, K + 1, AN1),
        holds(in_progress_activity(AN), I),
        holds(in_progress_activity(AN1), I),
        holds(next_action(AN1, AA), I).

%% (25)
holds(next_action(AN, stop(AN1)), I) :- holds(current_action_index(AN, K), I),
        activity_component(AN, K + 1, AN1),
        holds(in_progress_activity(AN), I),
        holds(active_activity(AN1), I),
        activity_goal(AN1, G1),
        -holds(active_goal(G1), I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Executability Conditions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (26)
impossible(start(AN), I) :- holds(active_activity(AN), I).
impossible(stop(AN), I) :- -holds(active_activity(AN), I).

%% (27)
impossible(PAA,I) :- occurs(MAA,I), #physical_agent_action(PAA), #mental_agent_action(MAA).
impossible(MAA,I) :- occurs(PAA,I), #physical_agent_action(PAA), #mental_agent_action(MAA).
impossible(MAA1,I) :- occurs(MAA2,I), #mental_agent_action(MAA1), #mental_agent_action(MAA2), MAA1 != MAA2.

%% (28) %impossible(wait,I) :- occurs(A1,I),  #physical_agent_action(A1).
%impossible(PAA,I) :- occurs(finish,I),  #physical_agent_action(PAA).
%impossible(MAA,I) :- occurs(finish,I),  #mental_agent_action(MAA).


%% (29)
impossible(select(G), I) :- holds(active_goal(G), I).
impossible(abandon(G), I) :- -holds(active_goal(G), I).
impossible(abandon(G), I) :- minor_goal(G).

%% (30)
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
%%  (31)
holds(F, 0) :- obs(F, true, 0).
-holds(F, 0) :- obs(F, false, 0).

%% (32)
:-current_step(I1), I <= I1, obs(F, false, I), holds(F,I).
:-current_step(I1), I <= I1, obs(F, true, I), -holds(F,I).

%% (33)
occurs(A,I) :- hpd(A, true, I), current_step(I1), I<I1.
-occurs(A,I) :- hpd(A, false, I), current_step(I1), I<I1.

% (34)
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

%% (35)
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

%  (36)
holds(current_action_index(AN,neg1),0).
-holds(active_goal(G),0).
holds(next_available_name(1),0).

% (37)
observed_result(AA,I) :- current_step(I1),
			I<=I1,
			hpd(AA,B,I),
			#agent_action(AA).

:- current_step(I1),
  	I<=I1,
   	attempt(AA,I),
    	not observed_result(AA,I),
	#agent_action(AA).

% (38)
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
%% (39)
occurs(PEA,I1) :+ current_step(I),
		explaining(I),
		I1<I,
		#physical_exogenous_action(PEA).
%%(40)
unobserved(PEA,I1) :- current_step(I),
		explaining(I),
		I1<I,
		occurs(PEA,I1),
		not hpd(PEA,true,I1),
		#physical_exogenous_action(PEA).

%%(41)
number_unobserved(N,I) :- current_step(I),
			explaining(I),
			N = #count{EX:unobserved(EX,IX)}.


%%(42)
%:-current_step(I),
%number_unobserved(N,I),
%explanation(X, I),
%N!= X.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rules for determining intended action %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (43)
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




%  (44)
no_activity_for_goal(G,I) :- current_step(I),
			explanation(N, I),
      not minor_goal(G),
			holds(active_goal(G),I),
      -holds(in_progress_goal(G),I).

%% (45)
no_goal_for_activity(AN,I) :- current_step(I),
			explanation(N, I),
      not minor_activity(AN),
			holds(active_activity(AN),I),
			activity_goal(AN,G),
			-holds(active_goal(G),I).

%% (46)
active_goal_activity(AN,I) :- current_step(I),
			explanation(N, I),
      not minor_activity(AN),
			holds(in_progress_activity(AN),I).


% (47)
intended_action(finish,I) :- current_step(I),
			explanation(N, I),
			no_goal_for_activity(AN,I).


%%%%%% Finding intended action for active_goal_activity
%% (48)
occurs(AA,I1) :- current_step(I),
		explanation(N, I),
		I <= I1,
    not minor_activity(AN),
		active_goal_activity(AN,I),
		holds(in_progress_activity(AN),I1),
		holds(next_action(AN,AA),I1),
		not impossible(AA,I1),
		#agent_action(AA).


%% (49)
projected_success(AN,I) :- current_step(I),
			explanation(N, I),
      not minor_activity(AN),
			I < I1,
		     	holds(active_activity(AN),I1),
		     	activity_goal(AN,G),
 			holds(G,I1).

%% (50)
-projected_success(AN,I) :-  current_step(I),
			explanation(N, I),
			not projected_success(AN,I).


%% (51)
intended_action(AA,I) :- current_step(I),
			explanation(N, I),
                        active_goal_activity(AN,I),
			holds(next_action(AN,AA),I),
			projected_success(AN,I),
			#agent_action(AA).

%% (52)
:- current_step(I),
	explanation(N, I),
	active_goal_activity(AN,I),
	-projected_success(AN,I),
	not futile_activity(AN,I).

%% (53)
futile_activity(AN,I) :+ current_step(I),
	explanation(N, I),
        active_goal_activity(AN,I),
	-projected_success(AN,I).

%% (54)
intended_action(stop(AN),I) :- current_step(I),
	explanation(N, I),
	active_goal_activity(AN,I),
	futile_activity(AN,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Creating a new activity by specifying its goal, activity_components and activity_length.%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (55)
candidate(AN,I) :-  current_step(I),
		explanation(N, I),
		no_activity_for_goal(G,I),
		holds(next_available_name(AN),I).


%% (56)
activity_goal(AN,G) :-  current_step(I),
		explanation(N, I),
            	no_activity_for_goal(G,I),
		candidate(AN,I).

%%  (57)
impossible(start(AN),I) :-  current_step(I),
			explanation(N, I),
                        no_activity_for_goal(G,I),
                        activity_goal(AN,G),
			occurs(start(AN1),I),
	 		AN != AN1.

%%  (58)
occurs(start(AN),I) :- current_step(I),
			explanation(N, I),
         	     	no_activity_for_goal(G,I),
			candidate(AN,I),
         	     	activity_goal(AN,G),
			not impossible(start(AN),I).

%% The following rule guarantees that candidates that are started by rule (Thesis: 5.31) achieve
%% the goal by forbidding all answer sets where there was not projected success. If none are left,
%% the the goal is futile and the intended action is defined by rules (Thesis: 5.34) and (Thesis: 5.35)

%%  (59)
%:- current_step(I),
%	explanation(N, I),
%	no_activity_for_goal(G,I),
%	occurs(start(AN),I),
%	-projected_success(AN,I),
%	not futile_goal(G,I).

%% (60)
%futile_goal(G,I) :- current_step(I),
%	explanation(N, I),
%	no_activity_for_goal(G,I),
%	occurs(start(AN),I),
%	-projected_success(AN,I).

%% (61)
intended_action(finish,I) :- current_step(I),
	explanation(N, I),
	no_activity_for_goal(G,I),
	futile_goal(G,I).

%% (62)
some_action_occurred(I1) :-  current_step(I),
			explanation(N, I),
			I <= I1,
			occurs(A,I1).


%% (63) original
occurs(PAA,I1) :+ current_step(I),
		explanation(N, I),
                no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		I < I1,
		some_action_occurred(I1-1),
		#physical_agent_action(PAA).


% (64)
activity_component(AN,I1-I,PAA) :- current_step(I),
		explanation(N, I),
		I < I1,
		no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		occurs(PAA,I1),
		#physical_agent_action(PAA).

% (65)
:- current_step(I),
	explanation(N, I),
	no_activity_for_goal(G,I),
	candidate(AN,I),
	activity_component(AN,K,PAA1),
	activity_component(AN,K,PAA2),
	PAA1 != PAA2,
	#physical_agent_action(PAA1),
	#physical_agent_action(PAA2).

% (66)
has_component(AN,K) :- current_step(I),
		explanation(N, I),
                no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		activity_component(AN,K,C).

% (67)
activity_length(AN,K) :- current_step(I),
		explanation(N, I),
                no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		has_component(AN,K),
                not has_component(AN,K+1).

% (68)
intended_action(start(AN),I) :- current_step(I),
		explanation(N, I),
		no_activity_for_goal(G,I),
		candidate(AN,I),
		occurs(start(AN),I),
		projected_success(AN,I).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Engine %%%%%%%%%%%%%%%%
% (69)
has_intention(I) :- intended_action(A,I).
:- current_step(I),
	explanation(N, I),
	0<I,
	not has_intention(I).



%%%%%%%%%%%%%%%%%
%% Attributes:
%%%%%%%%%%%%%%%%%%%
next_to(library, kitchen).
next_to(kitchen, office).
-next_to(L1,L2) :- not next_to(L1,L2).


-holds(in_hand(R,O),0) :- not holds(in_hand(R,O),0).

%%%%%%%%%
%% Goal:
%%%%%%%%%
%% @_@_@



%%%%%%%%%%%%%%%%%
%% Current Step:
%%%%%%%%%%%%%%%%%
%% *_*_*



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial State and history:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% #_#_# beginning
%% #_#_# end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obs.
hpd.
attempt.
activity_goal.
activity_component.
futile_activity.
futile_goal.
activity_length.
intended_action.
number_unobserved.
no_goal_for_activity.
unobserved.
