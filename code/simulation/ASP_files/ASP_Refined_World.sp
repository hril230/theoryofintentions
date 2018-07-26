#const numSteps = 1. % maximum number of steps.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#coarse_place = {library,kitchen,office1, office2}.
#robot = {rob1}.
#coarse_object = {book1, book2}.
#object = {ref_book1, ref_book2}.
#coarse_thing = #coarse_object + #robot.
#place = {c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16}.
#thing = #object + #robot.

#step = 0..numSteps.
#boolean = {true, false}.
#outcome = {true, false, undet}.
#physical_inertial_fluent = loc(#thing, #place) + in_hand(#robot, #object).
#physical_defined_fluent = coarse_loc(#coarse_thing, #coarse_place) + coarse_in_hand(#robot, #coarse_object).
#physical_fluent = #physical_inertial_fluent + #physical_defined_fluent.
#knowledge_inertial_fluent = can_be_tested(#robot, #physical_inertial_fluent) + directly_observed(#robot, #physical_inertial_fluent, #outcome) + indirectly_observed(#robot, #physical_defined_fluent, #outcome).
#knowledge_defined_fluent = may_discover(#robot, #physical_defined_fluent) + observed(#robot, #physical_fluent).
#inertial_fluent = #physical_inertial_fluent + #knowledge_inertial_fluent.
#defined_fluent = #physical_defined_fluent + #knowledge_defined_fluent.
#fluent = #inertial_fluent + #defined_fluent.
#rob_action = test(#robot, #physical_inertial_fluent, #outcome) + move(#robot, #place) + pickup(#robot, #object) + put_down(#robot, #object).
#exo_action = exo_move(#object,#place).
#action = #rob_action + #exo_action.

#refined_component = #place + #object.
#coarse_component = #coarse_place + #coarse_object.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
next_to(#place, #place).
coarse_next_to(#coarse_place, #coarse_place).
holds(#fluent, #step).
occurs(#action, #step).
obs(#fluent, #boolean, #step).
hpd(#action, #step).
success().
goal(#step).
something_happened(#step).
comp(#refined_component, #coarse_component).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Causal Laws %%

% Moving changes location to target room (if the door is not locked).
holds(loc(R, C), I+1) :- occurs(move(R, C), I).

%% Exogenous moving an object causes the object to be in a different location.
holds(loc(O,L),I+1) :- occurs(exo_move(O,L),I).

% Grasping an object causes object to be in hand.
holds(in_hand(R, OP), I+1) :- occurs(pickup(R, OP), I).

% Putting an object down causes it to no longer be in hand.
-holds(in_hand(R, OP), I+1) :- occurs(put_down(R, OP), I).



%% State Constraints %%

% Reflexive property of next_to relation.
next_to(C1, C2) :- next_to(C2, C1), #place(C1), #place(C2).

% Any object exists in only one location.
-holds(loc(T, C2), I) :- holds(loc(T, C1), I), C1!=C2.

% If a robot is holding an object, they have the same location.
holds(loc(OP, C), I) :- holds(loc(R, C), I), holds(in_hand(R, OP), I).

% Only one object can be held at any time.
-holds(in_hand(R, OP2), I) :- holds(in_hand(R, OP1), I), OP1!=OP2.

% Axioms relating coarse-resolution attributes and fine-resolution counterparts
holds(coarse_loc(T, Z), I) :- holds(loc(T, C), I), comp(C, Z).
holds(coarse_loc(B, Z),I) :- holds(loc(RB,C), I), comp(RB,B), comp(C,Z).
holds(coarse_in_hand(rob1, O), I) :- holds(in_hand(rob1, OP), I), comp(OP, O).
-holds(coarse_in_hand(rob1, O), I) :- -holds(in_hand(rob1, OP), I), comp(OP, O), not holds(coarse_in_hand(rob1, O), I).
coarse_next_to(Z1, Z2) :- next_to(C1, C2), comp(C1, Z1), comp(C2, Z2), #place(C1), #place(C2).



%% Executability Conditions %%

% Cannot move to a location if you are already there.
-occurs(move(R, C), I) :- holds(loc(R, C), I).

% Cannot move to a location if it is not next to it.
-occurs(move(R, C2), I) :- holds(loc(R, C1), I), -next_to(C1,C2).

% Cannot put down an object unless it is in hand.
-occurs(put_down(R, OP), I) :-  -holds(in_hand(R, OP), I).

% Cannot pick up an object if it has something in hand.
-occurs(pickup(R, OP1), I) :- holds(in_hand(R, OP2), I).

% Cannot pick up an object if you are not in the same room.
-occurs(pickup(R, OP), I) :- holds(loc(R, C), I), not holds(loc(OP, C), I).

% The next two executability conditions make sure that the robot has tested the objects location before trying to pick it up
-occurs(pickup(rob1, OP), I) :- holds(loc(rob1, C), I), not occurs(test(rob1, loc(OP, C), true), I-1).
-occurs(pickup(rob1, OP), I) :- I = 0.

%% An exogenous move of an object cannot be done to the same location.
-occurs(exo_move(O,L),I) :- holds(loc(O,L),I).

%% An exogenous move of an object cannot happen if it is being in hand
-occurs(exo_move(O,L),I) :- holds(in_hand(R,O),I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Axioms for observing the environment %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(can_be_tested(rob1, loc(T, C)), I) :- holds(loc(rob1, C), I).
holds(can_be_tested(rob1, in_hand(rob1, OP)), I).
holds(directly_observed(rob1, F, true), I+1) :- holds(F, I), occurs(test(rob1, F, true), I).
holds(directly_observed(rob1, F, false), I+1) :- -holds(F, I), occurs(test(rob1, F, false), I).
-occurs(test(rob1, F, O), I) :- -holds(can_be_tested(rob1, F, O), I).
holds(indirectly_observed(rob1, coarse_loc(T, R), true), I) :- holds(directly_observed(rob1, loc(T, C), true), I), comp(C, R).
holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I) :- holds(directly_observed(rob1, in_hand(rob1, OP), true), I), comp(OP, O).
holds(indirectly_observed(rob1, coarse_loc(T, R), false), I) :- -holds(indirectly_observed(rob1, coarse_loc(T, R), true), I), -holds(may_discover(rob1, coarse_loc(T, R), true), I).
holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), false), I) :- -holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I), -holds(may_discover(R, coarse_in_hand(rob1, O), true), I).
holds(may_discover(rob1, coarse_loc(T, R), true), I) :- -holds(indirectly_observed(rob1, coarse_loc(T, R), true), I), comp(C, R), holds(directly_observed(rob1, loc(T, C), undet), I).
holds(may_discover(rob1, coarse_in_hand(rob1, O), true), I) :- -holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I), comp(OP, O), holds(directly_observed(rob1, in_hand(rob1, OP), undet), I).
holds(directly_observed(rob1, F, undet), I) :- not holds(directly_observed(rob1, F, true), I), not holds(directly_observed(rob1, F, false), I).
holds(observed(rob1, F, O), I) :- holds(directly_observed(rob1, F, O), I).
holds(observed(rob1, F, O), I) :- holds(indirectly_observed(rob1, F, O), I).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inertial axiom + CWA

%% General inertia axioms.
holds(F,I+1) :- #inertial_fluent(F), holds(F,I), not -holds(F,I+1).

-holds(F,I+1) :- #inertial_fluent(F), -holds(F,I), not holds(F,I+1).

%% CWA for Actions.
-occurs(A,I) :- not occurs(A,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% History and initial state rules

%% Take what actually happened into account.
occurs(A,I) :- hpd(A,I).


%% Reality check axioms.
:- obs(F, true, I), -holds(F, I).
:- obs(F, false, I), holds(F, I).

%% Awareness axiom.
holds(F, 0) | -holds(F, 0) :- #physical_inertial_fluent(F).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planning Module


%% Cannot execute two actions at the same time.
:- occurs(A1,I), occurs(A2,I), A1 != A2, #rob_action(A1), #rob_action(A2).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Attributes.

next_to(c1, c2).
next_to(c1, c3).
next_to(c1, c2).
next_to(c1, c3).
next_to(c2, c4).
next_to(c3, c4).
next_to(c5, c6).
next_to(c5, c7).
next_to(c6, c8).
next_to(c7, c8).
next_to(c9, c10).
next_to(c9, c11).
next_to(c10, c12).
next_to(c11, c12).
next_to(c13, c14).
next_to(c13, c15).
next_to(c14, c15).
next_to(c4, c7).
next_to(c8, c11).
next_to(c12, c15).
-next_to(L1,L2) :- not next_to(L1,L2), #place(L1), #place(L2).
-next_to(L1,L2) :- not next_to(L1,L2), #coarse_place(L1), #coarse_place(L2).

comp(c1, library).
comp(c2, library).
comp(c3, library).
comp(c4, library).
comp(c5, kitchen).
comp(c6, kitchen).
comp(c7, kitchen).
comp(c8, kitchen).
comp(c9, office1).
comp(c10, office1).
comp(c11, office1).
comp(c12, office1).
comp(c13, office2).
comp(c14, office2).
comp(c15, office2).
comp(c16, office2).
comp(ref_book1, book1).
comp(ref_book2, book2).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%
%% *_*_*
obs(loc(rob1,c11),true,0).
obs(loc(ref_book1,c13),true,0).
obs(loc(ref_book2,c10),true,0).
obs(in_hand(rob1,ref_book1),false,0).
obs(in_hand(rob1,ref_book2),false,0).
hpd(test(rob1,loc(ref_book2,c11),true),0).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(loc(A,B),numSteps).
holds(in_hand(A,B),numSteps).
-holds(in_hand(rob1,B),numSteps).
holds(coarse_loc(A,B),numSteps).
holds(coarse_in_hand(A,B),numSteps).
-holds(coarse_in_hand(A,B),numSteps).
holds(directly_observed(rob1,F,true),1).
holds(directly_observed(rob1,F,false),1).
