#const numSteps = 2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This ASP is used for inferring indirect observations from the direct observations set
%% collected during the execution of a refined plan. This ASP does not contain planning
%% or diagnosis. It does not have understanding of exogeneous actions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#coarse_place = {zoneR,zoneG,zoneY,above}.
#robot = {rob1}.
#coarse_object = {green_box, blue_box}.
#object = {green_box_top, blue_box_top}.
#coarse_thing = #coarse_object + #robot.
#place = {c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, unknown_cell}.
#thing = #object + #robot.

#step = 0..numSteps.
#boolean = {true, false}.
#outcome = {true, false, undet}.
#physical_inertial_fluent = loc(#thing, #place) + in_hand(#robot, #object).
#physical_defined_fluent = coarse_loc(#coarse_thing, #coarse_place) + coarse_in_hand(#robot, #coarse_object).
#physical_fluent = #physical_inertial_fluent + #physical_defined_fluent.
#knowledge_inertial_fluent = can_be_tested(#robot, #physical_inertial_fluent, #boolean) + directly_observed(#robot, #physical_inertial_fluent, #outcome).
#knowledge_defined_fluent = may_discover(#robot, #physical_defined_fluent,#boolean) + observed(#robot, #physical_fluent, #outcome) + indirectly_observed(#robot, #physical_defined_fluent, #outcome).
#inertial_fluent = #physical_inertial_fluent + #knowledge_inertial_fluent.
#defined_fluent = #physical_defined_fluent + #knowledge_defined_fluent.
#fluent = #inertial_fluent + #defined_fluent.
#action = test(#robot, #physical_inertial_fluent, #boolean) + move(#robot, #place) + pickup(#robot, #object) + put_down(#robot, #object).


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
indirect_observation_at_step(#physical_defined_fluent,#outcome,#step).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Causal Laws %%

% Moving changes location to target room (if the door is not locked).
holds(loc(R, C), I+1) :- occurs(move(R, C), I).


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

-holds(coarse_loc(T, C2), I) :- holds(coarse_loc(T, C1), I), C1!=C2.
-holds(coarse_in_hand(R, OP2), I) :- holds(coarse_in_hand(R, OP1), I), OP1!=OP2.

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

%% The next two executability conditions make sure that the robot has tested the objects location before trying to pick it up
%-occurs(pickup(rob1, OP), I) :- holds(loc(rob1, C), I), not occurs(test(rob1, loc(OP, C), true), I-1).
%-occurs(pickup(rob1, OP), I) :- I = 0.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Axioms for observing the environment %%
holds(can_be_tested(rob1, loc(T, C), V), I) :- holds(loc(rob1, C), I).
holds(can_be_tested(rob1, in_hand(rob1, OP), V), I).
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
holds(indirectly_observed(rob1, coarse_loc(T, R), true), I) :- holds(directly_observed(rob1, loc(Z, C), true), I), comp(C,R), comp(Z,T).
holds(may_discover(rob1, coarse_loc(T, R), true), I) :- -holds(indirectly_observed(rob1, coarse_loc(T, R), true), I), comp(C, R), comp(Z, T), holds(directly_observed(rob1, loc(Z, C), undet), I).

-holds(indirectly_observed(R,F,O),I) :- not holds(indirectly_observed(R,F,O),I).
-holds(may_discover(R,F,B),I) :- not holds(may_discover(R,F,B),I).
-holds(directly_observed(rob1, F, undet), I) :- holds(directly_observed(rob1, F, true), I).
-holds(directly_observed(rob1, F, undet), I) :- holds(directly_observed(rob1, F, false), I).
-holds(directly_observed(R,loc(O,C1),true),I) :- holds(directly_observed(R,loc(O,C2),true),I), C1!=C2.


%%% predicate used for output proposes
indirect_observation_at_step(F,O,I) :- holds(indirectly_observed(R,F,O),I), not holds(indirectly_observed(R,F,O),I-1).

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
:- occurs(A1,I), occurs(A2,I), A1 != A2, #action(A1), #action(A2).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Attributes.
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
next_to(c4, c7).
next_to(c8, c11).
next_to(c4, unknown_cell).
next_to(c7, unknown_cell).
next_to(c8, unknown_cell).
next_to(c11, unknown_cell).

-next_to(L1,L2) :- not next_to(L1,L2), #place(L1), #place(L2).
-next_to(L1,L2) :- not next_to(L1,L2), #coarse_place(L1), #coarse_place(L2).

comp(c1, zoneR).
comp(c2, zoneR).
comp(c3, zoneR).
comp(c4, zoneR).
comp(c5, zoneG).
comp(c6, zoneG).
comp(c7, zoneG).
comp(c8, zoneG).
comp(c9, zoneY).
comp(c10, zoneY).
comp(c11, zoneY).
comp(c12, zoneY).
comp(unknown_cell, above).
comp(green_box_top, green_box).
comp(blue_box_top, blue_box).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%%%%%%
%% HISTORY GOES HERE
holds(coarse_loc(blue_box,zoneR),0).
holds(coarse_loc(green_box,zoneG),0).
holds(coarse_in_hand(rob1,green_box),0).
-holds(coarse_in_hand(rob1,blue_box),0).
holds(coarse_loc(rob1,zoneG),0).
holds(loc(rob1,c7),0).
holds(directly_observed(rob1,loc(blue_box_top,c7),false),2).
holds(directly_observed(rob1,in_hand(rob1,green_box_top),false),2).
holds(directly_observed(rob1,loc(green_box_top,c7),true),2).

%%%%%%%%%%%%%%%%%
%% End of History:
%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(indirectly_observed(rob1,F,O),numSteps).
