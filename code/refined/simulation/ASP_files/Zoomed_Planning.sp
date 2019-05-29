#const numSteps = 9.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#step = 0..numSteps.
#coarse_place = {kitchen,library}.
#coarse_object = {markerPen2}.
#object = {ref1_markerPen2}.
#place = {c1,c10,c11,c12,c13,c14,c15,c16,c17,c18,c19,c2,c20,c21,c22,c23,c24,c25,c26,c27,c28,c29,c3,c30,c31,c32,c4,c5,c6,c7,c8,c9}.
#robot = {rob1}.
#coarse_thing = #coarse_object + #robot.
#thing = #object + #robot.
#boolean = {false,true}.
#outcome = {false,true,undet}.
#physical_inertial_fluent = loc(#thing,#place) + in_hand(#robot,#object).
#physical_defined_fluent = coarse_loc(#coarse_thing,#coarse_place) + coarse_in_hand(#robot,#coarse_object).
#physical_fluent = #physical_inertial_fluent + #physical_defined_fluent.
#knowledge_inertial_fluent = directly_observed(#robot,#physical_inertial_fluent,#outcome) + can_be_tested(#robot,#physical_inertial_fluent,#boolean).
#knowledge_defined_fluent = observed(#robot,#physical_fluent,#outcome) + may_discover(#robot,#physical_defined_fluent,#boolean) + indirectly_observed(#robot,#physical_defined_fluent,#outcome).
#inertial_fluent = #knowledge_inertial_fluent + #physical_inertial_fluent.
#defined_fluent = #knowledge_defined_fluent + #physical_defined_fluent.
#fluent = #inertial_fluent + #defined_fluent.
#rob_action = test(#robot,#physical_inertial_fluent,#boolean) + pickup(#robot,#object) + move(#robot,#place) + put_down(#robot,#object).
#exo_action = exo_move(#object,#place).
#action = #exo_action + #rob_action.
#refined_component = #object + #place.
#coarse_component = #coarse_object + #coarse_place.

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

%%%%%%%%%%%%%%%%%
%% Causal Laws %%
%%%%%%%%%%%%%%%%%
%% CAUSAL LAWS GO HERE
% Moving changes location to target room (if the door is not locked).
holds(loc(R, C), I+1) :- occurs(move(R, C), I).

% Grasping an object causes object to be in hand.
holds(in_hand(R, OP), I+1) :- occurs(pickup(R, OP), I).

% Putting an object down causes it to no longer be in hand.
-holds(in_hand(R, OP), I+1) :- occurs(put_down(R, OP), I).


%%%%%%%%%%%%%%%%%%%%%%%
%% State Constraints %%
%%%%%%%%%%%%%%%%%%%%%%%
%% STATE CONSTRAINTS GO HERE
% Reflexive property of next_to relation.
next_to(C1, C2) :- next_to(C2, C1), #place(C1), #place(C2).

% Any object exists in only one refined location.
-holds(loc(T, C2), I) :- holds(loc(T, C1), I), C1!=C2.

% If a robot is holding an object, they have the same location.
holds(loc(OP, C), I) :- holds(loc(R, C), I), holds(in_hand(R, OP), I).

% Only one object can be held at any time.
-holds(in_hand(R, OP2), I) :- holds(in_hand(R, OP1), I), OP1!=OP2.
-holds(coarse_in_hand(R, OP2), I) :- holds(coarse_in_hand(R, OP1), I), OP1!=OP2.

% A place/coarse_place is not next to another unless specified.
-next_to(L1,L2) :- not next_to(L1,L2), #place(L1), #place(L2).
-coarse_next_to(L1,L2) :- not coarse_next_to(L1,L2), #coarse_place(L1), #coarse_place(L2).

% A thing cannot be at two places at the same time.
-holds(coarse_loc(T, C2), I) :- holds(coarse_loc(T, C1), I), C1!=C2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Coarse-resolution attributes and fine-resolution counterparts %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(coarse_loc(T, Z), I) :- holds(loc(T, C), I), comp(C, Z).
holds(coarse_loc(B, Z),I) :- holds(loc(RB,C), I), comp(RB,B), comp(C,Z).
holds(coarse_in_hand(rob1, O), I) :- holds(in_hand(rob1, OP), I), comp(OP, O).
holds(loc(RB2, C) ,I) :- holds(loc(RB1, C), I), comp(RB1, B), comp(RB2,B).
coarse_next_to(Z1, Z2) :- next_to(C1, C2), comp(C1, Z1), comp(C2, Z2), Z1!=Z2, #place(C1), #place(C2).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Executability Conditions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cannot move to a location if you are already there.
-occurs(move(R, C), I) :- holds(loc(R, C), I).

% Cannot move to a location if it is not next to it.
-occurs(move(R, C2), I) :- holds(loc(R, C1), I), -next_to(C1,C2).

% Cannot put down an object unless it is in hand.
-occurs(put_down(R, OP), I) :-  -holds(in_hand(R, OP), I).

% Cannot pick up an object if it has something in hand.
-occurs(pickup(R, O), I) :- holds(in_hand(R, CO), I).

% Cannot pick up an object if you are not in the same room.
-occurs(pickup(R, OP), I) :- holds(loc(R, C), I), not holds(loc(OP, C), I).

% Cannot execute two actions at the same time.
:- occurs(A1,I), occurs(A2,I), A1 != A2, #rob_action(A1), #rob_action(A2).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Axioms for observing the environment %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(can_be_tested(rob1, loc(T, C), V), I) :- holds(loc(rob1, C), I).
holds(can_be_tested(rob1, in_hand(rob1, OP), V), I).
holds(directly_observed(rob1, F, true), I+1) :- holds(F, I), occurs(test(rob1, F, true), I).
holds(directly_observed(rob1, F, false), I+1) :- -holds(F, I), occurs(test(rob1, F, false), I).
-occurs(test(rob1, F, O), I) :- -holds(can_be_tested(rob1, F, O), I).

holds(indirectly_observed(rob1, coarse_loc(T, R), true), I) :- holds(directly_observed(rob1, loc(T, C), true), I), comp(C, R).
holds(indirectly_observed(rob1, coarse_loc(T, R), true), I) :- holds(directly_observed(rob1, loc(Z, C), true), I), comp(C,R), comp(Z,T).
holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I) :- holds(directly_observed(rob1, in_hand(rob1, OP), true), I), comp(OP, O).

holds(indirectly_observed(rob1, coarse_loc(T, R), false), I) :- -holds(indirectly_observed(rob1, coarse_loc(T, R), true), I), -holds(may_discover(rob1, coarse_loc(T, R), true), I).
holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), false), I) :- -holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I), -holds(may_discover(R, coarse_in_hand(rob1, O), true), I).
holds(may_discover(rob1, coarse_loc(T, R), true), I) :- -holds(indirectly_observed(rob1, coarse_loc(T, R), true), I), comp(C, R), holds(directly_observed(rob1, loc(T, C), undet), I).
holds(may_discover(rob1, coarse_in_hand(rob1, O), true), I) :- -holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I), comp(OP, O), holds(directly_observed(rob1, in_hand(rob1, OP), undet), I).
holds(directly_observed(rob1, F, undet), I) :- not holds(directly_observed(rob1, F, true), I), not holds(directly_observed(rob1, F, false), I).
holds(observed(rob1, F, O), I) :- holds(directly_observed(rob1, F, O), I).
holds(observed(rob1, F, O), I) :- holds(indirectly_observed(rob1, F, O), I).
holds(may_discover(rob1, coarse_loc(T, R), true), I) :- -holds(indirectly_observed(rob1, coarse_loc(T, R), true), I), comp(C, R), comp(Z, T), holds(directly_observed(rob1, loc(Z, C), undet), I).

-holds(indirectly_observed(R,F,O),I) :- not holds(indirectly_observed(R,F,O),I).
-holds(may_discover(R,F,B),I) :- not holds(may_discover(R,F,B),I).
-holds(directly_observed(rob1, F, undet), I) :- holds(directly_observed(rob1, F, true), I).
-holds(directly_observed(rob1, F, undet), I) :- holds(directly_observed(rob1, F, false), I).
-holds(directly_observed(R,loc(O,C1),true),I) :- holds(directly_observed(R,loc(O,C2),true),I), C1!=C2.

%%%%%%%%%%%%%%%%%%%
%% Testing Actions
%%%%%%%%%%%%%%%%%%%
% Make sure the outcome of any concrete action is tested
occurs(test(R, loc(R, C), true), I+1) :- occurs(move(R, C), I).
occurs(test(R, in_hand(R, O), true), I+1) :- occurs(pickup(R, O), I).
occurs(test(R, in_hand(R, O), false), I+1) :- occurs(put_down(R, O), I).
-occurs(pickup(rob1, OP), I) :- holds(loc(rob1, C), I), not occurs(test(rob1, loc(OP, C), true), I-1).
-occurs(pickup(rob1, OP), I) :- I = 0.

%%%%%%%%%%%%%%%%%%%
%% Awareness axiom.
%%%%%%%%%%%%%%%%%%%
holds(F, 0) | -holds(F, 0) :- #physical_inertial_fluent(F).

%%%%%%%%%%%%%%%%%%%
%% Inertia Axioms.
%%%%%%%%%%%%%%%%%%%
holds(F,I+1) :- #inertial_fluent(F), holds(F,I), not -holds(F,I+1).
-holds(F,I+1) :- #inertial_fluent(F), -holds(F,I), not holds(F,I+1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CWA for Actions and Defined Fluents.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
-occurs(A,I) :- not occurs(A,I).
-holds(F,I) :- #defined_fluent(F), not holds(F,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% History and initial state rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Take what actually happened into account.
occurs(A,I) :- hpd(A,I).

%% Reality check axioms.
:- obs(F, true, I), -holds(F, I).
:- obs(F, false, I), holds(F, I).


%%%%%%%%%%%%%%%%%%%%
%% Planning Module
%%%%%%%%%%%%%%%%%%%%
%% Failure is not an option.
success :- goal(I).
:- not success.
%% Plan Actions minimally
occurs(A,I):+ not goal(I).
%% Preventing preASP_refined_domain_no_planning
something_happened(I) :- occurs(A, I).
:- not goal(I), not something_happened(I).
:- not something_happened(0).

%%%%%%%%%%%%%%%
%% Attributes:
%%%%%%%%%%%%%%%
next_to(c1, c2).
next_to(c2, c3).
next_to(c3, c4).
next_to(c5, c6).
next_to(c6, c7).
next_to(c7, c8).
next_to(c9, c10).
next_to(c10, c11).
next_to(c11, c12).
next_to(c13, c14).
next_to(c14, c15).
next_to(c15, c16).

next_to(c1, c5).
next_to(c2, c6).
next_to(c3, c7).
next_to(c4, c8).
next_to(c9, c5).
next_to(c10, c6).
next_to(c11, c7).
next_to(c12, c8).
next_to(c9, c13).
next_to(c10, c14).
next_to(c11, c15).
next_to(c12, c16).

next_to(c16, c29).


next_to(c17, c18).
next_to(c18, c19).
next_to(c19, c20).
next_to(c21, c22).
next_to(c22, c23).
next_to(c23, c24).
next_to(c25, c26).
next_to(c26, c27).
next_to(c27, c28).
next_to(c29, c30).
next_to(c30, c31).
next_to(c31, c32).

next_to(c21, c17).
next_to(c22, c18).
next_to(c23, c19).
next_to(c24, c20).
next_to(c21, c25).
next_to(c22, c26).
next_to(c23, c27).
next_to(c24, c28).
next_to(c25, c29).
next_to(c26, c30).
next_to(c27, c31).
next_to(c28, c32).



















comp(c1, library).
comp(c2, library).
comp(c3, library).
comp(c4, library).
comp(c5, library).
comp(c6, library).
comp(c7, library).
comp(c8, library).
comp(c9, library).
comp(c10, library).
comp(c11, library).
comp(c12, library).
comp(c13, library).
comp(c14, library).
comp(c15, library).
comp(c16, library).
comp(c17, kitchen).
comp(c18, kitchen).
comp(c19, kitchen).
comp(c20, kitchen).
comp(c21, kitchen).
comp(c22, kitchen).
comp(c23, kitchen).
comp(c24, kitchen).
comp(c25, kitchen).
comp(c26, kitchen).
comp(c27, kitchen).
comp(c28, kitchen).
comp(c29, kitchen).
comp(c30, kitchen).
comp(c31, kitchen).
comp(c32, kitchen).






comp(ref1_markerPen2, markerPen2).

%%%%%%%%%
%% Goal:
%%%%%%%%%
goal(I) :- holds(coarse_loc(rob1,kitchen),I), holds(coarse_loc(markerPen2,kitchen),I), holds(coarse_in_hand(rob1,markerPen2),I).

%%%%%%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%%%%%%
holds(loc(rob1,c6),0).
holds(in_hand(rob1,ref1_markerPen2),0).
holds(coarse_loc(rob1,library),0).
holds(coarse_loc(markerPen2,library),0).
holds(coarse_in_hand(rob1,markerPen2),0).

%%%%%%%%%
display
%%%%%%%%%

occurs.