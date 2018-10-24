#const numSteps = 10.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#coarse_place = {kitchen,office1}.
#coarse_object = {book4}.
#object = { ref1_book4,ref2_book4,ref3_book4,ref4_book4,ref1_book2,ref2_book2,ref3_book2,ref4_book2}.
#place = {c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c23,c24,c25,c177,c288}.
#robot = {rob1}.
#coarse_thing = #coarse_object + #robot.
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

% Defined fluents do not hold unless specified.
-holds(coarse_in_hand(rob1, O), I) :- not holds(coarse_in_hand(rob1, O), I).
-holds(in_hand(rob1,O),I) :- not holds(in_hand(rob1,O),I).
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
-occurs(pickup(R, OP1), I) :- holds(in_hand(R, OP2), I).

% Cannot pick up an object if you are not in the same room.
-occurs(pickup(R, OP), I) :- holds(loc(R, C), I), not holds(loc(OP, C), I).

% Cannot execute two actions at the same time.
:- occurs(A1,I), occurs(A2,I), A1 != A2, #action(A1), #action(A2).











%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Axioms for observing the environment %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(can_be_tested(rob1, loc(T, C), V), I) :- holds(loc(rob1, C), I).
holds(can_be_tested(rob1, in_hand(rob1, OP), V), I).
holds(directly_observed(rob1, F, true), I+1) :- holds(F, I), occurs(test(rob1, F, true), I).
holds(directly_observed(rob1, F, false), I+1) :- -holds(F, I), occurs(test(rob1, F, false), I).
-occurs(test(rob1, F, O), I) :- -holds(can_be_tested(rob1, F, O), I).
holds(indirectly_observed(rob1, coarse_loc(T, R), true), I) :- holds(directly_observed(rob1, loc(T, C), true), I), comp(C, R), holds(loc(T,C),I).
holds(indirectly_observed(rob1, coarse_loc(T, R), true), I) :- holds(directly_observed(rob1, loc(Z, C), true), I), comp(C,R), comp(Z,T), holds(loc(Z,C),I).
holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I) :- holds(directly_observed(rob1, in_hand(rob1, OP), true), I), comp(OP, O), holds(in_hand(rob1, OP), I).
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
%% TESTING RULES GO HERE

%%%%%%%%%%%%%%%%%%%
%% Inertia Axioms.
%%%%%%%%%%%%%%%%%%%
holds(F,I+1) :- #inertial_fluent(F), holds(F,I), not -holds(F,I+1).
-holds(F,I+1) :- #inertial_fluent(F), -holds(F,I), not holds(F,I+1).

%%%%%%%%%%%%%%%%%%%%
%% CWA for Actions.
%%%%%%%%%%%%%%%%%%%%
-occurs(A,I) :- not occurs(A,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% History and initial state rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Take what actually happened into account.
occurs(A,I) :- hpd(A,I).

%% Reality check axioms.
:- obs(F, true, I), -holds(F, I).
:- obs(F, false, I), holds(F, I).

%% Awareness axiom.
holds(F, 0) | -holds(F, 0) :- #physical_inertial_fluent(F).


%%%%%%%%%%%%%%%%%%%%
%% Planning Module
%%%%%%%%%%%%%%%%%%%%
%% PLANNING RULES GO HERE


%%%%%%%%%%%%%%%
%% Attributes.
%%%%%%%%%%%%%%%
next_to(c6, c7).
next_to(c7, c8).
next_to(c8, c9).
next_to(c9, c10).
next_to(c10, c11).
next_to(c11, c12).
next_to(c12, c13).
next_to(c13, c14).
next_to(c14, c15).

comp(c6, kitchen).
comp(c7, kitchen).
comp(c8, kitchen).
comp(c9, kitchen).
comp(c10, kitchen).
comp(c11, office1).
comp(c12, office1).
comp(c13, office1).
comp(c14, office1).
comp(c15, office1).

comp(ref1_book4, book4).
comp(ref2_book4, book4).
comp(ref3_book4, book4).
comp(ref4_book4, book4).

%%%%%%%%%
%% Goal:
%%%%%%%%%
%% GOAL GOES HERE

%%%%%%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%%%%%%
%% HISTORY GOES HERE
hpd(move(rob1,c7),0).
holds(loc(rob1,c6),0).
hpd(test(rob1,loc(rob1,c7),true),1).
hpd(move(rob1,c8),2).
obs(loc(ref1_book4,c7),false,2).
obs(loc(rob1,c7),true,2).
hpd(test(rob1,loc(rob1,c8),true),3).
hpd(move(rob1,c9),4).
obs(loc(ref1_book4,c8),false,4).
obs(loc(rob1,c8),true,4).
hpd(test(rob1,loc(rob1,c9),true),5).
obs(loc(ref1_book4,c9),false,6).
hpd(move(rob1,c10),6).
obs(loc(rob1,c9),true,6).
hpd(test(rob1,loc(rob1,c10),true),7).
obs(loc(rob1,c10),true,8).
hpd(move(rob1,c11),8).
obs(loc(ref1_book4,c10),false,8).
hpd(test(rob1,loc(rob1,c11),true),9).
obs(loc(ref1_book4,c11),false,10).
obs(loc(rob1,c11),true,10).
holds(directly_observed(rob1,loc(rob1,c7),true),2).
holds(directly_observed(rob1,loc(ref1_book4,c7),false),2).
holds(directly_observed(rob1,loc(ref1_book4,c8),false),4).
holds(directly_observed(rob1,loc(rob1,c8),true),4).
holds(directly_observed(rob1,loc(ref1_book4,c9),false),6).
holds(directly_observed(rob1,loc(rob1,c9),true),6).
holds(directly_observed(rob1,loc(rob1,c10),true),8).
holds(directly_observed(rob1,loc(ref1_book4,c10),false),8).
holds(directly_observed(rob1,loc(ref1_book4,c11),false),10).
holds(directly_observed(rob1,loc(rob1,c11),true),10).

%%%%%%%%%%%%%%%%%
%% End of History:
%%%%%%%%%%%%%%%%%

%%%%%%%%%
display
%%%%%%%%%

holds(indirectly_observed(rob1,B,C),numSteps).
