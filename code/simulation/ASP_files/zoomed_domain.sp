#const numSteps = 6. % maximum number of steps.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#robot = {rob1}.
#coarse_object = {book2}.
#object = {ref_book2}.
#coarse_thing = #coarse_object + #robot.
#thing = #object + #robot.

#step = 0..numSteps.
#boolean = {true, false}.
#outcome = {true, false, undet}.
#physical_inertial_fluent = in_hand(#robot,#object).
#physical_defined_fluent = coarse_in_hand(#robot,#coarse_object).
#physical_fluent = #physical_inertial_fluent + #physical_defined_fluent.
#knowledge_inertial_fluent = can_be_tested(#robot, #physical_inertial_fluent) + directly_observed(#robot, #physical_inertial_fluent, #outcome) + indirectly_observed(#robot, #physical_defined_fluent, #outcome).
#knowledge_defined_fluent = may_discover(#robot, #physical_defined_fluent) + observed(#robot, #physical_fluent).
#inertial_fluent = #physical_inertial_fluent + #knowledge_inertial_fluent.
#defined_fluent = #physical_defined_fluent + #knowledge_defined_fluent.
#fluent = #inertial_fluent + #defined_fluent.
#action = test(#robot,#physical_inertial_fluent,#outcome) + pickup(#robot,#object) + put_down(#robot,#object).

#refined_component = #object.
#coarse_component = #coarse_object.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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


% Grasping an object causes object to be in hand.
holds(in_hand(R, OP), I+1) :- occurs(pickup(R, OP), I).

% Putting an object down causes it to no longer be in hand.
-holds(in_hand(R, OP), I+1) :- occurs(put_down(R, OP), I).



%% State Constraints %%

% Reflexive property of next_to relation.



% Only one object can be held at any time.
-holds(in_hand(R, OP2), I) :- holds(in_hand(R, OP1), I), OP1!=OP2.

% Axioms relating coarse-resolution attributes and fine-resolution counterparts
holds(coarse_in_hand(rob1, O), I) :- holds(in_hand(rob1, OP), I), comp(OP, O).
-holds(coarse_in_hand(rob1, O), I) :- -holds(in_hand(rob1, OP), I), comp(OP, O), not holds(coarse_in_hand(rob1, O), I).



%% Executability Conditions %%



% Cannot put down an object unless it is in hand.
-occurs(put_down(R, OP), I) :-  -holds(in_hand(R, OP), I).

% Cannot pick up an object if it has something in hand.
-occurs(pickup(R, OP1), I) :- holds(in_hand(R, OP2), I).

% Cannot pick up an object if you are not in the same room.


-occurs(pickup(rob1, OP), I) :- I = 0.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Axioms for observing the environment %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(can_be_tested(rob1, in_hand(rob1, OP)), I).
holds(directly_observed(rob1, F, true), I+1) :- holds(F, I), occurs(test(rob1, F, true), I).
holds(directly_observed(rob1, F, false), I+1) :- -holds(F, I), occurs(test(rob1, F, false), I).
-occurs(test(rob1, F, O), I) :- -holds(can_be_tested(rob1, F, O), I).
holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I) :- holds(directly_observed(rob1, in_hand(rob1, OP), true), I), comp(OP, O).
holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), false), I) :- -holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I), -holds(may_discover(R, coarse_in_hand(rob1, O), true), I).
holds(may_discover(rob1, coarse_in_hand(rob1, O), true), I) :- -holds(indirectly_observed(rob1, coarse_in_hand(rob1, O), true), I), comp(OP, O), holds(directly_observed(rob1, in_hand(rob1, OP), undet), I).
holds(directly_observed(rob1, F, undet), I) :- not holds(directly_observed(rob1, F, true), I), not holds(directly_observed(rob1, F, false), I).
holds(observed(rob1, F, O), I) :- holds(directly_observed(rob1, F, O), I).
holds(observed(rob1, F, O), I) :- holds(indirectly_observed(rob1, F, O), I).

% Make sure the outcome of any concrete action is tested
occurs(test(rob1, F, true), I) :- -holds(F, I-1), holds(F, I), #physical_inertial_fluent(F).
occurs(test(rob1, F, false), I) :- holds(F, I-1), -holds(F, I), #physical_inertial_fluent(F), not -occurs(test(rob1, F, false), I).
-occurs(test(R, F, O), 0). % cannot test in the first step



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

%% Failure is not an option.
success :- goal(I).
:- not success.

%% Plan Actions All
%occurs(A, I)| -occurs(A,I).
%:- occurs(A, I), goal(I).

%% Plan Actions minimally
occurs(A,I):+ not goal(I).

%% Cannot execute two actions at the same time.
:- occurs(A1,I), occurs(A2,I), A1 != A2.

something_happened(I) :- occurs(A, I).
:- not goal(I), not something_happened(I).
:- not something_happened(0).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Attributes.



comp(ref_book2, book2).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%
%% Goal:
%%%%%%%%%
goal(I) :- -holds(coarse_in_hand(rob1,book2),I).


%%%%%%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%%%%%%
holds(in_hand(rob1,ref_book2), 0).

%%%%%%%%%%%%%%%%%
%% End of History:
%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
occurs.
