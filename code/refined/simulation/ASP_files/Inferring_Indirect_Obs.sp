#const numSteps = 52.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#step = 0..numSteps.
#coarse_place = {office1}.
#coarse_object = {book2}.
#object = {ref1_book2,ref2_book2}.
#place = {c33,c34,c35,c36,c37,c38,c39,c40,c41,c42,c43,c44,c45,c46,c47,c48}.
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
#refined_component = #object + #place.
#coarse_component = #coarse_object + #coarse_place.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
comp(#refined_component, #coarse_component).
holds(#fluent, #step).

%% NEXT LINES ARE RELEVANT TO INFERRING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NEXT LINES ARE RELEVANT TO INFERRING
%%%%%%%%%%%%%%%%%%%
%% Inertia Axioms.
%%%%%%%%%%%%%%%%%%%
holds(F,I+1) :- #inertial_fluent(F), holds(F,I), not -holds(F,I+1).
-holds(F,I+1) :- #inertial_fluent(F), -holds(F,I), not holds(F,I+1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CWA for Actions and Defined Fluents.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
-holds(F,I) :- #defined_fluent(F), not holds(F,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Coarse-resolution attributes and fine-resolution counterparts %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
holds(coarse_loc(T, Z), I) :- holds(loc(T, C), I), comp(C, Z).
holds(coarse_loc(B, Z),I) :- holds(loc(RB,C), I), comp(RB,B), comp(C,Z).
holds(coarse_in_hand(rob1, O), I) :- holds(in_hand(rob1, OP), I), comp(OP, O).
holds(loc(RB2, C) ,I) :- holds(loc(RB1, C), I), comp(RB1, B), comp(RB2,B).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Axioms for inferrnig observations %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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


%%%%%%%%%%%%%%%%%%%%
%% Planning Module
%%%%%%%%%%%%%%%%%%%%
%% PLANNING RULES GO HERE

%%%%%%%%%%%%%%%
%% Attributes:
%%%%%%%%%%%%%%%
comp(c44,office1).
comp(c45,office1).
comp(c46,office1).
comp(c47,office1).
comp(c40,office1).
comp(c41,office1).
comp(c42,office1).
comp(c43,office1).
comp(c48,office1).
comp(ref2_book2,book2).
comp(c39,office1).
comp(c38,office1).
comp(c35,office1).
comp(c34,office1).
comp(c37,office1).
comp(c36,office1).
comp(ref1_book2,book2).
comp(c33,office1).

%%%%%%%%%
%% Goal:
%%%%%%%%%
%% GOAL GOES HERE

%%%%%%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%%%%%%
holds(directly_observed(rob1,loc(ref1_book2,c37),false),47).
holds(directly_observed(rob1,loc(ref2_book2,c33),true),50).
holds(directly_observed(rob1,loc(ref2_book2,c37),false),47).
holds(directly_observed(rob1,loc(ref1_book2,c33),true),49).
holds(directly_observed(rob1,loc(ref2_book2,c33),true),52).
holds(directly_observed(rob1,loc(ref2_book2,c33),true),49).
holds(directly_observed(rob1,loc(rob1,c33),true),48).
holds(directly_observed(rob1,loc(ref1_book2,c33),true),50).
holds(directly_observed(rob1,in_hand(rob1,ref2_book2),true),51).
holds(directly_observed(rob1,loc(ref1_book2,c33),true),52).

%%%%%%%%%
display
%%%%%%%%%
holds(indirectly_observed(rob1,B,C),numSteps).