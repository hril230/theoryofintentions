#const numSteps = 1. % maximum number of steps.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#step = 0..numSteps.
#coarse_place = {library,kitchen,office1,office2,storage_room}.
#coarse_object = {book1,book2,book3,book4,cup1,cup2,cup3,cup4, pen1, pen2, pen3, pen4,noteBook1,noteBook2,noteBook3,noteBook4,plate1,plate2,plate3,plate4, markerPen1, markerPen2, markerPen3, markerPen4}.
#object = {ref1_book1,ref2_book1,ref1_book2,ref2_book2, ref1_book3,ref2_book3, ref1_book4,ref2_book4, ref1_cup1,ref2_cup1,ref1_cup2,ref2_cup2,ref1_cup3,ref2_cup3, ref1_cup4,ref2_cup4, ref1_pen1, ref1_pen2,ref1_pen3, ref1_pen4, ref1_noteBook1,ref2_noteBook1,ref1_noteBook2,ref2_noteBook2, ref1_noteBook3,ref2_noteBook3, ref1_noteBook4,ref2_noteBook4, ref1_plate1,ref2_plate1,ref1_plate2,ref2_plate2,ref1_plate3,ref2_plate3, ref1_plate4,ref2_plate4, ref1_markerPen1, ref1_markerPen2,ref1_markerPen3, ref1_markerPen4}.
#place = {c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25,c26,c27,c28,c29,c30,c31,c32,c33,c34,c35,c36,c37,c38,c39,c40,c41,c42,c42,c43,c44,c45,c46,c47,c48,c49,c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62, c63, c64, c65, c66, c67, c68, c69, c70, c71, c72, c73, c74, c75,c76,c77,c78,c79,c80}.

#robot = {rob1}.
#coarse_thing = #coarse_object + #robot.
#thing = #object + #robot.
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
#rob_action = test(#robot, #physical_inertial_fluent, #boolean) + move(#robot, #place) + pickup(#robot, #object) + put_down(#robot, #object).
#exo_action = exo_move(#object,#place).
#action = #rob_action + #exo_action.
#refined_component = #place + #object.
#coarse_component = #coarse_place + #coarse_object.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
comp(#refined_component, #coarse_component).
holds(#fluent, #step).

%% NEXT LINES NOT RELEVANT TO INFERRING
next_to(#place, #place).
coarse_next_to(#coarse_place, #coarse_place).
occurs(#action, #step).
obs(#fluent, #boolean, #step).
hpd(#action, #step).
success().
goal(#step).
something_happened(#step).
%% NEXT LINES ARE RELEVANT TO INFERRING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NEXT LINES NOT RELEVANT TO INFERRING

%%%%%%%%%%%%%%%%%
%% Causal Laws %%
%%%%%%%%%%%%%%%%%
-holds(in_hand(R,OP2),I+1) :- occurs(put_down(rob1,OP1),I), comp(OP1,B), comp(OP2,B), holds(coarse_in_hand(rob1,B),I).
% Moving changes location to target room (if the door is not locked).
holds(loc(R, C), I+1) :- occurs(move(R, C), I).

% Grasping an object causes object to be in hand.
holds(in_hand(R, OP), I+1) :- occurs(pickup(R, OP), I).

% Putting an object down causes it to no longer be in hand.
-holds(in_hand(R, OP), I+1) :- occurs(put_down(R, OP), I).

%% exogenous moving an object causes the object to be in a different location.
holds(loc(O,L),I+1) :- occurs(exo_move(O,L),I).

%%%%%%%%%%%%%%%%%%%%%%%
%% State Constraints %%
%%%%%%%%%%%%%%%%%%%%%%%
%% STATE CONSTRAINTS GO HERE
% Reflexive property of next_to relation.
next_to(C1, C2) :- next_to(C2, C1), #place(C1), #place(C2).
coarse_next_to(Z1, Z2) :- next_to(C1, C2), comp(C1, Z1), comp(C2, Z2), Z1!=Z2, #place(C1), #place(C2).

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Executability Conditions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cannot move to a location if you are already there.
-occurs(move(R, C), I) :- holds(loc(R, C), I).

% Cannot move to a location if it is not next to it.
-occurs(move(R, C2), I) :- holds(loc(R, C1), I), -next_to(C1,C2).

% Cannot put down an object unless it is in hand.
-occurs(put_down(R,OP),I) :- comp(OP,B), -holds(coarse_in_hand(rob1,B),I).

% Cannot pick up an object if it has something in hand.
-occurs(pickup(R, O), I) :- holds(in_hand(R, CO), I).

% Cannot pick up an object if you are not in the same room.
-occurs(pickup(R, OP), I) :- holds(loc(R, C), I), not holds(loc(OP, C), I).

% Cannot execute two actions at the same time.
:- occurs(A1,I), occurs(A2,I), A1 != A2, #rob_action(A1), #rob_action(A2).

%%%%%%%%%%%%%%%%%%%
%% CWA for Actions.
%%%%%%%%%%%%%%%%%%%
-occurs(A,I) :- not occurs(A,I).

%%%%%%%%%%%%%%%%%%%%%%%%
%% Axioms for testing %%
%%%%%%%%%%%%%%%%%%%%%%%%
holds(can_be_tested(rob1, loc(T, C), V), I) :- holds(loc(rob1, C), I).
holds(can_be_tested(rob1, in_hand(rob1, OP), V), I).
holds(directly_observed(rob1, F, true), I+1) :- holds(F, I), occurs(test(rob1, F, true), I).
holds(directly_observed(rob1, F, false), I+1) :- -holds(F, I), occurs(test(rob1, F, false), I).
-occurs(test(rob1, F, O), I) :- -holds(can_be_tested(rob1, F, O), I).

%%%%%%%%%%%%%%%%%%%
%% Testing Actions
%%%%%%%%%%%%%%%%%%%
%% TESTING RULES GO HERE

%%%%%%%%%%%%%%%%%%%
%% Awareness axiom.
%%%%%%%%%%%%%%%%%%%
holds(F, 0) | -holds(F, 0) :- #physical_inertial_fluent(F).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% History and initial state rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Take what actually happened into account.
occurs(A,I) :- hpd(A,I).

%% Reality check axioms.
:- obs(F, true, I), -holds(F, I).
:- obs(F, false, I), holds(F, I).

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

next_to(c32, c45).



next_to(c33, c34).
next_to(c34, c35).
next_to(c35, c36).
next_to(c37, c38).
next_to(c38, c39).
next_to(c39, c40).
next_to(c41, c42).
next_to(c42, c43).
next_to(c43, c44).
next_to(c45, c46).
next_to(c46, c47).
next_to(c47, c48).

next_to(c33, c37).
next_to(c34, c38).
next_to(c35, c39).
next_to(c36, c40).
next_to(c37, c41).
next_to(c38, c42).
next_to(c39, c43).
next_to(c40, c44).
next_to(c45, c41).
next_to(c46, c42).
next_to(c47, c43).
next_to(c48, c44).


next_to(c48, c61).


next_to(c49, c50).
next_to(c50, c51).
next_to(c51, c52).
next_to(c53, c54).
next_to(c54, c55).
next_to(c55, c56).
next_to(c57, c58).
next_to(c58, c59).
next_to(c59, c60).
next_to(c61, c62).
next_to(c62, c63).
next_to(c63, c64).

next_to(c49, c53).
next_to(c50, c54).
next_to(c51, c55).
next_to(c52, c56).
next_to(c57, c53).
next_to(c58, c54).
next_to(c59, c55).
next_to(c60, c56).
next_to(c57, c61).
next_to(c58, c62).
next_to(c59, c63).
next_to(c60, c64).

next_to(c64, c77).


next_to(c65, c66).
next_to(c66, c67).
next_to(c67, c68).
next_to(c69, c70).
next_to(c70, c71).
next_to(c71, c72).
next_to(c73, c74).
next_to(c74, c75).
next_to(c75, c76).
next_to(c77, c78).
next_to(c78, c79).
next_to(c79, c80).

next_to(c65, c69).
next_to(c66, c70).
next_to(c67, c71).
next_to(c68, c72).
next_to(c73, c69).
next_to(c74, c70).
next_to(c75, c71).
next_to(c76, c72).
next_to(c73, c77).
next_to(c74, c78).
next_to(c75, c79).
next_to(c76, c80).
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
comp(c33, office1).
comp(c34, office1).
comp(c35, office1).
comp(c36, office1).
comp(c37, office1).
comp(c38, office1).
comp(c39, office1).
comp(c40, office1).
comp(c41, office1).
comp(c42, office1).
comp(c43, office1).
comp(c44, office1).
comp(c45, office1).
comp(c46, office1).
comp(c47, office1).
comp(c48, office1).
comp(c49, office2).
comp(c50, office2).
comp(c51, office2).
comp(c52, office2).
comp(c53, office2).
comp(c54, office2).
comp(c55, office2).
comp(c56, office2).
comp(c57, office2).
comp(c58, office2).
comp(c59, office2).
comp(c60, office2).
comp(c61, office2).
comp(c62, office2).
comp(c63, office2).
comp(c64, office2).
comp(c65, storage_room).
comp(c66, storage_room).
comp(c67, storage_room).
comp(c68, storage_room).
comp(c69, storage_room).
comp(c70, storage_room).
comp(c71, storage_room).
comp(c72, storage_room).
comp(c73, storage_room).
comp(c74, storage_room).
comp(c75, storage_room).
comp(c76, storage_room).
comp(c77, storage_room).
comp(c78, storage_room).
comp(c79, storage_room).
comp(c80, storage_room).

comp(ref1_book1, book1).
comp(ref2_book1, book1).
comp(ref1_book2, book2).
comp(ref2_book2, book2).
comp(ref1_book3, book3).
comp(ref2_book3, book3).
comp(ref1_book4, book4).
comp(ref2_book4, book4).

comp(ref1_noteBook1, noteBook1).
comp(ref2_noteBook1, noteBook1).
comp(ref1_noteBook2, noteBook2).
comp(ref2_noteBook2, noteBook2).
comp(ref1_noteBook3, noteBook3).
comp(ref2_noteBook3, noteBook3).
comp(ref1_noteBook4, noteBook4).
comp(ref2_noteBook4, noteBook4).

comp(ref1_cup1, cup1).
comp(ref2_cup1, cup1).
comp(ref1_cup2, cup2).
comp(ref2_cup2, cup2).
comp(ref1_cup3, cup3).
comp(ref2_cup3, cup3).
comp(ref1_cup4, cup4).
comp(ref2_cup4, cup4).

comp(ref1_plate1, plate1).
comp(ref2_plate1, plate1).
comp(ref1_plate2, plate2).
comp(ref2_plate2, plate2).
comp(ref1_plate3, plate3).
comp(ref2_plate3, plate3).
comp(ref1_plate4, plate4).
comp(ref2_plate4, plate4).

comp(ref1_pen1, pen1).
comp(ref1_pen2, pen2).
comp(ref1_pen3, pen3).
comp(ref1_pen4, pen4).

comp(ref1_markerPen1, markerPen1).
comp(ref1_markerPen2, markerPen2).
comp(ref1_markerPen3, markerPen3).
comp(ref1_markerPen4, markerPen4).

%%%%%%%%%
%% Goal:
%%%%%%%%%
%% GOAL GOES HERE

%%%%%%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%%%%%%
%% HISTORY GOES HERE
-holds(coarse_in_hand(rob1,noteBook4),0).
holds(loc(ref2_book2,c66),0).
-holds(coarse_in_hand(rob1,pen2),0).
holds(loc(ref2_plate3,c69),0).
-holds(coarse_in_hand(rob1,plate4),0).
holds(loc(ref2_cup2,c77),0).
holds(loc(ref1_pen1,c27),0).
-holds(coarse_in_hand(rob1,book2),0).
holds(loc(ref2_book1,c8),0).
holds(loc(ref1_book2,c66),0).
holds(loc(ref1_markerPen2,c69),0).
holds(loc(ref1_plate1,c27),0).
holds(loc(ref2_plate2,c54),0).
holds(loc(ref1_noteBook3,c15),0).
holds(loc(ref1_plate3,c69),0).
-holds(coarse_in_hand(rob1,pen4),0).
holds(loc(ref2_noteBook3,c15),0).
-holds(coarse_in_hand(rob1,cup1),0).
holds(loc(ref2_cup1,c13),0).
holds(loc(ref2_noteBook1,c44),0).
-holds(coarse_in_hand(rob1,book3),0).
-holds(coarse_in_hand(rob1,cup4),0).
holds(loc(rob1,c57),0).
holds(loc(ref1_book3,c1),0).
holds(loc(ref2_book3,c1),0).
-holds(coarse_in_hand(rob1,pen1),0).
-holds(coarse_in_hand(rob1,noteBook3),0).
-holds(coarse_in_hand(rob1,markerPen3),0).
-holds(coarse_in_hand(rob1,markerPen4),0).
holds(loc(ref1_pen4,c33),0).
holds(loc(ref1_pen3,c27),0).
-holds(coarse_in_hand(rob1,plate2),0).
holds(loc(ref1_markerPen4,c27),0).
holds(loc(ref1_noteBook1,c44),0).
holds(loc(ref2_cup4,c45),0).
-holds(coarse_in_hand(rob1,book4),0).
holds(loc(ref1_plate4,c29),0).
holds(loc(ref2_noteBook2,c12),0).
-holds(coarse_in_hand(rob1,noteBook2),0).
holds(loc(ref1_cup1,c13),0).
holds(loc(ref1_book4,c12),0).
holds(loc(ref2_cup3,c43),0).
-holds(coarse_in_hand(rob1,book1),0).
holds(loc(ref1_cup2,c77),0).
holds(loc(ref1_noteBook2,c12),0).
holds(loc(ref1_pen2,c17),0).
holds(loc(ref1_cup4,c45),0).
holds(loc(ref2_plate4,c29),0).
-holds(coarse_in_hand(rob1,noteBook1),0).
-holds(coarse_in_hand(rob1,cup2),0).
-holds(coarse_in_hand(rob1,cup3),0).
holds(loc(ref2_book4,c12),0).
-holds(coarse_in_hand(rob1,pen3),0).
holds(loc(ref2_noteBook4,c16),0).
-holds(coarse_in_hand(rob1,markerPen2),0).
holds(loc(ref1_cup3,c43),0).
holds(loc(ref1_markerPen3,c76),0).
-holds(coarse_in_hand(rob1,plate3),0).
holds(loc(ref1_plate2,c54),0).
holds(loc(ref1_noteBook4,c16),0).
holds(loc(ref1_markerPen1,c3),0).
holds(loc(ref2_plate1,c27),0).
-holds(coarse_in_hand(rob1,markerPen1),0).
holds(loc(ref1_book1,c8),0).
-holds(coarse_in_hand(rob1,plate1),0).

%%%%%%%%%
display
%%%%%%%%%

occurs.
holds(loc(A,B),numSteps).
holds(in_hand(A,B),numSteps).
holds(coarse_loc(A,B),numSteps).
holds(coarse_in_hand(A,B),numSteps).
