#const numSteps = 1.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This ASP is used to update the abstract belief of the ControllerToI.
%% It does not have planning module.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#room = {library, kitchen, office1, office2}.
#book = {book1, book2, book3}.
#robot = {rob1}.
#object = #book.
#thing = #object + #robot.

#boolean = {true, false}.
#step = 0..numSteps.

#inertial_fluent = loc(#thing, #room) + in_hand(#robot, #object).
#fluent = #inertial_fluent.
#rob_action = move(#robot, #room) + pickup(#robot, #object)
	+ put_down(#robot, #object).
#exo_action = exo_move(#object,#room).
#action = #rob_action + #exo_action.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
next_to(#room,#room).
holds(#fluent,#step).
occurs(#action,#step).
obs(#fluent, #boolean, #step).

hpd(#action, #step).
diag(#exo_action, #step).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
%% Causal Laws %%
%%%%%%%%%%%%%%%%%
%% Moving changes location to target room (if the door is not locked).
holds(loc(R, L), I+1) :- occurs(move(R, L), I).

%% Grasping an object causes object to be in hand.
holds(in_hand(R, O), I+1) :- occurs(pickup(R, O), I).

%% Putting an object down causes it to no longer be in hand.
-holds(in_hand(R, O), I+1) :- occurs(put_down(R, O), I).

%% Exogenous moving an object causes the object to be in a different location.
holds(loc(O,L),I+1) :- occurs(exo_move(O,L),I).


%%%%%%%%%%%%%%%%%%%%%%%
%% State Constraints %%
%%%%%%%%%%%%%%%%%%%%%%%
%% Reflexive property of next_to relation.
next_to(L1,L2) :- next_to(L2,L1).

%% Any object exists in only one location.
-holds(loc(T, L2), I) :- holds(loc(T, L1), I), L1!=L2.

%% If a robot is holding an object, they have the same location.
holds(loc(O, L), I) :- holds(loc(R, L), I), holds(in_hand(R, O), I).

%% Only one object can be held at any time.
-holds(in_hand(R, O2), I) :- holds(in_hand(R, O1), I), O1 != O2.

%% a room is not next to another unless specified.
-next_to(L1,L2) :- not next_to(L1,L2).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Executability Conditions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Cannot move to a location if you are already there.
-occurs(move(R, L), I) :- holds(loc(R, L), I).

%% Cannot move to a location if it is not next to it.
-occurs(move(R, L2), I) :- holds(loc(R, L1), I), -next_to(L1,L2).

%% Cannot put down an object unless it is in hand.
-occurs(put_down(R, O), I) :-  -holds(in_hand(R, O), I).

%% Cannot pick up an object if it has something in hand.
-occurs(pickup(R, O1), I) :- holds(in_hand(R, O2), I).

%% Cannot pick up an object if you are not in the same room.
-occurs(pickup(R, O), I) :- holds(loc(R, L), I), not holds(loc(O, L), I).

%% An exogenous move of an object cannot be done to the same location.
-occurs(exo_move(O,L),I) :- holds(loc(O,L),I).

%% An exogenous move of an object cannot happen if it is being in hand
-occurs(exo_move(O,L),I) :- holds(in_hand(R,O),I).


%%%%%%%%%%%%%%
%% Defaults %%
%%%%%%%%%%%%%%
%holds(loc(O,library),0) :- #book(O), not -holds(loc(O,library),0).
%holds(loc(O,office1),0) :- #book(O), -holds(loc(O,library),0), not -holds(loc(O,office1),0).


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inertial axiom + CWA %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% General inertia axioms.
holds(F,I+1) :- #inertial_fluent(F),
                holds(F,I),
                not -holds(F,I+1).

-holds(F,I+1) :- #inertial_fluent(F),
                 -holds(F,I),
                 not holds(F,I+1).

%% CWA for Actions.
-occurs(A,I) :- not occurs(A,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% History and initial state rules %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Take what actually happened into account.
occurs(A,I) :- hpd(A,I).


%% Reality check axioms.
:- obs(F, true, I), -holds(F, I).
:- obs(F, false, I), holds(F, I).

%% Awareness axiom.
holds(F, 0) | -holds(F, 0) :- #inertial_fluent(F).


%%%%%%%%%%%%%%
%% Diagnosis
%%%%%%%%%%%%%%
occurs(A,I) :+ #exo_action(A), I<numSteps.
diag(A,I) :- occurs(A,I),
		not hpd(A,I),
		#exo_action(A).

%%%%%%%%%%%%%%%%
%% Attributes.
%%%%%%%%%%%%%%%%
next_to(library, kitchen).
next_to(kitchen, office1).
next_to(office1, office2).

%%%%%%%%%%%%
%% History:
%%%%%%%%%%%%
holds(loc(book1,kitchen),0).
-holds(in_hand(rob1,book1),0).
holds(loc(book2,kitchen),0).
holds(loc(book3,kitchen),0).
-holds(in_hand(rob1,book3),0).
holds(loc(rob1,kitchen),0).
-holds(in_hand(rob1,book2),0).
hpd(pickup(rob1,book2), 0).
obs(in_hand(rob1,book2),true,1).
obs(loc(book2,kitchen),true,1).
obs(loc(rob1,kitchen),true,1).


%%%%%%%%%%
display
%%%%%%%%%%
diag(A,I).
holds(F,numSteps).
-holds(in_hand(rob1,B), numSteps).
