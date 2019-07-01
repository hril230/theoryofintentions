#const n = 21. % maximum number of steps.
#const max_len = 20. % maximum activity_length of an activity.
#const max_name = 4.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#step = 0..n.
#integer = 0..n.

#secure_room = {library}.
#room = #secure_room + {kitchen, office1, office2}.
#robot = {rob1}.
#book = {book1}.
#object = #book.
#thing = #object + #robot.
#boolean = {true, false}.

#physical_inertial_fluent = loc(#thing, #room) + in_hand(#robot, #object) + locked(#secure_room).
#possible_goal = {my_goal}.
#physical_defined_fluent = #possible_goal.

#physical_fluent = #physical_inertial_fluent + #physical_defined_fluent.

#physical_agent_action = move(#robot,#room) + pickup(#robot,#object) + put_down(#robot,#object) + unlock(#robot,#secure_room).
#physical_exogenous_action = exo_move(#object, #room) + exo_lock(#secure_room).


#exogenous_action = #physical_exogenous_action.
#agent_action = #physical_agent_action.
#action = #agent_action + #exogenous_action.

#defined_fluent = #physical_defined_fluent.

#inertial_fluent = #physical_inertial_fluent.

#fluent = #inertial_fluent + #defined_fluent.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% static relation
next_to(#room,#room).

holds(#fluent,#step).
occurs(#action,#step).

%% used in history
obs(#fluent, #boolean, #step).
hpd(#action, #boolean, #step).
attempt(#action,#step).
impossible(#action, #step).
current_step(#step).
unobserved(#physical_exogenous_action, #step).

%% flags
finding_defaults(#step).
diagnosing(#step).


selected_goal_holds(#possible_goal,#step).
%% defaults:
ab_d1(#book). % default d1 is: books are in the office1.
ab_d2(#book). % default d2 is: cups are in office2.
ab_dL(#book). % default dL is: books are in the library.


defined_by_default(#inertial_fluent).

cost().

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

%% Awareness axiom.
holds(F, 0) | -holds(F, 0) :- #inertial_fluent(F).

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
-next_to(L1,L2) :- not next_to(L1,L2).

-holds(loc(T, L2), I) :- holds(loc(T, L1), I), L1!=L2.

holds(loc(O,L), I) :- holds(loc(R,L), I) , holds(in_hand(R,O),I).

-holds(in_hand(R, O2), I) :- holds(in_hand(R, O1), I), O1!=O2.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Physical Executability Condition %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
-occurs(A,I) :- impossible(A,I).
impossible(move(R,L),I) :- holds(loc(R,L),I).
impossible(move(R,L2),I) :- holds(loc(R,L1),I), -next_to(L1,L2).
impossible(move(R,L1),I) :- holds(locked(L1),I).
impossible(move(R,L2),I) :- holds(loc(R,L1),I), holds(locked(L1),I).
impossible(unlock(R,L), I) :- -holds(locked(L),I).
impossible(unlock(R,L1),I) :- holds(loc(R,L2),I), -next_to(L1,L2), L1 != L2.
impossible(put_down(R,O), I) :- -holds(in_hand(R,O), I).
impossible(pickup(R,O1), I) :- holds(in_hand(R,O2), I).
impossible(pickup(R,O), I) :- holds(loc(R,L1), I), holds(loc(O,L2), I) , L1 != L2.
impossible(exo_move(O,L),I) :- holds(loc(O,L),I).
%impossible(exo_move(O,L),I) :- holds(locked(L),I).
%impossible(exo_move(O,L2),I) :- holds(loc(O,L1),I), holds(locked(L1),I).
impossible(exo_move(O,L),I) :- holds(in_hand(R,L),I).
impossible(exo_lock(L),I) :- holds(locked(L),I).


%%%%%%%%%%%%%%%%%
%% History Rules
%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Diagnosys Generation %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (25)
occurs(PEA,I) :- unobserved(PEA,I).

%%(26)
unobserved(PEA,I1) :+ current_step(I),
		diagnosing(I),
		I1<I,
		not hpd(PEA,true,I1),
		#physical_exogenous_action(PEA).



%%%%%%%%%%%%%%%%%
%% Attributes:
%%%%%%%%%%%%%%%%%%%
next_to(office2,office1).
next_to(office1,kitchen).
next_to(kitchen, library).

%%%%%%%%%%%%
%% Defaults:
%%%%%%%%%%%%

holds(loc(B,library),0):- #book(B), not ab_d2(B), finding_defaults(I), current_step(I).
ab_d2(B) :+ #book(B), finding_defaults(I), current_step(I).
defined_by_default(loc(B,library)) :- #book(B), not ab_d2(B), finding_defaults(I), current_step(I).

holds(loc(B,office1),0):- #book(B), not ab_d1(B), finding_defaults(I), current_step(I).
ab_d1(B) :+ #book(B), finding_defaults(I), current_step(I).
defined_by_default(loc(B,office1)) :- #book(B), not ab_d1(B), finding_defaults(I), current_step(I).


ab_d2(B) :- not ab_d1(B), #book(B).

:- ab_d2(B), not ab_d1(B), #book(B), not cost.
cost :+ ab_d2(B), not ab_d1(B).

%% Initial choices of undetermined fluents.
holds(loc(B,library),0) | holds(loc(B,kitchen),0) | holds(loc(B,office1),0) | holds(loc(B,office2),0) :- #book(B).
%:- unobserved(E,I1), not cost, diagnosing(I).
%cost :+ unobserved(E,I1), diagnosing(I).


%%%%%%%%%
%% Goal:
%%%%%%%%%
%% GOAL GOES HERE
holds(my_goal,I) :- holds(loc(book1,library),I),  -holds(in_hand(rob1,book1),I) .

%%%%%%%%%%%%%%%%%
%% Current Step:
%%%%%%%%%%%%%%%%%
%% CURRENT STEP GOES HERE
current_step(1).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial State and history:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HISTORY GOES HERE
obs(locked(library),false,0).
obs(loc(rob1,office2),true,0).
obs(loc(book1,office2),false,0).
obs(in_hand(rob1,book1),false,0).
attempt(move(rob1,office1),0).

obs(loc(rob1,office1),true,1).
obs(loc(book1,office1),false,1).
obs(in_hand(rob1,book1),false,1).
attempt(move(rob1,kitchen),1).

%obs(loc(rob1,kitchen),true,2).
%obs(loc(book1,kitchen),true,2).
%obs(in_hand(rob1,book1),false,2).
%obs(locked(library),true,2).
%attempt(move(rob1,library),2).



diagnosing(1).
finding_defaults(1).
%%%%%%
display
%%%%%%
%defined_by_default.
ab_d1.
ab_d2.
unobserved.
holds(loc(book1,L),0).
