:- [f12_3_Astar].
:- [f13_10_RTA].
:- ['set_print_options.pl'].
:- dynamic cell/3, grid_size/3, s/3. % NEW

/******************************************************************************/
% Four fixed example calls.

t1(Path) :- run(astar, h_e, 6, 1-1, 6-6, Path).
t2(Path) :- run(astar, h_m, 6, 1-1, 6-6, Path).
t3(Path) :- run(rta,   h_e, 6, 1-1, 6-6, Path).
t4(Path) :- run(rta,   h_m, 6, 1-1, 6-6, Path).

/*******************************************************************************
The run predicate is the start of all test cases.

Example call for general case.
  run( astar,    % The functor for the predicate to call
       h_e,      % The functor for the heuristic function to be used
       Grid_size % Number of rows & columns for square grid
       Start,    % The start state for the robot
       Goal,     % The goal state
       Path      % The returned search path
       ).
*/

run(Algorithm, H_functor, Grid_size, Start, Goal, Path) :-
    statistics(walltime, [TimeSinceStart | [TimeSinceLastCall]]), % NEW
	 ( retract(h_functor(_)) ; true ),  % MODIFIED: repositioned
	 asserta(h_functor(H_functor)),

	 ( retract(goal(_)) ; true ), % New repositioned
    asserta(goal(Goal)), ! ,

    ( retract(grid_size(_)),
        clearFacts,              % NEW, originally retractall(s(_,_,_))
        assertz(grid_size(Grid_size)),
        createEdges(Grid_size), !
    ;
      assertz(grid_size(Grid_size)) , createEdges(Grid_size)
    ) ,

    Predicate =.. [Algorithm, Start, Path],
    call(Predicate), !,
	 statistics(walltime, [NewTimeSinceStart | [ExecutionTime]]), % NEW
	 write('The search took '), write(ExecutionTime), write(' ms.'), nl. % NEW


/******************************************************************************
New predicates
*******************************************************************************/

% createEdges(Size)
% Size = size of a square grid
% Uses rowloop (which uses columnloop) predicate to create the cells
% Takes each cell that is not the goal cell and create an edge using linkAdjacent predicate
% Also creates the calculates and creates the heuristic for each cell
createEdges(Size) :- rowloop(Size,Size,Size),
                     forall((cell(R-C,D), \+ goal(R-C)), linkAdjacent(R-C, Size)),
							forall(cell(R-C,D), createHeuristic(R-C)).

% rowloop(Row, Col, Size)
% base case is when Row = 0
% starts using columnloop predicate once it backs out of the base case
rowloop(0,_,_).
rowloop(Row,Col,Size) :- Row > 0, Row1 is Row-1, rowloop(Row1, Col, Size), columnloop(Row,Col,Size).

% columnloop(Row,Col,Size)
% base case is when Col = 0
% starts calculating the current Row-Col cell's Difficulty/Cost and asserts cells
% once it backs out of the base case
% cells are created from left to right of current Row
columnloop(_,0,_).
columnloop(Row,Col,Size) :- Col > 0, Col1 is Col-1, columnloop(Row, Col1, Size),
                            findDiff(Row-Col, Diff),
									 assertz(cell(Row-Col, Diff)).

% findDiff(R-C, Diff)
% R = row, C = column
% Diff = returns the difficulty of the cell R-C
findDiff(R-C, Diff) :- Diff is (((R**3) + (C**3)) mod 17).

% linkAdjacent(R-C,Size)
% R = Row, C = column, Size = size of grid
% main predicate for creating edges s(source, destination, cost)
% helper for linking cell to adjacent nodes if the cell is not the goal
% uses all variations of findAdjacent(R-C,Size) predicate to link cell to
% adjacent below, right, left, and above cells
linkAdjacent(R-C, Size) :- forall(findAdjacent(R-C, Size),true).

% link to cell below
findAdjacent(R-C, Size) :- R1 is R + 1, R1 =< Size, cell(R1-C, Cost),
                           assertz(s(R-C, R1-C, Cost)).

% link to cell on the right
findAdjacent(R-C, Size) :- C1 is C + 1, C1 =< Size, cell(R-C1, Cost),
                           assertz(s(R-C, R-C1, Cost)).

% link to cell on the left
findAdjacent(R-C, Size) :- C1 is C - 1, C1 > 0, cell(R-C1, Cost),
                           assertz(s(R-C, R-C1, Cost)).

% link to cell above
findAdjacent(R-C, Size) :- R1 is R - 1, R1 > 0, cell(R1-C, Cost),
                           assertz(s(R-C, R1-C, Cost)).

% h_e(X1-Y1, X2-Y2, D)
% X1 = Col1, Y1 = Row1, X2 = Col2, Y2 = Row2, D = euclidean distance from goal
% euclidean distance = squareroot( (X1-X2)^2 + (Y1-Y2)^2 )
% returns a float number
h_e(X1-Y1,X2-Y2,D) :- D is sqrt( ((X1-X2)**2) + ((Y1-Y2)**2) ). % debug , write(D), nl.

% h_m(X1-Y1, X2-Y2, D)
% X1 = Col1, Y1 = Row1, X2 = Col2, Y2 = Row2, D = manhattan distance from goal
% manhattan distance = |X1-X2| + |Y1-Y2|
% much easier distance to work with because it always returns an integer Number
% mahattan distance is the distance from the goal based on the sum of the
% change in columns and rows
h_m(X1-Y1,X2-Y2,D) :- D is ( abs(X1-X2) + abs(Y1-Y2) ). % debug, write(D), nl.

% createHeuristic(R-C)
% finds the h_functor's name from the knowledge base
% finds the Row and Col value of the goal
% create the heuristic function and asserts it to the knowledge base
% call the heuristic function and assert the heuristic facts h(Row-Col, HeuristicDist)
createHeuristic(R-C) :- h_functor(H_functor),
                        goal(R1-C1),
                        P=..[H_functor, R-C, R1-C1, Hdist],
										call(P), !,
										assertz(h(R-C,Hdist)).

% Debug: predicate for printing all cells
printCells :- forall(cell(R,C), (write(R), write(" ("), write(C), write(")"), nl)).

% Debug: predicate for printing all edges
printEdges :- forall(s(C1,C2,D), (write(C1), write(" to "), write(C2), write(" = ("), write(D), write(")"), nl)).

% Debug: predicate for printing heuristics
printH :- forall(h(Cell,Hdist), (write("h("), write(Cell), write(", "), write(Hdist), write(")"), nl)).

% Debug: predicate to count all possible transition/edges
printCount :- countnumbers(X), write(X), nl.
countnumbers(X) :- findall(X, s(_,_,_), Ns), length(Ns, X).

% Debug: predicate for printing all of the facts
printFacts :- write("Goal = "), goal(G), write(G), nl,
              write("===== Nodes ====="), nl, printCells,
              write("===== Edges ====="), nl, printEdges,
				  write("===== Heuristics ====="), nl, printH.

% Debug: predicate for clearing all the facts in the database
clearFacts :- retractall(s(_,_,_)), retractall(cell(_,_)), retractall(h(_,_)).

/****************************************************************
    TEST CASES
****************************************************************/

/**Difficulty Function Tests**/
:- begin_tests(difftests).
test(diff11, [nondet]) :- findDiff(1-1, Diff), Diff = 2.
test(diff52, [nondet]) :- findDiff(5-2, Diff), Diff = 14.
test(diff56, [nondet]) :- findDiff(5-6, Diff), Diff = 1.
test(diff85, [nondet]) :- findDiff(8-5, Diff), Diff = 8.
test(diff76, [nondet]) :- findDiff(7-6, Diff), Diff = 15.
test(diff27, [nondet]) :- findDiff(2-7, Diff), Diff = 11.
:- end_tests(difftests).

/**Difficulty/Cost Function Test**/
:- begin_tests(he_tests).
test(he_11_66, [nondet]) :- h_e(1-1, 6-6, D), Delta is D - 7.071068, Delta < 0.0000005.
test(he_11_33, [nondet]) :- h_e(1-1, 3-3, D), Delta is D - 2.828427, Delta < 0.0000005.
test(he_33_66, [nondet]) :- h_e(3-3, 6-6, D), Delta is D - 4.242641, Delta < 0.0000005.
test(he_45_88, [nondet]) :- h_e(4-5, 8-8, D), Delta is D - 5.000000, Delta < 0.0000005.
test(he_33_99, [nondet]) :- h_e(3-3, 9-9, D), Delta is D - 8.485281, Delta < 0.0000005.
test(he_31_92), [nondet]) :- h_e(3-1, 9-2, D), Delta is D - 6.082763, Delta < 0.0000005.

test(hm_11_66, [nondet]) :- h_m(1-1, 6-6, D), D = 10.
test(hm_11_33, [nondet]) :- h_m(1-1, 3-3, D), D = 4.
test(hm_33_66, [nondet]) :- h_m(3-3, 6-6, D), D = 6.
test(hm_45_88, [nondet]) :- h_m(4-5, 8-8, D), D = 7.
test(hm_33_99, [nondet]) :- h_m(3-3, 9-9, D), D = 12.
test(hm_31_92), [nondet]) :- h_m(3-1, 9-2, D), D = 7.
:- end_tests(he_tests).

/* A* algorithm *********************************************/
:- begin_tests(astartests).
% astar using euclidean distance
test(start_astar_he, [nondet]) :- nl, nl, write("----A* using Euclidean Distance: Grid 5 to 9----"), nl.
test(astar_5by5_he, [nondet]) :- run(astar, h_e, 5, 1-1, 5-5, Path), Path = [1-1,2-1,3-1,3-2,3-3,4-3,4-4,4-5,5-5].
test(astar_6by6_he, [nondet]) :- run(astar, h_e, 6, 1-1, 6-6, Path), Path = [1-1,1-2,1-3,2-3,3-3,4-3,4-4,5-4,6-4,6-5,6-6].
test(astar_7by7_he, [nondet]) :- run(astar, h_e, 7, 1-1, 7-7, Path), Path = [1-1,2-1,3-1,3-2,3-3,4-3,4-4,5-4,6-4,6-5,6-6,6-7,7-7].
test(astar_8by8_he, [nondet]) :- run(astar, h_e, 8, 1-1, 8-8, Path), Path = [1-1,2-1,3-1,3-2,3-3,3-4,4-4,4-5,4-6,5-6,6-6,7-6,7-7,7-8,8-8].
test(astar_9by9_he, [nondet]) :- run(astar, h_e, 9, 1-1, 9-9, Path), Path = [1-1,2-1,3-1,3-2,3-3,4-3,4-4,5-4,6-4,6-5,7-5,8-5,9-5,9-6,9-7,9-8,9-9].

% astar using mahattan distance
test(start_astar_hm, [nondet]) :- nl, nl, write("----A* using Manhattan Distance: Grid 5 to 9----"), nl.
test(astar_5by5_hm, [nondet]) :- run(astar, h_m, 5, 1-1, 5-5, Path), Path = [1-1,2-1,3-1,3-2,3-3,3-4,4-4,4-5,5-5].
test(astar_6by6_hm, [nondet]) :- run(astar, h_m, 6, 1-1, 6-6, Path), Path = [1-1,2-1,3-1,3-2,3-3,4-3,4-4,4-5,4-6,5-6,6-6].
test(astar_7by7_hm, [nondet]) :- run(astar, h_m, 7, 1-1, 7-7, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-4,4-4,5-4,6-4,6-5,6-6,6-7,7-7].
test(astar_8by8_hm, [nondet]) :- run(astar, h_m, 8, 1-1, 8-8, Path), Path = [1-1,2-1,3-1,3-2,3-3,3-4,4-4,4-5,4-6,5-6,6-6,7-6,7-7,7-8,8-8].
test(astar_9by9_hm, [nondet]) :- run(astar, h_m, 9, 1-1, 9-9, Path), Path = [1-1,2-1,3-1,3-2,3-3,3-4,4-4,4-5,4-6,5-6,5-7,5-8,5-9,6-9,7-9,8-9,9-9].
:- end_tests(astartests).


/* RTA* algorithm *******************************************/
:- begin_tests(rtatests).
% rta using euclidean distance
test(start_rta_he, [nondet]) :- nl, nl, write("----RTA* using Euclidean Distance: Grid 5 to 9----"), nl.
test(rta_5by5_he, [nondet]) :- run(rta, h_e, 5, 1-1, 5-5, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,5-5].
test(rta_6by6_he, [nondet]) :- run(rta, h_e, 6, 1-1, 6-6, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,5-6,6-6].
test(rta_7by7_he, [nondet]) :- run(rta, h_e, 7, 1-1, 7-7, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,
                                                                         5-6,6-6,6-5,6-4,6-3,6-2,7-2,7-1,6-1,5-1,5-2,
																								 5-3,5-4,5-5,5-6,4-6,3-6,2-6,2-7,1-7,1-6,1-5,
																								 2-5,2-4,2-3,3-3,3-2,3-1,2-1,1-1,1-2,1-3,2-3,
																								 2-4,3-4,4-4,4-5,5-5,6-5,7-5,7-6,7-7].
test(rta_8by8_he, [nondet]) :- run(rta, h_e, 8, 1-1, 8-8, Path),  Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,
                                                                         5-6,6-6,6-5,6-4,6-3,6-2,7-2,7-1,8-1,8-2,8-3,
																								 8-4,8-5,7-5,7-6,7-7,7-8,8-8].
test(rta_9by9_he, [nondet]) :- run(rta, h_e, 9, 1-1, 9-9, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,
                                                                         5-6,6-6,6-5,6-4,6-3,6-2,7-2,7-1,8-1,8-2,9-2,
																								 9-3,9-4,9-5,9-6,9-7,9-8,8-8,8-9,7-9,7-8,8-8,
																								 8-9,9-9].

% rta using mahattan distance
test(start_astar_hm, [nondet]) :- nl, nl, write("----RTA* using Manhattan Distance: Grid 5 to 9----"), nl.
test(rta_5by5_hm, [nondet]) :- run(rta, h_m, 5, 1-1, 5-5, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,5-5].
test(rta_6by6_hm, [nondet]) :- run(rta, h_m, 6, 1-1, 6-6, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,5-6,6-6].
test(rta_7by7_hm, [nondet]) :- run(rta, h_m, 7, 1-1, 7-7, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,5-6,6-6,
                                                                         6-5,6-4,5-4,5-5,5-6,4-6,3-6,2-6,2-7,1-7,1-6,1-5,2-5,
																								 2-4,2-3,3-3,3-2,3-3,2-3,2-4,3-4,4-4,4-5,5-5,6-5,6-4,
																								 6-3,6-2,7-2,7-1,6-1,5-1,5-2,4-2,3-2,3-1,2-1,1-1,1-2,
																								 1-3,2-3,3-3,4-3,4-2,3-2,3-3,3-4,2-4,2-5,2-6,3-6,3-7,
																								 4-7,5-7,6-7,7-7].
test(rta_8by8_hm, [nondet]) :- run(rta, h_m, 8, 1-1, 8-8, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,5-6,6-6,
                                                                         6-5,6-4,5-4,5-5,5-6,5-7,5-8,6-8,7-8,8-8].
test(rta_9by9_hm, [nondet]) :- run(rta, h_m, 9, 1-1, 9-9, Path), Path = [1-1,1-2,1-3,2-3,3-3,3-2,4-2,4-3,4-4,4-5,4-6,5-6,6-6,
                                                                         6-5,6-4,5-4,5-5,5-6,5-7,5-8,5-9,6-9,7-9,8-9,8-8,9-8,
																								 9-7,8-7,8-8,8-9,7-9,8-9,9-9].
:- end_tests(rtatests).
