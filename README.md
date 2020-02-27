# Generic-Task-Planner
This repository contains an implementation of a Task Planner that follows the STRIPS representation

*Academic Integrity*
If you are currently enrolled in the Graduate 16-782 Planning and Decision Making in Robotics, or the Undergraduate 16-350 Planning Techniques for Robotics course at Carnegie Mellon University, please refer to CMUs Academic Integrity Policy before referring to any of the contents of this repository.

The task at hand is to develop a general planner that outputs a sequence of actions that make my agent reach the
goal conditions from the start conditions. Actions are predefined and at every state that my agent is in, it has to
check which actions can it take in order to reach the next state. This is essentially a graph search problem now,
however, computing valid actions based on the agent's current state is a very expensive operation and in order to
achieve performance requirements with respect to speed, the agent needs to quickly know what its valid actions for
a given state are.
To tackle this, a precompute step is run before running my graph search where for every general action, all permu-
tations of arguments (symbols) are fed into this general action to get all possible Grounded Actions (note that a lot
of these actions will not be valid in any setting). Now that all possible Grounded Actions have been computed, it is
easy for my agent to quickly iterate over them and and valid actions.
However, as stated earlier, a lot of these Grounded Actions are invalid, and this collection of Grounded Actions
can be pruned. For a given environment, there are some Grounded Conditions in the initial conditions that never
change regardless of what action is applied, i.e., some literals never appear in the effects of any actions such as
Block(x) or NotTable(x) etc. Once these literals have been identified it is then checked whether Grounded Condi-
tions in the set of preconditions for every Grounded action corresponding to these literals are present in my initial
conditions. If they are not, this Grounded Action will never be valid and hence can be removed. This decreases the
number of possible Grounded Actions for my agent and hence increases the performance of the planner.
After this, the graph search is performed using the A* algorithm (Dijkstra's algorithm when there is no heuris-
tic). The algorithm is complete and will return a solution if there is one and returns no solution found if there isn't
a solution. The Symbolic Planner itself is completely domain independent as all computation is performed on the
environment file that is parsed.

1) No Heuristic: When no heuristic is selected, the graph search is just Dijkstra's and all states are expanded uni-
formly. The agent has no estimate of the goal. The number of states expanded in this case is large

2) Inadmissible Heuristic: A heuristic is said to be inadmissible if it gives an overestimate of the path from the
current state to the goal. Inadmissible heuristic can result in sub-optimality as the agent my traverse paths with low
heuristic values even though the path itself might have been high cost. The inadmissible heuristic used in this case is
the number of Grounded Conditions in the goal that are not being satisfied by the state at hand. This is inadmissible
as, for example, suppose 3 grounded conditions are not being satisfied by the current state and the heuristic will give
a value of 3 but it may be possible that all those grounded conditions are satisfied by performing just one action.
However, as this heuristic gives some idea of the goal, the planner planning time decreases substantially and less
states are expanded.

3) Admissible Heurist (relaxed search): The idea here is to only add positive effects of a given Grounded Action
and then return the number of actions it will take for my agent to reach the goal from this new state generated by
adding only positive grounded conditions. This heuristic is admissible as it is an underestimate of the number of
actions it will take for the agent to reach the goal as my agent will definitely need to perform more actions than what the heuristic says as only a portion of the effects of the action at hand is being applied. The number of states
expanded in this case reduces dramatically but the planning time increases. This is expected as for every new state
that is expanded, the planner has to perform a separate search to return the number of actions to go.

To Compile : 
g++ -std=c++11  SymbolicPlanner.cpp main.cpp -o planner
.\planner.exe filename.txt heuristic_val

