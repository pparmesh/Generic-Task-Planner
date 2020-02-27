// Class declaration file for Symbolic Planner
// Author : Prateek Parmeshwar
// Copyright 2019, Prateek Parmeshwar, All rights reserved

#pragma once

using namespace std;

class SymbolicPlanner
{
    private:
        vector<GroundedAction> m_GA_vector;
        vector<GroundedAction> m_pruned_GA_vector;
        Env* m_env_object; 
        struct graph_node
        {
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state_config;
            int parent_action = -10;
            double f = DBL_MAX, g = DBL_MAX, h = DBL_MAX; // Cells will have a total cost, heuristic cost and cost to go
            string parent_state = "";
        };
        typedef pair<double, string> f_COORDINATE;

    public:
        SymbolicPlanner(Env* env_object); // make constructor

        // Function to hash state. Return string of sorted GCs, which will be unique
        string hash_state(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state);

        // Function to create combinations
        void create_combinations(int offset, int k, vector<vector<string>>& result, const vector<string> vec_main, vector<string>& combination);

        // Function to get combinations
        vector<vector<string>> get_combinations(const vector<string> vec_main, int offset, int k);

        // Function to generate permutations
        vector<vector<string>> permute(vector<string> str);

        // This function generates a map of total number of possible actions
        void precompute();

        // Function to speed up planner by pruning action space
        void get_pruned_grounded_actions();

        // Function to check validty of action
        bool is_action_valid(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state,
        GroundedAction action);

        // Function to get new state
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_new_state(GroundedAction action,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state);

        // Function to get inadmissible state
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_new_relaxed_state(GroundedAction action,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state);

        // Checks if small state is a subset of big state
        bool is_subset_of(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> small_state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> big_state);

        // Function to get inadmissible heuristic val
        double get_inadmis_heuristic_val(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state);

        // Function to get admissible heuristic val
        double get_admis_heuristic_val(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state);

        // Function to get admissible relaxed heuristic val
        double get_relaxed_heuristic_val(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>parent_state,
        GroundedAction action, unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state);

        // Function to get solution path
        vector<GroundedAction> solutionPath(string goal_hash,unordered_map<string, graph_node> graph_map);

        // A* Search
        vector<GroundedAction> A_star(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state, int type_heuristic, bool verbos);

};

