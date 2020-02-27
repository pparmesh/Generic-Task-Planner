// Class definition for Symbolic Planner
// Author : Prateek Parmeshwar
// Copyright 2019, Prateek Parmeshwar, All rights reserved

#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <float.h>
#include <cfloat>
#include <chrono>
#include<ctime>

#include "GroundedCondition.hpp"
#include "Condition.hpp"
#include "GroundedAction.hpp"
#include "Action.hpp"
#include "Env.hpp"
#include "SymbolicPlanner.hpp"

using namespace std;

SymbolicPlanner::SymbolicPlanner(Env* env_object)
        {
            m_env_object = env_object;
        }

string SymbolicPlanner::hash_state(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state)
        {
            vector<string> current_state_str;
            for (GroundedCondition gpc : current_state)
            {
                string gc_str = gpc.toString();
                current_state_str.push_back(gc_str);
            }
            sort(current_state_str.begin(),current_state_str.end());
            string s;
            for (vector<string>::const_iterator i = current_state_str.begin(); i != current_state_str.end(); ++i)
                s += *i;
            return s;
        }

void SymbolicPlanner::create_combinations(int offset, int k, vector<vector<string>>& result, const vector<string> vec_main, vector<string>& combination) 
{
    if (k == 0) {
        result.push_back(combination);
        return;
    }
    for (int i = offset; i <= vec_main.size() - k; ++i) {
        combination.push_back(vec_main[i]);
        create_combinations(i+1, k-1, result, vec_main, combination);
        combination.pop_back();
    }
}

vector<vector<string>> SymbolicPlanner::get_combinations(const vector<string> vec_main, int offset, int k) 
{
    vector<vector<string>> new_vec;
    vector<string> combination;
    create_combinations(offset, k, new_vec, vec_main, combination);
    return new_vec;  
}

vector<vector<string>> SymbolicPlanner::permute(vector<string> str) 
{ 
    vector<vector<string>> ans;
    // Sort the string in lexicographically 
    // ascennding order 
    sort(str.begin(), str.end()); 

    // Keep printing next permutation while there 
    // is next permutation 
    do { 
        vector<string> tmp;
        for (int i = 0; i < str.size(); i++)
        {
            tmp.push_back(str[i]);
        }
        ans.push_back(tmp);
    } while (next_permutation(str.begin(), str.end())); 

    return ans;
}


void SymbolicPlanner::precompute()
{
    // This function generates a map of total number of possible actions
    unordered_set<string> env_symbols = m_env_object->get_symbols(); // get all the symbols in the environment
    vector<string> symbols_vector; // All symbols in environment are stored in vector. -->CBO
    auto it_1 = env_symbols.begin();  
    while (it_1 != env_symbols.end())
    {
        symbols_vector.push_back(*it_1);
        it_1++;
    }

    unordered_set<Action, ActionHasher, ActionComparator> total_actions = m_env_object->get_actions(); // get all actions
    for (auto temp_action : total_actions)
    {
    // Iterate over all actions. For each action get all possible Grounded Actions
        int number_args = temp_action.get_args_number();
        vector<vector<string>> symbol_combinations =  get_combinations(symbols_vector,0,number_args);
        for (int j=0; j<symbol_combinations.size(); j++)
        {
            vector<vector<string>> symbol_permutations = permute(symbol_combinations[j]);
            for (int k=0; k<symbol_permutations.size(); k++)
            {
                // Create GA. To create GA, we need name, args, GCs and GEFs
                string action_name = temp_action.get_name(); // Get action name
                list<string> action_symbols(symbol_permutations[k].begin(), symbol_permutations[k].end()); // Get symbols list
                list<string> action_args = temp_action.get_args();

                // Create mapping of general args to Grounded 
                list<string>::iterator it_2 = action_args.begin();
                list<string>::iterator it_3 = action_symbols.begin();
                // Iterate over both lists to make a mapping from args to symbols
                unordered_map<string, string> args_to_symbols;
                for(; it_2 != action_args.end() && it_3 != action_symbols.end(); ++it_2, ++it_3)
                {
                    args_to_symbols[*it_2] = *it_3;
                }

                // Iterate over preconditions to get grounded preconditions
                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
                unordered_set<Condition, ConditionHasher, ConditionComparator> action_preconditions = temp_action.get_preconditions();
                auto it_4 = action_preconditions.begin();
                while (it_4!=action_preconditions.end())
                {
                    string cond_predicate = it_4->get_predicate();// name of condition
                    list<string> cond_args = it_4->get_args(); // args
                    list<string> cond_arg_values;
                    for (auto it_5=cond_args.begin(); it_5!=cond_args.end(); it_5++)
                    {
                        if (args_to_symbols[*it_5]=="")
                            cond_arg_values.push_back(*it_5);
                        else
                            cond_arg_values.push_back(args_to_symbols[*it_5]); 
                    }
                    bool cond_truth = it_4->get_truth();
                    GroundedCondition temp_GC(cond_predicate,cond_arg_values,cond_truth);
                    gPreconditions.insert(temp_GC);
                    it_4++;
                }

                // Iterate over effects to get grounded effects
                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;
                unordered_set<Condition, ConditionHasher, ConditionComparator> action_effects = temp_action.get_effects();
                auto it_6 = action_effects.begin();
                while (it_6!=action_effects.end())
                {
                    string cond_predicate_eff = it_6->get_predicate();// name of condition
                    list<string> cond_args_eff = it_6->get_args(); // args
                    list<string> cond_arg_eff_values;
                    for (auto it_7=cond_args_eff.begin(); it_7!=cond_args_eff.end(); it_7++)
                    {
                        if (args_to_symbols[*it_7]=="")
                            cond_arg_eff_values.push_back(*it_7);
                        else
                            cond_arg_eff_values.push_back(args_to_symbols[*it_7]); 
                    }
                    bool cond_truth_eff = it_6->get_truth();
                    GroundedCondition temp_GC_eff(cond_predicate_eff,cond_arg_eff_values,cond_truth_eff);
                    gEffects.insert(temp_GC_eff);
                    it_6++;
                }

                // Create a Grounded Action
                GroundedAction GA_temp(action_name,action_symbols,gPreconditions,gEffects);
                // Store Grounded Action
                m_GA_vector.push_back(GA_temp);
            }
        }
    }
}


void SymbolicPlanner::get_pruned_grounded_actions()
{
    unordered_set<Action, ActionHasher, ActionComparator> total_actions = m_env_object->get_actions(); // get all actions
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_state = m_env_object->get_initial_condition();
    unordered_set<string> all_effects_predicates;
    unordered_set<string> permanent_conds;

    for (auto curr_act : total_actions)
    {
        unordered_set<Condition, ConditionHasher, ConditionComparator> act_effects = curr_act.get_effects();
        for (auto pc : act_effects)
        {
            string eff_predicate = pc.get_predicate();
            all_effects_predicates.insert(eff_predicate);
        }
    }
    // Iterate over initial conditions to see if the predicates are in all effects
    for (auto init_pc : initial_state)
    {
        string ground_predicate = init_pc.get_predicate();
        if(all_effects_predicates.find(ground_predicate)==all_effects_predicates.end())
            permanent_conds.insert(ground_predicate);
    }

    // Iterate over grounded actions and remove invalid ones
    for (int i=0; i<m_GA_vector.size(); i++)
    {
        bool flag = true;
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> all_gcs = m_GA_vector[i].get_preconditions();
        for (auto gc: all_gcs)
        {
            string predicate_gc = gc.get_predicate();
            if (permanent_conds.find(predicate_gc)!=permanent_conds.end())
            {
                if (initial_state.find(gc)==initial_state.end())
                {
                    flag = false;
                    break;
                }
            }
        }
        if (flag)
            m_pruned_GA_vector.push_back(m_GA_vector[i]);
        
    }
}


bool SymbolicPlanner::is_action_valid(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state,
        GroundedAction action) 
{
    // Iterate over action preconditions and if one of them is not in current_state break and return falso
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> preconds = action.get_preconditions();
    for (auto pc : preconds)
    {
        if (current_state.find(pc)==current_state.end())
            return false;
    }
    return true;
}


 unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> SymbolicPlanner::get_new_state(GroundedAction action,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state)
{
    // Iterate over effects 
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> action_effects = action.get_effects();
    for (auto effect : action_effects)
    {
        if (effect.get_truth())
            current_state.insert(effect);
        else
        {
            effect.set_truth(true);
            current_state.erase(effect);
        }
    }
    return current_state;
}

 unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> SymbolicPlanner::get_new_relaxed_state(GroundedAction action,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state)
{
    // Iterate over effects 
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> action_effects = action.get_effects();
    for (auto effect : action_effects)
    {
        if (effect.get_truth())
            current_state.insert(effect);
    }
    return current_state;
}       

bool SymbolicPlanner::is_subset_of(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> small_state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> big_state)
{
    // Checks if small state is a subset of big state
    // Iterate over GC in small state
    for (auto gc : small_state)
    {
        if(big_state.find(gc)==big_state.end())
            return false;
    }
    return true;
}

double SymbolicPlanner::get_inadmis_heuristic_val(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state)
{
    double h_val(0);
    for (auto gc : goal_state)
    {
        if (new_state.find(gc)==new_state.end())
            h_val++;
    }
    return h_val;
}

double SymbolicPlanner::get_admis_heuristic_val(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_state,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state)
        {
            vector<GroundedAction> actions_to_take = A_star(new_state,goal_state,1,false);
            return actions_to_take.size();
        }

double SymbolicPlanner::get_relaxed_heuristic_val(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>parent_state,
GroundedAction action, unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state)
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_relaxed_state = get_new_relaxed_state(action,parent_state);
    vector<GroundedAction> actions_to_take = A_star(new_relaxed_state,goal_state,1,false);
    return actions_to_take.size();
    
}

vector<GroundedAction> SymbolicPlanner::solutionPath(string goal_hash,unordered_map<string, graph_node> graph_map)
{
    vector<GroundedAction> solution;
    string curr_state = goal_hash;
    int backtrack_action = graph_map[curr_state].parent_action;

    while (backtrack_action>=0)
    {
        curr_state = graph_map[curr_state].parent_state;
        solution.push_back(m_pruned_GA_vector[backtrack_action]);
        backtrack_action = graph_map[curr_state].parent_action;
    }
    reverse(solution.begin(),solution.end());
    return solution;

}
        
vector<GroundedAction> SymbolicPlanner::A_star(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_state,
unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_state, int type_heuristic, bool verbos)
{
    int expanded_states(0);
    vector<GroundedAction> search_result;
    if (is_subset_of(goal_state,initial_state)) // Check to see if already at goal
        return search_result;

    // Initialize the open list 
    unordered_map<string, graph_node> graph_map;
    // Initialize the closed list to keep track of expanded nodes
    unordered_map <string, bool> closed_list;
    string init_state_hash = hash_state(initial_state);
    string goal_state_hash = hash_state(goal_state);

    // Inititalize the start cell. It has no parents and its f, g & h values are 0
    graph_map[init_state_hash].f = 0;
    graph_map[init_state_hash].g = 0;
    graph_map[init_state_hash].h = 0;
    graph_map[init_state_hash].state_config = initial_state;
    graph_map[init_state_hash].parent_action = -1; // start cell as its parent as itself
    graph_map[init_state_hash].parent_state = "None";

    string temp_goal_hash = "VOID";


    // Implement the open list to keep track of states to expand using a set
    // It is a set of f_COORINATE, i.e it has location of state and its f value
    set<f_COORDINATE> open_list; 
//     // Add my start cell to my open list
    open_list.insert(make_pair (0.0, init_state_hash)); 


//     // Expand till open list is not empty
    while(!open_list.empty() && closed_list[goal_state_hash]!=true) //change
    {   
        // Pick index with lowest f value from open list. Set will help in doing this as it is ordered.
        //Put this in closed list.
        // Find valid actions and new states of my current index and find f values only if they are not in closed list.
        // If they are not in closed list, find their f-values. If they are in the open list with a larger
        // f-value then update it, otherwise add this index to the open list. 
        // Loop till goal state has not been expanded.
        // Get index from openlist. Pop the first value from the open list.
        f_COORDINATE q = *open_list.begin();
        // Remove it from the open list
        open_list.erase(open_list.begin());
        // Get index of this node
        string q_current = q.second;
        closed_list[q_current] = true;
        expanded_states++; 

        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> current_state = graph_map[q_current].state_config;
        if (is_subset_of(goal_state, current_state))
        {
            closed_list[goal_state_hash] = true;
            temp_goal_hash = q_current;
        }

        // Loop through neighbors
        vector<int> valid_grounded_actions;
        for (int i = 0; i < m_pruned_GA_vector.size(); i++)
        {
            if (is_action_valid(current_state, m_pruned_GA_vector[i]))
                valid_grounded_actions.push_back(i);
        }

        // Iterate over valid actions
        for (int j=0; j<valid_grounded_actions.size(); j++)
        {
            double fNew, gNew, hNew; // Variables used to find f, g & h values
            // Get new state from action
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_state = get_new_state(m_pruned_GA_vector[valid_grounded_actions[j]],current_state);
            string new_state_hash = hash_state(new_state);
            // Only proceed if it is not in closed list
            if (closed_list[new_state_hash] != true)
            {
                // Compute fNew, gNew, hNew.
                gNew = graph_map[q_current].g + 1;
                if (type_heuristic==0)
                    hNew = 0;
                if (type_heuristic==1)
                    hNew = get_inadmis_heuristic_val(new_state,goal_state);
                if (type_heuristic==2)
                    hNew = get_admis_heuristic_val(new_state,goal_state);
                if (type_heuristic==3)
                    hNew = get_relaxed_heuristic_val(current_state,m_pruned_GA_vector[valid_grounded_actions[j]],goal_state);

                fNew = gNew + hNew;
                if (graph_map[new_state_hash].f == DBL_MAX || graph_map[new_state_hash].f > fNew)
                {
                    open_list.insert(make_pair (fNew, new_state_hash));
                    graph_map[new_state_hash].f = fNew;
                    graph_map[new_state_hash].g = gNew;
                    graph_map[new_state_hash].h = hNew;
                    graph_map[new_state_hash].state_config = new_state;
                    graph_map[new_state_hash].parent_action = valid_grounded_actions[j];
                    graph_map[new_state_hash].parent_state = q_current;
                }
            }
        }    
    }
    if (closed_list[goal_state_hash])
    {
        if (verbos)
        {
            cout<<"Solution found!"<<'\n'<<endl;
            cout<<"Number of states expanded: "<<expanded_states<<endl;
        }
        search_result = solutionPath(temp_goal_hash,graph_map);
        return search_result;
    }
    else
    {
        cout<<"No solution found"<<'\n'<<endl;;
        return search_result;
    }
}



