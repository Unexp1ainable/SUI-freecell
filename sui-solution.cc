#include <algorithm>
#include <iostream>
#include "memusage.h"
#include "search-strategies.h"
#include <stack>

/**
 * @brief Backtrack the path and return it
 *
 * @param howDidWeGetHere Mapping of parent states
 * @param actions Action to state mapping (index of action corresponds to the index of the state)
 * @param total End of vectors, since the
 * @return std::vector<SearchAction>
 */
std::vector<SearchAction> finalize(
    std::vector<int>& howDidWeGetHere,
    std::vector<SearchAction>& actions) {
    // ):
    std::vector<SearchAction> result{actions.back()};
    auto actionN = howDidWeGetHere.back();

    while (actionN != 0) {
        result.emplace_back(actions[actionN - 1]);
        actionN = howDidWeGetHere[actionN - 1];
    }

    std::reverse(result.begin(), result.end());

    return result;
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState& init_state) {
    if (init_state.isFinal()) {
        return {};
    }

    // pre-calculate maximum size of the vectors
    size_t size = (mem_limit_ - getCurrentRSS()) / (sizeof(SearchState) + sizeof(int) + sizeof(SearchAction) + 52 * sizeof(Card) + 52 * sizeof(Card*)) - 100;

    // pre-allocate space
    std::vector<SearchState> toBeSearched{};  // holds all states
    std::vector<int> howDidWeGetHere{};       // holds index of the parent state
    std::vector<SearchAction> actions{};      // holds all actions
    toBeSearched.reserve(size);
    howDidWeGetHere.reserve(size);
    actions.reserve(size);
    toBeSearched.emplace_back(init_state);

    size_t total = 0;

    // begin bfs
    for (size_t i = 0; i < toBeSearched.size(); i++) {
        auto currState = toBeSearched[i];

        for (const auto& action : currState.actions()) {
            if (total == size - 1) {
                return {};
            }

            actions.emplace_back(action);
            toBeSearched.emplace_back(action.execute(currState));
            howDidWeGetHere.emplace_back(i);

            if (toBeSearched.back().isFinal()) {
                toBeSearched.clear();
                return finalize(howDidWeGetHere, actions);
            }

            total++;
        }
    }

    // we should not ever reach this
    return {};
}

/*
std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    std::stack<std::vector<SearchState>> stack;
    stack.push(std::vector<SearchState>{init_state});

    std::vector<std::vector<SearchAction>> actions;
    

    int d = 0;
    while(!stack.empty()) {
        // processing current state
        std::vector<SearchState> curr_level = stack.top();
        
        for(auto curr_state : curr_level) {
            //slepa ulicka
            if(curr_state.actions().size()==0) {
                
            }
            actions.push_back(curr_state.actions());
            std::vector<SearchState> nextLevel;
            for(const auto& action : curr_state.actions()) {
                SearchState adj_state = action.execute(curr_state);
                nextLevel.push_back(adj_state);
            }
            stack.push(nextLevel);
        }

    }
    return {};


    
}*/


std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    if(init_state.isFinal()){
        return {};
    }

    std::vector<SearchState> stack{init_state};
    std::map<SearchState, bool> visited;
    std::map<SearchState, int> depths;
    std::map<SearchState, SearchAction> actions;
    std::map<SearchState, SearchState> prevState;
    visited[init_state] = false;
    depths[init_state] = 0;
    while(!stack.empty()){
        SearchState curr_state = stack.back();
        stack.pop_back();

        if(visited[curr_state]) {std::cout<<"already visited"<<std::endl;continue;};

        int curr_depth = depths[curr_state];
        if(curr_depth >= depth_limit_) {
            continue;
        }
        // path reconstruction
        if(curr_state.isFinal()){
            std::cout<<depths[curr_state]<<std::endl;
            std::cout<<curr_state<<std::endl;
            std::cout<<"======================================"<<std::endl;
            
            std::vector<SearchAction> path;
            for(int i = 0; i < curr_depth; i++){
                auto last_move = actions.find(curr_state)->second;
                std::cout<<last_move<<std::endl;
                path.emplace_back(last_move);
                
                auto prev = prevState.find(curr_state)->second;
                curr_state = std::move(prev);
                std::cout<<curr_state<<std::endl;
                
                std::cout<<"-------------------------"<<std::endl;
            }
            std::cout<<path.size()<<std::endl;
            std::reverse(path.begin(), path.end());
            return path;
        }



        std::cout<<curr_depth<<std::endl;
        std::cout<<curr_state<<std::endl;
        std::cout<<"----------------------------------------------"<<std::endl;
        visited[curr_state] = true;
        for(auto action : curr_state.actions()){
            SearchState adj_state = action.execute(curr_state);
            if(visited[adj_state]) continue;
            stack.emplace_back(adj_state);
            actions.insert({adj_state, action});
            prevState.insert({adj_state, curr_state});
            depths.insert({adj_state, curr_depth+1});
        }
        
        
    }
    return {};
}


double StudentHeuristic::distanceLowerBound(const GameState& state) const {
    state.all_storage.back();
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState& init_state) {
    init_state.actions();
    return {};
}
