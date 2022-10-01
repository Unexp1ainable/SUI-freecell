#include <algorithm>
#include <iostream>
#include "memusage.h"
#include "search-strategies.h"
#include <stack>
#include <functional>
#include <sstream>
#include <optional>
#include <set>

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


bool isExplored(std::shared_ptr<const SearchState> state, std::set<std::shared_ptr<const SearchState>> discovered){
    return std::find_if(discovered.begin(), discovered.end(), 
            [&](std::shared_ptr<const SearchState> s) {
                    return *s == *state;
                }) != discovered.end();
}

void setExplored(std::shared_ptr<const SearchState> state, std::set<std::shared_ptr<const SearchState>>& discovered) {
    discovered.insert(state);
}


bool operator==(const SearchState& a, const SearchState& b) {
    return a.state_ == b.state_;
}


std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    if(init_state.isFinal()){
        return {};
    }
    // stack of pairs (ss, depth)
    std::stack<std::pair<std::shared_ptr<const SearchState>, int>> stack;
    stack.push({{std::make_shared<const SearchState>(init_state)}, 0});
    // closed set
    std::set<std::shared_ptr<const SearchState>> explored;
    // path traversed
    std::map<std::shared_ptr<const SearchState>, 
                std::pair<std::shared_ptr<const SearchAction>,std::shared_ptr<const SearchState>>> history;

    int curr_depth;
    std::shared_ptr<const SearchState> curr_p, adj_p;
    std::shared_ptr<const SearchAction> action_p;
    while(!stack.empty()){
        // pop top state from stack
        curr_p = stack.top().first;
        curr_depth = stack.top().second;
        stack.pop();

        // if surpassed depth limit then dont go further
        if(curr_depth >= depth_limit_) {
            continue;
        }

        // path reconstruction
        if(curr_p->isFinal()){
            std::vector<SearchAction> path;
            for(int i = 0; i < curr_depth; i++){
                auto prev = history.at(curr_p);
                curr_p = prev.second;
                path.push_back(*(prev.first));
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // set current state as already explored
        setExplored(curr_p, explored);
        // push adjacent states to stack
        for(auto action : curr_p->actions()){
            const SearchState adj_state = action.execute(*curr_p);
            
            adj_p = std::make_shared<const SearchState>(adj_state);
            action_p = std::make_shared<const SearchAction>(action);

            // if state has been already explored, dont push him to stack
            // comparison by GameState, not by identity
            if(isExplored(adj_p, explored)) {
                continue;
            }
            history.insert({adj_p, {action_p, curr_p}});
            stack.push({adj_p,curr_depth+1});
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
