#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <stack>
#include "memusage.h"
#include "search-strategies.h"

constexpr int CARD_COUNT = 52;
// constexpr int TOTAL_CARD_PTRS = (nb_stacks + nb_freecells + nb_homes + nb_stacks + nb_freecells);
constexpr int TOTAL_CARD_PTRS = 0;
constexpr int MEMORY_RESERVE = 100;

bool operator==(const SearchState& a, const SearchState& b) {
    return a.state_ == b.state_;
}

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

    // pre-calculate maximum size of the containers
    size_t size = (mem_limit_ - getCurrentRSS()) /
                      (sizeof(SearchState) +
                       sizeof(int) +
                       sizeof(SearchAction) +
                       sizeof(std::set<SearchState>::iterator) +
                       CARD_COUNT * sizeof(Card)) -
                  MEMORY_RESERVE;

    // pre-allocate space
    std::set<SearchState> expanded{init_state};
    std::vector<std::set<SearchState>::iterator> toBeSearched{};  // holds all states
    std::vector<int> howDidWeGetHere{};                           // holds index of the parent state
    std::vector<SearchAction> actions{};                          // holds all actions

    toBeSearched.reserve(size);
    howDidWeGetHere.reserve(size);
    actions.reserve(size);
    toBeSearched.emplace_back(expanded.begin());

    size_t total = 0;

    // begin bfs
    for (size_t i = 0; i < toBeSearched.size(); i++) {
        auto currState = *toBeSearched[i];

        for (const auto& action : currState.actions()) {
            if (total == size - 1) {
                return {};
            }

            auto next = expanded.emplace(action.execute(currState));
            if (next.second) {  // if element was not in the expanded set
                actions.emplace_back(action);
                toBeSearched.emplace_back(next.first);
                howDidWeGetHere.emplace_back(i);
                if (toBeSearched.back()->isFinal()) {
                    toBeSearched.clear();
                    expanded.clear();
                    return finalize(howDidWeGetHere, actions);
                }
            }

            total++;
        }
    }

    // we should not ever reach this
    return {};
}

using SearchAction_p = std::shared_ptr<const SearchAction>;
using SearchState_p = std::shared_ptr<const SearchState>;

inline bool isExplored(SearchState_p state, std::set<SearchState_p> discovered){
    return std::find_if(discovered.begin(), discovered.end(), 
            [&](SearchState_p s) {
                    return *s == *state;
                }) != discovered.end();
}

inline void setExplored(SearchState_p state, std::set<SearchState_p>& discovered) {
    discovered.insert(state);
}





size_t max_states_count(size_t mem_limit) {
    size_t search_state_s = sizeof(const SearchState) + 52*(sizeof(Card));
    size_t search_state_ptr_s = sizeof(SearchState_p); //OK
    size_t depth_s = sizeof(int); //OK
    size_t pair_s_d = sizeof(std::pair<SearchState_p, int>); //OK
    size_t stack_s = sizeof(std::stack<std::pair<SearchState_p, int>>); // OK
    size_t stack_elem_s = depth_s + search_state_ptr_s + pair_s_d;// OK

    size_t set_elem_s = search_state_ptr_s;// OK
    size_t set_s = sizeof(std::set<SearchState_p>);//OK

    size_t pair_s_as = sizeof(std::pair<SearchState_p, std::pair<SearchAction_p,SearchState_p>>);
    size_t pair_as = sizeof(std::pair<SearchAction_p,SearchState_p>);
    size_t search_action_ptr_s = sizeof(SearchAction_p);
    size_t map_elem_s = pair_s_as + 2*search_state_ptr_s + pair_as + search_action_ptr_s;
    size_t map_s = sizeof(std::map<SearchState_p, std::pair<SearchAction_p,SearchState_p>>);

    size_t num_states = (mem_limit - getCurrentRSS() - stack_s - set_s - map_s) / 
                (set_elem_s +stack_elem_s  + search_state_s + map_elem_s);

    std::cout<<"MAP "<<map_s<<std::endl;
    std::cout<<"SET "<<set_s<<std::endl;
    std::cout<<"STACK "<<set_s<<std::endl;
    std::cout<<"MAP_ELEM "<<map_elem_s<<std::endl;
    std::cout<<"SET_ELEM "<<set_elem_s<<std::endl;
    std::cout<<"STACK_ELEM "<<stack_elem_s<<std::endl;
    return num_states; // reserve 10 percent for arbitrary variables and stuff
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    if (init_state.isFinal()) {
        return {};
    }

    auto num_elems = max_states_count(mem_limit_);
    std::cout<<"NUM_ELEMS: "<<num_elems<<std::endl;
    // stack of pairs (ss, depth)
    std::stack<std::pair<SearchState_p, int>> stack;
    stack.push({{std::make_shared<const SearchState>(init_state)}, 0});
    // closed set
    std::set<SearchState_p> explored;
    // path traversed
    std::map<SearchState_p, std::pair<SearchAction_p,SearchState_p>> history;
    

    int curr_depth;
    SearchState_p curr_p, adj_p;
    SearchAction_p action_p;
    while(!stack.empty()){
        // pop top state from stack
        curr_p = stack.top().first;
        curr_depth = stack.top().second;
        stack.pop();

        // if surpassed depth limit then dont go further
        if (curr_depth >= depth_limit_) {
            continue;
        }

        // path reconstruction
        if (curr_p->isFinal()) {
            std::vector<SearchAction> path;
            for (int i = 0; i < curr_depth; i++) {
                auto prev = history.at(curr_p);
                curr_p = prev.second;
                path.push_back(*(prev.first));
            }
            std::reverse(path.begin(), path.end());
            std::cout<<"MEMORY CAPACITY: "<<mem_limit_<<std::endl;
            std::cout<<"MEMORY SPOTREBOVANE: "<<(getCurrentRSS())<<std::endl;
            std::cout<<"MEMORY REZERVA: "<<(mem_limit_ - getCurrentRSS())<<std::endl<<std::endl;
            std::cout<<curr_p->nbExpanded()<<std::endl;
            
            return path;
        }

        // set current state as already explored
        setExplored(curr_p, explored);
        std::cout<<"NB "<<curr_p->nbExpanded()<<std::endl;
        // push adjacent states to stack
        for (auto action : curr_p->actions()) {
            const SearchState adj_state = action.execute(*curr_p);

            adj_p = std::make_shared<const SearchState>(adj_state);
            action_p = std::make_shared<const SearchAction>(action);

            // if state has been already explored, dont push him to stack
            // comparison by GameState, not by identity
            if (isExplored(adj_p, explored)) {
                continue;
            }
            history.insert({adj_p, {action_p, curr_p}});
            stack.push({adj_p,curr_depth+1});
         }    
         std::cout<<"MEMORY REZERVA: "<<(mem_limit_ - getCurrentRSS())<<std::endl<<std::endl;
         std::cout<<"___________________________________________________"<<std::endl;
         //if(curr_p->nbExpanded() > num_elems) break;
    }
    return {};
}

double StudentHeuristic::distanceLowerBound(const GameState& state) const {
    // cards not in order
    int turns = 0;
    for (const auto& stack : state.stacks) {
        const auto& storage = stack.storage();
        if (storage.empty()) {
            continue;
        }

        const Card& last = *stack.storage().begin();
        auto end = storage.end();
        for (auto it = storage.begin() + 1; it != end; it++) {
            if (last.value - 1 != it->value) {
                turns++;
            }
        }
    }

    // + cards in freecells
    for (const auto& freecell : state.free_cells) {
        if (freecell.topCard().has_value()) {
            turns++;
        }
    }

    return turns;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState& init_state) {
    init_state.actions();
    return {};
}
