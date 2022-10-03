#include <algorithm>
#include <functional>
#include <iostream>
#include <list>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include "memusage.h"
#include "search-strategies.h"

#include <chrono>
#include <thread>

constexpr int CARD_COUNT = 52;
// constexpr int TOTAL_CARD_PTRS = (nb_stacks + nb_freecells + nb_homes + nb_stacks + nb_freecells);
constexpr int TOTAL_CARD_PTRS = 0;
constexpr int MEMORY_RESERVE = 100;

// because RSS is not updating, after one solve try fails, the memory is shown as not free.
// Here will be the first number of items stored and reused in subsequent tries
size_t safe_item_count_global = 0;

bool operator==(const SearchState& a, const SearchState& b) {
    return a.state_ == b.state_;
}

/**
 * @brief Backtrack the path and return it
 *
 * @param howDidWeGetHere Mapping of parent states
 * @param actions Action to state mapping (index of action corresponds to the index of the state)
 * @return std::vector<SearchAction>
 */
std::vector<SearchAction> finalize(
    std::vector<int>& howDidWeGetHere,
    std::vector<SearchAction>& actions) {
    // ):
    std::vector<SearchAction> result{actions.back()};
    auto actionN = howDidWeGetHere.back();

    // backtrack
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
    size_t item_count;
    if (safe_item_count_global == 0) {
        item_count = (mem_limit_ - getCurrentRSS()) /
                         (sizeof(SearchState) +
                          sizeof(int) +
                          sizeof(SearchAction) +
                          sizeof(std::set<SearchState>::iterator) +
                          CARD_COUNT * sizeof(Card)) -
                     MEMORY_RESERVE;

        safe_item_count_global = item_count;
    } else {
        item_count = safe_item_count_global;
    }

    // pre-allocate space
    std::set<SearchState> expanded{init_state};
    std::vector<std::set<SearchState>::iterator> toBeSearched{};  // holds all states
    std::vector<int> howDidWeGetHere{};                           // holds index of the parent state
    std::vector<SearchAction> actions{};                          // holds all actions

    toBeSearched.reserve(item_count);
    howDidWeGetHere.reserve(item_count);
    actions.reserve(item_count);
    toBeSearched.emplace_back(expanded.begin());

    size_t curr_item_count = 0;

    // begin bfs
    for (size_t i = 0; i < toBeSearched.size(); i++) {
        auto currState = *toBeSearched[i];
        for (const auto& action : currState.actions()) {
            // number of items exceeded
            if (curr_item_count == item_count - 1) {
                return {};
            }

            auto next = expanded.emplace(action.execute(currState));
            if (next.second) {  // if element was not in the expanded set
                actions.emplace_back(action);
                toBeSearched.emplace_back(next.first);
                howDidWeGetHere.emplace_back(i);
                if (toBeSearched.back()->isFinal()) {
                    // free memory for backtracking (just to be sure)
                    toBeSearched.clear();
                    expanded.clear();
                    return finalize(howDidWeGetHere, actions);
                }
            }

            curr_item_count++;
        }
    }

    // we should never reach this
    return {};
}

using SearchAction_p = std::shared_ptr<const SearchAction>;
using SearchState_p = std::shared_ptr<const SearchState>;

namespace dfs {

inline bool isExplored(SearchState_p state, std::set<SearchState_p> discovered) {
    return std::find_if(discovered.begin(), discovered.end(),
                        [&](SearchState_p s) {
                            return *s == *state;
                        }) != discovered.end();
}

inline void setExplored(SearchState_p state, std::set<SearchState_p>& discovered) {
    if (!discovered.insert(state).second) {
        std::cout << "ALREDY THERE" << std::endl;
    }
}

size_t max_states_count(size_t mem_limit) {
    size_t search_state_s = sizeof(const SearchState) + CARD_COUNT * (sizeof(Card));
    size_t search_state_ptr_s = sizeof(SearchState_p);
    size_t depth_s = sizeof(int);
    size_t pair_s_d = sizeof(std::pair<SearchState_p, int>);
    size_t stack_s = sizeof(std::stack<std::pair<SearchState_p, int>>);
    size_t stack_elem_s = depth_s + search_state_ptr_s + pair_s_d;

    size_t set_elem_s = search_state_ptr_s;
    size_t set_s = sizeof(std::set<SearchState_p>);

    size_t pair_s_as = sizeof(std::pair<SearchState_p, std::pair<SearchAction_p, SearchState_p>>);
    size_t search_action_s = sizeof(const SearchAction);
    size_t pair_as = sizeof(std::pair<SearchAction_p, SearchState_p>);
    size_t search_action_ptr_s = sizeof(SearchAction_p);
    size_t map_elem_s = pair_s_as + 2 * search_state_ptr_s + pair_as + search_action_ptr_s + search_action_s;
    size_t map_s = sizeof(std::map<SearchState_p, std::pair<SearchAction_p, SearchState_p>>);

    size_t num_states = (mem_limit - getCurrentRSS() - stack_s - set_s - map_s) /
                        (set_elem_s + stack_elem_s + map_elem_s + search_state_s);
    return num_states * 0.8;
}

}  // namespace dfs

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    if (init_state.isFinal()) {
        return {};
    }

    size_t num_elems;
    if (safe_item_count_global == 0) {
        num_elems = dfs::max_states_count(mem_limit_);
        safe_item_count_global = num_elems;
    } else {
        num_elems = safe_item_count_global;
    }

    // stack of pairs (ss, depth)
    std::stack<std::pair<SearchState_p, int>> stack;
    stack.push({{std::make_shared<const SearchState>(init_state)}, 0});
    // closed set
    std::set<SearchState_p> explored;
    // path traversed
    std::map<SearchState_p, std::pair<SearchAction_p, SearchState_p>> history;

    int curr_depth;
    SearchState_p curr_p, adj_p;
    SearchAction_p action_p;
    size_t x = 1;
    while (!stack.empty()) {
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
            std::cout << "FINISH " << curr_p->nbExpanded() << " | " << num_elems << std::endl;
            return path;
        }

        // set current state as already explored
        dfs::setExplored(curr_p, explored);

        // push adjacent states to stack
        for (auto action : curr_p->actions()) {
            const SearchState adj_state = action.execute(*curr_p);

            adj_p = std::make_shared<const SearchState>(adj_state);
            action_p = std::make_shared<const SearchAction>(action);

            // if state has been already explored, dont push him to stack
            // comparison by GameState, not by identity
            if (dfs::isExplored(adj_p, explored)) {
                continue;
            }
            x++;
            history.insert({adj_p, {action_p, curr_p}});
            stack.push({adj_p, curr_depth + 1});
        }
        if (getCurrentRSS() > mem_limit_) {
            // this should never be reached, but just to be sure...
            break;
        }
        // check whether we surpassed predicted limit of expanded states
        if (x > num_elems)
            break;
    }
    std::cout << "FAIL " << curr_p->nbExpanded() << " | " << num_elems << std::endl;
    return {};
}

double StudentHeuristic::distanceLowerBound(const GameState& state) const {
    // int cards_out_of_home = king_value * colors_list.size();
    // for (const auto& home : state.homes) {
    //     auto opt_top = home.topCard();
    //     if (opt_top.has_value())
    //         cards_out_of_home -= opt_top->value;
    // }

    std::optional<Card> cCard{Card(Color::Club, 1)};  // Tesco
    std::optional<Card> dCard{Card(Color::Diamond, 1)};
    std::optional<Card> hCard{Card(Color::Heart, 1)};
    std::optional<Card> sCard{Card(Color::Spade, 1)};
    for (const auto& home : state.homes) {
        if (home.topCard().has_value()) {
            auto card = *home.topCard();
            switch (card.color) {
                case Color::Club:
                    if (card.value != king_value)
                        cCard.emplace(Card(card.color, card.value + 1));
                    break;

                case Color::Diamond:
                    if (card.value != king_value)
                        dCard.emplace(Card(card.color, card.value + 1));
                    break;

                case Color::Heart:
                    if (card.value != king_value)
                        hCard.emplace(Card(card.color, card.value + 1));
                    break;

                case Color::Spade:
                    if (card.value != king_value)
                        sCard.emplace(Card(card.color, card.value + 1));
                    break;

                default:
                    break;
            }
        }
    }

    int turns = 0;
    bool count = false;

    // std::cout << state;
    // how many cards till we get to the next card to go home
    for (const auto& stack : state.stacks) {
        count = false;

        for (const auto& card : stack.storage()) {
            if (count) {
                turns += 2;
                continue;
            }

            if (card == cCard || card == sCard || card == dCard || card == hCard) {
                count = true;
                turns++;
            }
        }
    }

    // for (const auto& cell : state.free_cells) {
    //     if (cell.topCard().has_value()) {
    //         turns += 2;
    //     }
    // }
    // // std::cout << turns << std::endl;
    // // exit(0);
    // return turns + cards_out_of_home / 2;

    // cards not in order weighted by measure of unorderness
    for (const auto& stack : state.stacks) {
        const auto& storage = stack.storage();
        if (storage.empty()) {
            continue;
        }

        const Card& last = *stack.storage().begin();
        auto end = storage.end();
        for (auto it = storage.begin() + 1; it != end; it++) {
            auto diff = abs(last.value - 1 - it->value);
            if (diff != 0) {
                turns += diff;
            }
        }
    }

    // + cards in freecells
    for (const auto& freecell : state.free_cells) {
        if (freecell.topCard().has_value()) {
            turns += 2;
        }
    }

    return turns;
}

namespace a_star {

typedef struct Node {
    SearchState_p state_p;
    SearchState_p prev_p;
    SearchAction_p action;
    double g;
    double f;
} Node;

struct NodeComparator {
    bool operator()(const Node& lhs,
                    const Node& rhs) const {
        return (rhs.f) < (lhs.f);
    }
};

struct SearchStatePointerComparator {
    bool operator()(const SearchState_p& lhs,
                    const SearchState_p& rhs) const {
        return (*lhs) < (*rhs);
    }
};

size_t max_states_count(size_t mem_limit) {
    size_t search_state_s = sizeof(const SearchState) + CARD_COUNT * (sizeof(Card));
    size_t node_s = sizeof(Node);
    size_t open_set_s = sizeof(std::priority_queue<a_star::Node>);
    size_t open_set_elem_s = node_s + search_state_s;

    size_t search_state_ptr_s = sizeof(SearchState_p);
    size_t set_elem_s = search_state_ptr_s;
    size_t set_s = sizeof(std::set<SearchState_p>);

    size_t pair_s_as = sizeof(std::pair<SearchState_p, std::pair<SearchAction_p, SearchState_p>>);
    size_t search_action_s = sizeof(const SearchAction);
    size_t pair_as = sizeof(std::pair<SearchAction_p, SearchState_p>);
    size_t search_action_ptr_s = sizeof(SearchAction_p);
    size_t map_elem_s = pair_s_as + 2 * search_state_ptr_s + pair_as + search_action_ptr_s + search_action_s;
    size_t map_s = sizeof(std::map<SearchState_p, std::pair<SearchAction_p, SearchState_p>>);

    size_t num_states = (mem_limit - getCurrentRSS() - open_set_s - set_s - map_s) /
                        (set_elem_s + open_set_elem_s + map_elem_s);
    return num_states * 0.8;
}

}  // namespace a_star

std::vector<SearchAction> AStarSearch::solve(const SearchState& init_state) {
    if (init_state.isFinal()) {
        return {};
    }
    // priority queue sorted by f
    // initialized with init_state
    std::priority_queue<a_star::Node, std::vector<a_star::Node>, a_star::NodeComparator> open_set;
    size_t num_elems;
    if (safe_item_count_global == 0) {
        num_elems = a_star::max_states_count(mem_limit_);
        safe_item_count_global = num_elems;
    } else {
        num_elems = safe_item_count_global;
    }
    open_set.push({std::make_shared<const SearchState>(init_state), nullptr, nullptr, 0., 0.});

    // set of already explored states
    std::set<SearchState_p, a_star::SearchStatePointerComparator> exp;
    // history map, needed for path tracing
    std::map<SearchState_p, std::pair<SearchAction_p, SearchState_p>> history;

    SearchState_p curr_state_p, adj_state_p;
    SearchAction_p action_p;

    a_star::Node curr_node;
    double heuristic;
    size_t x = 1;
    while (!open_set.empty()) {
        curr_node = open_set.top();  // get top object by f value
        open_set.pop();
        curr_state_p = curr_node.state_p;

        // if state already explored then we dont process it
        if (!exp.insert(curr_state_p).second) {
            continue;
        }

        for (auto action : curr_state_p->actions()) {
            // adjacent state
            adj_state_p = std::make_shared<const SearchState>(action.execute(*curr_state_p));
            x++;

            if (adj_state_p->isFinal()) {
                std::cout << "FINAL " << curr_state_p->nbExpanded() << " | " << num_elems << std::endl;
                std::vector<SearchAction> path{action};
                for (int i = 0; i < curr_node.g; i++) {
                    auto prev = history.at(curr_state_p);
                    curr_state_p = prev.second;
                    path.push_back(*(prev.first));
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            heuristic = compute_heuristic(*adj_state_p, *heuristic_);
            open_set.emplace(
                a_star::Node{adj_state_p, curr_state_p, std::make_shared<SearchAction>(action), curr_node.g + 1, curr_node.g + 1 + heuristic});
            history.insert({adj_state_p, {std::make_shared<SearchAction>(action), curr_state_p}});
        }
        if (x > num_elems) {
            break;
        }
    }
    std::cout << "FAIL " << curr_state_p->nbExpanded() << " | " << num_elems << std::endl;
    return {};
}

std::vector<SearchAction> solve(const SearchState& init_state);
