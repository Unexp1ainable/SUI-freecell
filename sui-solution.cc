#include <algorithm>
#include <iostream>
#include "memusage.h"
#include "search-strategies.h"

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

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    init_state.actions();
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
