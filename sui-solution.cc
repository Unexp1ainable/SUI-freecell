#include <algorithm>
#include <iostream>
#include "memusage.h"
#include "search-strategies.h"

std::vector<SearchAction> finalize(
    std::vector<SearchState>& toBeSearched,
    std::vector<int>& howDidWeGetHere,
    std::vector<SearchAction>& actions) {
    std::vector<SearchAction> result{actions.back()};
    auto actionN = howDidWeGetHere.back();

    std::cout << "Action:" << actions.back() << "\n";

    std::cout << "State" << result.back().execute(toBeSearched[howDidWeGetHere.back()]) << std::endl;

    while (actionN != 0) {
        result.push_back(actions[actionN - 1]);

        std::cout << "Action:" << actions[actionN] << "\n";
        std::cout << "State" << toBeSearched[actionN] << "\n";
        actionN = howDidWeGetHere[actionN - 1];
    }

    std::reverse(result.begin(), result.end());
    std::cout << "----------\n";
    for (const auto& r : result)
        std::cout << r << "\n";

    return result;
}


std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState& init_state) {
    if (init_state.isFinal()) {
        return {};
    }

    std::cout << "State" << init_state << "\n";

    std::vector<SearchState> toBeSearched{init_state};
    std::vector<int> howDidWeGetHere{};
    std::vector<SearchAction> actions{};

    int j = 0;
    for (int i = 0; i < toBeSearched.size(); i++) {
        auto currState = toBeSearched[i];
        int k = 0;
        for (const auto& action : currState.actions()) {
            actions.emplace_back(action);
            toBeSearched.emplace_back(action.execute(currState));

            howDidWeGetHere.push_back(i);
            j++;
            if (toBeSearched.back().isFinal()) {
                // std::cout << toBeSearched.back() << "\n";

                return finalize(toBeSearched, howDidWeGetHere, actions);
            }
            k++;
        }
    }

    return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    return {};
}

double StudentHeuristic::distanceLowerBound(const GameState& state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState& init_state) {
    return {};
}
