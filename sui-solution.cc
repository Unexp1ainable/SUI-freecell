#include <algorithm>
#include <iostream>
#include "memusage.h"
#include "search-strategies.h"
#include <stack>
#include <functional>
#include <sstream>
#include <optional>

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

std::string hash(SearchState state) {
    std::ostringstream ss;
    ss << state;
    /*for(auto a : state.actions()){
        ss<<a;
    }*/
    std::hash<std::string> hash;
    std::string h = ss.str();
    ss.str("");
    std::ostringstream ss2;
    ss2<<hash(h);
    return ss2.str();
}




bool isExplored(std::shared_ptr<const SearchState> state, std::vector<std::shared_ptr<const SearchState>> discovered){
    return std::find_if(discovered.begin(), discovered.end(), 
            [&](std::shared_ptr<const SearchState> s) {
                    return *s == *state;
                }) != discovered.end();
}

void setExplored(std::shared_ptr<const SearchState> state, std::vector<std::shared_ptr<const SearchState>>& discovered) {
    discovered.push_back(state);
}


bool operator==(const SearchState& a, const SearchState& b) {
    return a.state_ == b.state_;
}


std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    std::cout<<"DFS"<<std::endl;
    if(init_state.isFinal()){
        return {};
    }
    std::vector<std::shared_ptr<const SearchState>> stack{std::make_shared<const SearchState>(init_state)};
    std::vector<std::shared_ptr<const SearchState>> explored;
    std::vector<int> depths{0};
    std::map<std::shared_ptr<const SearchState>, std::pair<std::shared_ptr<const SearchAction>,std::shared_ptr<const SearchState>>> history;


    while(!stack.empty()){
        std::shared_ptr<const SearchState> curr_p = stack.back();
        stack.pop_back();

        int curr_depth = depths.back();
        depths.pop_back();

        if(curr_depth >= depth_limit_) {
            continue;
        }

        if(isExplored(curr_p, explored)) {
            // std::cout<<" already explored"<<std::endl;
            continue;
        }

        if(curr_p->isFinal()){
            
            // std::cout<<curr_depth<<std::endl;
            // std::cout<<hash(*curr_p)<<std::endl;
            // std::cout<<"LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL"<<std::endl;
            std::vector<SearchAction> path;
            for(int i = 0; i < curr_depth; i++){
                auto prev = history.at(curr_p);
                curr_p = prev.second;
                path.push_back(*(prev.first));
                // std::cout<<hash(*curr_p)<<std::endl;
                
                // std::cout<<"-------------------------"<<std::endl;
            }
            // std::cout<<path.size()<<std::endl;
            std::reverse(path.begin(), path.end());
            return path;
        }

        // std::cout<<"depth "<<curr_depth<<std::endl;
        // std::cout<<hash(*curr_p)<<std::endl;
        
        setExplored(curr_p, explored);
        // std::cout<<"-----------------adjacent----------------"<<std::endl;
        
        for(auto action : curr_p->actions()){
            const SearchState adj_state = action.execute(*curr_p);
            
            
            
            auto adj_p = std::make_shared<const SearchState>(adj_state);
            auto action_p = std::make_shared<const SearchAction>(action);

            if(isExplored(adj_p, explored)) {
                // std::cout<<"explored"<<std::endl;
                continue;
            }
            // std::cout<<hash(adj_state)<<std::endl;//<<adj_state<<std::endl;
            history.insert({adj_p, {action_p, curr_p}});
            stack.push_back(adj_p);
            depths.push_back(curr_depth+1);
            
         }
        
        // std::cout<<"-------------end adjacent----------------"<<std::endl;
        // std::cout<<"-------------visited----------------"<<std::endl;
        // for(auto x : explored) {
        //     std::cout<<hash(*x)<<std::endl;
        // }
        // std::cout<<"----------------end visited-----------------------"<<std::endl;
        // std::cout<<"================================================="<<std::endl;
        
    }
    return {};
}


/*
std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    if(init_state.isFinal()){
        return {};
    }
    std::vector<SearchState> stack{init_state};
    std::vector<std::shared_ptr<const SearchState>> explored;
    std::vector<int> depths{0};
    std::map<std::shared_ptr<const SearchState>, SearchAction> actions;
    std::map<std::shared_ptr<const SearchState>, std::shared_ptr<const SearchState>> prevState;


    while(!stack.empty()){
        SearchState curr_state = stack.back();
        stack.pop_back();

        int curr_depth = depths.back();
        depths.pop_back();

        if(curr_depth >= depth_limit_) {
            continue;
        }

        if(isExplored(curr_state, explored)) {
            std::cout<<" already explored"<<std::endl;
            continue;
        }

        if(curr_state.isFinal()){
            
            std::cout<<curr_depth<<std::endl;
            std::cout<<hash(curr_state)<<std::endl;
            std::cout<<"LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL"<<std::endl;
            std::vector<SearchAction> path;
            return{};
            //for(int i = 0; i < curr_depth-2; i++){
            while(true) {
                //auto last_move = actions.at(curr_state);//actions.find(curr_state)->second;
                //std::cout<<last_move<<std::endl;
                //path.emplace_back(last_move);
                //auto prev = prevState.at(curr_state);
                //curr_state = std::move(prev);
                //std::cout<<hash(curr_state)<<std::endl;
                
                std::cout<<"-------------------------"<<std::endl;
            }
            std::cout<<path.size()<<std::endl;
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::cout<<"depth "<<curr_depth<<std::endl;
        std::cout<<hash(curr_state)<<std::endl;
        
        setExplored(std::shared_ptr<const SearchState>{&curr_state}, explored);
        std::cout<<"-----------------adjacent----------------"<<std::endl;
        
        for(auto action : curr_state.actions()){
            SearchState adj_state = action.execute(curr_state);
            if(isExplored(adj_state, explored)) {
                continue;
            }
            std::cout<<hash(adj_state)<<std::endl;//<<adj_state;
            /*if(prevState.find(adj_state) != prevState.end()) {
                std::cout<<" already in";
            }
            std::cout<<std::endl;
            stack.emplace_back(adj_state);
            depths.push_back(curr_depth+1);
            //actions.insert({adj_state, action});
            
            prevState.insert(
                {std::shared_ptr<const SearchState>{&adj_state}, std::shared_ptr<const SearchState>{&curr_state}});
         }
        
        std::cout<<"-------------end adjacent----------------"<<std::endl;
        std::cout<<"-------------visited----------------"<<std::endl;
        for(auto x : explored) {
            std::cout<<hash(*x)<<std::endl;
        }
        std::cout<<"----------------end visited-----------------------"<<std::endl;
        std::cout<<"================================================="<<std::endl;
        
    }
    return {};
}*/


/*
std::vector<SearchAction> DepthFirstSearch::solve(const SearchState& init_state) {
    if(init_state.isFinal()){
        return {};
    }

    std::vector<SearchState> stack{init_state};
    std::map<SearchState, bool> visited;
    //std::map<SearchState, int> depths;
    std::vector<int> depth_stack;
    std::map<SearchState, SearchAction> actions;
    std::map<SearchState, SearchState> prevState;

    visited[init_state] = false;
    //depths[init_state] = 0;

    depth_stack.push_back(0);

    while(!stack.empty()){
        SearchState curr_state = stack.back();
        stack.pop_back();

        if(visited[curr_state]) {std::cout<<"already visited two times"<<std::endl;continue;};

        int curr_depth = depth_stack.back();
        depth_stack.pop_back();
        if(curr_depth >= depth_limit_) {
            continue;
        }
        // path reconstruction
        if(curr_state.isFinal()){
            std::cout<<hash(curr_state)<<std::endl;
            std::cout<<"LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL"<<std::endl;
            
            std::vector<SearchAction> path;
            for(int i = 0; i < curr_depth; i++){
                auto last_move = actions.find(curr_state)->second;
                std::cout<<last_move<<std::endl;
                path.emplace_back(last_move);
        
                auto prev = prevState.find(curr_state)->second;
                curr_state = std::move(prev);
                std::cout<<hash(curr_state)<<std::endl;
                
                std::cout<<"-------------------------"<<std::endl;
            }
            std::cout<<path.size()<<std::endl;
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::cout<<"depth "<<curr_depth<<std::endl;
        std::cout<<hash(curr_state)<<std::endl;
        
        visited[curr_state] = true;
        std::cout<<"-----------------adjacent----------------"<<std::endl;
        for(auto action : curr_state.actions()){
            SearchState adj_state = action.execute(curr_state);
            std::cout<<hash(adj_state, action)<<" "<<action;
            if(visited[adj_state]) {
                std::cout<<" already_seen"<<std::endl;
                continue;
            }
            std::cout<<std::endl;
            stack.emplace_back(adj_state);
            depth_stack.push_back(curr_depth+1);
            // set action taken to get to this state
            actions.insert({adj_state, action});
            // set state from which we got here
            prevState.insert({adj_state, curr_state});
        }
        for(auto x : visited) {
            std::cout<<hash(x.first)<<" "<<x.second<<std::endl;
        }
        std::cout<<"-------------end adjacent----------------"<<std::endl;
        std::cout<<"================================================="<<std::endl;
        
    }
    return {};
}*/

double StudentHeuristic::distanceLowerBound(const GameState& state) const {
    state.all_storage.back();
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState& init_state) {
    init_state.actions();
    return {};
}
