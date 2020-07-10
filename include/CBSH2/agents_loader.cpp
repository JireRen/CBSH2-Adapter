//=======================================================================

#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <algorithm>  // for remove_if
#include <ctime>
#include <chrono>
#include <random>
using namespace boost;
using namespace std;
namespace CBSH{
  int RANDOM_WALK_STEPS = 100000;

  AgentsLoader::AgentsLoader(std::vector<std::pair<int, int> > starts, std::vector<std::pair<int, int> > goals)
  {

  	this->num_of_agents = starts.size();
  	this->initial_locations = starts;
  	this->goal_locations = goals;
  }

  void AgentsLoader::printAgentsInitGoal () const
  {
    cout << "AGENTS:" << endl;;
    for (int i=0; i<num_of_agents; i++) 
    {
      cout << "Agent" << i << " : I=(" << initial_locations[i].first << "," << initial_locations[i].second << ") ; G=(" <<
        goal_locations[i].first << "," << goal_locations[i].second << ")" << endl;
    }
    cout << endl;
  }

  AgentsLoader::~AgentsLoader()
  {
    // vectors are on stack, so they are freed automatically
  }

  // create an empty object
  AgentsLoader::AgentsLoader() 
  {
    num_of_agents = 0;
  }

  // returns the agents' ids if they occupy [row,col] (first for start, second for goal)
  pair<int, int> AgentsLoader::agentStartOrGoalAt(int row, int col) {
    int f = -1;
    int s = -1;
    for (vector< pair<int, int> >::iterator it = initial_locations.begin(); it != initial_locations.end(); ++it)
      if ( it->first == row && it->second == col )
        f = std::distance(initial_locations.begin(), it);
    for (vector< pair<int, int> >::iterator it = goal_locations.begin(); it != goal_locations.end(); ++it)
      if ( it->first == row && it->second == col )
        s = std::distance(goal_locations.begin(), it);
    return make_pair(f, s);
  }


  void AgentsLoader::clearLocationFromAgents(int row, int col) {
    pair<int, int> idxs = agentStartOrGoalAt(row, col);
    if ( idxs.first != -1 ) 
    {  // remove the agent who's start is at [row,col]
      initial_locations.erase( initial_locations.begin() + idxs.first );
      goal_locations.erase ( goal_locations.begin() + idxs.first );
      num_of_agents--;
    }
    idxs = agentStartOrGoalAt(row, col);
    if ( idxs.second != -1 ) 
    {  // remove the agent who's goal is at [row,col]
      initial_locations.erase( initial_locations.begin() + idxs.second );
      goal_locations.erase( goal_locations.begin() + idxs.second );
      num_of_agents--;
    }
  }


  // add an agent
  void AgentsLoader::addAgent(int start_row, int start_col, int goal_row, int goal_col) 
  {
    this->initial_locations.push_back(make_pair(start_row, start_col));
    this->goal_locations.push_back(make_pair(goal_row, goal_col));
    num_of_agents++;
  }

  void AgentsLoader::saveToFile(std::string fname) 
  {
    ofstream myfile;
    myfile.open(fname);
    myfile << num_of_agents << endl;
    for (int i = 0; i < num_of_agents; i++)
      myfile << initial_locations[i].first << "," << initial_locations[i].second << ","
             << goal_locations[i].first << "," << goal_locations[i].second << "," << endl;
    myfile.close();
  }
}