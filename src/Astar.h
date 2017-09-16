#pragma once
#include <queue>
#include<list>
#include "PointCloudGraph.h"
#include "Statistics.h"
#include <fstream>
#include <iostream>

class Astar
{
public:
  Astar(void);
  std::list<int> Compute(PointCloudGraph & Graph ); 
private: 

  std::set<std::pair<float, int>> _OPEN_QUEUE_h_id; 
  std::map<int, float> _OPEN_id_h ; 
  std::set<int> _CLOSED; 
  std::vector<float> _F; 
  std::map<int, int> _child_parent_map; 


  Statistics stats; 
  void Astar::Initialize(int size); 
  void setNodeParent(int c, int p);
  std::list<int> getPath(int start, int goal); 
  bool _goal_found; 

  bool isOpenNode(int node); 
  bool isClosedNode(int node); 
  bool isCurrentEvaluationBetterThanTheStoredOne(int node, float eval); 
 
  void removeNodeFromOpenQueue(int node); 
  void addToOpenQueue(int node, float eval);
  void addToClosed(int node); 
  void removeNodeFromClosed(int node); 

  void saveResults(PointCloudGraph &  Graph); 

  float getCost(PointCloudGraph & Graph, int node, int succ);

  void printQueue(); 
 
  static std::ofstream& Astar::log(); 
  static std::ofstream& Astar::err(); 
  static std::ofstream log_file; 
  static std::ofstream err_file; 

  static bool log_on; 
  static int computation_iter; 

};

