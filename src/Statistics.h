#pragma once
#include <vector>
class Statistics
{
public:
  Statistics(void);
  ~Statistics(void);

  int cloud_size; 
  std::vector<float> cloud_dim; 
  bool is2D; 
  int n_open_nodes; 
  int n_closed_nodes; 
  float queue_mean_size;
  float path_lenght; 
  float start_goal_distance; 
  double execution_time; 
  int itr; 

  void log(); 
  void logHeaders(); 
private: 
  static std::string SEP; 
  static std::ofstream csv_file; 
 
};

