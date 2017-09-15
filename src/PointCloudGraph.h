#pragma once

#include <set>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>


class PointCloudGraph{

public:
  PointCloudGraph(void);
  ~PointCloudGraph(void);

  int Start() const ; 
  int Goal()  const ; 
  std::set<int> GetNodeNeighbours(int node);
  std::vector<float> PointCloudGraph::GetFloatArrayOfGraphSize(float defalut = 0.0F) const; 
  float G(int node); 

  virtual float H(int node) = 0; 
  virtual void  ComputeGraph()= 0; 
  virtual float GetDistance(int x, int y) = 0; 
  virtual int   GetGraphSize() const; 
  

protected: 

  std::vector<std::set<int>> _graph; 

  std::vector<float> _G; 
  std::vector<float> _H;

  int _start; 
  int _goal;

  void CreateCostFunctionDataStructure(); 
  void SetRandomStartAndGaol(); 
  void printNeighbours(int node, std::ofstream & file); 

  static std::ofstream& log(); 
  static std::ofstream& err(); 
  static std::ofstream log_file; 
  static std::ofstream err_file; 



};

