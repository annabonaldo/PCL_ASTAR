#pragma once
#include <set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

//#include <pcl/io/openni_grabber.h>
//#include <pcl/common/time.h>

//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/random_sample.h>
//#include <pcl/common/transforms.h>
//#include <pcl/common/eigen.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <fstream>

class PointCloudGraph{

public:
  PointCloudGraph(void);
  ~PointCloudGraph(void);

  int Start() const ; 
  int Goal()  const ; 
  std::set<int> GetNodeNeighbours(int node);
  std::vector<float> PointCloudGraph::GetFloatArrayOfGraphSize(float defalut = 0.0F) const; 
  float G(int node); 
  void setStartAndGoal(int start, int goal); 
  void SetRandomStartAndGaol(); 

  void UpdateCosts(std::vector<float> costs); 

  virtual float H(int node) = 0; 
  virtual void  ComputeGraph(); 
  virtual void  ComputeGraphKNN(int k) = 0; 
  virtual void  ComputeGraphRadiusSearch(float radius) = 0; 
  virtual float GetDistance(int x, int y) = 0; 
  virtual int   GetGraphSize() const = 0; 
  


protected: 

  std::vector<std::set<int>> _graph; 

  std::vector<float> _G; 
  std::vector<float> _H;

  int _start; 
  int _goal;

  bool precomputed_costs; 

  void CreateCostFunctionDataStructure(); 

  void printNeighbours(int node, std::ofstream & file); 

  static std::ofstream& log(); 
  static std::ofstream& err(); 
  static std::ofstream log_file; 
  static std::ofstream err_file; 



};

