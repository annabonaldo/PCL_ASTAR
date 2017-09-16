#pragma once
#include "PointCloudGraph.h"

class PointCloudGraph2D: public PointCloudGraph
{
public:
  PointCloudGraph2D(void);
  ~PointCloudGraph2D(void);
  PointCloudGraph2D(pcl::PointCloud<pcl::PointXY>::Ptr cloud); 

 // virtual void ComputeGraph(); 
  virtual float GetDistance(int x, int y); 
  virtual int   GetGraphSize() const; 
  virtual float H(int node); 
  virtual void  ComputeGraphKNN(int k); 
  virtual void  ComputeGraphRadiusSearch(float radius); 

  static boost::shared_ptr<pcl::PointCloud<pcl::PointXY>> RandomCloud(int size); 
protected: 
   pcl::PointCloud<pcl::PointXY>::Ptr _cloud;  

   static float distance(const pcl::PointXY & p1,const pcl::PointXY & p2); 

   static bool LOG_CONSOLE_GRAPH; 



};

