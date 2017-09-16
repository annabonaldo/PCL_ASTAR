#pragma once
#include "PointCloudGraph.h"

class PointCloudGraph3D: public PointCloudGraph
{
public:
  PointCloudGraph3D(void);
  ~PointCloudGraph3D(void);
  PointCloudGraph3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud); 

  virtual float GetDistance(int x, int y); 
  virtual int   GetGraphSize() const; 
  virtual float ComputeH(int node); 
  virtual void PointCloudGraph3D::ComputeGraphKNN(int k); 
  virtual void  ComputeGraphRadiusSearch(float radius); 

  static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> RandomCloud(int size); 

protected: 
   pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;  

};

