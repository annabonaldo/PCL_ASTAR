#pragma once
#include "PointCloudGraph.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

class PointCloudGraph2D: public PointCloudGraph
{
public:
  PointCloudGraph2D(void);
  ~PointCloudGraph2D(void);
protected: 
   pcl::PointCloud<pcl::PointXY>::Ptr _cloud;  
};

