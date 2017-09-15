#include "PointCloudGraph3D.h"
#include "IOmanager.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

using namespace pcl;

PointCloudGraph3D::PointCloudGraph3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  _cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
  _graph = std::vector<std::set<int>>(); 
  ComputeGraph(); 

}


PointCloudGraph3D::~PointCloudGraph3D(void)
{
}

int PointCloudGraph3D::GetGraphSize() const
{
  return this->_cloud->points.size(); 
}

void PointCloudGraph3D::ComputeGraph()
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(_cloud);

  // start point 
  pcl::PointXYZ searchPoint;

  // K nearest neighbor search
  int K = Params::K_SIZE; 

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points = _cloud->points; 
  if(points.size() > 0)
  {
    int perc_25 = points.size()/4;  
    int perc_50 = points.size()/2;
    int perc_75 = perc_25 *3; 

    for(int point_i=0; point_i< points.size(); point_i++)
    {
      if(point_i == perc_25)
      {  std::cout <<" 25% of graph computed "<< std::endl; 
      }
      if(point_i > perc_25)
      {
        if(point_i == perc_50)
        {
          log() <<" 50% of graph computed "<< std::endl; 
        }
        else if((point_i > perc_50) && (point_i == perc_75))
        {
          log() <<" 75% of graph computed "<< std::endl; 
        }
      }

      pcl::PointXYZ searchPoint = points.at(point_i); 

#ifdef VERBOSE_OUTPUT       
      std::cout << "K nearest neighbor search at (" << searchPoint.x 
        << " " << searchPoint.y 
        << " " << searchPoint.z
        << ") with K=" << K << std::endl;
#endif

      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        this->_graph.push_back(std::set<int>()); 
        for (int neighbor_i= 0; neighbor_i < pointIdxNKNSearch.size (); ++neighbor_i)
        {
          if(this->_graph.at(point_i).find(neighbor_i) == this->_graph.at(point_i).end() && (pointIdxNKNSearch.at(neighbor_i)!= point_i) )
            this->_graph.at(point_i).insert(pointIdxNKNSearch.at(neighbor_i)); 
        }
      }
    }
    std::cout<< "Graph of connectivity " << Params::K_SIZE << " correctly computed: " << points.size() << " nodes into the graph" <<std::endl;

  }
  else 
    cout << "no points found" << std::endl; 

  // Neighbors within radius search
  /*
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x 
  << " " << searchPoint.y 
  << " " << searchPoint.z
  << ") with radius=" << radius << std::endl;


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
  for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
  std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
  << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
  << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
  << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }
  */

  for(int i = 0; i<this->_graph.size(); i++)
  {
    log()<<"i= "<<i <<"..."; 
    for(std::set<int>::iterator it = _graph.at(i).begin(); it!= _graph.at(i).end(); it++)
      log()<<*it<<" "; 
    log()<<std::endl; 
  }
  SetRandomStartAndGaol(); 
  CreateCostFunctionDataStructure(); 
}

float PointCloudGraph3D::GetDistance(int x, int y)
{
  pcl::PointXYZ point_x =  this->_cloud->points.at(x); 
  pcl::PointXYZ point_y =  this->_cloud->points.at(y); 

  return pcl::euclideanDistance(point_x, point_y); 
}

float PointCloudGraph3D::H(int node)
{
  pcl::PointXYZ current =  this->_cloud->points.at(node); 
  pcl::PointXYZ start =  this->_cloud->points.at(_start); 
  pcl::PointXYZ goal =  this->_cloud->points.at(_goal); 

  switch(Params::H_TYPE)
  {
  case(HEURISTIC::MIN_DISTANCE_FROM_GOAL):
    {
      return pcl::euclideanDistance(current, goal); 
    }
  case(HEURISTIC::MIN_DISTANCE_FROM_STRAIGTH_LINE):
    {
      float actual_distance = pcl::euclideanDistance(current, goal)+pcl::euclideanDistance(current, start); 
      float best_distance = pcl::euclideanDistance(start, goal); 
      return actual_distance - best_distance; 
    }
  }
  return -1.0F; 
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudGraph::RandomCloud(int size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  
 cloud.reset(new pcl::PointCloud<pcl::PointXYZ>()); 
 cloud->width = size; 
 cloud->points.resize(size); 
  for(int i =0; i< size; i++)
    {
      cloud->points[i].x= 1024 * rand() / (RAND_MAX +1.0F); 
      cloud->points[i].y= 1024 * rand() / (RAND_MAX +1.0F); 
      cloud->points[i].z= 1024 * rand() / (RAND_MAX +1.0F); 
  }
  return cloud; 
}