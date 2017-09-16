#include "PointCloudGraph3D.h"
#include "IOmanager.h"
using namespace std; 
//#include "IOmanager.h"
//#include <iostream>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/common/time.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/random_sample.h>
//#include <pcl/common/transforms.h>
//#include <pcl/common/eigen.h>

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

  switch(Params::H_type)
  {
  case(HEURISTIC::MIN_DISTANCE_FROM_GOAL):
    {
      if((_H.size() > node) && (_H[node] >0.0f))
        return _H[node]; 
      return pcl::euclideanDistance(current, goal); 
    }
  case(HEURISTIC::MIN_DISTANCE_FROM_STRAIGTH_LINE):
    {
      if((_H.size() > node) && (_H[node] >0.0f))
        return _H[node]; 

      float actual_distance = pcl::euclideanDistance(current, goal)+pcl::euclideanDistance(current, start); 
      float best_distance = pcl::euclideanDistance(start, goal); 

      return (actual_distance - best_distance) +  pcl::euclideanDistance(current, goal); 
    }
  }
  return -1.0F; 
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudGraph3D::RandomCloud(int size)
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

void PointCloudGraph3D::ComputeGraphKNN(int k)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(_cloud);

  // start point 
  pcl::PointXYZ searchPoint;

  // K nearest neighbor search
  int K = Params::N_param; 

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
          std::cout <<" 50% of graph computed "<< std::endl; 
        }
        else if((point_i > perc_50) && (point_i == perc_75))
        {
          std::cout <<" 75% of graph computed "<< std::endl; 
        }
      }

      pcl::PointXYZ searchPoint = points.at(point_i); 

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
    //  std::cout<< "Graph of connectivity " << Params::N_param << " correctly computed: " << points.size() << " nodes into the graph" <<std::endl;
  }
}

void  PointCloudGraph3D::ComputeGraphRadiusSearch(float radius)
{ 
  radius = 200.0F,
  std::cout <<"ComputeGraphRadiusSearch "<< std::endl; 
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(_cloud);

  std::vector<int>   pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points = _cloud->points; 
  if(points.size() > 0)
  {
    std::cout <<"points n "<<points.size()<<std::endl; 
    int perc_25 = points.size()/4;  
    int perc_50 = points.size()/2;
    int perc_75 = perc_25 *3; 
    float rand = 256.0f * std::rand() / (RAND_MAX + 1.0f);
    std::cout <<" RAND "<< (int)rand << " "<<rand<<std::endl;
    for(int point_i=0; point_i< points.size(); point_i++)
    {
      std::cout <<point_i<<std::endl; 
      if(point_i == perc_25)
      {  std::cout <<" 25% of graph computed "<< std::endl; 
      }
      if(point_i > perc_25)
      {
        if(point_i == perc_50)
        {
          std::cout <<" 50% of graph computed "<< std::endl; 
        }
        else if((point_i > perc_50) && (point_i == perc_75))
        {
          std::cout <<" 75% of graph computed "<< std::endl; 
        }
      }

      pcl::PointXYZ searchPoint = points.at(point_i); 
      if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      {
        this->_graph.push_back(std::set<int>()); 
        for (int neighbor_i= 0; neighbor_i < pointIdxRadiusSearch.size (); ++neighbor_i)
        {
          if(this->_graph.at(point_i).find(neighbor_i) == this->_graph.at(point_i).end() 
            && 
            (pointIdxRadiusSearch.at(neighbor_i)!= point_i) )
            this->_graph.at(point_i).insert(pointIdxRadiusSearch.at(neighbor_i)); 
        }
      }
       else std::cout<< "none"<< std::endl; 
    }
  }

}
