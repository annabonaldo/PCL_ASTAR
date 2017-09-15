#include "PointCloudGraph.h"
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

//#ifndef VERBOSE_OUTPUT
//#define VERBOSE_OUTPUT 
#define LOG_DETAILS
//#endif

std::ofstream PointCloudGraph::log_file; 
std::ofstream PointCloudGraph::err_file; 


PointCloudGraph::PointCloudGraph(void)
{
}

PointCloudGraph::~PointCloudGraph(void)
{
}


//void PointCloudGraph::ComputeGraph()
//{
//  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//  kdtree.setInputCloud(_cloud);
//
//  // start point 
//  pcl::PointXYZ searchPoint;
//
//  // K nearest neighbor search
//  int K = Params::K_SIZE; 
//
//  std::vector<int> pointIdxNKNSearch(K);
//  std::vector<float> pointNKNSquaredDistance(K);
//  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points = _cloud->points; 
//  if(points.size() > 0)
//  {
//    int perc_25 = points.size()/4;  
//    int perc_50 = points.size()/2;
//    int perc_75 = perc_25 *3; 
//
//    for(int point_i=0; point_i< points.size(); point_i++)
//    {
//      if(point_i == perc_25)
//      {  std::cout <<" 25% of graph computed "<< std::endl; 
//      }
//      if(point_i > perc_25)
//      {
//        if(point_i == perc_50)
//        {
//          log() <<" 50% of graph computed "<< std::endl; 
//        }
//        else if((point_i > perc_50) && (point_i == perc_75))
//        {
//          log() <<" 75% of graph computed "<< std::endl; 
//        }
//      }
//
//      pcl::PointXYZ searchPoint = points.at(point_i); 
//
//#ifdef VERBOSE_OUTPUT       
//      std::cout << "K nearest neighbor search at (" << searchPoint.x 
//        << " " << searchPoint.y 
//        << " " << searchPoint.z
//        << ") with K=" << K << std::endl;
//#endif
//
//      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//      {
//        this->_graph.push_back(std::set<int>()); 
//        for (int neighbor_i= 0; neighbor_i < pointIdxNKNSearch.size (); ++neighbor_i)
//        {
//          if(this->_graph.at(point_i).find(neighbor_i) == this->_graph.at(point_i).end() && (pointIdxNKNSearch.at(neighbor_i)!= point_i) )
//            this->_graph.at(point_i).insert(pointIdxNKNSearch.at(neighbor_i)); 
//        }
//      }
//    }
//    std::cout<< "Graph of connectivity " << Params::K_SIZE << " correctly computed: " << points.size() << " nodes into the graph" <<std::endl;
//
//  }
//  else 
//    cout << "no points found" << std::endl; 
//
//  // Neighbors within radius search
//  /*
//  std::vector<int> pointIdxRadiusSearch;
//  std::vector<float> pointRadiusSquaredDistance;
//
//  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);
//
//  std::cout << "Neighbors within radius search at (" << searchPoint.x 
//  << " " << searchPoint.y 
//  << " " << searchPoint.z
//  << ") with radius=" << radius << std::endl;
//
//
//  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
//  {
//  for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//  std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
//  << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
//  << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
//  << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//  }
//  */
//
//  for(int i = 0; i<this->_graph.size(); i++)
//  {
//    log()<<"i= "<<i <<"..."; 
//    for(std::set<int>::iterator it = _graph.at(i).begin(); it!= _graph.at(i).end(); it++)
//      log()<<*it<<" "; 
//    log()<<std::endl; 
//  }
//  SetRandomStartAndGaol(); 
//  CreateCostFunctionDataStructure(); 
//}

float PointCloudGraph::G(int node)
{
  if(_G.size() > node)
    return _G[node];

  else return -1.0F; 
}

std::set<int> PointCloudGraph::GetNodeNeighbours(int node)
{
  if(this->_graph.size() > node)
    return _graph.at(node); 
  return std::set<int>(); 
}

int PointCloudGraph::Start() const
{
  return _start; 
}

int PointCloudGraph::Goal() const
{
  return _goal; 
}

std::vector<float> PointCloudGraph::GetFloatArrayOfGraphSize(float defalut) const
{
  std::vector<float> structure; 
  for(int j=0; j< this->GetGraphSize(); j++)
    structure.push_back(defalut);
 return structure; 

}

//################# PRIVATE METHODS
void PointCloudGraph::SetRandomStartAndGaol()
{
  int graph_size = this->_graph.size(); 
  if(graph_size > 0)
  {
    this->_goal = (  graph_size * rand () / (RAND_MAX + 1.0f)); 
    this->_start = ( graph_size * rand () / (RAND_MAX + 1.0f)); 
    if(_goal == _start)
    {

      err() << "goal cannot be equal to start .. retring..."<<std::endl; 
      this->_start = ( graph_size * rand () / (RAND_MAX + 1.0f)); 
     // _goal =_start + ( std::rand() % ( graph_size - _start + 1 ) ); 
    }

    if((_goal != _start) && (_goal < graph_size) && (_start < graph_size))
    { 

      log() << _goal<<" goal point index (in bounds) " <<std::endl; 
      log() << _start<<" start point index (in bounds)" <<std::endl;

    }
    else
    {
        err() << "goal cannot be equal to start: all set to 0.0"<<std::endl; 
      _start = 0; 
      _goal = 0; 
    }

  }
}

void PointCloudGraph::CreateCostFunctionDataStructure() 
{
  log()<< "Cost function data structure: creating ... "<<std::endl;
  _G = GetFloatArrayOfGraphSize(); 
  log()<< "Cost function data structure create correctly"<<std::endl;
}

std::ofstream& PointCloudGraph::log()
{
  if(!log_file.is_open())
  log_file.open("logGraph.txt", ios::out |ios::app ); 

  return log_file; 
}

std::ofstream& PointCloudGraph::err()
{
  if(!err_file.is_open())
  err_file.open("errGraph.txt", ios::out |ios::app ); 

  return err_file; 
}

void PointCloudGraph::printNeighbours(int node, std::ofstream & file)
{

}
