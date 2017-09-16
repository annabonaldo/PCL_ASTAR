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

float PointCloudGraph3D::GetDistance(int x, int y)
{
  pcl::PointXYZ point_x =  this->_cloud->points.at(x); 
  pcl::PointXYZ point_y =  this->_cloud->points.at(y); 
  float distance = pcl::euclideanDistance(point_x, point_y); 
  return distance; 
}

float PointCloudGraph3D::ComputeH(int node)
{
  pcl::PointXYZ current =  this->_cloud->points.at(node); 
  pcl::PointXYZ start =  this->_cloud->points.at(_start); 
  pcl::PointXYZ goal =  this->_cloud->points.at(_goal); 

  switch(Params::H_type)
  {
     case(HEURISTIC::MIN_DISTANCE_FROM_GOAL):
    {
      return pcl::euclideanDistance(current, goal); 
    }

     case(HEURISTIC::MIN_MAIN_DISTANCE):
    {
        Eigen::Vector3f p_start= start.getArray3fMap();
        Eigen::Vector3f p_goal = goal.getArray3fMap();
        Eigen::Vector3f p    = current.getArray3fMap();
         
        Eigen::Vector3f main_dir= (p_start - p_goal).normalized(); 
        Eigen::Vector3f dir     = (p - p_goal).normalized(); 
        double cosAlpha = main_dir.dot(dir); 

        double eval = this->GetDistance(node, Goal())* cosAlpha; 
        if(eval < 0.0)
          eval = std::fabs(eval) + GetDistance(Start(), Goal()); 

        return eval;

    }
     case(HEURISTIC::MIN_DISTANCE_FROM_STRAIGTH_LINE_PLUS_GOAL_DISTANCE):
    {
        if(node == Start()) return  GetDistance(Start(), Goal());

        float prj_on_start_to_goal_line = 0; 
        float distance_to_start_to_goal_line = 0; 

        Eigen::Vector3f p_start= start.getArray3fMap();
        Eigen::Vector3f p_goal = goal.getArray3fMap();
        Eigen::Vector3f p      = current.getArray3fMap();

        double StartToGoal_distance = GetDistance(Start(), Goal()); 
        double StartToNode_distance = GetDistance(Start(), node); 

        Eigen::Vector3f StartToGoal_versor = (p_goal-p_start); 
        Eigen::Vector3f StartToPoint_versor  = (p -p_start);

    /*    double cos_Start_angle = (StartToGoal_versor.x() *StartToPoint_versor.x()) + 
                                 (StartToGoal_versor.y() *StartToPoint_versor.y() )+
                                 (StartToGoal_versor.z()*StartToPoint_versor.z());

        cos_Start_angle = cos_Start_angle/ (StartToNode_distance* StartToGoal_versor);*/
         double cos_Start_angle = StartToGoal_versor.dot(StartToPoint_versor)/(StartToGoal_versor.norm()*StartToPoint_versor.norm()); 

        double sin_Start_angle = std::sqrt(1-(cos_Start_angle * cos_Start_angle)); 
        
        if(cos_Start_angle >=0)
        { 
          prj_on_start_to_goal_line = StartToNode_distance * cos_Start_angle; 
          distance_to_start_to_goal_line = StartToNode_distance * std::fabs(sin_Start_angle); 
        }
        else
        {
          cos_Start_angle = -1*cos_Start_angle;  
          prj_on_start_to_goal_line = (StartToNode_distance * cos_Start_angle) + StartToGoal_distance ; 
          distance_to_start_to_goal_line = StartToNode_distance * sin_Start_angle ; 

        }
        float total = prj_on_start_to_goal_line+distance_to_start_to_goal_line; 
        return total; 
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
  log()  <<"ComputeGraphKNN "<< std::endl; 
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
      {  log() <<" 25% of graph computed "<< std::endl; 
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
     log()<< "Graph of neighbours connectivity " << Params::N_param << " correctly computed: " << points.size() << " nodes into the graph" <<std::endl;
  }
}

void  PointCloudGraph3D::ComputeGraphRadiusSearch(float radius)
{ 
  int min =-1;
  int max =0;
  log()  <<"ComputeGraphRadiusSearch "<< std::endl; 
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(_cloud);

  std::vector<int>   pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points = _cloud->points; 
  if(points.size() > 0)
  {
    log() <<"points n "<<points.size()<<std::endl; 
    int perc_25 = points.size()/4;  
    int perc_50 = points.size()/2;
    int perc_75 = perc_25 *3; 
    //float rand = 256.0f * std::rand() / (RAND_MAX + 1.0f);
    //log() <<" RAND "<< (int)rand << " "<<rand<<std::endl;
    for(int point_i=0; point_i< points.size(); point_i++)
    {
      log() <<point_i<< " - "; 
      if(point_i == perc_25)
      { 
        log() << std::endl; 
        log()  <<" 25% of graph computed "<< std::endl; 
      }
      if(point_i > perc_25)
      {
        if(point_i == perc_50)
        {
          log() << std::endl; 
          log()  <<" 50% of graph computed "<< std::endl; 
        }
        else if((point_i > perc_50) && (point_i == perc_75))
        {
          log() << std::endl; 
          log()  <<" 75% of graph computed "<< std::endl; 
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
        if (pointIdxRadiusSearch.size () < min || min == -1)  min = pointIdxRadiusSearch.size (); 
        if (pointIdxRadiusSearch.size () > max ) max = pointIdxRadiusSearch.size (); 
      }
       else std::cout<< "none"<< std::endl; 
    }
    log() << "Graph of connectivity-radius " << Params::N_param << " correctly computed: " << points.size() << " nodes into the graph" <<std::endl;
    log() << "min connectivity: " <<min <<" max connectivity: "<<max<<std::endl; 
  }

}
