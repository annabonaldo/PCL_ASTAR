#include "PointCloudGraph2D.h"
#include "IOmanager.h"
using namespace std; 

bool PointCloudGraph2D::LOG_CONSOLE_GRAPH = false; 

PointCloudGraph2D::PointCloudGraph2D(void)
{
}

PointCloudGraph2D::~PointCloudGraph2D(void)
{
}

PointCloudGraph2D::PointCloudGraph2D(pcl::PointCloud<pcl::PointXY>::Ptr cloud)
{
  _cloud.reset(new pcl::PointCloud<pcl::PointXY>(*cloud));
  _graph = std::vector<std::set<int>>(); 
  ComputeGraph();

}

int PointCloudGraph2D::GetGraphSize() const
{
  return this->_cloud->points.size(); 
}

float PointCloudGraph2D::GetDistance(int x, int y)
{
  pcl::PointXY point_x =  this->_cloud->points.at(x); 
  pcl::PointXY point_y =  this->_cloud->points.at(y); 

  return PointCloudGraph2D::distance(point_x, point_y); 
}

float PointCloudGraph2D::ComputeH(int node)
{
  pcl::PointXY current =  this->_cloud->points.at(node); 
  pcl::PointXY start   =  this->_cloud->points.at(_start); 
  pcl::PointXY goal    =  this->_cloud->points.at(_goal); 

  switch(Params::H_type)
  {
     case(HEURISTIC::MIN_DISTANCE_FROM_GOAL):
       {

         return distance(current, goal);
       }

     case(HEURISTIC::MIN_DISTANCE_FROM_STRAIGTH_LINE_PLUS_GOAL_DISTANCE):
       {
         float actual_distance = distance(current, goal)+distance(current, start); 
         float best_distance = distance(start, goal); 
         return actual_distance - best_distance; 
       }
  }
  return -1.0F; 
}

 float PointCloudGraph2D::distance(const pcl::PointXY & p1,const pcl::PointXY & p2)
 {
   float dist_x = p1.x -p2.x; 
   float dist_y = p1.y -p2.y; 
   return std::sqrt((dist_x*dist_x)+(dist_y*dist_y)); 
 }

boost::shared_ptr<pcl::PointCloud<pcl::PointXY>> PointCloudGraph2D::RandomCloud(int size)
{
  pcl::PointCloud<pcl::PointXY>::Ptr cloud;  
 cloud.reset(new pcl::PointCloud<pcl::PointXY>()); 
 cloud->width = size; 
 cloud->points.resize(size); 
  for(int i =0; i< size; i++)
    {
      cloud->points[i].x= 1024 * rand() / (RAND_MAX +1.0F); 
      cloud->points[i].y= 1024 * rand() / (RAND_MAX +1.0F); 
  }
  return cloud; 
}


void PointCloudGraph2D::ComputeGraphKNN(int k)
{
  pcl::KdTreeFLANN<pcl::PointXY> kdtree;
  kdtree.setInputCloud(_cloud);

  // start point 
  pcl::PointXY searchPoint;

  // K nearest neighbor search
  int K = Params::N_param; 

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > points = _cloud->points; 
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

      pcl::PointXY searchPoint = points.at(point_i); 

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

void  PointCloudGraph2D::ComputeGraphRadiusSearch(float radius)
{ 
  pcl::KdTreeFLANN<pcl::PointXY> kdtree;
  kdtree.setInputCloud(_cloud);

  std::vector<int>   pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > points = _cloud->points; 
  if(points.size() > 0)
  {
    int perc_25 = points.size()/4;  
    int perc_50 = points.size()/2;
    int perc_75 = perc_25 *3; 
    //float radius = 256.0f * rand () / (RAND_MAX + 1.0f);
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

      pcl::PointXY searchPoint = points.at(point_i); 
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
    }
  }

}
