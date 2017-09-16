#include "PointCloudGraph.h"
#include "IOmanager.h"
using namespace pcl;
using namespace std; 

//#ifndef VERBOSE_OUTPUT
//#define VERBOSE_OUTPUT 
#define LOG_DETAILS

std::ofstream PointCloudGraph::log_file; 
std::ofstream PointCloudGraph::err_file; 

PointCloudGraph::PointCloudGraph(void)
{
  precomputed_costs = false; 
}

PointCloudGraph::~PointCloudGraph(void)
{
}


void PointCloudGraph::ComputeGraph()
{
  if(Params::N_type == NEIGHBORHOOD::KNN)
    ComputeGraphKNN(Params::N_param);
  else   if(Params::N_type == NEIGHBORHOOD::RADIUS)
    ComputeGraphRadiusSearch((float)Params::N_param); 

  SetRandomStartAndGaol(); 
  CreateCostFunctionDataStructure(); 
}

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

void PointCloudGraph::UpdateCosts(std::vector<float> costs)
{
  _H  = this->GetFloatArrayOfGraphSize(-1.0); 
  //Params::
  int perc = 1; 
  if(perc >1) 
  {
    for(int i = 0; i<costs.size(); i++)
    {
      if(i% perc == 0) _H[i] = costs[i]; 
    }
  }
}

//################# PRIVATE METHODS
void PointCloudGraph::SetRandomStartAndGaol()
{
  int graph_size = this->_graph.size(); 
  if(graph_size > 0)
  {
     _start =( std::rand() % ( graph_size - _start + 1 ) ); 
     _goal =_start + ( std::rand() % ( graph_size - _start + 1 ) ); 
    if(_goal == _start)
    {
      _goal =_start + ( std::rand() % ( graph_size - _start + 2 ) ); 
    }
    else
    {
      _start = 0; 
      _goal = 0; 
    }

  }
}

void PointCloudGraph::CreateCostFunctionDataStructure() 
{
  _G = GetFloatArrayOfGraphSize(); 
}


void PointCloudGraph::printNeighbours(int node, std::ofstream & file)
{

}

void PointCloudGraph::setStartAndGoal(int start, int goal)
{
  if(this->GetGraphSize() > start)  this->_start = start; 
  else _start = 0; 

  if(this->GetGraphSize() > goal) this->_goal = goal; 
  else _goal =0; 
}

std::ofstream& PointCloudGraph::log()
{
  if(!PointCloudGraph::log_file.is_open())
  PointCloudGraph::log_file.open("logGraph2D.txt", std::ios::out |std::ios::app ); 

  return log_file; 
}


std::ofstream& PointCloudGraph::err()
{
  if(!PointCloudGraph::err_file.is_open())
  err_file.open("errGraph2D.txt", std::ios::out |std::ios::app ); 

  return err_file; 
}


