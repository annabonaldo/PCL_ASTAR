#include "Astar.h"
#include "IOmanager.h"
#include <pcl/common/time.h>
std::ofstream Astar::log_file; 
std::ofstream Astar::err_file; 
bool Astar::log_on = true; 
int Astar::computation_iter = 0; 

Astar::Astar(void)
{
  //log_on = (computation_iter > 0); 
}

std::list<int> Astar::Compute(PointCloudGraph & Graph )
{
    int itr = 0; 
 // if(log_on) 
    log() <<"Astar computation n " << (computation_iter + 1)<<".... "<<std::endl; 

  if(Graph.Start() == Graph.Goal())
  { 
    if(log_on) log()<< "START== GOAL!!! START "<<Graph.Start() << ", GOAL "<<Graph.Goal()<<std::endl; 
     return std::list<int>(); 
  }


  pcl::StopWatch timer; 

  int start_i =  Graph.Start();
  _goal_found = false;
  _F.resize(Graph.GetGraphSize()); 
  for(int i= 0; i<_F.size(); i++)
    _F[i] = 0.0F; 

  if(log_on) log()<< " START node: "<<Graph.Start() << ", GOAL node:"<<Graph.Goal()<<std::endl; 
  addToOpenQueue(Graph.Start(), Graph.H(Graph.Start())); 
  setNodeParent(Graph.Start(), -1); 


  // WHILE QUEUE NOT EMPT
  while((!_OPEN_QUEUE_h_id.empty()) && (!_goal_found))
  {
    itr++; 
    printQueue(); 
    stats.queue_mean_size += _OPEN_QUEUE_h_id.size();
    
    std::pair<float, int> cur_node =*_OPEN_QUEUE_h_id.begin();
    int node = cur_node.second; // GET NODE ID
    float node_H = cur_node.first; // GET NODE HEURITSICS FUNCTION VALUE
    removeNodeFromOpenQueue(node);
    addToClosed(node); 

    //GENERATE ALL NODE NEIGHBOURS
    std::set<int> neighbours = Graph.GetNodeNeighbours(node);
    std::set<int>::iterator n_it = neighbours.begin(); 
    n_it = neighbours.begin(); 

    // NEIGHBOURS ITER
    for(; n_it != neighbours.end() && (!_goal_found); n_it++)
    {
      if(Graph.Goal() == *n_it)  
      {  
        setNodeParent(*n_it, node); 
        _goal_found = true; 
        if(log_on)  log()<< "Goal found on NODE: "<< *n_it<<" Distance: " << _F[node] + Graph.GetDistance(node, *n_it) <<std::endl; 
      }
      else // IS NOT GOAL 
      {
        if(log_on)  log()<< "NODE: "<< *n_it<<" Distance: " << _F[node]<< " "<< Graph.GetDistance(node, *n_it) <<std::endl; 

        float successor_node_F = getCost(Graph, node, *n_it); 

        // if current n_it NOT in open nodes
        if(isOpenNode(*n_it) && isCurrentEvaluationBetterThanTheStoredOne(*n_it,successor_node_F))
        {
          if(this->computation_iter> 0) log() << " better eval on OPEN node "<< std::endl; 
          removeNodeFromOpenQueue(*n_it);
          addToOpenQueue(*n_it, successor_node_F); 
          setNodeParent(*n_it, node);  
        }
        // if current n_it NOT in closed nodes
        else if(isClosedNode(*n_it) && isCurrentEvaluationBetterThanTheStoredOne(*n_it,successor_node_F))
        {
          if(this->computation_iter> 0) log() << " better eval on CLOSED node "<< std::endl; 
         removeNodeFromClosed (*n_it); 
         addToOpenQueue(*n_it, successor_node_F); 
         setNodeParent(*n_it, node);
        }
        else if((!isOpenNode(*n_it)) && (!isClosedNode(*n_it)))
        {
          addToOpenQueue(*n_it, successor_node_F); 
          setNodeParent(*n_it, node); 
        }
      }//end "IS NOT GOAL"

    }// END NEIGHBOURS ITER

  }// END WHILE QUEUE NOT EMPTY

  stats.execution_time = timer.getTime(); 
  log()<<"explored nodes: " << itr <<std::endl;
  saveResults(Graph); 
  return getPath(Graph.Start(), Graph.Goal()); 

}

std::list<int> Astar::getPath(int start, int goal)
{
  std::list<int> path; 
  if(_goal_found)
  {
    int child = goal; 
    int step =  0; 
    int max_step = _F.size(); 
    while(this->_child_parent_map.find(child)!= _child_parent_map.end() || (child == start)||(step > max_step))
    {
      step++; 
      path.push_front(child); 
      int parent = _child_parent_map.find(child)->second; 
      _child_parent_map.erase(child); 
      child = parent; 
    }

      if(log_on) log()<<"PATH: "; 
    for(std::list<int>::iterator it = path.begin(); it!=path.end(); it++)
      if(log_on) log()<<*it <<" "; 
    if(log_on) log()<<" ||"<<std::endl; 
  }
  else  if(log_on) log()<<" goal not -found "<< std::endl; 

  stats.path_lenght = path.size(); 

 if(log_on) log()<<"A star end ---------------------------------"<< std::endl<<std::endl; 
  return path; 
}

bool Astar::isOpenNode(int node)
{
  return (_OPEN_id_h.find(node) != _OPEN_id_h.end()); 
}

bool Astar::isClosedNode(int node)
{
   return (_CLOSED.find(node) != _CLOSED.end()); 
}

bool Astar::isCurrentEvaluationBetterThanTheStoredOne(int node, float eval)
{
  std::map<int, float> ::iterator old_value = _OPEN_id_h.find(node); 
  bool is_current_better = false; 

  if(old_value != _OPEN_id_h.end())
  {
    float old_evaluation = old_value->second; 
    if(eval < old_evaluation) 
      is_current_better =  true;
  }
  else
  {
    if((_F[node]>0.0F )&&(_F[node] > eval)) is_current_better = true; 
  }

  return is_current_better; 
}

void Astar::removeNodeFromOpenQueue(int node)
{
  std::map<int, float>::iterator id_to_h = _OPEN_id_h.find(node); // get node-id-key map element
  
  if(id_to_h != _OPEN_id_h.end())
  {
    std::set<std::pair<float, int>>::iterator h_to_id = _OPEN_QUEUE_h_id.find(std::pair<float, int>(id_to_h->second, node));  // get current enqueued (eval, node) pair
    if(h_to_id != _OPEN_QUEUE_h_id.end())
      _OPEN_QUEUE_h_id.erase(h_to_id); // remove it from queue 
    _OPEN_id_h.erase(id_to_h); 
  }
}

void Astar::removeNodeFromClosed(int node)
{
  if(_CLOSED.find(node) != _CLOSED.end())
     _CLOSED.erase(node); 
  else err()<< "Not erasable node "<<std::endl;  
}

void Astar::addToOpenQueue(int node, float eval)
{
  if (_OPEN_QUEUE_h_id.find(std::pair<float, int>(eval, node)) != _OPEN_QUEUE_h_id.end())
    _OPEN_QUEUE_h_id.erase(std::pair<float, int>(eval, node)); 

   if (_OPEN_id_h.find(node)!= _OPEN_id_h.end())
    _OPEN_id_h.erase(node); 
  
  _OPEN_QUEUE_h_id.insert(std::pair<float, int>(eval, node));  // add the eval-node pair to ordered queue
  _OPEN_id_h.insert(std::pair<int, float>(node, eval)); // add the node-id-key map element eval value
  _F[node] = eval; 
  log()<< _F[node] << " eval "<<std::endl; 
}

void Astar::addToClosed(int node)
{
  if(!isClosedNode(node))
    _CLOSED.insert(node); 
}

void Astar::setNodeParent(int c, int p)
{
  if(_child_parent_map.find(c) != _child_parent_map.end())
  { 
    this->_child_parent_map.find(c)->second = p; 
  }
  else
  {
    _child_parent_map.insert(std::pair<int, int>(c, p)); 
  }
}

std::ofstream& Astar::log()
{
  if(!log_file.is_open())
  log_file.open("logAstar.txt", std::ios::out |std::ios::app ); 

  return log_file; 
}

std::ofstream& Astar::err()
{
  if(!err_file.is_open())
  err_file.open("errAstar.txt", std::ios::out |std::ios::app ); 

  return err_file; 
}

void Astar::printQueue()
{
  log()<< "QUEUE:{{"; 
  for(std::set<std::pair<float, int>>::iterator it = this->_OPEN_QUEUE_h_id.begin();it != _OPEN_QUEUE_h_id.end(); it++)
    log()<< "- ("<< (*it).first << "," <<(*it).second <<")"; 
  log()<<" }} "<<std::endl; 
}

void Astar::saveResults(PointCloudGraph & Graph)
{

  if(Params::PREPROC) 
    Graph.SaveCostsOnGraph(this->_F); 

  stats.itr = 0; 
  stats.n_closed_nodes = _CLOSED.size(); 
  stats.n_open_nodes   = _OPEN_QUEUE_h_id.size(); 
  stats.start_goal_distance =  Graph.GetDistance(Graph.Start(), Graph.Goal()); 
  stats.cloud_size = Graph.GetGraphSize(); 
  stats.queue_mean_size = stats.queue_mean_size / stats.itr; 
  stats.logHeaders(); 
  stats.log(); 

  computation_iter++; 
}

float Astar::getCost(PointCloudGraph & Graph, int node, int succ)
{
  float successor_node_F = 0.0F; 
  if(_F[node] <= 0.0F )_F[node] = 0.0F; 

  if(Params::PREPROC && this->computation_iter > 0)
  {
    successor_node_F =  Graph.H(succ);
    if(successor_node_F <= 0.0F ) successor_node_F = 0.0F; 
  }
  else
  {

      float successor_node_G = _F[node] + Graph.GetDistance(node, succ); 
      successor_node_F = Graph.H(succ) + successor_node_G; 
  }

  return successor_node_F;
}