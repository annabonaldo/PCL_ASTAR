#include "Astar.h"
std::ofstream Astar::log_file; 
std::ofstream Astar::err_file; 
Astar::Astar(void){}

std::list<int> Astar::Compute(PointCloudGraph & Graph )
{
  int start_i =  Graph.Start();
  bool goal_found = false;
  _F = Graph.GetFloatArrayOfGraphSize(-1.0); 
  log()<< "START "<<Graph.Start() << ", GOAL "<<Graph.Goal()<<std::endl; 
  addToOpenQueue(Graph.Start(), Graph.H(Graph.Start())); 
  setNodeParent(Graph.Start(), -1); 

  while((!_OPEN_QUEUE_h_id.empty()) && (!goal_found))
  {
    printQueue(); 
    
    std::pair<float, int> cur_node =*_OPEN_QUEUE_h_id.begin();
    int node = cur_node.second; // GET NODE ID
    float node_H = cur_node.first; // GET NODE HEURITSICS FUNCTION VALUE
    removeNodeFromOpenQueue(node);
    addToClosed(node); 

    //GENERATE ALL NODE NEIGHBOURS
    std::set<int> neighbours = Graph.GetNodeNeighbours(node);
    std::set<int>::iterator n_it = neighbours.begin(); 
    n_it = neighbours.begin(); 
    for(; n_it != neighbours.end() && (!goal_found); n_it++)
    {
      if(Graph.Goal() == *n_it)
      {  
        setNodeParent(*n_it, node); 
        goal_found = true; 
        log()<< "Goal found on NODE: "<< *n_it<<std::endl; 
      }
      else
      {
        float successor_node_G = Graph.G(node) + Graph.GetDistance(node, *n_it); 
        float successor_node_F = Graph.H(*n_it) + successor_node_G; 

        // if current n_it NOT in open nodes
        if(isOpenNode(*n_it) && isCurrentEvaluationBetterThanTheStoredOne(*n_it,successor_node_F))
        {
          removeNodeFromOpenQueue(*n_it);
          addToOpenQueue(*n_it, successor_node_F); 
          setNodeParent(*n_it, node); 
        }
        // if current n_it NOT in closed nodes
        else if(isClosedNode(*n_it) && isCurrentEvaluationBetterThanTheStoredOne(*n_it,successor_node_F))
        {
         removeNodeFromClosed (*n_it); 
         addToOpenQueue(*n_it, successor_node_F); 
         setNodeParent(*n_it, node); 
        }
        else if((!isOpenNode(*n_it)) && (!isClosedNode(*n_it)))
        {
          addToOpenQueue(*n_it, successor_node_F); 
          setNodeParent(*n_it, node); 
        }
        

      }

      //OPEN = priority queue containing START
      //  CLOSED = empty set
      //  while lowest rank in OPEN is not the GOAL:
      //  current = remove lowest rank item from OPEN
      //  add current to CLOSED
      //  for neighbors of current:
      //cost = g(current) + movementcost(current, neighbor)
      //  if neighbor in OPEN and cost less than g(neighbor):
      //remove neighbor from OPEN, because new path is better
      //  if neighbor in CLOSED and cost less than g(neighbor): ⁽²⁾
      //    remove neighbor from CLOSED
      //    if neighbor not in OPEN and neighbor not in CLOSED:
      //set g(neighbor) to cost
      //  add neighbor to OPEN
      //  set priority queue rank to g(neighbor) + h(neighbor)
      //  set neighbor's parent to current
    }

  }
  if(goal_found)
    return getPath(Graph.Start(), Graph.Goal()); 
  else 
    return std::list<int>(); 
}

std::list<int> Astar::getPath(int start, int goal)
{
  std::list<int> path; 
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
   path.push_front(start); 
  log()<<"PATH: "; 
  for(std::list<int>::iterator it = path.begin(); it!=path.end(); it++)
    log()<<*it <<" "; 
  log()<<" ||"<<std::endl; 

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
    if(_F[node] >eval) is_current_better = true; 
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
  else err()<< "Not erasable node "<<endl;  
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
  log_file.open("logAstar.txt", ios::out |ios::app ); 

  return log_file; 
}

std::ofstream& Astar::err()
{
  if(!err_file.is_open())
  err_file.open("errAstar.txt", ios::out |ios::app ); 

  return err_file; 
}

void Astar::printQueue()
{
  log()<< "QUEUE:{{"; 
  for(std::set<std::pair<float, int>>::iterator it = this->_OPEN_QUEUE_h_id.begin();it != _OPEN_QUEUE_h_id.end(); it++)
    log()<< "- ("<< (*it).first << "," <<(*it).second <<")"; 
  log()<<" }} "<<std::endl; 
}
