#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    //Initializing the start and end node based on the coordinates.
	start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return end_node->distance(*node);
}

//Add unvisited neighbors to open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
  	for (RouteModel::Node* node : current_node->neighbors)
    {
      	if (node->visited)
       		continue;  
      
  		node->parent = current_node;
      	node->h_value = CalculateHValue(node);
      	node->g_value = current_node->g_value + current_node->distance(*node);
      	node->visited = true;
      	open_list.push_back(node);
      
    }
  
}

//Compare function using f value which can be used as a functor.
bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
  float f1 = a->g_value + a->h_value;
  float f2 = b->g_value + b->h_value;
  return f1 > f2; 
}

//This function guarantees next node returned is the closest node.
RouteModel::Node *RoutePlanner::NextNode() {
  	if ( open_list.size() > 0)
    {
		sort(open_list.begin(), open_list.end(), Compare);
      	RouteModel::Node* closestNode = open_list.back();
      	open_list.pop_back();
      	return closestNode;
	}
	
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found = {};

	while ( current_node != nullptr)
    {
      	//Node to be inserted at start of vector as we are traversing from end to start.
      	std::vector<RouteModel::Node>::iterator it = path_found.begin();
		path_found.insert(it, *current_node); 
      	if (current_node->parent)
      		distance += current_node->distance(*current_node->parent);
      	current_node = current_node->parent;
    }
  	 // Multiply the distance by the scale of the map to get meters.
  	distance *= m_Model.MetricScale();
    return path_found;
  
}

void RoutePlanner::AStarSearch() { 
  
    RouteModel::Node *current_node = nullptr;
  
  	current_node = start_node;
    current_node->h_value = CalculateHValue(current_node);
    current_node->g_value = 0;
    current_node->visited = true;
  	current_node->parent = nullptr;
  	open_list.push_back(current_node);
  
  	while (open_list.size() > 0)
    {
    	current_node = NextNode();
      	if (current_node == end_node) //Found the destination node.
        {
      		break;
        }
      	AddNeighbors(current_node);
    }

    m_Model.path = ConstructFinalPath(current_node);
  	
}