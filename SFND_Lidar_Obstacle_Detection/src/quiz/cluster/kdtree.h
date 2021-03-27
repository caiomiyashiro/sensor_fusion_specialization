/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point; // [0] is x dimension and [1] is y dimension
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		Node **eval_node = &root;
		uint level = 0;
		// std::cout << "------------------ point = " << point[0] << ", " << point[1] << std::endl;
		while(*eval_node != NULL){
			// std::cout << "------ eval_node = " << (*eval_node)->point[0] << ", " << (*eval_node)->point[1] << std::endl;
			float compared_value, node_value;
			// std::cout << "level is " << level << std::endl;
			if(level % 2 == 0){				
				compared_value = point[0];
				node_value = (*eval_node)->point[0];
			}else{
				compared_value = point[1];
				node_value = (*eval_node)->point[1];
			}
			level++;
			// std::cout << "compared_value " << compared_value << " node_value " << node_value << std::endl;
			if(compared_value < node_value){
				// std::cout << "if" << std::endl;
				eval_node = &(*eval_node)->left;
			}else{
				// std::cout << "else" << std::endl;
				eval_node = &(*eval_node)->right;
			}
		}
		*eval_node = new Node(point, id);

	}

	bool point_is_inside_box(std::vector<float> target, std::vector<float> node, float distanceTol){
		if(node[0] > (target[0]-distanceTol) && node[0] < (target[0]+distanceTol) && node[1] > (target[1]-distanceTol) && node[1] < (target[1]+distanceTol)){
			return true;
		}else{
			return false;
		}
	}

	void search_helper(std::vector<float> target, float distanceTol, int depth, Node *node, std::vector<int>& ids){
		if(node != NULL){
			if(point_is_inside_box(target, node->point, distanceTol)){
				ids.push_back(node->id);
			}
			
			if(target[depth%2] + distanceTol > node->point[depth%2]){
				search_helper(target, distanceTol, depth+1, node->right, ids);
			}
			if(target[depth%2] - distanceTol < node->point[depth%2]){
				search_helper(target, distanceTol, depth+1, node->left, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, distanceTol, 0, root, ids);
		return ids;
	}
	

};




