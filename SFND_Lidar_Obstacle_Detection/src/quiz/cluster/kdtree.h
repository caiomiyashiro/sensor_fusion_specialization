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

	void search_helper(std::vector<float> target, float distanceTol, int depth, Node *node, std::vector<int>& ids){
		if(node != NULL){
			if(node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol) && node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)){
				float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]));
				if(distance <= distanceTol){
					ids.push_back(node->id);
				}
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
		// auto startTime = std::chrono::steady_clock::now();
		std::vector<int> ids;
		search_helper(target, distanceTol, 0, root, ids);
		// auto endTime = std::chrono::steady_clock::now();
		// auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		// std::cout << " Tree Search Took " << elapsedTime.count() << " milliseconds" << std::endl;
		return ids;
	}
	

};

void cluster_helper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol){
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree-> search(points[indice], distanceTol);
	for(int id: nearest){
		if(!processed[id]){
			cluster_helper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while(i < points.size()){
		// auto startTime2 = std::chrono::steady_clock::now();
		if(processed[i]){
			i++;
			continue;
		}
		std::vector<int> cluster;
		cluster_helper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
		// auto endTime2 = std::chrono::steady_clock::now();
		// auto elapsedTime2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2);
		// std::cout << " 1 while cycle was " << elapsedTime2.count() << " milliseconds" << std::endl;
	}

	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << " Clustering (Including TreeSearch) took " << elapsedTime.count() << " milliseconds" << std::endl;

	return clusters;

};

// 1st attempt - see euclideanClusterCustom
// bool point_is_inside_box(std::vector<float> target, std::vector<float> node, float distanceTol){
// 	if(node[0] > (target[0]-distanceTol) && node[0] < (target[0]+distanceTol) && node[1] > (target[1]-distanceTol) && node[1] < (target[1]+distanceTol)){
// 		return true;
// 	}else{
// 		return false;
// 	}
// }

// void search_helper_custom(std::vector<float> target, float distanceTol, int depth, Node *node, std::vector<int>& ids){
// 	if(node != NULL){
// 		if(point_is_inside_box(target, node->point, distanceTol)){
// 			ids.push_back(node->id);
// 		}
		
// 		if(target[depth%2] + distanceTol > node->point[depth%2]){
// 			search_helper_custom(target, distanceTol, depth+1, node->right, ids);
// 		}
// 		if(target[depth%2] - distanceTol < node->point[depth%2]){
// 			search_helper_custom(target, distanceTol, depth+1, node->left, ids);
// 		}
// 	}
// }


// 1st attempt - it creates lots of small clusters
// std::vector<std::vector<int>> euclideanClusterCustom(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
// {

// 	// TODO: Fill out this function to return list of indices for each cluster
// 	std::vector<std::vector<int>> clusters;
// 	std::set<int> already_processed;

// 	int yet_to_process_counter = 0;
// 	while(yet_to_process_counter < points.size()){
// 		int eval_id = yet_to_process_counter;
// 		std::vector<float> eval_point = points[yet_to_process_counter];
// 		// only process if id hasn't been processed yet
// 		if(already_processed.find(eval_id) == already_processed.end() || already_processed.size() == 0 ){  
// 			std::vector<int> cluster_pre_filtering, cluster;

// 			// retrieve all close points
// 			cluster_pre_filtering = tree->search(eval_point, distanceTol);
// 			// for all the found points, create cluster with points that were not processed yet.
// 			std::vector<int>::iterator itr; 
// 			for (itr = cluster_pre_filtering.begin(); itr != cluster_pre_filtering.end(); itr++)
// 			{
// 				if(already_processed.find(*itr) == already_processed.end() || already_processed.size() == 0){
// 					cluster.push_back(*itr);
// 					already_processed.insert(*itr);
// 				}
// 			}
// 			clusters.push_back(cluster);
// 			already_processed.insert(eval_id);
// 		}
// 		yet_to_process_counter++;
// 	}
 
// 	return clusters;

// };




