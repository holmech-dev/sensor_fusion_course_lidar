/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	// create insertHelper method, node double pointer so the pointer is pointing to root mem address on the tree
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{

		//if Tree is empty
		if(*node==NULL)
			// if NULL, dereference our node and set it new here
			// reassigning this node in the tree
			*node = new Node(point,id);
		else
		{

			//Calculate the current dimension
			// unsinged int for postive only depths 
			// we're using a 2D case so just do depth mod 2
			uint cd = depth % 2;

			// compare new x value to the current node x value
			// if less than, branch off to the left
			if(point[cd] < ((*node)->point[cd]))
				// call the insertHelper method, pass in the address with &
				// branch off to the dereferenced node left child
				// and increment depth by 1 and give it the point and id
				insertHelper(&((*node)->left), depth+1, point, id);
			else 
				// if not less than (greater than or equal to)
				// branch off to the right
				insertHelper(&((*node)->right), depth+1, point, id);

		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		// call the insertHelper recursive function (created by me also)
		// start at root with depth 0
		insertHelper(&root,0,point,id);

	}

	// create searchHelper method
	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		//
		if(node!=NULL)
		{
			// look at the region + and - for x (0) and do the same for y (1)
			// tell me if x y are in the distnanceTol box region
			if( node->point[0] >= target[0] - distanceTol && node->point[0] <= target[0] + distanceTol && node->point[1] >= target[1] - distanceTol && node->point[1] <= target[1] + distanceTol) {

				// if so then find the distance between node x y in the target x y region
				float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]));
				// if distnace is <= distance tolerance than push back the node onto the ids
				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			// now check which way we to flow through the tree, check box boundary
			// if the box left edge is less than the node x or y value
			// go the left node and then add one to the depth
			if((target[depth%2]-distanceTol) < node->point[depth%2])
				searchHelper(target,node->left,depth+1,distanceTol,ids);
			// if the box left edge is more than the node x or y value
			// go the right node and then add one to the depth
			if((target[depth%2]+distanceTol) > node->point[depth%2])
				searchHelper(target,node->right,depth+1,distanceTol,ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		// call the searchHelper method (created by me also)
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};