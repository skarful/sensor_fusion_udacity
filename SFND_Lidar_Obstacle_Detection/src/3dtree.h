//Basic class for 3D kd tree

//Copying template from kdtree.h


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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		inserthelper(&root,0, point, id);

	}

	void inserthelper(Node** rooter, uint depth, std::vector<float> point, int id){
		if(*rooter == NULL){
			*rooter = new Node(point, id);
		}
		else{
			uint d = depth %3;
			if(d == 0){
				if(point[0] < (*rooter)->point[0]){
					inserthelper(&((*rooter)->left), depth + 1, point, id);
				}
				else{
					inserthelper(&((*rooter)->right), depth +1, point, id);
				}
			}
			else if(d == 1){
				if(point[1] < (*rooter)->point[1]){
					inserthelper(&((*rooter)->left), depth + 1, point, id);
				}
				else{
					inserthelper(&((*rooter)->right), depth +1, point, id);
				}
			}
			else{
				if(point[2] < (*rooter)->point[2]){
					inserthelper(&((*rooter)->left), depth + 1, point, id);
				}
				else{
					inserthelper(&((*rooter)->right), depth +1, point, id);
				}
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		helpersearch(target, root, 0, distanceTol, ids);
		return ids;


	}

	void helpersearch(std::vector<float> target, Node* rooter, uint depth, float distanceTol, std::vector<int> &ids){

		if(rooter!= NULL){



			//If within limits
			if((rooter->point[0] >= target[0] - distanceTol) && (rooter->point[0] <= target[0] + distanceTol) &&
				(rooter->point[1] >= target[1] - distanceTol) && (rooter->point[1] <= target[1] + distanceTol) &&
				(rooter->point[2] >= target[2] - distanceTol) && (rooter->point[2] <= target[2] + distanceTol)){

				float distance  = sqrt((rooter->point[0] - target[0])*(rooter->point[0] - target[0]) + (rooter->point[1] - target[1])*(
					rooter->point[1] - target[1]) + (rooter->point[2] - target[2])*(rooter->point[2] - target[2]));

				// std::cout<<"Distance: "<< distance <<std::endl;
				if(distance < distanceTol){
					ids.push_back(rooter->id);
				}
			}

			if((target[depth%3]-distanceTol) < rooter->point[depth%3]){
				helpersearch(target, rooter->left, depth+1, distanceTol, ids);
			}
			if((target[depth%3]+distanceTol) > rooter->point[depth%3]){
				helpersearch(target, rooter->right, depth+1, distanceTol, ids);
			}

		}
	}
	

};