#pragma once
#include<iostream>
#include<vector>
#include<cmath>
#include "Tree.h"
#include "PointS.h"
#include "hourglass_operation.h"
#include "polygon_operation.h"
using namespace std;

int find_diagonal_index(int first, int second);

class SPTnode
{
	int vertexID; //the id of the vertex the node represents
	SPTnode* parent;
	vector<SPTnode*> children;

public:
	SPTnode()
	{
		vertexID = -1;
		parent = NULL;
		children = vector<SPTnode*>();
	}
	SPTnode(int point_num)
	{
		vertexID = point_num;
		parent = NULL;
		children = vector<SPTnode*>();
	}
	SPTnode(int _vertexID, SPTnode* _parent)
	{
		vertexID = _vertexID;
		parent = _parent;
		children = vector<SPTnode*>();
	}
	/* Searches all its descendents and returns a pointer to the SPT node with the corresponding vertex ID */
	SPTnode* find_node(int ID)
	{
		if (vertexID == ID)
			return this;
		else
		{
			for (int i = 0; i < children.size(); i++)
			{
				SPTnode* node= children[i]->find_node(ID);
				if (node != NULL)
					return node;					
			}
			return NULL;
		}
	}
	/* Returns a pointer to the parent node of the current node */
	SPTnode* get_parent()
	{
		return parent;
	}
	/* Adds the pointer to the leaf SPT node to in the children list of the current node */
	void add_child(SPTnode* new_child)
	{
		children.push_back(new_child);
	}
};



class SPT
{
	SPTnode* root;
	vector<int> components;	//maintains all the point IDs in the tree

public:
	SPT()
	{
		root = NULL;
		components = vector<int>();
	}
	SPT(int _root)
	{
		root = new SPTnode(_root, NULL);
		components.push_back(_root);
	}
	/*  Add the leaf as a member of the tree and link it to the parent node and return the vertex ID of the leaf
		If the parent is not part of the tree in the first place, return -1 */
	int set_pred(int leaf, int parent)
	{
		//if parent is not in the tree, then it's an error
		if (find(components.begin(), components.end(), parent) == components.end())
		{
			return -1;
		}
		
		if (find(components.begin(), components.end(), leaf) != components.end())
			return -1;
		//now we can be sure that the parent is a node in the tree!!
		
		//add the leaf to components list and set the parent
		
		SPTnode* parent_node = get_node(parent);
		SPTnode* new_leaf = new SPTnode(leaf,parent_node);
		parent_node->add_child(new_leaf);

		components.push_back(leaf);
		
		return leaf;
	}
	/* Returns the SPT node pointer with the vertex ID */
	SPTnode* get_node(int vertex)
	{
		if (find(components.begin(), components.end(), vertex) == components.end())
			return NULL; //NO CORRESPONDING NODE IN THE TREE SO FAR

		return root->find_node(vertex);	
	}
	/* Returns the SPT node pointer of the parent node of node in question*/
	SPTnode* get_pred(int vertex)
	{
		//find the parent of the node representing 'vertex' in the tree!
		SPTnode* node = get_node(vertex);

		//will return null for the root
		return node->get_parent();	
	}
	void set_root(int _root)
	{
		root = new SPTnode(_root);
		components.push_back(_root);
	}
	bool is_set(int id)
	{
		if (find(components.begin(), components.end(), id) == components.end())
			return false;
		else
			return true;
	}
};

SPT spt = SPT();


class Funnel
{
	int apex;	// apex's vertex id in point_list
	int diag;
	int alpha;	// an edge's endpoint id
	int beta;	//other point of the edge
	vector<int> vertex_list; //starts with alpha and ends with beta

public:
	Funnel()
	{
		apex = -1;
		diag = -1;
		alpha = -1;
		beta = -1;
	}
	Funnel(int _apex, int _diag)
	{
		apex = _apex;
		diag = _diag;
		if (_diag < diagonal_with_edge_list.size())
		{
			//treats the origin of the diagonal as alpha
			alpha = diagonal_with_edge_list[_diag].get_origin();
			beta = diagonal_with_edge_list[_diag].get_dest();
			vertex_list.push_back(alpha);
			vertex_list.push_back(apex);
			vertex_list.push_back(beta);
		}
		else//error case
		{
			alpha = -1;
			beta = -1;
		}
	}
	Funnel(int _apex, int _alpha, int _beta)
	{
		//diag = -1;
		apex = _apex;
		alpha = _alpha;
		beta = _beta;
		//need to set the diagonal number if it exists!!
		diag = find_diagonal_index(_alpha, _beta);
		vertex_list.push_back(_alpha);
		vertex_list.push_back(apex);
		vertex_list.push_back(_beta);
	}
	Funnel(int _apex, vector<int> _vertex_list)
	{
		apex = _apex;
		alpha = _vertex_list.front();
		beta = _vertex_list.back();
		diag = find_diagonal_index(alpha, beta);
		vertex_list = _vertex_list;
	}
	int get_apex() { return apex; }
	int get_alpha() { return alpha; }
	int get_beta() { return beta; }
	int get_diag() { return diag; }
	vector<int> get_vertex_list() { return vertex_list; }
};

void find_shortest_path_tree(int);
void split_funnel(Funnel*);

/*constructs the shortest path tree 'SPT' with root as 's'*/
void find_shortest_path_tree(int s)
{
	Point root = point_list[s];//probably index point_list.size()-2
	spt.set_root(s);
	int triangle_with_s_id = point_state.find_triangle(root);

	vector<int> triangle_with_s = polygon_list[triangle_with_s_id];

	//set predecessor of each vertex!
	spt.set_pred(triangle_with_s[0], s);
	spt.set_pred(triangle_with_s[1], s);
	spt.set_pred(triangle_with_s[2], s);

	//divide the triangle into three
	for (int i = 0; i < 3; i++)
	{
		Funnel* temp = new Funnel(s, triangle_with_s[i % 3], triangle_with_s[(i + 1) % 3]);
		//call split
		split_funnel(temp);
	}

	printf("we have the spt set\n");
}

int choose_v(Funnel* funnel)
{
	int alpha = funnel->get_alpha();
	int beta = funnel->get_beta();
	int diag = funnel->get_diag();

	if (diag == -1)
		return -1;

	//array of two indexes of triangles in polygon_list that are adjacent to the diagonal
	int* triangles = diagonal_list[diag].get_triangle();
	int v = -1;
	if (triangles[0] == 1 || triangles[1] == 1)
		printf("gotcha\n");
	for (int i = 0; i < 2; i++)
	{
		int v_cand;
		for (int j = 0; j < 3; j++)
		{
			int vertex = polygon_list[triangles[i]][j];
			if (vertex != alpha && vertex != beta)
			{
				v_cand = vertex;
				break;
			}
		}
		
		if (spt.is_set(v_cand) == false)
		{
			v = v_cand;
			break;
		}
	}

	return v;
}

/* returns true iff vector (curr, v) lies between vectors (prev, curr) and (curr, next)
(the smaller side of the pie)
*/
bool in_between(int prev, int curr, int next, int v)
{
	float prev_curr = calculate_angle(prev, curr);
	float curr_next = calculate_angle(curr, next);
	float curr_v = calculate_angle(curr, v);

	float diff_first = calculate_angle_between(curr, next, v);
	float diff_second = curr_v - prev_curr;
	
	if (diff_second > PI)
		diff_second -= 2 * PI;
	else if (diff_second < -PI)
		diff_second += 2 * PI;

	if (diff_first*diff_second >= 0)
		return true;
	else return false;
}

int compute_pred(Funnel* funnel, int v)
{
	int apex = funnel->get_apex();
	int pred;
	vector<int> vertex_list = funnel->get_vertex_list();

	if (vertex_list.empty())
	{
		printf("funnel's vertex list should not be empty!\n");
		exit(234);
	}
	else if (vertex_list.size() == 2)
		return apex;

	vector<int>::iterator apex_ptr = find(vertex_list.begin(), vertex_list.end(), apex);
	vector<int> alpha_list, beta_list;

	//always start with alpha!!
	alpha_list.insert(alpha_list.end(), vertex_list.begin(), apex_ptr);
	alpha_list.push_back(apex);
	reverse(alpha_list.begin(), alpha_list.end());
	beta_list.insert(beta_list.end(), apex_ptr, vertex_list.end());

	float angle_alpha = calculate_angle_between(apex, v, alpha_list[1]);
	float angle_beta = calculate_angle_between(apex, v, beta_list[1]);

	//the case that the apex is the predecessor -> different signature (or zero)
	if (angle_alpha*angle_beta <= 0)
		return apex;
	
	//not the case that the apex is the predecessor
	vector<int> chain;

	//select alpha / beta so that the chain is closer to v
	chain = (abs(angle_alpha) > abs(angle_beta)) ? beta_list : alpha_list;

	vector<int>::iterator predecessor = chain.begin();
	predecessor++;

	while ((predecessor + 1) != chain.end())
	{
		if (in_between(*(predecessor-1), *predecessor, *(predecessor + 1), v))
			return *(predecessor);
		predecessor++;
	}
	
	//when the predecessor is the last element
	return *predecessor;
}


void split_funnel(Funnel* funnel)
{
	int alpha = funnel->get_alpha();
	int beta = funnel->get_beta();
	int apex = funnel->get_apex();

	//if alpha and beta are adjacent numbers (meaning the diagonal is a polygon edge), return
	if (abs(alpha - beta) == 1 || abs(alpha - beta) == (v_num - 1))
		return;

	int v = choose_v(funnel);
	if (v == -1)//cannot choose v
		return;
	int pred = compute_pred(funnel, v);
	//add pred info to tree
	spt.set_pred(v, pred);

	vector<int> chain = funnel->get_vertex_list();
	vector<int>::iterator pred_ptr = find(chain.begin(), chain.end(), pred);
	vector<int> first_chain , second_chain;

	if (find(chain.begin(), pred_ptr, apex) != pred_ptr)
	{
		reverse(chain.begin(), chain.end());
		pred_ptr = find(chain.begin(), chain.end(), pred);
	}
	first_chain.insert(first_chain.end(), chain.begin(), pred_ptr);
	first_chain.push_back(pred);
	first_chain.push_back(v);
	second_chain.push_back(v);
	second_chain.insert(second_chain.end(), pred_ptr, chain.end());

	Funnel* first = new Funnel(pred, first_chain);
	Funnel* second = new Funnel(apex, second_chain);

	if (pred == alpha || pred == beta)
	{		
		if (first_chain.size() == 2)//second_chain should have the origin apex
		{
			first = new Funnel(pred, first_chain);
			second = new Funnel(apex, second_chain);
		}
		else if (second_chain.size() == 2)//first_chain should have the original apex
		{
			first = new Funnel(apex, first_chain);
			second = new Funnel(pred, second_chain);
		}
		else {
			printf("this doesn't make sense\n");
			exit(-10);
		}
	}

	split_funnel(first);
	split_funnel(second);
	return;
}

int find_diagonal_index(int first, int second)
{
	for (int i = 0; i < diagonal_list.size(); i++) {
		if (diagonal_list[i].check_same_edge(first, second))
			return i;
	}

	return -1;
}
