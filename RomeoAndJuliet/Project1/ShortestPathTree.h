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


int compute_pred(Funnel* funnel, int v);

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
	vector<int> find_node_save_path(vector<int> path, int ID)
	{
		path.push_back(vertexID);
		if (vertexID == ID)
		{
			return path;
		}
		else
		{
			vector<int> child;
			//if all the children have no match, return an empty vector
			for (int i = 0; i < children.size(); i++)
			{
				child = children[i]->find_node_save_path(path, ID);
				if (!child.empty())
				{
					return child;
				}
			}

			return vector<int>();
		}
		//check if current node
		//if true, return path so far
		//if not, add current nod to the path list and invoke on children

	}
	int get_id()
	{
		return vertexID;
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
	int s;
	int t;
	int t_tri_num;
	vector<int> t_tri_vertices;
	int t_pred;
public:
	SPT()
	{
		root = NULL;
		components = vector<int>();
	}
	SPT(int _s,int _t) //must be called after decomposition is complete
	{
		root = new SPTnode(_s, NULL);
		components.push_back(_s);
		t = _t;
		s = _s;
		Point point_t = point_list[t];
		t_tri_num = point_state.find_triangle(point_t);
		t_tri_vertices = polygon_list[t_tri_num];
		t_pred = -1;
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
	bool get_size() {
		return components.size();
	}
	bool compute_shortest_path_tree()
	{
		//Point root = point_list[s];//probably index point_list.size()-2
		//set_root(s);
		int triangle_with_s_id = point_state.find_triangle(point_list[s]);
		vector<int> triangle_with_s = polygon_list[triangle_with_s_id];

		//set predecessor of each vertex!
		set_pred(triangle_with_s[0], s);
		set_pred(triangle_with_s[1], s);
		set_pred(triangle_with_s[2], s);

		//divide the triangle into three
		for (int i = 0; i < 3; i++)
		{
			Funnel* temp = new Funnel(s, triangle_with_s[i % 3], triangle_with_s[(i + 1) % 3]);
			//call split
			split_funnel(temp);
		}

		return components.size();
	}
	vector<int> find_shortest_path_default()
	{
		return find_shortest_path(t);
	}
	vector<int> find_shortest_path(int _t)
	{
		compute_shortest_path_tree();
		vector<int> path;

		if (is_set(_t)) // t is a polygon vertex, not a test point
		{
			path = root->find_node_save_path(path, _t);
		}
		else
		{
			printf("no such t in the shortest path tree\n");
			exit(39);
		}
		/*
		else {//the destination test point
			if (t_pred == -1)
			{
				printf("predecessor of t was not successfully computed! Check the `choose_v function\n");
				exit(27);
			}
			else
			{
				path = root->find_node_save_path(path, t_pred);
				path.push_back(t);
			}
		}*/

		//basic validity check
		if (path.front() == s && path.back() == _t)
			return path;
		else
		{
			printf("error! not a valid shortest path\n");
			exit(103);
		}

		/*
		vector<int> path;

		if (is_set(t))
		{
			path = root->find_node_save_path(path, t);
		}
		else //the second test point input
		{
			//find the triangle t is in
			Point dest = point_list[t];
			int triangle_num = point_state.find_triangle(dest);
			vector<int> tri_vertex = polygon_list[triangle_num];
			

			//find the closest to the root......

			//push_back t

		}
		
		if (path.front() == root->get_id() && path.back() == t)
			return path;
		else
		{
			printf("what is this\n");
			return vector<int>();
		}*/
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

		/*
		if (find(t_tri_vertices.begin(), t_tri_vertices.end(), alpha) != t_tri_vertices.end() &&
			find(t_tri_vertices.begin(), t_tri_vertices.end(), beta) != t_tri_vertices.end() &&
			find(t_tri_vertices.begin(), t_tri_vertices.end(), v) != t_tri_vertices.end())
		{
			t_pred = compute_pred(funnel, t);
		}*/

		int pred = compute_pred(funnel, v);
		//add pred info to tree
		set_pred(v, pred);

		vector<int> chain = funnel->get_vertex_list();
		vector<int>::iterator pred_ptr = find(chain.begin(), chain.end(), pred);
		vector<int> first_chain, second_chain;

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
	/* chooses the next vertex to expand 
		also calculates the predecessor of t if possible*/
	int choose_v(Funnel* funnel)
	{
		int alpha = funnel->get_alpha();
		int beta = funnel->get_beta();
		int diag = funnel->get_diag();

		if (diag == -1)
			return -1;

		//array of two indexes of triangles in polygon_list that are adjacent to the diagonal
		int* triangles = diagonal_list[diag].get_triangle();
		int selected_tri_index = -1;
		int v = -1;
		if (triangles[0] == 1 || triangles[1] == 1)
			printf("gotcha\n");
		for (int i = 0; i < 2; i++)
		{
			int v_cand;
			selected_tri_index = i;
			for (int j = 0; j < 3; j++)
			{
				int vertex = polygon_list[triangles[i]][j];
				if (vertex != alpha && vertex != beta)
				{
					v_cand = vertex;
					break;
				}
			}

			if (is_set(v_cand) == false)
			{
				v = v_cand;
				break;
			}
		}

		if (triangles[selected_tri_index] == t_tri_num)
		{
			t_pred = compute_pred(funnel, t);
			set_pred(t, t_pred);
		}

		return v;
	}
};



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

int find_diagonal_index(int first, int second)
{
	for (int i = 0; i < diagonal_list.size(); i++) {
		if (diagonal_list[i].check_same_edge(first, second))
			return i;
	}

	return -1;
}
