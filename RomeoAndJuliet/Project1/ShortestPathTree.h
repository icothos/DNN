#pragma once
#include<iostream>
#include<vector>
#include<cmath>
#include "Tree.h"
#include "PointS.h"

//using namespace std;


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
	SPTnode(int _vertexID, SPTnode* _parent)
	{
		vertexID = _vertexID;
		parent = _parent;
		children = vector<SPTnode*>();
	}
	SPTnode(int _vertexID, int _parent)
	{
		vertexID = _vertexID;
		parent = find_node(_parent);
		children = vector<SPTnode*>();
	}
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
	SPTnode* get_parent()
	{
		return parent;
	}
	SPTnode* add_child(SPTnode* new_child)
	{
		children.push_back(new_child);
	}
};
class SPT
{
	SPTnode* root;
	vector<int> components;

public:
	SPT()
	{
		root = NULL;
		components = vector<int>();
	}
	SPT(int root)
	{
		SPTnode root = SPTnode(root, NULL);
		components.push_back(root);
	}
	int set_pred(int leaf, int parent)
	{
		//if parent is not in the tree, then it's an error
		if (find(components.begin(), components.end(), parent) == components.end())
		{
			return -1;
		}
		
		//now we can be sure that the parent is a node in the tree!!
		
		//add the leaf to components list and set the parent
		SPTnode new_leaf = SPTnode(leaf, parent);
		new_leaf.get_parent()->add_child(&new_leaf);
	}
	SPTnode* get_node(int vertex)
	{
		if (find(components.begin(), components.end(), vertex) == components.end())
			return NULL; //NO CORRESPONDING NODE IN THE TREE SO FAR

		return root->find_node(vertex);	
	}
	SPTnode* get_pred(int vertex)
	{
		//find the parent of the node representing 'vertex' in the tree!
		SPTnode* node = get_node(vertex);
		return node->get_parent();	
	}
};


class Funnel
{
	int apex;	// apex's vertex id in point_list
	int alpha;	// an edge's endpoint id
	int beta;	//other point of the edge
	vector<int> vertex_list; //starts with alpha and ends with beta

public:
	Funnel()
	{
		apex = -1;
		alpha = -1;
		beta = -1;
	}
	Funnel(int _apex, int _alpha, int _beta)
	{
		apex = _apex;
		alpha = _alpha;
		beta = _beta;
	}
	int get_apex() { return apex; }
	int get_alpha() { return alpha; }
	int get_beta() { return beta; }
	vector<int> get_vertex_list() { return vertex_list; }
};

SPT spt = SPT();

void find_shortest_path_tree(int);
void split_funnel(Funnel);

/*constructs the shortest path tree 'SPT' with root as 's'*/
void find_shortest_path_tree(int s)
{
	//find the triangle that s is included in and divide it into three!!
	//we can get the candidates... but how do we get the nearby ones? -> at least one of them is not a "proper triangle"
	//looking for a method to check this
	//¿œ¥‹ find_all_triangles

	Point root = point_list[s];//probably index point_list.size()-2
	int triangle_with_s_id = point_state.find_triangle(root);

	vector<int> triangle_with_s = polygon_list[triangle_with_s_id];
	//divide the triangle into three
	for (int i = 0; i < 3; i++)
	{
		Funnel* temp = new Funnel(s, triangle_with_s[i % 3], triangle_with_s[(i + 1) % 3]);

		//set predecessor of each vertex!
		spt.set_pred(triangle_with_s[i % 3], s);
		spt.set_pred(triangle_with_s[(i + 1) % 3], s);

		//call split
		split_funnel(temp);
	}


}

void split_funnel(Funnel* funnel)
{
	int alpha = funnel->get_alpha();
	int beta = funnel->get_beta();
	int apex = funnel->get_apex();

	//if alpha and beta are adjacent numbers (meaning the diagonal is a polygon edge), return
	if (abs(alpha - beta) == 1 || abs(alpha - beta) == (v_num - 1))
		return;


	
}