#pragma once
#include<iostream>
#include<vector>
#include "Tree.h"
#include "PointS.h"
using namespace std;

Tree SPT = Tree();

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
};
class SPT
{
	SPTnode* root;
	vector<int> components;

public:
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
		//we will use the function get_node



	}
	SPTnode* get_node(int vertex)
	{
		if (find(components.begin(), components.end(), vertex) == components.end())
			return NULL; //NO CORRESPONDING NODE IN THE TREE SO FAR

		
	}
	SPTnode* get_pred(int vertex)
	{
		//find the parent of the node representing 'vertex' in the tree!
		
	}
};


class Funnel
{
	int apex;	// apex's vertex id in point_list
	int alpha;	// an edge's endpoint id
	int beta;	//other point of the edge
	vector<int> vertex_list;

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
};

void find_shortest_path_tree(int);
void split_funnel(int, Funnel, int);

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

		//call split
	}


}

void split_funnel(int diag, Funnel funnel, int apex)
{

}