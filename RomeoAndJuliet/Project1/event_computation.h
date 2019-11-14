#pragma once

#include<iostream>
#include<vector>
#include"Point.h"
#include "ShortestPathTree.h"

#define INT_MAX 100000000
using namespace std;

int line_of_sight_id;

float compute_slope(int _p1, int _p2)
{
	//p1 and p2 shouldn't be vertical lines
	Point p1 = point_list[_p1];
	Point p2 = point_list[_p1];

	float x1 = p1.get_x();
	float x2 = p2.get_x();
	float y1 = p1.get_y();
	float y2 = p2.get_y();

	if (y1 == y2)
		return 0;
	
	if (x1 == x2)
		return INT_MAX;

	return (float)(y1 - y2) / (x1 - x2);
}
class LOS {
	int id;
	int endpoint1;
	int endpoint2;
	float slope;
	int rotation_vertex;
public:
	LOS(int _id, int p1, int p2, int rot_vertex)
	{
		id = _id;
		endpoint1 = p1;
		endpoint2 = p2;
		rotation_vertex = rot_vertex;
		//slope calculation
		slope = compute_slope(p1, p2);
	}
	int get_endpoint1()
	{
		return endpoint1;
	}
	int get_endpoint2()
	{
		return endpoint2;
	}
	float get_slope()
	{
		return slope;
	}
	int get_rotation_vertex()
	{
		return rotation_vertex;
	}
};

class EVENTS {
	int next_line_id;
	vector<vector<LOS*>> events;
	vector<int> shortest_path;
public:
	EVENTS(vector<int> _shortest_path)
	{
		next_line_id = 0;
		shortest_path = _shortest_path;
		for (int i = 0; i < shortest_path.size(); i++)
		{
			events.push_back(vector<LOS*>());
		}
	}
	vector<int> get_shortest_path()
	{
		return shortest_path;
	}
	void compute_path_events();
	void compute_boundary_events(SPT* spt);
	void compute_bend_events();
};

void EVENTS::compute_path_events()
{
	line_of_sight_id = 0;

	for (int i = 0; i < shortest_path.size()-1; i++)
	{
		LOS* los = new LOS(next_line_id++, shortest_path[i], shortest_path[i + 1], shortest_path[i + 1]);

		events[i].push_back(los);
	}
}

void EVENTS::compute_boundary_events(SPT* spt)
{
	//search the tree for candidates 
	for (int i = 0; i < shortest_path.size(); i++)
	{
		//find the vertex in the spt and the direct children will be the candidate
		SPTnode* vertex = spt->get_node(shortest_path[i]);
		if (vertex == NULL)
		{
			printf("couldn't find node in tree\n");
			exit(-1);
		}

		vector<SPTnode*> candidates = vertex->get_children();

		//then check the tangent thing...
		for (int j = 0; j < candidates.size(); j++)
		{
			//must check alg
		}


	}
}