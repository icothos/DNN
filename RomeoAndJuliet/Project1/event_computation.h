#pragma once

#include<iostream>
#include<vector>
#include"Point.h"
#include "ShortestPathTree.h" //includes polygon_operation.h

#define INT_MAX 100000000
using namespace std;

int line_of_sight_id;

float compute_slope(int _p1, int _p2)
{
	//p1 and p2 shouldn't be vertical lines
	Point p1 = point_list[_p1];
	Point p2 = point_list[_p2];

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

enum event_type {
	PATH,
	BOUNDARY,
	BEND
};
class LOS {
	int id;
	int endpoint1;
	int endpoint2;
	float slope;
	float path_event_angle; //the angle between the previous path event (used to sort the boundary events)
	event_type type;
	int rotation_vertex;
public:
	LOS(int _id, int p1, int p2, int rot_vertex,float angle, event_type _type)
	{
		id = _id;
		endpoint1 = p1;
		endpoint2 = p2;
		type = _type;
		rotation_vertex = rot_vertex;
		//slope calculation
		slope = compute_slope(p1, p2);
		path_event_angle = angle;
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
	float get_path_angle()
	{
		return path_event_angle;
	}
	event_type get_type()
	{
		return type;
	}
};

class EVENTS {
	int next_line_id;
	vector<vector<LOS*>> queue;
	vector<int> shortest_path;
public:
	EVENTS(vector<int> _shortest_path)
	{
		next_line_id = 0;
		shortest_path = _shortest_path;
		for (int i = 0; i < shortest_path.size()-1; i++)
		{
			queue.push_back(vector<LOS*>());
		}
	}
	vector<int> get_shortest_path()
	{
		return shortest_path;
	}
	void compute_path_events();
	void compute_boundary_events(SPT* spt_s, SPT* spt_t);
	void compute_bend_events();
	void sort_boundary_events();
};

bool compare_by_angle(LOS* a, LOS* b)
{
	if (a->get_path_angle() < b->get_path_angle())
		return true;
	else
		return false;
}

/* sorts boundary events in the order they appear on the polygon 
   uses the slope information stored in LOS class
   must be called only after all the path events are set*/
void EVENTS::sort_boundary_events()
{
	for (int i = 0; i < queue.size()-1; i++)
	{
		sort(queue[i].begin(), queue[i].end(), compare_by_angle);
		
		float angle = calculate_angle_between_positive(shortest_path[i + 1], shortest_path[i + 2], shortest_path[i], shortest_path[i + 1]);
		if (angle < queue[i].back()->get_path_angle())
		{
			vector<LOS*> sorted;
			sorted.push_back(queue[i][0]);
			
			reverse(queue[i].begin(), queue[i].end());
	
			sorted.insert(sorted.end(), queue[i].begin(), queue[i].end()-1);

			queue[i] = sorted;
		}
	}
}

/* adds a LOS to the queue vector for every edge in the shortest path (s,t) */
void EVENTS::compute_path_events()
{
	for (int i = 0; i < shortest_path.size()-1; i++)
	{
		int prev = shortest_path[i];
		int cur = shortest_path[i + 1];
		
		//float angle = i == 0 ? 0 : calculate_angle_between_positive(shortest_path[i], shortest_path[i + 1], shortest_path[i], shortest_path[i - 1]);
		LOS* los = new LOS(next_line_id++, prev, cur, cur, 0, PATH);

		queue[i].push_back(los);
	}
}

/* determines whether the line (*not vector) (CUR, P) is tangent to the path (PREV~CUR~NEXT) at vertex CUR */
bool is_tangent(int prev, int cur, int next, int p)
{
	float first = calculate_angle(prev, cur);
	float second = calculate_angle(cur, next);

	float p_cur = calculate_angle(p, cur);
	float cur_p = calculate_angle(cur, p);

	float angle1 = normalize_angle(first - p_cur);
	float angle2 = normalize_angle(p_cur - second);

	if (angle1*angle2 > 0)
		return true;
	
	angle1 = normalize_angle(first - cur_p);
	angle2 = normalize_angle(cur_p - second);

	if(angle1*angle2 > 0)
		return true;
	
	return false;
}

/* computes all boundary events
	sorts the boundary events by slope after inserting into the queue */
void EVENTS::compute_boundary_events(SPT* spt_s, SPT* spt_t)
{
	//search the tree for candidates 
	for (int i = 1; i < shortest_path.size()-1; i++)
	{
		int prev = shortest_path[i - 1];
		int cur = shortest_path[i];
		int next = shortest_path[i + 1];

		//find the vertex in the spt and the direct children will be the candidate
		SPTnode* vertex = spt_s->get_node(cur);
		if (vertex == NULL)
		{
			printf("couldn't find node in tree\n");
			exit(-1);
		}

		vector<SPTnode*> candidates = vertex->get_children();

		SPTnode* vertex_t = spt_t->get_node(cur);
		if (vertex_t == NULL)
		{
			printf("couldn't find node in tree\n");
			exit(100);
		}

		vector<SPTnode*> t_candidates = vertex_t->get_children();
		candidates.insert(candidates.end(), t_candidates.begin(), t_candidates.end());

		//then check the tangent thing...
		for (int j = 0; j < candidates.size(); j++)
		{
			int vertex_id = candidates[j]->get_id();
			if (vertex_id != next && vertex_id != shortest_path[i - 1])
			{
				if (is_tangent(prev,cur,next, vertex_id))
				{
					float angle = calculate_angle_between_positive(cur, vertex_id, prev, cur);
					LOS* los = new LOS(next_line_id++, cur, vertex_id, cur, angle, BOUNDARY);

					queue[i-1].push_back(los);
				}
			}
		}
	}
	sort_boundary_events();
}

void EVENTS::compute_bend_events()
{

}
