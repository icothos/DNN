#pragma once

#include<iostream>
#include<vector>
#include"Point.h"
#include "ShortestPathTree.h" //includes polygon_operation.h
#include "polygon_decomposition.h"

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
	event_type type;
	int endpoint1;//in case of boundary events, rotation vertices are stored as endpoint1
	int endpoint2;//in case of boundary events, polygon vertices that are not rotation vertices are stored as endpoint2
	Point other_endpoint;//the point of intersection between the l.o.s. and the polygon in stored here
	float slope;
	float path_event_angle; //the angle between the previous path event (used to sort the boundary events)
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
	Point get_other_endpoint()
	{
		return other_endpoint;
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
	bool compute_other_endpoint();
};



bool check_penetration(int from, int to, int apex, int first, int second)
{
	double firstA = calculate_angle_between(apex, first, from, to);
	double secondA = calculate_angle_between(apex, second, from, to);

	if (firstA*secondA > 0)
		return false;
	else if (abs(firstA) + abs(secondA) >= PI)
		return false;
	else
		return true;
}
/* returns the index of the triangle in polygon_list that (endpoint, rotation) penentrates through
   @rotation : the rotation vertex
   @endpoint : the other vertex of the boundary event
   @vertex : the name of the int[2] array that contains the indices of the two other triangle vertices 
 */
int choose_triangle(int rotation, int endpoint, int* vertex)
{
	//list of all triangles adjacent to vertex `rotation'
	vector<int> candidates = point_state.find_all_triangles(point_list[rotation]);
	int triangle;

	//find the triangle that (endpoint,rotation) penentrates through!!
	for (int i = 0; i < candidates.size(); i++)
	{
		triangle = candidates[i];
		for (int j = 0; j < 3; j++)
		{
			if (polygon_list[triangle][j] == rotation)
			{
				vertex[0] = polygon_list[triangle][(j + 1) % 3];
				vertex[1] = polygon_list[triangle][(j + 2) % 3];
				break;
			}
		}

		if (vertex[0] == -1 || vertex[1] == -1)
			break;

		if (check_penetration(endpoint, rotation, rotation, vertex[0], vertex[1]))
			return triangle;
	}

	return -1;
}

/* returns the pointer the to Point such that the point is the intersection of
the extend line of (p1, p2) and the bounded line (q1,q2) (segment) */
Point* get_line_intersection(int p1, int p2, int q1, int q2)
{
	Point A = point_list[p1];
	Point B = point_list[p2];
	Point C = point_list[q1];
	Point D = point_list[q2];

	point_type a1 = B.get_y() - A.get_y();
	point_type b1 = A.get_x() - B.get_x();
	point_type c1 = a1 * (A.get_x()) + b1 * (A.get_y());

	point_type a2 = D.get_y() - C.get_y();
	point_type b2 = C.get_x() - D.get_x();
	point_type c2 = a2 * (C.get_x()) + b2 * (C.get_y());

	point_type determinant = a1 * b2 - a2 * b1;

	//the two lines are parallel
	if (determinant == 0)
		return NULL;

	point_type x = (b2*c1 - b1 * c2) / determinant;
	point_type y = (a1*c2 - a2 * c1) / determinant;
	Point newP = Point(x, y);
	return &newP;
}

Point* get_endpoint(int from, int to,int tri, int vertex1, int vertex2)
{
	Triangle triangle = t_list[tri];
	int* diag_list = triangle.get_d_list();
	int* p_list = triangle.get_p_list();
	int other_vertex = -1;
	int chosen_vertex = -1;
	int diag = -1;
	int new_tri = -1;;

	for (int i = 0; i < 3; i++)
	{
		if (p_list[i] != vertex1 && p_list[i] != vertex2)
		{
			other_vertex = p_list[i];
			break;
		}
	}

	if (check_penetration(from, to, to, vertex1, other_vertex))
		chosen_vertex = vertex1;
	else if (check_penetration(from, to, to, vertex2, other_vertex))
		chosen_vertex = vertex2;
	else
		return NULL;	

	//check if polygon edge
	int diff = abs(chosen_vertex - other_vertex);
	if (diff == 1 || diff == (v_num - 1))
		return get_line_intersection(from, to, chosen_vertex, other_vertex);
	else
	{
		//find new triangle 
		for (int i = 0; i < 3; i++)
		{
			if (diag_list[i]!=-1 && diagonal_list[diag_list[i]].check_same_point(chosen_vertex) != -1 && diagonal_list[diag_list[i]].check_same_point(other_vertex) != -1)
				diag = diag_list[i];
		}

		if (diag == -1)
			return NULL;

		if (diagonal_list[diag].get_triangle()[0] == tri)
			new_tri = diagonal_list[diag].get_triangle()[1];
		else
			new_tri = diagonal_list[diag].get_triangle()[0];

		return get_endpoint(from, to, new_tri, chosen_vertex, other_vertex);
	}
}


bool LOS::compute_other_endpoint()
{
	int rotation = endpoint1;
	int endpoint = endpoint2;
	printf("%d %d\n", rotation, endpoint);
	int vertex[2] = { -1,-1 };
	
	//compute triangle that the boundary event penetrates through
	int triangle = choose_triangle(rotation, endpoint, vertex);
	if (triangle == -1)
		return false;
	
	//polygon vertex
	int diff = abs(vertex[0]-vertex[1]);
	if (diff == 1 || diff == (v_num - 1))
	{
		other_endpoint = *get_line_intersection(rotation, endpoint, vertex[0], vertex[1]);
		return true;
	}
	else
	{
		int new_tri = -1;
		int* d_list = t_list[triangle].get_d_list();
		int diag = -1;
		for (int i = 0; i < 3; i++)
		{
			if (d_list[i]!=-1 && diagonal_list[d_list[i]].check_same_point(vertex[0])!=-1 && diagonal_list[d_list[i]].check_same_point(vertex[1])!=-1)
			{
				diag = d_list[i];
				break;
			}
		}
		if (diag == -1)
			return false;

		if (diagonal_list[diag].get_triangle()[0] == triangle)
			new_tri = diagonal_list[diag].get_triangle()[1];
		else
			new_tri = diagonal_list[diag].get_triangle()[0];
		

		Point* ptr =  get_endpoint(endpoint, rotation, new_tri, vertex[0], vertex[1]);
		if (ptr == NULL)
			return false;
		other_endpoint = *ptr;
		return true;
	}
}


class EVENTS {
	int next_line_id;
	vector<vector<LOS*>> queue;
	vector<int> shortest_path;
public:
	EVENTS() {}
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
	vector<vector<LOS*>> get_queue() {
		return queue;
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
		vector<SPTnode*> s_candidates = vertex->get_children();

		SPTnode* vertex_t = spt_t->get_node(cur);
		if (vertex_t == NULL)
		{
			printf("couldn't find node in tree\n");
			exit(100);
		}
		vector<SPTnode*> t_candidates = vertex_t->get_children();

		s_candidates.insert(s_candidates.end(), t_candidates.begin(), t_candidates.end());

		//then check the tangent thing...
		for (int j = 0; j < s_candidates.size(); j++)
		{
			int vertex_id = s_candidates[j]->get_id();
			if (vertex_id != next && vertex_id != shortest_path[i - 1])
			{
				if (is_tangent(prev,cur,next, vertex_id))
				{
					float angle = calculate_angle_between_positive(cur, vertex_id, prev, cur);
					LOS* los = new LOS(next_line_id++, cur, vertex_id, cur, angle, BOUNDARY);
					los->compute_other_endpoint();
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
