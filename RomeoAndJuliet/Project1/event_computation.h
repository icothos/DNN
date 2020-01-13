#pragma once

#include<iostream>
#include<vector>

#include "LOS.h"
#define INT_MAX 100000000
using namespace std;


class EVENTS {
	int next_line_id;
	vector<vector<LOS*>> queue;
	vector<int> shortest_path;
	SPT* spt[2]; //[0]: spt_s (s as root) , [1]: spt_t (t as root)
public:
	EVENTS() {}
	EVENTS(vector<int> _shortest_path, SPT* _spt_s, SPT* _spt_t)
	{
		next_line_id = 0;
		shortest_path = _shortest_path;
		for (int i = 0; i < shortest_path.size()-1; i++)
		{
			queue.push_back(vector<LOS*>());
		}
		spt[0] = _spt_s;
		spt[1] = _spt_t;
	}
	vector<int> get_shortest_path()
	{
		return shortest_path;
	}
	vector<vector<LOS*>> get_queue() {
		return queue; 
	}
	void compute_path_events();
	void compute_boundary_events();
	void compute_bend_events();
	void compute_shortest_path_to_line(int i, int j);
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
		LOS* path = queue[i][0];
		for (int j = 0; j < queue[i].size(); j++)
		{
			LOS* boundary = queue[i][j];
			float angle = calculate_angle_between_positive(path->get_p1(), path->get_p2(), boundary->get_p1(), boundary->get_p2());
			boundary->set_path_angle(angle);
		}
		sort(queue[i].begin(), queue[i].end(), compare_by_angle);

		/*
		//set path_angle separately
		//sort by path_angle
		sort(queue[i].begin(), queue[i].end(), compare_by_angle);
		
		float angle = calculate_angle_between_positive(shortest_path[i + 1], shortest_path[i + 2], shortest_path[i], shortest_path[i + 1]);
		if (angle < queue[i].back()->get_path_angle())
		{
			vector<LOS*> sorted;
			sorted.push_back(queue[i][0]);
			
			reverse(queue[i].begin(), queue[i].end());
	
			sorted.insert(sorted.end(), queue[i].begin(), queue[i].end()-1);

			queue[i] = sorted;
		}*/
	}

	printf("sorting boundary events is complete\n");
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
		los->compute_other_endpoint(true);
		//los->extend_path_event();
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
void EVENTS::compute_boundary_events()
{
	//search the tree for candidates 
	for (int i = 1; i < shortest_path.size()-1; i++)
	{
		int prev = shortest_path[i - 1];
		int cur = shortest_path[i];
		int next = shortest_path[i + 1];

		//find the vertex in the spt and the direct children will be the candidate
		SPTnode* vertex = spt[0]->get_node(cur);
		if (vertex == NULL)
		{
			printf("couldn't find node in tree\n");
			exit(-1);
		}
		vector<SPTnode*> s_candidates = vertex->get_children();
		int s_size = s_candidates.size();

		SPTnode* vertex_t = spt[1]->get_node(cur);
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
					LOS* los = new LOS(next_line_id++, cur, vertex_id, cur, angle, j < s_size ? BOUNDARY_S : BOUNDARY_T);// BOUNDARY);
					los->compute_other_endpoint(false);
					queue[i-1].push_back(los);
				}
			}
		}
	}
	sort_boundary_events();
}

/* Sets the foot_bool, pi_s_l, pi_t_l for the given los in the queue */
/*
void EVENTS::compute_shortest_path_to_line(int i, int j)
{
	LOS* los = queue[i][j];

	if (los->get_type() == PATH)
	{
		vector<int> s_to_l(shortest_path.begin(), shortest_path.begin() + i + 1);
		vector<int> t_to_l(shortest_path.rbegin(), shortest_path.rbegin() + shortest_path.size() - i);
		los->set_foot_bool(true);
		los->set_pi_s_l(s_to_l);
		los->set_pi_t_l(t_to_l);
		return;
	}
	else //BOUNDARY CASE (_S or _T)
	{
		//let's first think about BOUNDARY_S
		int rotation = los->get_endpoint1();
		int polygon_vertex = los->get_endpoint2();
		Point other_vertex = los->get_other_endpoint();
		int p_edge_num = los->get_polygon_edge();

		vector<int> s_to_v = spt_s->retrieve_shortest_path(rotation);
		vector<int> s_to_e1 = spt_s->retrieve_shortest_path(p_edge_num);
		vector<int> s_to_e2 = spt_s->retrieve_shortest_path((p_edge_num + v_num - 2) % (v_num - 1));

		//first get the shortest path from s to the other_vertex
		vector<int> s_to_other_endpoint;
		int max_size = s_to_e1.size() <= s_to_e2.size() ? s_to_e1.size() : s_to_e2.size();
		int apex_idx = 0;
		for (apex_idx; apex_idx < max_size; apex_idx++)
		{
			if (s_to_e1[apex_idx] != s_to_e2[apex_idx])
				break;
		}
		s_to_other_endpoint.insert(s_to_other_endpoint.end(),s_to_e1.begin(), (s_to_e1.begin() + apex_idx));
		vector<int> temp1, temp2;
		temp1.insert(temp1.end(), s_to_e1.begin() + apex_idx, s_to_e1.end());
		temp2.insert(temp2.end(), s_to_e2.begin() + apex_idx, s_to_e2.end());
		get_remaining_path(temp1, temp2, los)


		for (int i = 0; i < s_to_e1.size(); i++)
		{
			if (s_to_e1[i] == s_to_e2[i])
			{
				s_to_other_endpoint.push_back(s_to_e1[i]);
			}
			else
			{
				//a function that directly computes the shortest path from foot to apex...? or chain
				break;
			}
		}
		//now get the shortest path from s to the line!!

		//S_TO_L은 v(endpoint1) 랑 other_endpoint이은에에서 구해야할듯
		//T_TO_L은 v랑 아마 endpoint1고 ㅏendpoint2 ㅏ사이에서
		los->set_foot_bool(false);
	}
	
}*/

void print_vector(vector<int> vec)
{
	for (int i = 0; i < vec.size(); i++)
	{
		printf("%d ", vec[i]);
	}
	printf("\n");
}

bool is_polygon_edge(int diag)
{
	int p1 = diagonal_list[diag].get_origin();
	int p2 = diagonal_list[diag].get_dest();

	if (abs(p1 - p2) == 1)
		return true;
	if (p1 * p2 == 0 && p1 + p2 == v_num - 1)
		return true;
	else
		return false;
}
bool is_polygon_edge(int p1, int p2)
{
	if (abs(p1 - p2) == 1)
		return true;
	if (p1 * p2 == 0 && p1 + p2 == v_num - 1)
		return true;
	else
		return false;
}
int find_diagonal(int p1, int p2)
{
	for (int i = 0; i < diagonal_list.size(); i++)
	{
		if (diagonal_list[i].check_same_edge(p1, p2))
			return i;
	}

	return -1;
}

int opposite_tri(int current_tri, int diag)
{
	int* tri_cand = diagonal_list[diag].get_triangle();
	int new_tri;

	if (tri_cand[0] == current_tri)
		new_tri = tri_cand[1];
	else
		new_tri = tri_cand[0];

	if (new_tri == -1)
	{
		return -1;
	}
	else
		return new_tri;
}

/*computes the endpoint of bend event that is perpendicular to line (p1, p2) and passes through the rotation vertex*/
Point compute_bend_event_endpoint(int p1, int p2, int rotation_vertex,SPT* spt)
{
	Point foot = foot_of_perpendicular(rotation_vertex, point_list[p1], point_list[p2]);
	double x = foot.get_x();
	double diff1 = x - point_list[p1].get_x();
	double diff2 = x - point_list[p2].get_x();
	if (diff1*diff2 < 0)
	{
		//return the parent in the spt
		SPTnode* parent = spt->get_pred(rotation_vertex);
		int vertex = parent->get_id();
		if (vertex == p1 || vertex == p2)
			return point_list[vertex];
		else
		{
			printf("what is this\n");
			exit(30);
		}
	}

	int foot_tri = point_state.find_triangle(foot);
	int vertex[2];

	int tri = choose_triangle(p2, p1, vertex);
	if (tri == foot_tri)
		return foot;	

	while (!is_polygon_edge(vertex[0],vertex[1]))
	{
		//set the new diag (vertex[0], vertex[1])
		int* diag_list = t_list[tri].get_d_list();
		int diag = -1;
		for (int i = 0; i < 3; i++)
		{
			int d = diag_list[i];
			if (d != -1 && diagonal_list[d].check_same_edge(vertex[0], vertex[1]))
			{
				diag = d;
				break;
			}
		}
		//find opposite triangle to diag
		tri = opposite_tri(tri, diag);
		if (tri == foot_tri)
			return foot;

		//getting the other endpoint
		Triangle t = t_list[tri];
		int* p_list = t.get_p_list();
		int other_p;
		for (int i = 0; i < 3; i++)
		{
			if (p_list[i] != vertex[0] && p_list[i] != vertex[1])
			{
				other_p = p_list[i];
				break;
			}
		}

		//setting vertex[0] and vertex[1] -> the next diag
		if (check_penetration(p1, p2, p2, vertex[0], other_p))
		{
			vertex[1] = other_p;
		}
		else if (check_penetration(p1, p2, p2, vertex[1], other_p))
		{
			vertex[0] = other_p;
		}
	}

	//diag should be the polygon edge
	return *get_line_intersection(p1, p2,vertex[0],vertex[1]);

}
LOS* add_bend_event(LOS* path_event, int rotation_vertex, bool first)
{
	//get the foot of perpendicular from the rotation vertex to line(p1,p2)
	int p1 = path_event->get_p1();
	int p2 = path_event->get_p2();
	Point foot = foot_of_perpendicular(rotation_vertex, point_list[p1], point_list[p2]);
	
	//if the foot is in the polygon boundary
	int valid = point_state.find_triangle(foot);
	if (valid != -1)
	{
		LOS los(-1, rotation_vertex, -1, rotation_vertex, -1, BEND);
		los.set_endpoint(0, foot);
		return &los;
		//los connects points rotation_vertex and foot
	}
	else //not inside the polygon ->we find the intersection with the polygon boundary
	{
		//guess what!? we already computed it!! it's part of the path event
		foot = path_event->get_endpoint(first);
		LOS los(-1, rotation_vertex, -1, rotation_vertex, -1, BEND);
		los.set_endpoint(0, foot);
		return &los;
	}

}

void EVENTS::compute_bend_events()
{
	for (int i = 0; i < queue.size(); i++)
	{
		for (int j = 0; j < queue[i].size(); j++)
		{
			queue[i][j]->compute_shortest_path_to_los(shortest_path, spt);
		}
	}

	printf("done computing all the shortest paths\n");

	//gathering the turning points (변곡점)
	//they can't have bend events with themselves as the rotation vertex
	vector<int> turningP;
	if (shortest_path.size() > 3) {
		bool prev_side = is_left(shortest_path[2], shortest_path[0], shortest_path[1]);
		for (int i = 1; i <= shortest_path.size() - 3; i++)
		{
			bool side = is_left(shortest_path[i + 2], shortest_path[i], shortest_path[i + 1]);
			if (side != prev_side)
			{
				turningP.push_back(shortest_path[i + 1]);
				prev_side = side;
			}
		}
	}
	
	//search all the pi_s_l's and detect the differences
	vector<int> prev = queue[0][0]->get_pi_s_l();
	vector<int> cur;
	for (int i = 0; i < queue.size()-1; i++)
	{
		int rotation = shortest_path[i+1];
		for (int j = 0; j < queue[i].size(); j++)
		{

			if (queue[i][j]->get_type() != BEND) {
				cur = queue[i][j]->get_pi_s_l();

				//(probably) equivalent to a change in the combinatorial structure

				if (prev.back() != cur.back())
				{
					if (j == 0)
						rotation = shortest_path[i];

					if (find(turningP.begin(), turningP.end(), rotation) == turningP.end())
					{
						Point test = compute_bend_event_endpoint(prev.back(), cur.back(), rotation, spt[0]);
						LOS* bend = new LOS(-1, rotation, -1, rotation, 0, BEND);
						bend->set_endpoint(0, test); //only setting one endpoint for now
						//bend->compute_other_endpoint(true);
						if (j == 0)
							queue[i - 1].insert(queue[i - 1].end(), bend);
						else
							queue[i].insert(queue[i].begin() + j, bend);
					}
				}
				prev = cur;
			}
		}
	}

	/*
	prev = queue[0][0]->get_pi_t_l();
	for (int i = 0; i < queue.size() - 1; i++)
	{
		int rotation = shortest_path[i+1];
		for (int j = 0; j < queue[i].size(); j++)
		{
			if (queue[i][j]->get_type() != BEND) {
				cur = queue[i][j]->get_pi_t_l();

				//(probably) equivalent to a change in the combinatorial structure

				if (prev.back() != cur.back())
				{
					if (j == 0)
						rotation = shortest_path[i];

					if (find(turningP.begin(), turningP.end(), rotation) == turningP.end())
					{
						Point test = compute_bend_event_endpoint(prev.back(), cur.back(), rotation, spt[1]);
						LOS* bend = new LOS(-1, rotation, -1, rotation, 0, BEND);
						bend->set_endpoint(0, test); //only setting one endpoint for now
						if (j == 0)
							queue[i - 1].insert(queue[i - 1].end(), bend);
						else
							queue[i].insert(queue[i].begin() + j, bend);
					}
				}
				prev = cur;
			}
		}
	}*/
	/*
	vector<int> prev = queue[0][0]->get_pi_s_l();// int prev_size = queue[0][0]->get_pi_s_l();
	for (int i = 0; i < queue.size(); i++)
	{
		for (int j = 0; j < queue[i].size(); j++)
		{
			vector<int> cur = queue[i][j]->get_pi_s_l();
			if (prev.size() != cur.size())
			{
				vector<int>* bigger = (prev.size() > cur.size()) ? &prev : &cur;
				int a = bigger->at(bigger->size() - 2);
				int b = bigger->at(bigger->size() - 1);
				if (a != shortest_path[i] && b != shortest_path[i]) {
					Point test = compute_bend_event_endpoint(a, b, shortest_path[i]);
					printf("blob\n");
				}
			}
			prev = cur;
		}
	}*/
	//for every consecutive event... we have to see whether 
	//there is a change in the combinatorial structure of the path


	printf("done computing bend events\n");
	
}
