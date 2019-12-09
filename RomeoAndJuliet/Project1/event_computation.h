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
	SPT* spt_s;
	SPT* spt_t;
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
		spt_s = _spt_s;
		spt_t = _spt_t;
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
void EVENTS::compute_boundary_events()
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
		int s_size = s_candidates.size();

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
					LOS* los = new LOS(next_line_id++, cur, vertex_id, cur, angle, j < s_size ? BOUNDARY_S : BOUNDARY_T);// BOUNDARY);
					los->compute_other_endpoint();
					queue[i-1].push_back(los);
				}
			}
		}
	}
	sort_boundary_events();
}

/* Sets the foot_bool, pi_s_l, pi_t_l for the given los in the queue */
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
	
}

/* Chain1 and 2 are chains of the funnel each with the apex as its first member*/
void get_remaining_path(vector<int> chain1, vector<int> chain2, vector<int>* final_path,Point* Foot)
{
	int apex = chain1[0];

	int foot_idx = point_list.size();
	Point foot = foot_of_perpendicular(apex, point_list[chain1.back()], point_list[chain2.back()]);
	point_list.push_back(foot);
	bool direct = check_penetration(apex, foot_idx, apex, chain1[1], chain2[1]);
	

	if (direct) {
		*Foot = foot; 
		point_list.pop_back();
		return;
	}

	vector<int> main_chain = (calculate_angle_between_positive(apex, chain1[1], apex, foot_idx) > calculate_angle_between_positive(apex, chain2[1], apex, foot_idx))
		? chain2 : chain1;
	bool side = is_left(main_chain[1], apex, foot_idx);/////////////not sure about the order of the arguments
	point_list.pop_back();

	for (int i = 1; i < main_chain.size() - 1; i++)
	{
		apex = main_chain[i];
		final_path->push_back(apex);
		//only for the boundary case
		foot = foot_of_perpendicular(apex, point_list[chain1.back()], point_list[chain2.back()]);
		point_list.push_back(foot);
		if (side != is_left(main_chain[i + 1], apex, foot_idx))
		{
			//correct foot
			*Foot = foot;
			point_list.pop_back();
			return;
		}
		point_list.pop_back();
	}

	//no foot of perpendicular
	final_path->pop_back();
	*Foot = point_list[main_chain.back()];
	return;
	

	/*
	int apex_index;
	int max_size = chain1.size() >= chain2.size() ? chain1.size() : chain2.size();
	
	if(chain1[0]!=chain2[0])
		printf("this is an error\n");
	
	for (apex_index = 1; apex_index < max_size; apex_index++)
	{
		if (chain1[apex_index] != chain2[apex_index])
		{
			apex_index--;
			break;
		}
	}
	
	if (apex_index < 0)
	{
		printf("something went wrong here\n");
		exit(94);
	}

	for (int i = 0; i <= apex_index; i++)
		final_path->push_back(chain1[i]);

	int foot_idx = point_list.size();
	point_list.push_back(*foot);
	vector<int> main_chain = calculate_angle_between_positive()
	point_list.pop_back();*/

}
void EVENTS::compute_bend_events()
{
	for (int i = 0; i < queue.size(); i++)
	{
		for (int j = 0; j < queue[i].size(); j++)
		{
			LOS* los = queue[i][j];
			
			vector<int> s_to_e1 = spt_s->retrieve_shortest_path(los->get_endpoint1());
			vector<int> s_to_e2 = spt_s->retrieve_shortest_path(los->get_endpoint2());

			vector<int> chain1, chain2, common_chain;

			int last_common_index;
			for (last_common_index = 0; last_common_index<s_to_e1.size() && last_common_index<s_to_e2.size() &&
				s_to_e1.at(last_common_index) == s_to_e2.at(last_common_index); last_common_index++)
			{
				common_chain.push_back(s_to_e1[last_common_index]);
			}
			chain1.insert(chain1.end(), s_to_e1.begin() + last_common_index-1, s_to_e1.end());
			chain2.insert(chain2.end(), s_to_e2.begin() + last_common_index-1, s_to_e2.end());
			los->compute_shortest_path_to_los(true, common_chain, chain1, chain2);


			//the same for s_to_e2
		}
	}
}
