#pragma once
#include"Point.h"
#include "ShortestPathTree.h" //includes polygon_operation.h
#include "polygon_decomposition.h"

int line_of_sight_id;
bool check_penetration(int from, int to, int apex, int first, int second);
Point foot_of_perpendicular(int p, Point origin, Point dest);
/*
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
}*/

enum event_type {
	PATH,
	BOUNDARY,
	BOUNDARY_S,
	BOUNDARY_T,
	BEND
};
class LOS {
	int id;
	event_type type;
	int endpoint1;//in case of boundary events, rotation vertices are stored as endpoint1
	int endpoint2;//in case of boundary events, polygon vertices that are not rotation vertices are stored as endpoint2
	Point other_endpoint;//the point of intersection between the l.o.s. and the polygon in stored here
	//float slope;
	float path_event_angle; //the angle between the previous path event (used to sort the boundary events)
	int rotation_vertex;
	vector<int> pi_s_l;//shortest path from s to l (only the polygon vertices registered in point_list)
	vector<int> pi_t_l;//shortest path from t to l (only the polygon vertices registered in point_list)
	Point foot_s;
	Point foot_t;
	bool foot_is_P_vertex;
	int e1; //vertex of the edge that the `other_endpoint' passes through
	int e2;
	Point extend1;
	Point extend2;
public:
	LOS(int _id, int p1, int p2, int rot_vertex, float angle, event_type _type)
	{
		id = _id;
		endpoint1 = p1;
		endpoint2 = p2;
		type = _type;
		rotation_vertex = rot_vertex;
		//slope calculation
		//slope = compute_slope(p1, p2);
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
	
	/*
	float get_slope()
	{
		return slope;
	}*/
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
	vector<int> get_pi_s_l()
	{
		return pi_s_l;
	}
	vector<int> get_pi_t_l()
	{
		return pi_t_l;
	}
	void set_pi_s_l(vector<int> pi)
	{
		pi_s_l = pi;
	}
	void set_pi_t_l(vector<int> pi)
	{
		pi_t_l = pi;
	}
	void set_foot_bool(bool is_polygon_vertex)
	{
		foot_is_P_vertex = is_polygon_vertex;
	}
	void compute_shortest_path_to_los(vector<int> shortest_path, SPT* spt_s, SPT* spt_t);
	bool compute_other_endpoint();
	vector<Point> get_shortest_path_to_line(bool s);
	Point* get_endpoint(int from, int to, int tri, int vertex1, int vertex2);
	vector<int> compute_shortest_path_line_nonP_vertex(Point vertex, SPT* spt, int e1, int e2);
	void extend_path_event();
};
/*
Point foot_of_perpendicular(int p, Edge e)
{
	Point pp = point_list[p];
	Point origin = point_list[e.get_origin()];
	Point dest = point_list[e.get_dest()];

	double ax = origin.get_x();
	double ay = origin.get_y();
	double bx = dest.get_x();
	double by = dest.get_y();
	double px = pp.get_x();
	double py = pp.get_y();

	if (ax == bx) //vertical line
	{
		return Point(ax, py);
	}
	else if (ay == by)//horizontal line
	{
		return Point(px, ay);
	}
	else {
		double slope = (double)(ay - by) / (ax - bx);
		double qx = slope / (1 + slope * slope) * (py + (double)px / slope + slope * ax - ay);
		double qy = ay + slope * (qx - ax);

		return Point(qx, qy);
	}
}*/


vector<Point> LOS::get_shortest_path_to_line(bool s)
{
	vector<Point> sp;
	vector<int>* sp_line = s?&pi_s_l:&pi_t_l;
	for (int i = 0; i < sp_line->size(); i++)
	{
		sp.push_back(point_list[sp_line->at(i)]);
	}

	if(type!=PATH)
	sp.push_back(s?foot_s:foot_t);

	return sp;
}
/* Returns the foot of perpendicular from point p to edge (p1, p2) */
Point foot_of_perpendicular(int p, Point origin, Point dest)
{
	Point pp = point_list[p];
	double ax = origin.get_x();
	double ay = origin.get_y();
	double bx = dest.get_x();
	double by = dest.get_y();
	double px = pp.get_x();
	double py = pp.get_y();

	if (ax == bx) //vertical line
	{
		return Point(ax, py);
	}
	else if (ay == by)//horizontal line
	{
		return Point(px, ay);
	}
	else {
		double slope = (double)(ay - by) / (ax - bx);
		double qx = slope / (1 + slope * slope) * (py + (double)px / slope + slope * ax - ay);
		double qy = ay + slope * (qx - ax);

		return Point(qx, qy);
	}
}

/* Returns whether vector(from,to) is  in the smaller angle that the two vectors v(apex,first) and v(apex,second) make */
bool check_penetration(int from, int to, int apex, int first, int second)
{
	double firstA = calculate_angle_between(apex, first, from, to);
	double secondA = calculate_angle_between(apex, second, from, to);

	if (firstA * secondA > 0)
		return false;
	else if (abs(firstA) + abs(secondA) >= PI)
		return false;
	else
		return true;
}


void get_remaining_path(vector<int> chain1, vector<int> chain2, vector<int>* final_path, Point* Foot)
{


	int apex = chain1[0];
	Point foot = foot_of_perpendicular(apex, point_list[chain1.back()], point_list[chain2.back()]);

	if (chain1.size() == 1 || chain2.size() == 1)
	{
		*Foot = foot;
		return;
	}
	int foot_idx = point_list.size();
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
	//final_path->pop_back();
	*Foot = point_list[main_chain.back()];
	return;
}


vector<int> LOS::compute_shortest_path_line_nonP_vertex(Point vertex, SPT* spt, int e1, int e2)
{
	vector<int> shortest_path;
	vector<int> _e1 = spt->retrieve_shortest_path(e1);
	vector<int> _e2 = spt->retrieve_shortest_path(e2);

	int size = _e1.size() <= _e2.size() ? _e1.size() : _e2.size();
	int idx = 0;
	for (; idx < size; idx++)
	{
		if (_e1[idx] != _e2[idx])
			break;
	}

	//the common path for the two points of the edge (e1, e2) -> inserted from the beginning to the apex
	shortest_path.insert(shortest_path.end(), _e1.begin(), _e1.begin() + idx);

	
	//the different part
	vector<int> temp1(_e1.begin() + idx-1, _e1.end());
	vector<int> temp2(_e2.begin() + idx-1, _e2.end());

	if (temp1.size() == 1 || temp2.size() == 1)
		return shortest_path;
	int v_id = point_list.size();
	point_list.push_back(vertex);
	bool success = check_penetration(shortest_path.back(), v_id, temp1[0], temp1[1], temp2[1]);
	//point_list.pop_back();
	if (success) {
		point_list.pop_back();
		return shortest_path;
	}


	vector<int>* main_chain;
	int apex = temp1[0];
	double temp1A = calculate_angle_between_positive(apex, v_id, apex, temp1[1]);
	double temp2A = calculate_angle_between_positive(apex, v_id, apex, temp2[1]);
	if(temp1A > temp2A)
	//if(calculate_angle_between_positive(apex,v_id,apex,temp1[1])>calculate_angle_between_positive(apex, v_id,apex,temp2[1]))
		main_chain = &temp2;
	else
		main_chain = &temp1;

	point_list.pop_back();
	bool left = is_Left(point_list[main_chain->at(1)], point_list[apex], vertex);

	for (int i = 1; i < main_chain->size() - 1; i++)
	{
		apex = main_chain->at(i);
		shortest_path.push_back(apex);
		if (left!=is_Left(point_list[main_chain->at(i + 1)], point_list[apex], vertex))
		{
			return shortest_path;
		}
	}

	printf("this should be an error\n");
	exit(40);
}

void LOS::compute_shortest_path_to_los(vector<int> shortest_path, SPT* spt_s, SPT* spt_t)
{
	if (type == PATH)
	{
		vector<int>::iterator it = find(shortest_path.begin(), shortest_path.end(), endpoint2);

		pi_s_l.insert(pi_s_l.end(), shortest_path.begin(), it);
		pi_t_l.insert(pi_t_l.end(), it, shortest_path.end());
		reverse(pi_t_l.begin(), pi_t_l.end());
		foot_is_P_vertex = true;
	}
	else
	{
		SPT* spt_other, * spt_endpoint2;
		Point* foot_other, * foot_endpoint2;
		vector<int>* other, * endpoint2;
		if (type == BOUNDARY_S)
		{
			spt_other = spt_s;
			spt_endpoint2 = spt_t;
			other = &pi_s_l;
			endpoint2 = &pi_t_l;
			foot_other = &foot_s;
			foot_endpoint2 = &foot_t;
		}
		else
		{
			spt_other = spt_t;
			spt_endpoint2 = spt_s;
			other = &pi_t_l;
			endpoint2 = &pi_s_l;
			foot_other = &foot_t;
			foot_endpoint2 = &foot_s;
		}
		vector<int> to_v = spt_other->retrieve_shortest_path(rotation_vertex);
		point_list.push_back(other_endpoint);
		vector<int> to_other_endpoint = compute_shortest_path_line_nonP_vertex(other_endpoint, spt_other, e1, e2);
		to_other_endpoint.push_back(point_list.size() - 1);
		int idx = 0;
		for (; idx < to_v.size() && idx < to_other_endpoint.size(); idx++)
		{
			if (to_v[idx] != to_other_endpoint[idx])
				break;
		}
		other->insert(other->end(), to_v.begin(), to_v.begin() + idx);
		vector<int> temp1(to_v.begin() + idx - 1, to_v.end());
		vector<int> temp2(to_other_endpoint.begin() + idx - 1, to_other_endpoint.end());
		get_remaining_path(temp1, temp2, other, foot_other);
		point_list.pop_back();

		to_v = spt_endpoint2->retrieve_shortest_path(rotation_vertex);
		vector<int> to_endpoint2 = spt_endpoint2->retrieve_shortest_path(this->endpoint2);
		idx = 0;
		for (; idx < to_v.size() && idx < to_endpoint2.size(); idx++)
		{
			if (to_v[idx] != to_endpoint2[idx])
				break;
		}
		endpoint2->insert(endpoint2->end(), to_v.begin(), to_v.begin() + idx);
		temp1 = vector<int>(to_v.begin() + idx - 1, to_v.end());
		temp2 = vector<int>(to_endpoint2.begin() + idx - 1, to_endpoint2.end());
		get_remaining_path(temp1, temp2, endpoint2, foot_endpoint2);
	}

	return;
}


/* computes the shortest path from the root of the spt to the line of sight*/
/*
void LOS::compute_shortest_path_to_los(bool spt_s, vector<int> point_to_apex, vector<int> chain1, vector<int> chain2)
{
	//Inserts into the shortest path all the vertices from the root to the common apex
	vector<int> shortest_path(point_to_apex);
	vector<int>* dest = spt_s ? &pi_s_l : &pi_t_l;
	int apex = chain1[0];

	if (chain1.front() != chain2.front())
	{
		printf("not a valid chain (compute_shortest_path_to_los)\n");
		exit(35);
	}
	else if (chain1.size() == 1 || chain2.size() == 1) // the apex is the foot
	{
		shortest_path.pop_back(); //remove the apex (it's going to be the foot instead);
		foot = point_list[apex];
		*dest = shortest_path;
		return;
	}

	//calculate the foot of perpendicular
	Point foot = (type == PATH) ? foot_of_perpendicular(apex, point_list[endpoint1], point_list[endpoint2]) : foot_of_perpendicular(apex, point_list[endpoint1], other_endpoint);
	int foot_idx = point_list.size();
	point_list.push_back(foot);

	bool direct = check_penetration(apex, foot_idx, apex, chain1[1], chain2[1]);
	if (direct)
	{
		this->foot = foot;
		point_list.pop_back();
		*dest = shortest_path;
		return;
	}
	vector<int> main_chain = (calculate_angle_between_positive(apex, chain1[1], apex, foot_idx) > calculate_angle_between_positive(apex, chain2[1], apex, foot_idx))
		? chain2 : chain1;
	bool side = is_left(apex, foot_idx, main_chain[1]);
	for (int i = 1; i < main_chain.size() - 1; i++)
	{
		apex = main_chain[i];
		shortest_path.push_back(apex);
		point_list.pop_back();
		//foot = recalculate new foot of perpendicular
		foot = (type == PATH) ? foot_of_perpendicular(apex, point_list[endpoint1], point_list[endpoint2]) : foot_of_perpendicular(apex, point_list[endpoint1], other_endpoint);
		point_list.push_back(foot);

		//check if our foot is correct
		if (side != is_left(apex, foot_idx, main_chain[i + 1]))
		{
			//chosen
			point_list.pop_back();
			*dest = shortest_path;
			return;
		}
	}

	// no foot of perpendicular
	point_list.pop_back();
	foot = point_list[main_chain.back()];
	*dest = shortest_path;
	return;
}*/

/* Returns whether the line (start, foot) doesn't overlap with the chain */
bool check_valid_foot(vector<int> chain, bool left, int start, Point foot)
{
	vector<int>::iterator it = find(chain.begin(), chain.end(), start);
	
	if (it + 1 == chain.end())
	{
		printf("this shouldn't be happening\n");
		exit(47);
	}

	if (left != is_Left(point_list[(*it)], foot, point_list[*(it + 1)]))
		return true;
	else
		return false;
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

	vertex[0] = -1;
	vertex[1] = -1;
	//find the triangle that (endpoint,rotation) penentrates through!!

	if (rotation >= v_num)
	{
		triangle = candidates.front();
		for (int i = 0; i < 3; i++)
		{
			if (polygon_list[triangle][i] == endpoint)
			{
				vertex[0] = polygon_list[triangle][(i + 1) % 3];
				vertex[1] = polygon_list[triangle][(i + 2) % 3];
				break;
			}
		}
		return triangle;
	}

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

	point_type x = (b2 * c1 - b1 * c2) / determinant;
	point_type y = (a1 * c2 - a2 * c1) / determinant;
	Point newP = Point(x, y);
	return &newP;
}

Point * LOS::get_endpoint(int from, int to, int tri, int vertex1, int vertex2)
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
	if (diff == 1 || diff == (v_num - 1)) {
		e1 = chosen_vertex;
		e2 = other_vertex;
		return get_line_intersection(from, to, chosen_vertex, other_vertex);
	}
	else
	{
		//find new triangle 
		for (int i = 0; i < 3; i++)
		{
			if (diag_list[i] != -1 && diagonal_list[diag_list[i]].check_same_point(chosen_vertex) != -1 && diagonal_list[diag_list[i]].check_same_point(other_vertex) != -1)
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
	int vertex[2] = { -1,-1 };

	//compute triangle that the boundary event penetrates through
	int triangle = choose_triangle(rotation, endpoint, vertex);
	if (triangle == -1)
		return false;

	//polygon vertex
	int diff = abs(vertex[0] - vertex[1]);
	if (diff == 1 || diff == (v_num - 1))
	{
		other_endpoint = *get_line_intersection(rotation, endpoint, vertex[0], vertex[1]);
		e1 = vertex[0];
		e2 = vertex[1];
		return true;
	}
	else
	{
		int new_tri = -1;
		int* d_list = t_list[triangle].get_d_list();
		int diag = -1;
		for (int i = 0; i < 3; i++)
		{
			if (d_list[i] != -1 && diagonal_list[d_list[i]].check_same_point(vertex[0]) != -1 && diagonal_list[d_list[i]].check_same_point(vertex[1]) != -1)
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


		Point * ptr = get_endpoint(endpoint, rotation, new_tri, vertex[0], vertex[1]);
		if (ptr == NULL)
			return false;
		other_endpoint = *ptr;
		return true;
	}
}