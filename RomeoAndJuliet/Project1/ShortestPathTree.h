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
	}/*
	SPTnode(int _vertexID, int _parent)
	{
		vertexID = _vertexID;
		parent = spt.get_node(_parent);
		children = vector<SPTnode*>();
	}*/
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
	//find the triangle that s is included in and divide it into three!!
	//we can get the candidates... but how do we get the nearby ones? -> at least one of them is not a "proper triangle"
	//looking for a method to check this
	//일단 find_all_triangles

	Point root = point_list[s];//probably index point_list.size()-2
	spt.set_root(s);
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

/*  Chooses the next v on the other side of the diagonal compared to the apex in the funnel
	Returns the vertex ID of the chosen v
	Returns -1 on error */
int choose_v(Funnel * funnel)
{
	int alpha = funnel->get_alpha();
	int beta = funnel->get_beta();
	int apex = funnel->get_apex();

	//when there exists a valid Edge with an s_node set...
	if (funnel->get_diag() != -1)
	{
		SNode* diagonal = diagonal_list[funnel->get_diag()].get_SNode();
		SNode* child = NULL;
		if (diagonal != NULL)
		{
			//get diagonal's chliren....
			//locate in which of the child's cell the apex is located...
			//choose a vertex v in the other cell

			bool included_in_left_child = check_inclusive_id(diagonal->get_left_children()->get_polygon_with_edge(), apex);
			bool included_in_right_child = check_inclusive_id(diagonal->get_right_children()->get_polygon_with_edge(), apex);

			if (included_in_left_child)
			{
				//choose a vertex v in the right child cell that is not alpha nor beta
				child = diagonal->get_right_children();
			}
			else if (included_in_right_child)
			{
				//choose a vertex v in the left child cell that is not alpha nor beta
				child = diagonal->get_left_children();
			}
			else
			{
				//shit.. this should not be happening
				printf("apex is included in neither two children\n");
			}

			vector<int> next_v_candidates = diag_id_to_point_id(child->get_polygon_with_edge());
			for (int i = 0; i < next_v_candidates.size(); i++)
			{
				if (next_v_candidates[i] != alpha && next_v_candidates[i] != beta)
				{
					return next_v_candidates[i];
				}
			}
			//shouldn't reach here
			return -1;
		}
		else {
			printf("Shortest Path Tree!! SNode was not correctly set in the diagonal!!\n");
		}
	}
}

bool in_between(int prev, int curr, int next, int v)
{
	//cur->prev, cur->next 사이 angle 계산

	//둔각인 경우 (curr가 apex일때, 가장 처음 경우)
	//alpha side의 벡터를 180도 회전한다....

	//curr->v 이루는 각도 계산 후 1-3.3-2의 부호 비교....
	//같은 부호이면 영역 밖이라는 뜻이므로 return false
	//다른 부호이면 영역 안이라는 뜻이므로 return true
	float check_first = calculate_angle_between(curr, prev, next);

	float angle_prev = calculate_angle(prev, curr);
	float angle_next = calculate_angle(curr, next);
	if (abs(angle_prev - angle_next) >= PI / 2) //둔각인 경우 - curr가 apex일때
	{
		float v_prev = calculate_angle_between(curr, prev, v);
		float next_v = calculate_angle_between(curr, v, next);

		if (v_prev * next_v > 0)
			return true;
		else return false;
	}
	
	float angle_v = calculate_angle(curr, v);

	if ((angle_next - angle_v) * (angle_v - angle_prev) > 0)
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

	vector<int>::iterator apex_ptr = find(vertex_list.begin(), vertex_list.end(), apex);
	vector<int> alpha_list, beta_list;

	//always start with alpha!!
	alpha_list.insert(alpha_list.end(), vertex_list.begin(), apex_ptr);
	alpha_list.push_back(apex);
	reverse(alpha_list.begin(), alpha_list.end());
	beta_list.insert(beta_list.end(), apex_ptr, vertex_list.end());

	//when v's predecessor is the apex (apex directly reachable from v)
	if (in_between(beta_list.at(1), alpha_list.front(), alpha_list.at(1), v))
		return apex;
	
	//predecessor is a vertex from alpha_list or beta list
	
	float angle_alpha = calculate_angle_between(apex, alpha_list[1], v);
	float angle_beta = calculate_angle_between(apex, beta_list[1], v);
	vector<int> chain;
	if (angle_alpha*angle_beta < 0)//different booho ///this we need to change!!!!!1////////////////////////
	{
		return apex;
	}
	else
	{
		if (abs(angle_alpha) > abs(angle_beta))
		{
			chain = beta_list;
		}
		else
			chain = alpha_list;
	}

	vector<int>::iterator predecessor = chain.begin();
	//predecessor++;

	while ((predecessor + 2) != chain.end())
	{
		if (in_between(*predecessor, *(predecessor + 1), *(predecessor + 2), v))
		{
			return *(predecessor + 1);
		}

		predecessor++;
	}
	
	//when the predecessor is the last element
	return *predecessor;
	/*
	if (angle_alpha + angle_beta > 0)//they are both tilted to the right
	{
		while (predecessor +1 != chain.end())
		{
			float angle = calculate_angle_between(*predecessor, *(predecessor + 1), v);
			if (angle < 0)
				break;
			predecessor++;
		}
	}
	else
	{
		while (predecessor +1 != chain.end())
		{
			float angle = calculate_angle_between(*predecessor, *(predecessor + 1), v);
			if (angle > 0)
				break;
			predecessor++;
		}
	}
	
	return *predecessor;
	*/
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

	int pred = compute_pred(funnel, v);
	
	//add pred info to tree
	spt.set_pred(v, pred);

	vector<int> chain = funnel->get_vertex_list();
	vector<int>::iterator pred_ptr = find(chain.begin(), chain.end(), pred);
	vector<int> first_chain;
	vector<int> second_chain;

	if (find(chain.begin(), pred_ptr, apex) != chain.end())
	{
		reverse(chain.begin(), chain.end());
	}
	pred_ptr = find(chain.begin(), chain.end(), pred);
	first_chain.insert(first_chain.end(), chain.begin(), pred_ptr);
	first_chain.push_back(pred);
	first_chain.push_back(v);
	second_chain.push_back(v);
	second_chain.insert(second_chain.end(), pred_ptr, chain.end());

	Funnel* first = new Funnel(pred, first_chain);
	Funnel* second = new Funnel(apex, second_chain);

	split_funnel(first);
	split_funnel(second);

}

int find_diagonal_index(int first, int second)
{
	for (int i = 0; i < diagonal_list.size(); i++) {
		if (diagonal_list[i].check_same_edge(first, second))
			return i;
	}

	return -1;
}