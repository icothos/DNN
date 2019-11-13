#pragma once

#include<iostream>
#include<vector>

using namespace std;

int line_of_sight_id;
class LOS {
	int id;
	int endpoint1;
	int endpoint2;
	int slope;
	int rotation_vertex;
public:
	LOS()
	{
		id = line_of_sight_id++;
	}
	bool get_endpoint1()
	{
		return endpoint1;
	}
	bool get_endpoint2()
	{
		return endpoint2;
	}
};

class EVENTS {
	vector<vector<int>> events;
	vector<int> shortest_path;
public:
	EVENTS(vector<int> _shortest_path)
	{
		shortest_path = _shortest_path;
		for (int i = 0; i < shortest_path.size(); i++)
		{
			events.push_back(vector<int>());
		}
	}
	vector<int> get_shortest_path()
	{
		return shortest_path;
	}
	void compute_path_events();
};

void EVENTS::compute_path_events()
{
	line_of_sight_id = 0;

	for (int i = 0; i < shortest_path.size(); i++)
	{

	}
}