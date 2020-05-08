#pragma once
#include "Point.h"

class Edge {
protected:
	Point *s, *t;
public:
	Edge();
	Edge(Point* _s, Point* _t);
	Edge(Edge*);
	~Edge();
	bool operator==(Edge);
	Point* crossing(Edge*, bool);
	Point* gets();
	Point* gett();
	double length();
};