#include "Edge.h"

Edge::Edge() {
	this->s = new Point<>();
	this->t = new Point<>();
}


Edge::Edge(Point *_s, Point *_t) {
	this->s = _s;
	this->t = _t;
}

Edge::Edge(Edge *_e) {
	this->s = _e->gets();
	this->t = _e->gett();
}

Edge::~Edge() {

}

bool Edge::on(Point* p) {
	double d = this->gett()->getx() - this->gets()->getx();
	if (d == 0) {
		if (p->getx() == this->gett()->getx() && p->gety() <= max(this->gett()->gety(), this->gets()->gety()) && p->gety() >= min(this->gett()->gety(), this->gets()->gety())) {
			return true;
		}
		else return false;
	}
	else {
		double y = ((this->gett()->gety() - this->gets()->gety()) / d) * (p->getx() - this->gets()->getx()) + this->gets()->gety();
		if (y == p->gety() && p->gety() <= max(this->gett()->gety(), this->gets()->gety()) && p->gety() >= min(this->gett()->gety(), this->gets()->gety())) {
			return true;
		}
		else return false;
	}
}

Point* Edge::crossing(Edge* _e, bool closed = true) {
	double d = (_e->gett()->gety() - _e->gets()->gety()) * (this->t->getx() - this->s->getx()) - (_e->gett()->getx() - _e->gets()->getx()) * (this->t->gety() - this->s->gety());
	if (d == 0) {
		if (t == 0 && closed) {
			if (this->on(_e->gets())) {
				return _e->gets();
			}
			else if (this->on(_e->gett())) {
				return _e->gett();
			}
			else {
				return this->gets();
			}
		}
		else return nullptr;
	}
	else {
		double t = (_e->gett()->getx() - _e->gets()->getx()) * (this->s->gety() - _e->gets()->gety()) - (_e->gett()->gety() - _e->gets()->gety()) * (this->s->getx() - _e->gets()->getx());
		double s = (this->t->getx() - this->s->getx()) * (this->s->gety() - _e->gets()->gety()) - (this->t->gety() - this->s->gety()) * (this->s->getx() - _e->gets()->getx());
		t = t / d;
		s = s / d;
		if (t > 1 || s > 1 || t < 0 || s < 0) {
			printf("dont intersect");
			return nullptr;
		}
		else if (((t == 0 || t == 1) || (s == 0 || s == 1)) && !closed) {
			printf("if open");
			return nullptr;
		}
		else {
			double x = (1 - t) * this->s->getx() + t * this->t->getx();
			double y = (1 - t) * this->s->gety() + t * this->t->gety();
			Point* P = new Point(x, y);
			return P;
		}
	}
}

Point* Edge::gets() {
	return this->s;
}

Point* Edge::gett() {
	return this->t;
}


double Edge::length() {
	return this->s->distance(*this->t);
}
