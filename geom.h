#ifndef __geom_h
#define __geom_h

#include <vector>

using namespace std; 


typedef struct _point2d {
  int x,y; 
} point2d;

typedef struct _config {
    int x,y;
    double dist;
    double f;
    bool finished;
    _config* parent;
    int angle;
} config;

struct CompareConfigByF {
  bool operator()(const pair<config*, double>& a, const pair<config*, double>& b) const {
    return a.second > b.second; // Order by f in non-decreasing order
  }
};

struct compareconfig {
    bool operator()(const config* a, const config* b) const {
        return a->f < b->f;
    }
};



typedef struct _robot {
    vector<point2d> vertices;
    config c;
} robot;

typedef struct _edge2d {
  point2d *a, *b;
} edge;

typedef struct _adjnode {
  point2d* v;
  int i;
  vector<point2d> neighbours;
} adjnode;

typedef struct _triangle {
  point2d a,b,c;
} triangle;


/* returns 2 times the signed area of triangle abc. The area is
   positive if c is to the left of ab, 0 if a,b,c are collinear and
   negative if c is to the right of ab
 */
int signed_area2D(point2d a, point2d b, point2d c); 


/* return 1 if p,q,r collinear, and 0 otherwise */
int collinear(point2d p, point2d q, point2d r);

bool on_segment(point2d p, point2d q, point2d a);

/*// function used when sorting points to compare the angle of a point with respect to p0
bool compare_angles(point_angle p1, point_angle p2);*/


/* return 1 if c is  strictly left of ab; 0 otherwise */
int left_strictly (point2d a, point2d b, point2d c); 


/* return 1 if c is left of ab or on ab; 0 otherwise */
int left_on(point2d a, point2d b, point2d c); 

//returns the distance between two points
double distance_2d(point2d p1, point2d p2);

// Check if a point is inside a triangle
bool pointInTriangle(point2d p, point2d a, point2d b, point2d c);

//return true if point c is between a and b (a,b,c are assumed to be collinear)
bool between(point2d a, point2d b, point2d c);

char parallel_intersection(point2d a, point2d b, point2d c, point2d d);

// returns true if the line segments ab and cd intersect
//point p stores the point of intersection
char SegSegInt(point2d& a, point2d& b, point2d& c, point2d& d, point2d& p);

  

#endif
