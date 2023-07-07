#include "geom.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vector>

using namespace std; 

/* **************************************** */
/* returns the signed area of triangle abc. The area is positive if c
  is to the left of ab, and negative if c is to the right of ab
 */
int signed_area2D(point2d a, point2d b, point2d c) {
  int Ax = b.x - a.x;
  int Ay = b.y - a.y;
  int Bx = c.x - a.x;
  int By = c.y - a.y;

  return (Ax*By - Ay*Bx)/2;
}



/* **************************************** */
/* return 1 if p,q,r collinear, and 0 otherwise */
int collinear(point2d p, point2d q, point2d r) {
  const double tolerance = 1e-9; // set tolerance value
  double area = signed_area2D(p, q, r);
  return fabs(area) < tolerance;
}

// return true if a is on pq
bool on_segment(point2d p, point2d q, point2d a) {
    const double tolerance = 1e-6; // set tolerance value
    if (p.x == q.x) {  // vertical segment, avoid division by 0
        return a.x == p.x && a.y >= min(p.y, q.y) - tolerance && a.y <= max(p.y, q.y) + tolerance;
    }
    if (p.y == q.y) {  // horizontal segment, avoid division by 0
        return a.y == p.y && a.x >= min(p.x, q.x) - tolerance && a.x <= max(p.x, q.x) + tolerance;
    }
    double tx = (a.x - p.x) / (q.x - p.x);
    double ty = (a.y - p.y) / (q.y - p.y);
    return fabs(tx - ty) < tolerance && tx >= 0 - tolerance && tx <= 1 + tolerance;
}



/* **************************************** */
/* return 1 if c is  strictly left of ab; 0 otherwise */
int left_strictly(point2d a, point2d b, point2d c) {
  return signed_area2D(a,b,c) > 0;
}



/* return 1 if c is left of ab or on ab; 0 otherwise */
int left_on(point2d a, point2d b, point2d c) {
  int area = signed_area2D(a,b,c);
  if (abs(area) < 1e-9){
    return 1;
  }
  return area > 0;
}

//returns the distance between two points
double distance_2d(point2d p1, point2d p2){
    //printf("calculating distance...\n");
    //printf("\tp1: (%f, %f)\n", p1.x, p1.y);
    //printf("\tp2: (%f, %f)\n", p2.x, p2.y);
  return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

// Check if a point is inside a triangle
bool pointInTriangle(point2d p, point2d a, point2d b, point2d c) {
    bool b1, b2, b3;

    b1 = left_strictly(a, b, p);
    b2 = left_strictly(b, c, p);
    b3 = left_strictly(c, a, p);

    return (b1 && b2 && b3);
}


//return true if point c is between a and b (a,b,c are assumed to be collinear)
bool between(point2d a, point2d b, point2d c){
  if (a.x != b.x){ //not vertical 
    if ((a.x <= c.x && c.x <= b.x) || (b.x <= c.x && c.x <= a.x)){
      return true;
    }
  return false;
  } // else vertical 
  if ((a.y <= c.y && c.y <= b.y) || (b.y <= c.y && c.y <= a.y)){
    return true;
  }
  return false;
}


char parallel_intersection(point2d a, point2d b, point2d c, point2d d, point2d p){
  if (!collinear(a,b,c)){ // parallel but not collinear, so will never intersect even if the segments were extended infinitely
    return '0';
  }
  if (between(a,b,c)){
    p.x = c.x;
    p.y = c.y;
    return 'e';
  }
  if (between(a,b,d)){
    p.x = d.x;
    p.y = d.y;
    return 'e';
  }
  if (between(c,d,a)){
    p.x = a.x;
    p.y = a.y;
    return 'e';
  }
  if (between(c,d,b)){
    p.x = b.x;
    p.y = b.y;
    return 'e';
  }
  return false;
}



// returns a code describing the type of intersection between the line segments ab and cd 
// 'e': the segments collinearly overlap, sharing a point; 'e' stands for edge
// 'v': an endpoint of one segment is on the other segment, but 'e' doesn't hold; 'v' stands for vertex
// '1': the segments intersect properly (they share a point and neither 'v' nor 'e' holds)
// '0': the segments do not intersect (they share no points)
//point p stores the point of intersection
char SegSegInt(point2d& a, point2d& b, point2d& c, point2d& d, point2d& p){
  double s,t;
  double num, denom;
  char code = '?';
  
  denom = 
  a.x * (d.y - c.y) +
  b.x * (c.y - d.y) +
  d.x * (b.y - a.y) +
  c.x * (a.y - b.y);

  // if denom is zero then segments are parallel
  if (denom == 0.0){
    return parallel_intersection(a,b,c,d,p);
  }

  num = 
  a.x * (double)(d.y - c.y) +
  c.x * (double)(a.y - d.y) +
  d.x * (double)(c.y - a.y);

  double eps = 1e-6; // set a small tolerance value
  if ((fabs(num) < eps) || (fabs(num - denom) < eps)){
    code = 'v';
  }

  s = num/denom;

  num =
  -(a.x * (double)(c.y - b.y) +
  b.x * (double)(a.y - c.y) +
  c.x * (double)(b.y - a.y));

  if ((fabs(num) < eps) || (fabs(num - denom) < eps)){
    code = 'v';
  }

  t = num/denom;

  if ((0.0 < s) && (s < 1.0) && (0.0 < t) && (t < 1.0)){
    code = '1';
  }
  else if ((0.0 > s) || (s > 1.0) || (0.0 > t) || (t > 1.0)){
    code = '0';
  }

  p.x = a.x + s * (b.x - a.x);
  p.y = a.y + s * (b.y - a.y);

  //printf("intersection point from seg seg method: (%f, %f):\n", p.x, p.y);

  return code;
}

