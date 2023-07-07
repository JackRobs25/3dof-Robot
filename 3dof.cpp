// visgraph.cpp

#include "geom.h" 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <float.h>
#include <tuple>
#include <unordered_map>
#include <queue>
#include <set>
#include <unistd.h>



#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


#include <vector> 

using namespace std; 



GLfloat red[3] = {1.0, 0.0, 0.0};
GLfloat green[3] = {0.0, 1.0, 0.0};
GLfloat blue[3] = {0.0, 0.0, 1.0};
GLfloat black[3] = {0.0, 0.0, 0.0};
GLfloat white[3] = {1.0, 1.0, 1.0};
GLfloat gray[3] = {0.5, 0.5, 0.5};
GLfloat yellow[3] = {1.0, 1.0, 0.0};
GLfloat magenta[3] = {1.0, 0.0, 1.0};
GLfloat cyan[3] = {0.0, 1.0, 1.0};



/* global variables */
const int WINDOWSIZE = 750; 

int Kx = 75;
int Ky = 75;
int Ktheta = 10;

// global variables for the size of each grid cell
int grid_width = WINDOWSIZE / Kx;
int grid_height = WINDOWSIZE / Ky;
int grid_depth = 360 / Ktheta;

//obstacles 
vector<vector<point2d> >  obstacles;

// start point 
point2d start_point = {270,555};

// end point
point2d end_point = {581, 49};

// robot 
vector<point2d> robot_points;

robot global_robot;

int animation_index = 0;

//coordinates of the last mouse click
double mouse_x=-10, mouse_y=-10;  //initialized to a point outside the window

//used to give mouse presses a variety of functions
int poly_init_mode = 0;

int current_obstacle = 0;

int graph_mode = 0;

int fill_mode = 1; // press 'f' to turn off

int path_mode = 0;

int animation_mode = 0;

/* forward declarations of functions */
void display(void);
void keypress(unsigned char key, int x, int y);
void mousepress(int button, int state, int x, int y);
void timerfunc(); 


bool isVisible(point2d p, point2d q, vector<vector<point2d> > obstacles, point2d& intersection);
int predecessor(vector<point2d>& poly, int index);
int successor(vector<point2d>& poly, int index);
vector<edge> CreateSegmentsFromPolygon(vector<point2d>& poly);
vector<edge> CreateSegmentsFromObstacles(vector<vector<point2d> >& obstacles);
bool polygonSimple(vector<point2d>& poly);
void draw_polygon(vector<point2d>& poly, GLfloat* color);
void draw_robot(vector<point2d>& robot);
void draw_obstacles(vector<vector<point2d> > obstacles);
void print_obstacles(vector<vector<point2d> > obstacles);
bool pointInPolygon(vector<point2d>& poly, point2d p);
void print_graph(vector<edge>& graph);
void dijkstra_shortest_path(vector<edge>& graph, point2d start_point, point2d end_point, vector<adjnode> adj_list);
void find_neighbours(point2d p, vector<edge> graph, adjnode& node);
void print_adjacency_list(vector<adjnode> adj_list);
vector<int> find_neighbour_indices(vector<adjnode> adj_list, int vertexIndex);
void print_path(vector<config*> path);
void print_previous_edges(unordered_map<int, int> previous, vector<adjnode> adj_list);
void draw_filled_obstacles(vector<vector<point2d> >  obstacles);
bool compare_2dpoints (point2d p1, point2d p2);
robot init_robot(vector<point2d> robot_points);
void draw_path(vector<config*> path);
void print_config(config* c);
point2d cell_center(point2d p);
void draw_complete_path(vector<config*>& path, robot starter_robot);
void animate_path(vector<config*>& path, robot starter_robot);



// given the starting position of the robot, change its configuration to match c

/*To orient the robot to match the new placement p, you need to update both the 
placement of the robot r and its vertices to reflect the change in position and 
orientation.

To update the placement of the robot, you can simply assign the refp and angle 
fields of r.p to the corresponding fields of p. This will update the robot's 
reference point and rotation angle to match the new placement.

To update the robot's vertices, you need to apply a transformation to each vertex. 
The transformation will depend on the difference between the old placement and 
the new placement. Specifically, you need to translate the vertices by the 
difference between the old and new reference points, and then rotate them by 
the difference between the old and new rotation angles.
*/
robot orient_robot(robot starter_robot, config c) {
  robot r = starter_robot;

  // Compute the transformation matrix
  double dx = c.x - r.c.x;
  double dy = c.y - r.c.y;
  double dtheta = c.angle - r.c.angle;


  // Apply the transformation to each vertex
  for (int i = 0; i < r.vertices.size(); i++) {
    // translate the vertex
    r.vertices[i].x += dx;
    r.vertices[i].y += dy;

    // rotate the vertex
    double cos_theta = cos(dtheta);
    double sin_theta = sin(dtheta);
    int rotated_x = round((r.vertices[i].x - c.x) * cos_theta - (r.vertices[i].y - c.y) * sin_theta + c.x);
    int rotated_y = round((r.vertices[i].x - c.x) * sin_theta + (r.vertices[i].y - c.y) * cos_theta + c.y);
    r.vertices[i].x = rotated_x;
    r.vertices[i].y = rotated_y;
  }

  // Update the robot's config
  r.c = c;

  return r;
}



bool is_free(config c, robot& starter_robot, vector<vector<point2d> > obstacles){
  robot new_robot = orient_robot(starter_robot, c);
  // now we want to check if the newly oriented robot intersects any of the obstacles
  vector<edge> obstacle_edges = CreateSegmentsFromObstacles(obstacles);
  vector<edge> new_robot_edges = CreateSegmentsFromPolygon(new_robot.vertices);
  for (int i = 0; i<obstacle_edges.size(); i++){
    for(int j = 0; j<new_robot_edges.size(); j++){
      point2d intersection;
      if (SegSegInt(*obstacle_edges[i].a, *obstacle_edges[i].b, *new_robot_edges[j].a, *new_robot_edges[j].b, intersection) != '0'){
        return false;
      }
    }
  }
  return true;
}


bool same_config(config c1, config c2){
  return ((c1.x == c2.x && c1.y == c2.y) && (c1.dist == c2.dist) && (c1.f == c2.f) && (c1.finished == c2.finished) && (c1.angle == c2.angle ));
}


// return a vector of the neighbour configurations of config n
vector<config*> get_neighbour(config* n, config***& graph, vector<config>& seen_before){
    vector<config*> neighbours;
    neighbours.clear(); // to be safe
    // compute the new refp/angle of the 6 different neighbours 
    // find the corresponding gnode from the graph
    // add to the vector of neighbour

    int x_index = (n->x)/grid_width;
    int y_index = (n->y)/grid_height;
    int t_index = (n->angle)/grid_depth;


    if (x_index + 1 < Kx) { // if the config is in bounds
      if (graph[x_index + 1][y_index][t_index].finished == false) { // if the config isn't finished
        bool seen = false;
        config x1;
        x1.x = (n->x) + grid_width;
        x1.y = n->y;
        x1.angle = n->angle;
        x1.finished = false;
        x1.dist = numeric_limits<double>::infinity();
        x1.f = numeric_limits<double>::infinity();
        x1.parent = nullptr;
        for (int i = 0; i < seen_before.size(); i++) {
          if (same_config(seen_before[i], graph[x_index + 1][y_index][t_index])) { // if config has been seen before
            // add the existing config to neighbours
            neighbours.push_back(&graph[x_index + 1][y_index][t_index]);
            seen = true;
            break;
          }
        }
        if (!seen) {
          // add the updated config to graph and neighbours
          graph[x_index + 1][y_index][t_index] = x1;
          neighbours.push_back(&graph[x_index + 1][y_index][t_index]);
          seen_before.push_back(graph[x_index + 1][y_index][t_index]);
        }
      }
    }


    if (x_index - 1 >= 0) { // if the config is in bounds
      if (graph[x_index - 1][y_index][t_index].finished == false) { // if the config isn't finished
      bool seen = false;
      config x2;
      x2.x = (n->x) - grid_width;
      x2.y = n->y;
      x2.angle = n->angle;
      x2.finished = false;
      x2.dist = numeric_limits<double>::infinity();
      x2.f = numeric_limits<double>::infinity();
      x2.parent = nullptr;
      for (int i = 0; i < seen_before.size(); i++) {
        if (same_config(seen_before[i], graph[x_index - 1][y_index][t_index])) { // if config has been seen before
          // add the existing config to neighbours
          neighbours.push_back(&graph[x_index - 1][y_index][t_index]);
          seen = true;
          break;
        }
      }
      if (!seen) {
        // add the updated config to graph and neighbours
        graph[x_index - 1][y_index][t_index] = x2;
        neighbours.push_back(&graph[x_index - 1][y_index][t_index]);
        seen_before.push_back(graph[x_index - 1][y_index][t_index]);
      }
    }
  }



    if (y_index + 1 < Ky) { // if the config is in bounds
      if (graph[x_index][y_index + 1][t_index].finished == false) { // if the config isn't finished
      bool seen = false;
      config y1;
      y1.x = n->x;
      y1.y = (n->y) + grid_height;
      y1.angle = n->angle;
      y1.finished = false;
      y1.dist = numeric_limits<double>::infinity();
      y1.f = numeric_limits<double>::infinity();
      y1.parent = nullptr;
      for (int i = 0; i < seen_before.size(); i++) {
        if (same_config(seen_before[i], graph[x_index][y_index + 1][t_index])) { // if config has been seen before
          // add the existing config to neighbours
          neighbours.push_back(&graph[x_index][y_index + 1][t_index]);
          seen = true;
          break;
        }
      }
      if (!seen) {
        // add the updated config to graph and neighbours
        graph[x_index][y_index + 1][t_index] = y1;
        neighbours.push_back(&graph[x_index][y_index + 1][t_index]);
        seen_before.push_back(graph[x_index][y_index + 1][t_index]);
      }
    }
  }


    if (y_index - 1 >= 0) { // if the config is in bounds
      if (graph[x_index][y_index - 1][t_index].finished == false) { // if the config isn't finished
      bool seen = false;
      config y2;
      y2.x = n->x;
      y2.y = (n->y) - grid_height;
      y2.angle = n->angle;
      y2.finished = false;
      y2.dist = numeric_limits<double>::infinity();
      y2.f = numeric_limits<double>::infinity();
      y2.parent = nullptr;
      for (int i = 0; i < seen_before.size(); i++) {
        if (same_config(seen_before[i], graph[x_index][y_index - 1][t_index])) { // if config has been seen before
          // add the existing config to neighbours
          neighbours.push_back(&graph[x_index][y_index - 1][t_index]);
          seen = true;
          break;
        }
      }
      if (!seen) {
        // add the updated config to graph and neighbours
        graph[x_index][y_index - 1][t_index] = y2;
        neighbours.push_back(&graph[x_index][y_index - 1][t_index]);
        seen_before.push_back(graph[x_index][y_index - 1][t_index]);
      }
    }
  }


  if (t_index + 1 < Ktheta) { // if the config is in bounds
      if (graph[x_index][y_index][t_index + 1].finished == false) { // if the config isn't finished
        bool seen = false;
        config t1;
        t1.x = n->x;
        t1.y = n->y;
        t1.angle = (n->angle) + grid_depth;
        t1.finished = false;
        t1.dist = numeric_limits<double>::infinity();
        t1.f = numeric_limits<double>::infinity();
        t1.parent = nullptr;
        for (int i = 0; i < seen_before.size(); i++) {
          if (same_config(seen_before[i], graph[x_index][y_index][t_index + 1])) { // if config has been seen before
            // add the existing config to neighbours
            neighbours.push_back(&graph[x_index][y_index][t_index + 1]);
            seen = true;
            break;
          }
        }
        if (!seen) {
          // add the updated config to graph and neighbours
          graph[x_index][y_index][t_index + 1] = t1;
          neighbours.push_back(&graph[x_index][y_index][t_index + 1]);
          seen_before.push_back(graph[x_index][y_index][t_index + 1]);
        }
      }
    }



    if (t_index - 1 < 0){
      // we want to be able to rotate both ways from 0 so if the index is negative adjust it to be valid
      // -1 would adjust to index 9, -2 to 8
      t_index += 9;
    }
    if (t_index - 1 >= 0) { // if the config is in bounds
      if (graph[x_index][y_index][t_index - 1].finished == false) { // if the config isn't finished
        bool seen = false;
        config t2;
        t2.x = n->x;
        t2.y = n->y;
        t2.angle = (n->angle) - grid_depth;
        if (t2.angle < 0){ // adjust so that angle wraps around 
          t2.angle = 360 + t2.angle;
        }
        t2.finished = false;
        t2.dist = numeric_limits<double>::infinity();
        t2.f = numeric_limits<double>::infinity();
        t2.parent = nullptr;
        for (int i = 0; i < seen_before.size(); i++) {
          if (same_config(seen_before[i], graph[x_index][y_index][t_index - 1])) { // if config has been seen before
            // add the existing config to neighbours
            neighbours.push_back(&graph[x_index][y_index][t_index - 1]);
            seen = true;
            break;
          }
        }
        if (!seen) {
          // add the updated config to graph and neighbours
          graph[x_index][y_index][t_index - 1] = t2;
          neighbours.push_back(&graph[x_index][y_index][t_index - 1]);
          seen_before.push_back(graph[x_index][y_index][t_index - 1]);
        }
      }
    }


    if (x_index + 1 < Kx && y_index + 1 < Ky) { // if the config is in bounds
      if (graph[x_index + 1][y_index + 1][t_index].finished == false) { // if the config isn't finished
        bool seen = false;
        config p1;
        p1.x = (n->x) + grid_width;
        p1.y = (n->y) + grid_height;
        p1.angle = n->angle;
        p1.finished = false;
        p1.dist = numeric_limits<double>::infinity();
        p1.f = numeric_limits<double>::infinity();
        p1.parent = nullptr;
        for (int i = 0; i < seen_before.size(); i++) {
          if (same_config(seen_before[i], graph[x_index + 1][y_index + 1][t_index])) { // if config has been seen before
            // add the existing config to neighbours
            neighbours.push_back(&graph[x_index + 1][y_index + 1][t_index]);
            seen = true;
            break;
          }
        }
        if (!seen) {
          // add the updated config to graph and neighbours
          graph[x_index + 1][y_index + 1][t_index] = p1;
          neighbours.push_back(&graph[x_index + 1][y_index + 1][t_index]);
          seen_before.push_back(graph[x_index + 1][y_index + 1][t_index]);
        }
      }
    }


    if (x_index + 1 < Kx && y_index - 1 >= 0) { // if the config is in bounds
      if (graph[x_index + 1][y_index - 1][t_index].finished == false) { // if the config isn't finished
        bool seen = false;
        config p2;
        p2.x = (n->x) + grid_width;
        p2.y = (n->y) - grid_height;
        p2.angle = n->angle;
        p2.finished = false;
        p2.dist = numeric_limits<double>::infinity();
        p2.f = numeric_limits<double>::infinity();
        p2.parent = nullptr;
        for (int i = 0; i < seen_before.size(); i++) {
          if (same_config(seen_before[i], graph[x_index + 1][y_index - 1][t_index])) { // if config has been seen before
            // add the existing config to neighbours
            neighbours.push_back(&graph[x_index + 1][y_index - 1][t_index]);
            seen = true;
            break;
          }
        }
        if (!seen) {
          // add the updated config to graph and neighbours
          graph[x_index + 1][y_index - 1][t_index] = p2;
          neighbours.push_back(&graph[x_index + 1][y_index - 1][t_index]);
          seen_before.push_back(graph[x_index + 1][y_index - 1][t_index]);
        }
      }
    }



    if (x_index - 1 >= 0 && y_index - 1 >= 0) { // if the config is in bounds
      if (graph[x_index - 1][y_index - 1][t_index].finished == false) { // if the config isn't finished
        bool seen = false;
        config p3;
        p3.x = (n->x) + grid_width;
        p3.y = (n->y) - grid_height;
        p3.angle = n->angle;
        p3.finished = false;
        p3.dist = numeric_limits<double>::infinity();
        p3.f = numeric_limits<double>::infinity();
        p3.parent = nullptr;
        for (int i = 0; i < seen_before.size(); i++) {
          if (same_config(seen_before[i], graph[x_index - 1][y_index - 1][t_index])) { // if config has been seen before
            // add the existing config to neighbours
            neighbours.push_back(&graph[x_index - 1][y_index - 1][t_index]);
            seen = true;
            break;
          }
        }
        if (!seen) {
          // add the updated config to graph and neighbours
          graph[x_index - 1][y_index - 1][t_index] = p3;
          neighbours.push_back(&graph[x_index - 1][y_index - 1][t_index]);
          seen_before.push_back(graph[x_index - 1][y_index - 1][t_index]);
        }
      }
    }



    if (x_index - 1 >= 0 && y_index + 1 < Ky) { // if the config is in bounds
      if (graph[x_index - 1][y_index + 1][t_index].finished == false) { // if the config isn't finished
        bool seen = false;
        config p4;
        p4.x = (n->x) + grid_width;
        p4.y = (n->y) - grid_height;
        p4.angle = n->angle;
        p4.finished = false;
        p4.dist = numeric_limits<double>::infinity();
        p4.f = numeric_limits<double>::infinity();
        p4.parent = nullptr;
        for (int i = 0; i < seen_before.size(); i++) {
          if (same_config(seen_before[i], graph[x_index - 1][y_index + 1][t_index])) { // if config has been seen before
            // add the existing config to neighbours
            neighbours.push_back(&graph[x_index - 1][y_index + 1][t_index]);
            seen = true;
            break;
          }
        }
        if (!seen) {
          // add the updated config to graph and neighbours
          graph[x_index - 1][y_index + 1][t_index] = p4;
          neighbours.push_back(&graph[x_index - 1][y_index + 1][t_index]);
          seen_before.push_back(graph[x_index - 1][y_index + 1][t_index]);
        }
      }
    }

    
    

    return neighbours;
}

void print_priority_queue(    priority_queue<pair<config*, double>, vector<pair<config*, double> >, CompareConfigByF> pq){
    while (!pq.empty()) {
        pair<config*, double> element = pq.top();
        pq.pop();
        printf("((x,y) --> f) ((%d, %d) --> %.2f) ", element.first->x, element.first->y, element.second);
    }
    printf("\n");
}

void print_config(config* c) {
    printf("x: %d, y: %d, angle: %d, dist: %f, f: %f, finished: %d\n", c->x, c->y, c->angle, c->dist, c->f, c->finished);
    if (c->parent != nullptr) {
      printf("parent: ");
      printf("x: %d, y: %d, dist: %f, f: %f, finished: %d\n", c->parent->x, c->parent->y, c->parent->dist, c->parent->f, c->parent->finished);
    }
}

// checks that if the configuration c is applied to the robot that all edges are in the window
bool all_edges_inbounds(config* c, robot r){
  robot new_robot = orient_robot(r, *c);
  // now we can check the vertices of new_robot and make sure they are within bounds
  for (int i = 0; i<new_robot.vertices.size(); i++){
    if (new_robot.vertices[i].x < 0 || new_robot.vertices[i].x > WINDOWSIZE){
      return false;
    }
    else if (new_robot.vertices[i].y < 0 || new_robot.vertices[i].y > WINDOWSIZE){
      return false;
    }
  }
  return true;
}


// same idea as dijkstra of visgraph except instead of a graph of nodes (x,y) we have a graph of placements (x,y,theta) made by our grid
// returns the shortest path
vector<config*> Astar(config***& graph, robot r, vector<config>& seen_before, vector<config>& in_pq){
    printf("running A*!\n");

    int final_angle = 0;
    int rotation_cost = 1;

    // Initialize a priority queue that holds a configuration and its f value
    priority_queue<pair<config*, double>, vector<pair<config*, double> >, CompareConfigByF> pq;  
    config* start_node = &graph[start_point.x / grid_width][start_point.y / grid_height][0];
    pq.push(make_pair(start_node, start_node->f));
    while(!pq.empty()){
        config* u = pq.top().first;
        pq.pop();
        u->finished = true;
        point2d endp = cell_center(end_point);
        if (u->x == (endp.x) && u->y == (endp.y)) { // found the goal node
        // log the angle so the config can be used to back track later to find the shortest path
          printf("Goal node found!\n");
          final_angle = u->angle;
          break;
        }
        vector<config*> neighbours = get_neighbour(u, graph, seen_before);
        for (int i = 0; i<neighbours.size(); i++){ // each placement node has 6 neighbours x+-, y+-, theta+-
          if (neighbours[i]->finished == false){
            if (all_edges_inbounds(neighbours[i], r)){ // make sure that the robot would be in bounds for that neighbour config
              point2d neighbor;
              neighbor.x = neighbours[i]->x;
              neighbor.y = neighbours[i]->y;
              point2d up;
              up.x = u->x;
              up.y = u->y;
              double alt = u->dist + distance_2d(up, neighbor);
              // Check if a rotation is required
              if (u->angle != neighbours[i]->angle) {
                alt += rotation_cost;  // Add a cost for rotation to discourage unnecessary rotation
              }
              if ((is_free(*neighbours[i], r, obstacles)) && (alt < neighbours[i]->dist)){
                neighbours[i]->dist = alt;
                neighbours[i]->f = neighbours[i]->dist + distance_2d(neighbor, end_point);
                neighbours[i]->parent = u;
                // Check if the neighbor has already been seen before
                bool seen = false;
                for (int j = 0; j < in_pq.size(); j++) {
                  if (same_config(*neighbours[i], in_pq[j])) {
                    seen = true;
                    break;
                  }
                }
                // If the neighbor has not been seen before, add it to the updated priority queue
                if (!seen) {
                  pq.push(make_pair(neighbours[i], neighbours[i]->f));
                  in_pq.push_back(*neighbours[i]);              
                }
              }
            }
          }
        }
      }
    printf("out of while loop!\n");
    vector<config*> path;
    config* current_node = &graph[end_point.x / grid_width][end_point.y / grid_height][final_angle / grid_depth];
    if (current_node->parent == nullptr){
      printf("No path possible.\n");
      exit(1);
    }
    printf("end node: ");
    print_config(current_node);
    while (!same_config(*current_node, *start_node)) {
      path.push_back(current_node);
      current_node = current_node->parent;
    }
    path.push_back(current_node);  // Add the start_node to the path

    // Reverse the path so that it goes from start to end
    reverse(path.begin(), path.end());

    return path;
}


void draw_path(vector<config> path){
    // Draw the shortest path
    glColor4f(0.0f, 1.0f, 0.0f, 1.0f); // set color to green
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < path.size(); i++) {
      glVertex2f(path[i].x, path[i].y);
    }
    glEnd();
}

// draw the config specified by the animation index
void draw_config(vector<config> path){
  printf("drawing config\n");
  if (animation_index >= path.size()){
    // reset animation once it reaches the end
    animation_index = 0;
  }
  config c = path[animation_index];
  robot r = orient_robot(global_robot, c);
  vector<point2d> pts = r.vertices;
  draw_polygon(pts, blue);
}



vector<config> planner(){
    printf("planner!\n");
    // generate a graph of configurations of robots with all the different placements Kx x Ky x Ktheta 
    
    // allocate memory for the 2D array and initialize all elements to a default config object
    config*** graph = (config***)malloc(Kx * sizeof(config**));
    for (int i = 0; i < Kx; i++) {
      graph[i] = (config**)malloc(Ky * sizeof(config*));
      for (int j = 0; j < Ky; j++) {
        graph[i][j] = (config*)malloc(Ktheta * sizeof(config));
      }
    }


    vector<config> seen_before;
    vector<config> in_pq;

    

    // start_node object to the 2D array
    config start;
    start.x = start_point.x;
    start.y = start_point.y;
    start.angle = 0;
    start.dist = 0;
    start.finished = false;
    start.f = distance_2d(start_point, end_point);
    start.parent = nullptr;
    graph[start.x / grid_width][start.y / grid_height][start.angle / grid_depth] = start;
    seen_before.push_back(start);


    config end;
    end.x = end_point.x;
    end.y = end_point.y;
    end.angle = 0;
    end.dist = numeric_limits<double>::infinity();
    end.f = numeric_limits<double>::infinity();
    end.finished = false;
    end.parent = nullptr;
    graph[end.x / grid_width][end.y / grid_height][end.angle / grid_depth] = end;

    seen_before.push_back(end);


    robot r = init_robot(robot_points);
    global_robot = r;
    printf("starter robot config: (%d, %d, %d)\n", r.c.x, r.c.y, r.c.angle);

    vector<config*> path = Astar(graph, r, seen_before, in_pq);

    vector<config> final_path;

    for (int i = 0; i<path.size(); i++){
      final_path.push_back(*path[i]);
    }

    // Free the memory allocated for the 3D array
    for (int i = 0; i < Kx; i++) {
      for (int j = 0; j < Ky; j++) {
        free(graph[i][j]);
      }
    free(graph[i]);
    }
    free(graph);

    return final_path;
}







// split the points into pairs of points where each pair represents a line segment in a polygon
vector<edge> CreateSegmentsFromPolygon(vector<point2d>& poly) {
  vector<edge> segments;
  for(int i = 0; i < poly.size()-1; i++) {
    edge e;
    e.a = &poly[i];
    e.b = &poly[i+1];
    segments.push_back(e);
  }
  edge d;
  d.a = &poly.back();
  d.b = &poly.front();
  segments.push_back(d); // add last segment to close polygon
  return segments;
}

vector<edge> CreateSegmentsFromObstacles(vector<vector<point2d> >& obstacles){
    vector<edge> obstacle_edges;
    for (int i = 0; i<obstacles.size(); i++){
      for (int j = 0; j<obstacles[i].size() - 1; j++){
        edge e;
        e.a = &obstacles[i][j];
        e.b = &obstacles[i][j+1];
        obstacle_edges.push_back(e);
      }
      edge start_end;
      start_end.a = &obstacles[i].back();
      start_end.b = &obstacles[i].front();
      obstacle_edges.push_back(start_end);
    }
    return obstacle_edges;
}

void print_edges(vector<edge> segments){
    printf("edges: \n");
    for (int i = 0; i<segments.size(); i++){
        printf("\t(%d, %d) (%d, %d)\n", segments[i].a->x,segments[i].a->y, segments[i].b->x,segments[i].b->y);
    }
}

//return true if vertex p is visible from point q
bool isVisible(point2d p, point2d q, vector<vector<point2d> > obstacles, point2d& intersection) {
  // we want to cast a ray from point p to q and return false if there are any intersections
  // first we can create a line segment from p to q
  edge pq;
  pq.a = &p;
  pq.b = &q;

  // first we want to make sure that the ray does not pass through any obstacles on its way from p to q
  point2d testq;
  testq.x = p.x + (q.x - p.x)/10;
  testq.y = p.y + (q.y - p.y)/10;
  while (distance_2d(testq, q) > 0.001){
    for (int i = 0; i<obstacles.size(); i++){
      if (pointInPolygon(obstacles[i], testq)){
        return false;
      }
  }
  testq.x += (q.x - p.x)/10;
  testq.y += (q.y - p.y)/10;
  }



  // we want to shorten the ray just short of the vertex so that it doesn't confuse that as an intersection
  pq.b->x -= (q.x - p.x)/100;
  pq.b->y -= (q.y - p.y)/100;


  // next we want to check if the ray intersects any of the obstacles on its way to q
  vector<edge> segments = CreateSegmentsFromObstacles(obstacles);
  //print_edges(segments);
  char code = '?';
  for (int i = 0; i<segments.size(); i++){
    code = SegSegInt(*pq.a, *pq.b, *segments[i].a, *segments[i].b, intersection);
    if (code == '1'){ // the ray has intersected an edge 
      return false;
    }
  }
  // no intersections so point p is visible from poly[i]
  return true;

}


// returns the index of the predecessor of index i in the vector poly
int predecessor(vector<point2d>& poly, int i){
  if (i == 0){
    return poly.size() - 1;
  }
    return i - 1;
}

// returns the index of the successor of index i in the vector poly
int successor(vector<point2d>& poly, int index){
  if (index == poly.size()-1){
    return 0;
  }
  return index + 1;
}


//return true if the polygon is simple, i.e there are no intersecting lines
bool polygonSimple(vector<point2d>& poly){
  //create pairs of line segments
  vector<edge> segments = CreateSegmentsFromPolygon(poly);
  for (int i = 0; i < segments.size(); i++){
    for (int j = i + 1; j < segments.size(); j++){
      point2d p; //point to store intersection returned by SegSegInt
      if (SegSegInt(*segments[i].a, *segments[i].b, *segments[j].a, *segments[j].b, p) == '1'){
        return false;
      }
    }
  }
  return true;
}

void initialise_debug_scene(vector<vector<point2d> >& obstacles, vector<point2d>& robot_points){
  printf("initialising scene to debug!\n");
  obstacles.clear();
  robot_points.clear();

  vector<point2d> obstacle1;
  vector<point2d> obstacle2;
  vector<point2d> obstacle3;


  point2d p;
  p.x = 240;
  p.y = 432;
  obstacle1.push_back(p);
  p.x = 224;
  p.y = 365;
  obstacle1.push_back(p);
  p.x = 321;
  p.y = 346;
  obstacle1.push_back(p);
  p.x = 350;
  p.y = 451;
  obstacle1.push_back(p);
  p.x = 176;
  p.y = 501;
  obstacle1.push_back(p);

  p.x = 494;
  p.y = 381;
  obstacle2.push_back(p);
  p.x = 490;
  p.y = 285;
  obstacle2.push_back(p);
  p.x = 620;
  p.y = 285;
  obstacle2.push_back(p);
  p.x = 634;
  p.y = 456;
  obstacle2.push_back(p);
  p.x = 444;
  p.y = 474;
  obstacle2.push_back(p);

  p.x = 29;
  p.y = 259;
  obstacle3.push_back(p);
  p.x = 57;
  p.y = 78;
  obstacle3.push_back(p);
  p.x = 552;
  p.y = 58;
  obstacle3.push_back(p);
  p.x = 589;
  p.y = 174;
  obstacle3.push_back(p);
  p.x = 399;
  p.y = 277;
  obstacle3.push_back(p);

  obstacles.push_back(obstacle1);
  obstacles.push_back(obstacle2);
  obstacles.push_back(obstacle3);




  vector<point2d> robot;

  p.x = 574;
  p.y = 630;
  robot.push_back(p);
  p.x = 572;
  p.y = 608;
  robot.push_back(p);
  p.x = 610;
  p.y = 611;
  robot.push_back(p);
  

  robot_points = robot;
}

void print_path(vector<config> path){
  for (int i = 0; i<path.size(); i++){
    print_config(&path[i]);
  }
}



/* ****************************** */
int main(int argc, char** argv) {

  //initialise_debug_scene(obstacles, robot_points);


  // initialize GLUT  
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(WINDOWSIZE, WINDOWSIZE);
  glutInitWindowPosition(100,100);
  glutCreateWindow(argv[0]);

  // register callback functions 
  glutDisplayFunc(display); 
  glutKeyboardFunc(keypress);
  glutMouseFunc(mousepress); 
  glutIdleFunc(timerfunc); //register this if you want it called at every frame

  // init GL 
  // set background color black
  glClearColor(0, 0, 0, 0);   
  
  // give control to event handler 
  glutMainLoop();

  return 0;
}

/* ****************************** */
/* draw the polygon */
void draw_polygon(vector<point2d>& poly, GLfloat* color){
   if (poly.size() == 0) return; 

  //set color  and polygon mode 
  glColor3fv(color);   
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
 
  int i;
  for (i=0; i<poly.size()-1; i++) {
    //draw a line from point i to i+1
    glBegin(GL_LINES);
    glVertex2f(poly[i].x, poly[i].y); 
    glVertex2f(poly[i+1].x, poly[i+1].y);
    glEnd();
  }
  //draw a line from the last point to the first  
  int last=poly.size()-1; 
    glBegin(GL_LINES);
    glVertex2f(poly[last].x, poly[last].y); 
    glVertex2f(poly[0].x, poly[0].y);
    glEnd();
}

void draw_obstacles(vector<vector<point2d> > obstacles){
    for (int i = 0; i<obstacles.size(); i++){
        draw_polygon(obstacles[i], yellow);
    }
}

void draw_start(point2d start){
  glColor3fv(red);
  glPointSize(5.0f);
  glBegin(GL_POINTS);
  glVertex2f(start_point.x, start_point.y);
  glEnd();
}

void draw_end(point2d end){
  glColor3fv(red);
  glPointSize(5.0f);
  glBegin(GL_POINTS);
  glVertex2f(end_point.x, end_point.y);
  glEnd();
}

void draw_robot(vector<point2d>& robot){
  draw_polygon(robot, blue);
}

// check if a point is in the polygon
bool pointInPolygon(vector<point2d>& poly, point2d p){
  int n = poly.size();
  int crossings = 0;
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    // Check if segment intersects the ray cast right from p
    if (((poly[i].y <= p.y && p.y < poly[j].y) || (poly[j].y <= p.y && p.y < poly[i].y))
        && (poly[i].x > p.x || poly[j].x > p.x)) {
      // Compute the x-coordinate of the intersection of the segment with the ray
      double x = poly[i].x + (p.y - poly[i].y) * (poly[j].x - poly[i].x) / (poly[j].y - poly[i].y);
      if (p.x < x) {
        crossings++;
      }
    }
  }
  // If number of crossings is odd, the point is inside the polygon
  return (crossings % 2 == 1);
}

// return true if p1 and p2 are identical
bool compare_2dpoints (point2d p1, point2d p2){
  return ((p1.x == p2.x) && (p1.y == p2.y));
}

// return true if edge1 < edge 2
// ordered by x, then y
bool edgesCmp(edge edge1, edge edge2){
  if (compare_2dpoints(*(edge1.a), *(edge2.a))){
    if (edge1.b->x < edge2.b->x){
      return true;
    }
    else if (edge1.b->x == edge2.b->x){
      if (edge1.b->y < edge2.b->y){
        return true;
      }
    return false;
  }
  }
  if (edge1.a->x < edge2.a->x){
    return true;
  }
  else if (edge1.a->x == edge2.a->x){
    if (edge1.a->y < edge2.a->y){
      return true;
    }
  return false;
  }
  return false;
}

// return true if e1 and e2 are the same edge
bool same_edge(edge e1, edge e2){
    return ((compare_2dpoints(*e1.a, *e2.a) && compare_2dpoints(*e1.b, *e2.b)) || (compare_2dpoints(*e1.a, *e2.b) && compare_2dpoints(*e1.b, *e2.a)));
}

void removeDuplicateEdges(vector<edge>& graph) {
    vector<edge> no_duplicates;
    sort(graph.begin(), graph.end(), edgesCmp);
    no_duplicates.push_back(graph[0]);
    edge last_added = graph[0];
    for (int i = 1; i<graph.size(); i++){
        if (same_edge(last_added, graph[i])){
            continue; // don't want to add to vector again
        }
        else{
            no_duplicates.push_back(graph[i]);
            last_added = graph[i];
        }
    }
    graph = no_duplicates;
}


void find_neighbours(point2d p, vector<edge> graph, adjnode& node){
    vector<point2d> neigh;
    for (int k = 0; k<graph.size(); k++){
        //go through the graph and find the neighbours of the current vertex
        if (compare_2dpoints(*graph[k].a, p)){
            neigh.push_back(*graph[k].b);
        }
        else if (compare_2dpoints(*graph[k].b, p)){
            neigh.push_back(*graph[k].a);
        }
    }
    node.neighbours = neigh;
}


// return true if vertex v is a neighbour of vertex u
bool neighbour(int u, int v, vector<adjnode> adj_list){
    vector<int> neighbours = find_neighbour_indices(adj_list, u);
    for (int i = 0; i<neighbours.size(); i++){
        if (v == neighbours[i]){
            return true;
        }
    }
    return false;
}

vector<int> find_neighbour_indices(vector<adjnode> adj_list, int vertexIndex){
    set<int> neighbour_indices; // use a set instead of a vector
    for (int k = 0; k<adj_list[vertexIndex].neighbours.size(); k++){
        // for each neighbour of the specified vertex
        point2d neighbour = adj_list[vertexIndex].neighbours[k];
        for (int j = 0; j<adj_list.size(); j++){ //for each vertex determine what the index of the neighbour is
            if (compare_2dpoints(neighbour, *adj_list[j].v)){
                int neighbour_index = adj_list[j].i;
                neighbour_indices.insert(neighbour_index); // insert index into the set
                break; // exit loop since index has been found
            }
        }
    }
    // convert the set to a vector and return it so that there are no duplicates
    return vector<int>(neighbour_indices.begin(), neighbour_indices.end());
}



void print_graph(vector<edge>& graph){
  printf("graph: ");
  for (int i = 0; i<graph.size(); i++){
    printf("\t(%d, %d) (%d, %d)\n", graph[i].a->x, graph[i].a->y, graph[i].b->x, graph[i].b->y);
  }
}

// Returns true if vertex b is an ear of the polygon defined by vertices a, b, and c
bool is_ear(vector<point2d>& polygon, int a, int b, int c) {
    const point2d& pa = polygon[a];
    const point2d& pb = polygon[b];
    const point2d& pc = polygon[c];
    if (!left_strictly(pa, pb, pc)) {
        return false;
    }
    for (int i = 0; i < polygon.size(); ++i) {
        if (i == a || i == b || i == c) {
            continue;
        }
        if (pointInTriangle(polygon[i], pa, pb, pc)) {
            return false;
        }
    }
    return true;
}

vector<triangle> triangulate_polygon(vector<point2d>& poly) {
    vector<triangle> triangles;
    int n = poly.size();
    vector<int> indices(n);
    for (int i = 0; i < n; ++i) {
        indices[i] = i;
    }
    while (n > 3) {
        bool ear_found = false;
        for (int i = 0; i < n; ++i) {
            int a = indices[(i + n - 1) % n];
            int b = indices[i];
            int c = indices[(i + 1) % n];
            if (is_ear(poly, a, b, c)) {
                triangle t;
                t.a = poly[a];
                t.b = poly[b];
                t.c = poly[c];
                triangles.push_back(t);
                indices.erase(indices.begin() + i);
                n--;
                ear_found = true;
                break;
            }
        }
        if (!ear_found) {
            // The polygon cannot be triangulated
            return vector<triangle>();
        }
    }
    triangle tri;
    tri.a = poly[indices[0]];
    tri.b = poly[indices[1]];
    tri.c = poly[indices[2]];
    triangles.push_back(tri);
    return triangles;
}



void draw_filled_obstacles(vector<vector<point2d> >  obstacles){

    // Define an array of colors for each obstacle
    GLfloat colors[][4] = {
    { 1.0f, 0.0f, 0.0f, 0.2f },  // Red
    { 0.0f, 1.0f, 0.0f, 0.2f },  // Green
    { 0.0f, 0.0f, 1.0f, 0.2f },  // Blue
    { 1.0f, 1.0f, 0.0f, 0.2f },  // Yellow
    { 1.0f, 0.0f, 1.0f, 0.2f },  // Magenta
    { 0.0f, 1.0f, 1.0f, 0.2f },  // Cyan
    // Add more colors as needed
    };
    // for each obstacle
    for (int i = 0; i<obstacles.size(); i++){
        vector<triangle> obstacle_triangulation = triangulate_polygon(obstacles[i]);

        if (obstacle_triangulation.size() == 0) return;

        // Enable blending for transparency
        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Specify polygon color and transparency
        glColor4fv(colors[i % (sizeof(colors) / sizeof(colors[0]))]); // 50% transparency

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Begin rendering the filled polygon
        glBegin(GL_TRIANGLES);

        // loop through each triangulation and render it translucent and filled
        for (int i = 0; i < obstacle_triangulation.size(); i++){
        glVertex2f(obstacle_triangulation[i].a.x, obstacle_triangulation[i].a.y);
        glVertex2f(obstacle_triangulation[i].b.x, obstacle_triangulation[i].b.y);
        glVertex2f(obstacle_triangulation[i].c.x, obstacle_triangulation[i].c.y);

        }

        // End rendering the polygon
        glEnd();

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        // Disable blending
        glDisable(GL_BLEND);
    }
}


// initialise a robot object from the points drawn
robot init_robot(vector<point2d> robot_points){
  printf("initialising robot\n");
    robot r;
    r.vertices = robot_points;
    config cf;
    printf("start point: (%d, %d): \n", start_point.x, start_point.y);
    cf.x = robot_points[0].x;
    cf.y = robot_points[0].y;
    cf.parent = nullptr;
    cf.f = 0; // doesn't matter, never used
    cf.dist = 0; // doesn't matter, never used
    cf.angle = 0;
    r.c = cf;
    printf("in init: (%d, %d): \n", r.c.x, r.c.y);
    return r;
}


void draw_space() {
  // Set the color of the grid lines to light gray
  glColor3f(0.7, 0.7, 0.7);
  // Set the line width to 1 pixel
  glLineWidth(1.0);
  
  // Draw vertical grid lines
  for (int i = 1; i < Kx; i++) {
    // Calculate the x-coordinate of the current grid line
    float x = i * (float)WINDOWSIZE / Kx;
    glBegin(GL_LINES);
      glVertex2f(x, 0);
      glVertex2f(x, WINDOWSIZE);
    glEnd();
  }
  
  // Draw horizontal grid lines
  for (int i = 1; i < Ky; i++) {
    // Calculate the y-coordinate of the current grid line
    float y = i * (float)WINDOWSIZE / Ky;
    glBegin(GL_LINES);
      glVertex2f(0, y);
      glVertex2f(WINDOWSIZE, y);
    glEnd();
  }
}




void draw_complete_path(vector<config*>& path, robot starter_robot){
  // given the starter robot drawn by the user
  // now we want to orient the new_robot so that it matches the config of the current node in the path
  // draw it 
  // and then we can move onto the next node and repeat
  for (int i = 0; i<path.size(); i++){
    config c = *path[i];
    robot new_robot = orient_robot(starter_robot, c);
    draw_polygon(new_robot.vertices, blue);
    // move to next config of path 
  }
}

/* ****************************** */
/* This is the main render function. We registered this function to be
   called by GL to render the window. 
 */
void display(void) {

  glClear(GL_COLOR_BUFFER_BIT);
  //clear all modeling transformations 
  glMatrixMode(GL_MODELVIEW); 
  glLoadIdentity();


  /* The default GL window is [-1,1]x[-1,1] with the origin in the
     center.  The camera is at (0,0,0) looking down negative
     z-axis.  

     The points are in the range (0,0) to (WINSIZE,WINSIZE), so they
     need to be mapped to [-1,1]x [-1,1] */
  
  //First we scale down to [0,2] x [0,2] */ 
  glScalef(2.0/WINDOWSIZE, 2.0/WINDOWSIZE, 1.0);  
  /* Then we translate so the local origin goes in the middle of teh
     window to (-WINDOWSIZE/2, -WINDOWSIZE/2) */
  glTranslatef(-WINDOWSIZE/2, -WINDOWSIZE/2, 0); 
  
  //now we draw in our local coordinate system (0,0) to
  //(WINSIZE,WINSIZE), with the origin in the lower left corner.

 // draw_space();

  draw_obstacles(obstacles); 
  //printf("obstacles drawn\n");
  draw_start(start_point);
  //printf("start drawn\n");
  draw_end(end_point);
  //printf("end drawn\n");
  draw_robot(robot_points);
  //printf("robot drawn\n");
  if (fill_mode == 1){
    draw_filled_obstacles(obstacles);
  }

  

  if (path_mode == 1){
    vector<config> path = planner();
    draw_path(path);
    if (animation_mode == 1){
      draw_config(path);
    }
  }

  /* execute the drawing commands */
  glFlush();
}



/* ****************************** */
void keypress(unsigned char key, int x, int y) {
  vector<point2d> new_obstacle;
  switch(key) {
  case 'q':	
    exit(0);
    break;
  case 's': // to override the default obstacles and start drawing your own 
    printf("drawing obstacles...\n");
    obstacles.clear();
    current_obstacle = 0;
    obstacles.push_back(new_obstacle);
    poly_init_mode = 1;
    break;
  case 'n': // draw a new obstacle
    printf("drawing another obstacle!\n");
    current_obstacle++;
    new_obstacle.clear();
    obstacles.push_back(new_obstacle);
    break;
  case 'r': // draw the robot
    robot_points.clear();
    printf("drawing the robot\n");
    poly_init_mode = 4;
    break;
  case 'f': // fill the polygon
    if(fill_mode == 0) {
        fill_mode = 1;
    } else {
        fill_mode = 0;
    }
    glutPostRedisplay();
    break;
  case 'a': // choose the start position
    printf("click to choose the start position\n");
    poly_init_mode = 2;
    glutPostRedisplay();
    break;
  case 'z': // choose the end position
    printf("click to choose the end position\n");
    poly_init_mode = 3;
    glutPostRedisplay();
    break;
  case 'h': // compute approx path
    path_mode = 1;
    glutPostRedisplay();
  case 'g': // start/stop animation
    if(animation_mode == 0) {
        animation_mode = 1;
    } else {
        animation_mode = 0;
    }
    break;
  }
}

// return the adjusted coordinates for the center of the cell that p falls into
point2d cell_center(point2d p){
  // Calculate the index of the cell that the mouse click falls in
    int i = (int)(p.y / grid_height);
    int j = (int)(p.x / grid_width);

    // Calculate the center point of the cell
    double cell_center_x = (j + 0.5) * grid_width;
    double cell_center_y = (i + 0.5) * grid_height;

    point2d adjusted_p;
    adjusted_p.x = cell_center_x;
    adjusted_p.y = cell_center_y;

    return adjusted_p;
}

/* 
void glutMouseFunc(void (*func)(int button, int state, int x, int y));

glutMouseFunc sets the mouse callback for the current window. When a
user presses and releases mouse buttons in the window, each press and
each release generates a mouse callback. The button parameter is one
of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON. For
systems with only two mouse buttons, it may not be possible to
generate GLUT_MIDDLE_BUTTON callback. For systems with a single mouse
button, it may be possible to generate only a GLUT_LEFT_BUTTON
callback. The state parameter is either GLUT_UP or GLUT_DOWN
indicating whether the callback was due to a release or press
respectively. The x and y callback parameters indicate the window
relative coordinates when the mouse button state changed. If a
GLUT_DOWN callback for a specific button is triggered, the program can
assume a GLUT_UP callback for the same button will be generated
(assuming the window still has a mouse callback registered) when the
mouse button is released even if the mouse has moved outside the
window.
*/
void mousepress(int button, int state, int x, int y) {

  if(state == GLUT_DOWN) { //mouse click detected 
    printf("poly_init_mode: %d\n",  poly_init_mode);

    //(x,y) are in window coordinates, where the origin is in the upper
    //left corner; our reference system has the origin in lower left
    //corner, this means we have to reflect y
    mouse_x = (double) x;
    mouse_y = (double) (WINDOWSIZE - y); 
    point2d mouse;
    mouse.x = mouse_x;
    mouse.y = mouse_y;
    printf("mouse pressed at (%.1f,%.1f)\n", mouse_x, mouse_y); 

    

    point2d p = cell_center(mouse);


    if (poly_init_mode == 1){ // s has been pressed and we want to add each mouse press to the current obstacle
      //add this point to poly
      obstacles[current_obstacle].push_back(p);
    }
    else if (poly_init_mode == 2){ // we want to make the click the start position
      for (int i = 0; i < obstacles.size(); i++){
        if (pointInPolygon(obstacles[i], p)){
            printf("whoopsie, you can't put the start point inside an obstacle!\n Have another go...\n");
        }
        else{
          start_point = p;
        }
      }
    }
    else if (poly_init_mode == 3){ // we want to make the click the end position
      for (int i = 0; i < obstacles.size(); i++){
        if (pointInPolygon(obstacles[i], p)){
            printf("whoopsie, you can't put the end point inside an obstacle!\n Have another go...\n");
        }
        else{
          end_point = p;
        }
      }  
    }
    else if (poly_init_mode == 4){
      robot_points.push_back(p);
    }
  }
  glutPostRedisplay();
}





//this function is called every frame. Use for animations 
void timerfunc() {
  //printf("hello\n");
  if (animation_mode == 1){
    printf("animation_index: %d\n", animation_index);
    animation_index++;
    glutPostRedisplay();
  }
  
}


