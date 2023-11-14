//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Exercise]
// Stephen J. Guy <sjguy@umn.edu>

/*
TODO:
  1. Add a function so that when you press the 'r' key, the obstacles move to
     new random locations with new random radii. Also, be sure to make a new
     PRM and plan a path over these new obstacles.
     [Hint: Look at setup() to see how I place obstacles, build the PRM, and run the BFS]
  2. Currently edge connections of any length are allowed. Update the connectNeighbors()
     function to only allow edges if their length is less than 200 pixels. What use is there
     for this? Why do you think people sometimes put a maximum length on PRM edges?
  3. The function, closestNode() is supposed to return the ID of the PRM node 
     that is closest to the passed-in point. However, it currently returns a 
     random node. Once you fix the function, you should see that clicking with 
     the mouse lets you select the red goal node.
  4. You can see we define a box obstacle with a top left corner, and a width and height.
     I have already written code to draw this box with the rect() command. Uncomment
     the code, to see the box drawn.
  5. For the PRM to be collision free w.r.t. the box we need to make sure no nodes are
     inside and box, and that none of the between-none edges between nodes intersect the box:
       A. Complete in the pointInBox() routine in the CollisionLibrary
       B. Use pointInBox() inside generateRandomNodes() to ensure all PRM nodes
          generated are outside of the box
       C. Use the already existing rayBoxIntersect() to test the edges between 
          PRM nodes with the box
    Test each sub-step individually. If all three work, the planned path should
    be collision free with all of the circles and the box.
  6. Pressing the arrow keys will move the pink box. Update the code to make
    sure the PRM and the planned path get updated as the box moves.
    
Challenge:
  1. Add support for a list of rectangle obstacles.
  2. Let the user use the mouse to click and drag the obstacles.
*/
import java.util.*;

//A list of circle obstacles
static int numObstacles = 50;
Vec2 circlePos[] = new Vec2[numObstacles]; //Circle positions
float circleRad[] = new float[numObstacles];  //Circle radii

static int numNodes = 500;

//A box obstacle
Vec2 boxTopLeft = new Vec2(100,100);
float boxW = 100;
float boxH = 250;
float goalSpeed = 200;

int numAgents = 30;
Vec2 startPos = new Vec2(100,500);
Vec2 goalPos = new Vec2(500,200);
Vec2[] agentPos = new Vec2[numAgents];
Vec2[] agentVel = new Vec2[numAgents];
Vec2[] agentAcc = new Vec2[numAgents];
Vec2[] agentGoal = new Vec2[numAgents];
ArrayList<Vec2>[] agentPath = new ArrayList[numAgents];
int agentSize = 10;

float tH = 10;
float maxF = 1000;
float maxSep = 40;

void placeRandomObstacles(){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
  }
}

int strokeWidth = 2;

void resetPaths() {
   for (int i = 0; i < numAgents; i++) {
    agentGoal[i] = new Vec2(random(goalPos.x - goalRad,goalPos.x + goalRad), random(goalPos.y - goalRad,goalPos.y + goalRad));
    runBFS(closestNode(agentPos[i]),closestNode(agentGoal[i]));
    agentPath[i] = new ArrayList<>((ArrayList)path.clone());
    agentPath[i].add(agentGoal[i]);
    vis[i] = new ArrayList<>(Collections.nCopies(agentPath[i].size(), false));
  }
  runBFS(closestNode(startPos), closestNode(goalPos));
}
void setup(){
  size(1024,768);
  placeRandomObstacles();
  placeAgents();
  buildPRM(circlePos, circleRad, boxTopLeft, boxW, boxH);
  resetPaths();
}

int startRad = 200, goalRad = 20;
void placeAgents() {
   for (int i = 0; i < numAgents; i++){
    agentPos[i] = new Vec2(random(startPos.x - startRad,startPos.x + startRad),random(startPos.y - startRad,startPos.y + startRad));
    agentGoal[i] = new Vec2(random(goalPos.x - goalRad,goalPos.x + goalRad), random(goalPos.y - goalRad,goalPos.y + goalRad));
    //agentGoal[i] = new Vec2(goalPos.x, goalPos.y);
    agentVel[i] = agentGoal[i].minus(agentPos[i]);
    agentVel[i].setToLength(goalSpeed);
    agentAcc[i] = new Vec2(0, 0);
    movingTo[i] = -1;  
  }
}

void draw(){
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(200); //Grey background
  stroke(0,0,0);
  
  if (!paused){
    moveAgent(1.0/frameRate);
  }
  
  //Draw the circle obstacles
   fill(250,30,50);
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    circle(c.x,c.y,r*2);
  }
  
  //Draw the box obstacles
  //TODO: Uncomment this to draw the box
  //fill(250,200,200);
  //rect(boxTopLeft.x, boxTopLeft.y, boxW, boxH);
  
  //Draw Start and Goal
  fill(20,60,250);
  circle(nodePos[startNode].x,nodePos[startNode].y,20);
  //circle(startPos.x,startPos.y,20);
  
  for (int i = 0; i < numAgents; i++) {
    fill(20,250,60);
    circle(agentPos[i].x,agentPos[i].y,agentSize*2);
    Vec2 dir = agentVel[i];
    dir.setToLength(agentSize);
    Vec2 tip = agentPos[i].plus(dir);
    Vec2 neg_tip = agentPos[i].minus(dir);
    Vec2 neg_tip_rel = neg_tip.minus(agentPos[i]);
    Vec2 left = neg_tip_rel.rotated((float)Math.toRadians(25.)).plus(agentPos[i]);
    Vec2 right = neg_tip_rel.rotated((float)Math.toRadians(-25.)).plus(agentPos[i]);
    fill(#ff8c00);
    triangle(tip.x, tip.y, left.x, left.y, right.x, right.y);
    //circle(left.x, left.y, 10);
  }
  fill(#ffae42);
  circle(nodePos[goalNode].x,nodePos[goalNode].y,20);
  //circle(goalPos.x,goalPos.y,20);
  
  //Draw Planned Path
 
  stroke(20,255,40);
  strokeWeight(5);
  for (int i = 0; i < path.size() - 1; i++) {
      Vec2 curNode = path.get(i);
      Vec2 nextNode =path.get(i + 1);
      line(curNode.x,curNode.y,nextNode.x, nextNode.y);
  }
 
}

int[] movingTo = new int[numNodes];

float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
 
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
 
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length()*l_dir.length();
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
 
  float d = b*b - 4*a*c; //discriminant
 
  if (d >=0 ){
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision!
    if (t >= 0) return t;
    return -1;
  }
 
  return -1; //We are not colliding, so there is no good t to return
}

//Return at what time agents 1 and 2 collide if they keep their current velocities
// or -1 if there is no collision.
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  return rayCircleIntersectTime(pos1, radius1+radius2, pos2, vel1.minus(vel2)); 
}

// Compute attractive forces to draw agents to their goals,
// and avoidance forces to anticipatory avoid collisions

Vec2 computeAgentForces(int i){
  //TODO: Make this better
  Vec2 acc = new Vec2(0, 0);
  for (int j = 0; j < numAgents; j++) {
    float d = agentPos[i].distanceTo(agentPos[j]);
    if (d < 1e-3 || d > maxSep) continue;
    float t = computeTTC(agentPos[i], agentVel[i], agentSize, agentPos[j], agentVel[j], agentSize);
    Vec2 fAvoid = agentPos[i].plus(agentVel[i].times(t)).minus(agentPos[j]).minus(agentVel[j].times(t));
    float mag = 0;
    if (t >= 0 && t <= tH)
      mag = (tH - t) / (t + 1e-6);
    mag = max(mag, maxF);
    fAvoid.setToLength(mag);
    acc.add(fAvoid);
  }
  
  for (int j = 0; j < numObstacles; j++) {
    Vec2 center = circlePos[j];
    float r = circleRad[j];
    float d = agentPos[i].distanceTo(center);
    if (d < 1e-3 || d > maxSep) continue;
    if (d <= agentSize + r) {
      Vec2 dir = agentPos[i].minus(center);
      agentPos[i].plus(dir);
      dir.setToLength(agentVel[i].length());
      agentVel[i].plus(dir);
      continue;
    }
    float t = computeTTC(agentPos[i], agentVel[i], agentSize, center, new Vec2(0, 0), r);
    Vec2 fAvoid = agentPos[i].plus(agentVel[i].times(t)).minus(center);
    float mag = 0;
    if (t >= 0 && t <= tH)
      mag = (tH - t) / (t + 1e-6);
    mag = max(mag*3, maxF);
    fAvoid.setToLength(mag);
    acc.add(fAvoid);
  }
  return acc;
}

Vec2[] newVel = new Vec2[numAgents];
Vec2[] newPos = new Vec2[numAgents];  
//boolean[] vis = new boolean[numNodes];
ArrayList<Boolean>[] vis = new ArrayList[numAgents];
int closestNodeInPath(int i) {
  int res = -1;
  float min_d = Float.POSITIVE_INFINITY;
  for (int j = 0; j < agentPath[i].size(); j++) {
     Vec2 p = agentPath[i].get(j);
     float d = p.distanceTo(agentPos[i]);
      if (!vis[i].get(j) && d < min_d) {
        min_d = d;
        res = j;
      }
  }
  return res;
}

void moveAgent(float dt) {
  for (int i = 0; i < numAgents; i++) {
    //println(Arrays.toString(agentVel));
    agentAcc[i] = computeAgentForces(i);
    /*if (movingTo[i] == -1 || (movingTo[i] < agentPath[i].size() && nodePos[agentPath[i].get(movingTo[i])].distanceTo(agentPos[i]) < 10)) {
      movingTo[i]++;
      if (movingTo[i] < agentPath[i].size()) {
        float prevSpeed = agentVel[i].length();
        agentVel[i] = nodePos[agentPath[i].get(movingTo[i])].minus(agentPos[i]);
        agentVel[i].setToLength(prevSpeed);
      } 
    }*/
    int closest = closestNodeInPath(i);
    //println("agentPath " + i + ": " + agentPath[i]);
    println("agent " + i + ": " + closest);
  // println("agent " + i + " vel: " + agentVel[i]);
    if (closest != -1) {
        //float prevSpeed = agentVel[i].length();
        Vec2 p = agentPath[i].get(closest);
        agentVel[i] = p.minus(agentPos[i]);
        agentVel[i].setToLength(goalSpeed);
        if (p.distanceTo(agentPos[i]) <= 10) {
          vis[i].set(closest, true);
        }
    }
    newVel[i] = agentVel[i].plus(agentAcc[i].times(dt));
    newPos[i] = agentPos[i].plus(agentVel[i].times(dt));
  }
  println("agentAcc: " + Arrays.toString(agentAcc));
 /*
  for (int i = 0; i < numAgents - 1; i++) {
    for (int j = i + 1; j < numAgents; j++) {
      float d = newPos[i].distanceTo(newPos[j]);
      if (d <= agentSize + agentSize) {
        Vec2 dir = newPos[i].minus(newPos[j]).times(.5);
        newPos[i].plus(dir);
        newPos[j].plus(dir.negate());
      }
   }
  }
  for (int i = 0; i < numAgents; i++) {
    for (int j = 0; j < numObstacles; j++) {
      Vec2 center = circlePos[j];
      float r = circleRad[j];
      float d = newPos[i].distanceTo(center);
      if (d <= agentSize + r) {
        Vec2 dir = newPos[i].minus(center);
        newPos[i].plus(dir);
      }
    }
  }*/
  agentVel = newVel.clone();
  agentPos = newPos.clone();
}

Boolean paused = true;
void keyPressed(){
  if (key == ENTER) {
    paused = !paused;
  }
}

int closestNode(Vec2 point){
  //TODO: Return the closest node the passed in point
  float minDist = point.distanceTo(nodePos[0]);
  int minIdx = 0;
  for (int i = 1; i < numNodes; i++) {
    float curDist =  point.distanceTo(nodePos[i]);
    if (curDist < minDist) {
      minDist = curDist;
      minIdx = i;
    }
  }
  return minIdx;
}

void mousePressed(){
  goalPos = new Vec2(mouseX, mouseY);
  println("New Goal is",goalPos.x, goalPos.y);
  resetPaths();
}



/////////
// Point Intersection Tests
/////////

//Returns true if the point is inside a box
boolean pointInBox(Vec2 boxTopLeft, float boxW, float boxH, Vec2 pointPos){
  //TODO: Return true if the point is actually inside the box
  return false;
}

//Returns true if the point is inside a circle
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos){
  float dist = pointPos.distanceTo(center);
  if (dist <= r+agentSize){ //small safety factor
    return true;
  }
  return false;
}

//Returns true if the point is inside a list of circle
boolean pointInCircleList(Vec2[] centers, float[] radii, Vec2 pointPos){
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (pointInCircle(center,r,pointPos)){
      return true;
    }
  }
  return false;
}




/////////
// Ray Intersection Tests
/////////

class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

hitInfo rayBoxIntersect(Vec2 boxTopLeft, float boxW, float boxH, Vec2 ray_start, Vec2 ray_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.hit = true;
  
  float t_left_x, t_right_x, t_top_y, t_bot_y;
  t_left_x = (boxTopLeft.x - ray_start.x)/ray_dir.x;
  t_right_x = (boxTopLeft.x + boxW - ray_start.x)/ray_dir.x;
  t_top_y = (boxTopLeft.y - ray_start.y)/ray_dir.y;
  t_bot_y = (boxTopLeft.y + boxH - ray_start.y)/ray_dir.y;
  
  float t_max_x = max(t_left_x,t_right_x);
  float t_max_y = max(t_top_y,t_bot_y);
  float t_max = min(t_max_x,t_max_y); //When the ray exists the box
  
  float t_min_x = min(t_left_x,t_right_x);
  float t_min_y = min(t_top_y,t_bot_y);
  float t_min = max(t_min_x,t_min_y); //When the ray enters the box
  
  
  //The the box is behind the ray (negative t)
  if (t_max < 0){
    hit.hit = false;
    hit.t = t_max;
    return hit;
  }
  
  //The ray never hits the box
  if (t_min > t_max){
    hit.hit = false;
  }
  
  //The ray hits, but further out than max_t
  if (t_min > max_t){
    hit.hit = false;
  }
  
  hit.t = t_min;
  return hit;
}

hitInfo rayCircleIntersect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Length of l_dir (we normalized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r+strokeWidth)*(r+strokeWidth); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the length of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only need the first collision
      float t2 = (-b + sqrt(d))/(2*a); //Optimization: we only need the first collision
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      }
      else if (t1 < 0 && t2 > 0){
        hit.hit = true;
        hit.t = -1;
      }
      
    }
    
  return hit;
}

hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.t = max_t;
  for (int i = 0; i < numObstacles; i++){
    Vec2 center = centers[i];
    float r = radii[i];
    
    hitInfo circleHit = rayCircleIntersect(center, r, l_start, l_dir, hit.t);
    if (circleHit.t > 0 && circleHit.t < hit.t){
      hit.hit = true;
      hit.t = circleHit.t;
    }
    else if (circleHit.hit && circleHit.t < 0){
      hit.hit = true;
      hit.t = -1;
    }
  }
  return hit;
}




/////////////////////////////////
// A Probabilistic Roadmap (PRM)
////////////////////////////////


//The optimal path found along the PRM
ArrayList<Vec2> path = new ArrayList();
int startNode, goalNode; //The actual node the PRM tries to connect do

//Represent our graph structure as 3 lists
ArrayList<Integer>[] neighbors = new ArrayList[numNodes];  //A list of neighbors can can be reached from a given node
float[] dist= new float[numNodes]; //A list which store if a given node has been visited
int[] parent = new int[numNodes]; //A list which stores the best previous node on the optimal path to reach this node

//The PRM uses the above graph, along with a list of node positions
Vec2[] nodePos = new Vec2[numNodes];

//Generate non-colliding PRM nodes
void generateRandomNodes(Vec2[] circleCenters, float[] circleRadii, Vec2 boxTopLeft, float boxW, float boxH){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,randPos);
    }
    nodePos[i] = randPos;
  }
}


//Set which nodes are connected to which neighbors based on PRM rules
void connectNeighbors(){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(circlePos, circleRad, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//Build the PRM
// 1. Generate collision-free nodes
// 2. Connect mutually visible nodes as graph neighbors
void buildPRM(Vec2[] circleCenters, float[] circleRadii, Vec2 boxTopLeft, float boxW, float boxH){
  generateRandomNodes(circleCenters, circleRadii, boxTopLeft, boxW, boxH);
  connectNeighbors();
}

class QueueItem {
  int node;
  float weight;
  QueueItem(int _node, float _weight) {
    node = _node;
    weight = _weight;
  }
};
//BFS
void runBFS(int startID, int goalID){
  startNode = startID;
  goalNode = goalID;
  Queue<QueueItem> fringe = new PriorityQueue<>(new Comparator<QueueItem>() {
        @Override
        public int compare(QueueItem lhs, QueueItem rhs) {
            return Float.compare(lhs.weight, rhs.weight);
        }
  });  //Make a new, empty fringe
  path = new ArrayList(); //Reset path
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    dist[i] = Float.POSITIVE_INFINITY;
    parent[i] = -1; //No parent yet
  }
  
  dist[startID] = 0;
  fringe.add(new QueueItem(startID, 0));
  
  while (fringe.size() > 0){
    QueueItem top = fringe.poll();
    int currentNode = top.node;
    float currentDist = top.weight;
    if (currentNode == goalID){
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      float neighborDist = currentDist + nodePos[currentNode].distanceTo(nodePos[neighborNode]);
      if (dist[neighborNode] > neighborDist){
        dist[neighborNode] = neighborDist;
        parent[neighborNode] = currentNode;
        fringe.add(new QueueItem(neighborNode, neighborDist));
      }
    } 
  }
  
  int prevNode = parent[goalID];
  path.add(nodePos[goalID]);
  while (prevNode >= 0){
    path.add(nodePos[prevNode]);
    prevNode = parent[prevNode];
  }
  Collections.reverse(path);
}




/////////////////////////////////
// Vec2 Library
////////////////////////////////

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public float lengthSqr(){
    return x*x+y*y;
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  public Vec2 negate() {
    return new Vec2(-x, -y);
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  public Vec2 rotated(float theta) {
    return new Vec2(x*cos(theta) - y*sin(theta), x*sin(theta) + y*cos(theta));
  }
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}
