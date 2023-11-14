//Inverse Kinematics
//CSCI 5611 IK [Solution]
// Stephen J. Guy <sjguy@umn.edu>



void setup(){
  size(612,574);
  surface.setTitle("Best Snow man >:)");
  //Square obstacle
  pos4 = new Vec2(100, 100);
  pos5 = new Vec2(530, 300);
  inPresent1 = false;
  inPresent2 = false;
  right_arm_control = true;
}
//right_arm_control
void mousePressed(){
  right_arm_control = !right_arm_control;
  prev = new Vec2(mouseX, mouseY);
}
Vec2 prev;
Vec2 mega_root = new Vec2(300, 300);

Vec2 root = new Vec2(355, 240);
Vec2 root2 = new Vec2(245, 240);

//connect to mega root
float mega_l = 80; 
float mega_a = 0.3;

float mega_l2 = 80;
float mega_a2 = 0.3;
//Upper Arm
float l0 = 40; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 40;
float a1 = 0.3; //Elbow joint

//Hand
float l2 = 40;
float a2 = 0.3; //Wrist joint

//Hand
float l3 = 40;
float a3 = 0.3; //4th joint



//other arm
//pre arm 1
float p2_l0 = 40; 
float p2_a0 = 0.3;
//Upper Arm
float l02 = 40; 
float a02 = 0.3; //Shoulder joint

//Lower Arm
float l12 = 40;
float a12 = 0.3; //Elbow joint

//Hand
float l22 = 40;
float a22 = 0.3; //Wrist joint

//Hand
float l32 = 40;
float a32 = 0.3; //4th joint
//connect to mega root

Vec2 start_r, start_l1,start_l2,start_l3,endPoint;
Vec2 start_r2, start_l12,start_l22,start_l32,endPoint2;

//creating obstacle square
Vec2 pos4;

Vec2 pos5;
boolean inPresent1;
boolean inPresent2;

boolean right_arm_control;
class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}
boolean sameSide(Vec2 l1_start, Vec2 l1_end, Vec2 l2_start, Vec2 l2_end){
  float cp1 = cross(l1_end.minus(l1_start), l2_start.minus(l1_start));
  float cp2 = cross(l1_end.minus(l1_start), l2_end.minus(l1_start));
  if (cp1*cp2 >= 0){
    return true;
  }
  else {
    return false;
  }
}

//line line intersection check
hitInfo LineLineIntesect(Vec2 l1_start, Vec2 l1_end, Vec2 l2_start, Vec2 l2_end){
  hitInfo hit = new hitInfo();
  //check l2
  boolean same_side_l2 = sameSide(l1_start, l1_end, l2_start, l2_end);
  boolean same_side_l1 = sameSide(l2_start, l2_end, l1_start, l1_end);
  //println(same_side_l2, same_side_l1);
  
  if (!same_side_l2 && !same_side_l1){
    hit.hit = true;
  }
  return hit;
}
//square square
hitInfo SquareSquareIntesect(Vec2 sq1, float w1, float h1, Vec2 sq2, float w2, float h2){
  hitInfo hit = new hitInfo();
      
      boolean comb_h = ((h1/2) + (h2/2)) > abs(sq2.y - sq1.y);
      boolean comb_w = ((w1/2) + (w2/2)) > abs(sq2.x - sq1.x);
      
      if (comb_h && comb_w){
        hit.hit = true;
        hit.t = 1;
      }
    
  return hit;
}

void solve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  //Update 4th joint
  startToGoal = goal.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a3 += angleDiff;
  else
    a3 -= angleDiff;
  
  Vec2 sq2 = new Vec2(pos5.x-(100/2), pos5.y-(100/2));
  println(goal);
  hitInfo hitSquareSquare = SquareSquareIntesect(sq2, 100, 100, goal, l2, armW);
  if(hitSquareSquare.hit){
    inPresent1 = true;
    println("IN THE PRESENT");
  }else {
    inPresent1 = false;
    println("NOPE");
  }
  
  fk(); //Update link positions with fk (e.g. end effector changed)

  //Update wrist joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2 += angleDiff;
  else
    a2 -= angleDiff;
   
  ///*TODO: Wrist joint limits here*/
  ////wrist joint, and limit it to be within +/- 90
  //float wristangle = (mega_a+a0+a1+a2) * l1;
  ////relative to the lower arm.
  
  //wristangle = clamp(wristangle,-90,90);
  //println("CUR ANGLE", wristangle);

  //////println(wristangle, a2, (wristangle/l1)-a0-a1);
  //if (wristangle == -90 || wristangle == 90){
  //  a2 = (wristangle/l1)-a0-a1-mega_a;
  //}
  
  
  
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff;
  else
    a1 -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update shoulder joint
  startToGoal = goal.minus(start_r);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(start_r);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0 += angleDiff;
  else
    a0 -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //megalink
  startToGoal = goal.minus(mega_root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(mega_root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    mega_a += angleDiff;
  else
    mega_a -= angleDiff;
  
  fk(); //Update link positions with fk (e.g. end effector changed)
}

void fk(){
  start_r = new Vec2(cos(mega_a)*mega_l,sin(mega_a)*mega_l).plus(mega_root);
  start_l1 = new Vec2(cos(a0+mega_a)*l0,sin(a0+mega_a)*l0).plus(start_r);
  start_l2 = new Vec2(cos(a0+mega_a+a1)*l1,sin(a0+mega_a+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+mega_a+a1+a2)*l1,sin(a0+mega_a+a1+a2)*l2).plus(start_l2);
  endPoint = new Vec2(cos(a0+mega_a+a1+a2+a3)*l3,sin(a0+mega_a+a1+a2+a3)*l3).plus(start_l3);
}

void solve2(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  //Update 4th joint
  startToGoal = goal.minus(start_l32);
  startToEndEffector = endPoint2.minus(start_l32);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a32 += angleDiff;
  else
    a32 -= angleDiff;
  
  Vec2 sq1 = new Vec2(pos4.x-(100/2), pos4.y-(100/2));
  //hitInfo SquareSquareIntesect(Vec2 sq1, float w1, float h1, Vec2 sq2, float w2, float h2){
  println(goal);
  hitInfo hitSquareSquare = SquareSquareIntesect(sq1, 100, 100, goal, l2, armW);
  if(hitSquareSquare.hit){
    inPresent2 = true;
    println("IN THE PRESENT");
  }else {
    inPresent2 = false;
    println("NOPE");
  }
  //println(start_l32);

  fk2(); //Update link positions with fk (e.g. end effector changed)
  
  //WORKING
  //Update wrist joint
  startToGoal = goal.minus(start_l22);
  startToEndEffector = endPoint2.minus(start_l22);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a22 += angleDiff;
  else
    a22 -= angleDiff;
   
  /*TODO: Wrist joint limits here*/
  //wrist joint, and limit it to be within +/- 90
  //float wristangle = (a0+a1+a2) * l1;
  //relative to the lower arm.
  
  //wristangle = clamp(wristangle,-90,90);
  ////println(wristangle, a2, (wristangle/l1)-a0-a1);
  //if (wristangle == -90 || wristangle == 90){
  //  a2 = (wristangle/l1)-a0-a1;
  //}
  
  
  fk2(); //Update link positions with fk (e.g. end effector changed)
  
  
  //WORKING
  
  //Update elbow joint
  startToGoal = goal.minus(start_l12);
  startToEndEffector = endPoint2.minus(start_l12);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a12 += angleDiff;
  else
    a12 -= angleDiff;
  fk2(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update shoulder joint
  startToGoal = goal.minus(start_r2);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint2.minus(start_r2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a02 += angleDiff;
  else
    a02 -= angleDiff;
  
  fk2(); //Update link positions with fk (e.g. end effector changed)
 
  //megalink
  startToGoal = goal.minus(mega_root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(mega_root);
  //WORKING
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    mega_a2 += angleDiff;
  else
    mega_a2 -= angleDiff;
  
  fk2(); 
}

void fk2(){
  start_r2 = new Vec2(cos(mega_a2)*mega_l2,sin(mega_a2)*mega_l2).plus(mega_root);
  start_l12 = new Vec2(cos(a02+mega_a2)*l02,sin(a02+mega_a2)*l02).plus(start_r2);
  start_l22 = new Vec2(cos(a02+mega_a2+a12)*l12,sin(a02+mega_a2+a12)*l12).plus(start_l12);
  start_l32 = new Vec2(cos(a02+mega_a2+a12+a22)*l12,sin(a02+mega_a2+a12+a22)*l22).plus(start_l22);
  endPoint2 = new Vec2(cos(a02+mega_a2+a12+a22+a32)*l32,sin(a02+mega_a2+a12+a22+a32)*l32).plus(start_l32);
}
float armW = 10;
float handH = 10;
void draw(){
  fk();
  fk2();
  solve();
  solve2();
  PImage img;
  img = loadImage("others.jpeg");
  background(img);
  //snow man :)
  //background(250,250,250);
  fill(250,250,250);
  //fill(200,0,0);
  circle(300, 130, 100);
  circle(300, 240, 130);
  circle(300, 400, 200);
  
  fill(0,0,0);
  //left
  circle(280, 120, 10);
  circle(270, 140, 5);
  circle(275, 150, 5);
  circle(290, 155, 5);
  //right
  circle(320, 120, 10);
  circle(330, 140, 5);
  circle(320, 150, 5);
  circle(305, 155, 5);
  //button coat
  circle(300, 210, 10);
  circle(300, 235, 10);
  circle(300, 255, 10);
  
  rect(285, 30, 30, 50);
  rect(265, 80, 70, 20);
  
  fill(255,165,0);
  triangle(300, 130, 330, 130, 300, 140);
  
  //end snow man
  
  //presents
  
  //left stick pres
  fill(255,0,0);
  square(pos4.x-(100/2), pos4.y-(100/2), 100);
  fill(255,215,0);
  rect(pos4.x-(100/2), pos4.y-(100/2)+40, 100, 15);
  
  rect(pos4.x-(100/2)+40, pos4.y-(100/2), 15, 100);
  
  //right stick pres
  fill(255,0,0);
  square(pos5.x-(100/2), pos5.y-(100/2), 100);
  fill(255,215,0);
  rect(pos5.x-(100/2), pos5.y-(100/2)+40, 100, 15);
  
  rect(pos5.x-(100/2)+40, pos5.y-(100/2), 15, 100);
  
  //arm1
  fill(0,0,0);
  
  //end node
  pushMatrix();
  translate(mega_root.x,mega_root.y);
  rect(-armW/2, -armW/2, armW/2, mega_l);
  popMatrix();
  
  //start_r
  //mega_l
  //mega_a
  
  //pre - node 1
  pushMatrix();
  translate(mega_root.x,mega_root.y);
  rotate(mega_a);
  rect(0, -armW/2, mega_l, armW/2);
  popMatrix();
  
  fill(160,82,45);
  pushMatrix();
  translate(start_r.x,start_r.y);
  rotate(a0+mega_a);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+mega_a+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+mega_a+a1+a2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  if (inPresent1){
    fill(255,255,255);
  }else{
    fill(160,82,45);
  }
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+mega_a+a1+a2+a3);
  rect(0, -armW/2, l3, armW);
  popMatrix();
  
  //arm2
  fill(0,0,0);
  //start_r
  //mega_l
  //mega_a
  
  pushMatrix();
  translate(mega_root.x,mega_root.y);
  rotate(mega_a2);
  println("LEFT UPDATE a:", mega_a2, mega_a);
  rect(0, -armW/2, mega_l2, armW/2);
  popMatrix();
  
  fill(160,82,45);
  pushMatrix();
  translate(start_r2.x,start_r2.y);
  rotate(a02+mega_a2);
  println("Right UPDATE", a02);
  rect(0, -armW/2, l02, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l12.x,start_l12.y);
  rotate(a02+mega_a2+a12);
  rect(0, -armW/2, l12, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l22.x,start_l22.y);
  rotate(a02+mega_a2+a12+a22);
  rect(0, -armW/2, l22, armW);
  popMatrix();
  
  if (inPresent2){
    fill(255,255,255);
  }else{
    fill(160,82,45);
  }
  pushMatrix();
  translate(start_l32.x,start_l32.y);
  rotate(a02+mega_a2+a12+a22+a32);
  rect(0, -armW/2, l32, armW);
  popMatrix();
  
}



//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

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
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
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

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
