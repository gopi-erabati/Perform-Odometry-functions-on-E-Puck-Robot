// Gopikrishna Erabati and Pamir Ghimire
// For Autonomous Robotics coursework
// Graduate Students, M1
// MSCV (Computer Vision and Robotics CVR)
// Universite De Bourgogne, France 

#include <webots/robot.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <stdio.h>
#include <math.h>


#define TIME_STEP 32
#define WHEEL_RADIUS 0.0205 // in m
#define AXLE_LENGTH 0.052 // in m
#define STEPS_ROT 1000 // 1000 steps per rotatation
#define PI 3.14
#define PI2 6.28


 // to compute odometry
void compute_odometry(double *dleft, double *dright);
//function to get updated pos
void updatePose();
// function to check update position by making a circle
void getPosition();
// to travel 10cms forward and stop
void move10cms();
// to rotate 90 degree clockwise
void rotate90cw();
// to move in square trajectory
void moveSquare();

// Global variable: Robot's pose
double pos[3];
    

//---------------  MAIN : -----------------------------------------------
// NOTE: Uncomment the functions according to the task required

int main(int argc, char **argv)
{
  // Robot's pose
  pos[0] = 0; pos[1] = 0; pos[2] = 0;
  
  // initialize robot
  wb_robot_init();
  
  //intialise encoder
  wb_differential_wheels_enable_encoders(TIME_STEP);
  
  //wb_differential_wheels_set_encoders(0,0);
  
  // uncomment the function to check update position function
  //getPosition();
  
  // uncomment the function to move the robot 10cms and stop
  //move10cms();
  
  // uncomment to rotate 90 degree clockwise
  //rotate90cw();
  
  //uncomment to move the robot in square trajectory of 10cms square
  moveSquare();
  
  // main end
  return 0;
}

// function to compute odometry and return 
//distance covered by left and right wheel 
void compute_odometry(double *dleft, double *dright) {
  double left = wb_differential_wheels_get_left_encoder();
  double right = wb_differential_wheels_get_right_encoder();
  //printf("left = %f and right = %f", left, right);
  *dleft = left / STEPS_ROT * 2 * PI * WHEEL_RADIUS; // distance covered by left wheel in meter
  *dright = right / STEPS_ROT * 2 * PI * WHEEL_RADIUS; // distance covered by right wheel in meter
}

//function to get updated pos
void updatePose(){

  double dleft, dright;
  compute_odometry(&dleft,&dright);
  printf("estimated distance covered by left wheel : %f m.\n",dleft);
  printf("estimated distance covered by right wheel : %f m.\n",dright);  
  double drobot = (dleft + dright)/2.0; // distance travelled by robot
  pos[0] +=  drobot * cos(pos[2]); // robot position w.r.to X direction
  pos[1] +=  drobot * sin(pos[2]); // robot position w.r.to Y direction
  pos[2] += (dright - dleft)/AXLE_LENGTH; // orientation
  pos[2] = fmodf(pos[2], PI2) ; // to get angle in o to 2*pi radians
  // if angle turns neagtive to make it postitve to get counter closkwise angle
  /*if (pos[2] < 0){
    pos[2] = pos[2] + PI2;
  }*/
  printf("current position of robot:\n");
  printf("estimated robot_x : %f m\n",pos[0]);
  printf("estimated robot_y : %f m\n",pos[1]);
  printf("estimated robot_theta : %f radians\n", pos[2]);

}

// function to check update position by making a circle
void getPosition(){
  
  for(;;){
  wb_differential_wheels_set_encoders(0,0);
  /* set speed values */
  wb_differential_wheels_set_speed(-100,-200);
    
  /* perform a simulation step */
  wb_robot_step(TIME_STEP);
  
  updatePose();
  
   
  }
}

// function to move the robot 10cms straight and stop
void move10cms(){
  
  // to store previous pose x and y
  double x0 = pos[0]; 
  double y0 = pos[1];
  
  for(;;){
    
    // to set encoders back to zero to calculate rotation ticks
    wb_differential_wheels_set_encoders(0,0);
  
    // set speed values 
    wb_differential_wheels_set_speed(100,100);
    // perform a simulation step 
    wb_robot_step(TIME_STEP);
    
    updatePose();
    
    double distance = sqrt((pos[0]-x0) * (pos[0]-x0) + (pos[1]-y0) * (pos[1]-y0));
    // check if distance estimated by odometry is around 10cms with a threshold of 1mm
    if (distance >= 0.097 && distance <= 0.103){
      wb_differential_wheels_set_speed(0,0);
      wb_robot_step(TIME_STEP);
      break;
    }
  }
}

// function to rotate the robot 90 degrees clockwise
void rotate90cw(){
  // to store previous radians
  double theta0 = pos[2];
  
  for (;;){
    // to set encoders back to zero to calculate rotation ticks
    wb_differential_wheels_set_encoders(0,0);
  
    //set speed values 
    wb_differential_wheels_set_speed(100,-100);
    //perform a simulation step 
    wb_robot_step(TIME_STEP);
    
    updatePose();
    
    double angle = fabs(pos[2] - theta0);
    printf("angle = %f \n",angle);
    double tolerance = 1.57/36; //5 degrees on both sides
    
    // check if distance estimated by odometry of wheel distance is 1/3 of its circumference with a threshold of 1mm
    if (angle > (1.57 - tolerance) && angle < (1.57 + tolerance)){
      wb_differential_wheels_set_speed(0,0);
      wb_robot_step(TIME_STEP);
      break;  
    }
    
  }
}

//function to move the robot in a square trajectory of side 10cms
void moveSquare(){
  move10cms();
  rotate90cw();
  move10cms();
  rotate90cw();
  move10cms();
  rotate90cw();
  move10cms();
  rotate90cw();
}
