
#include<RMCS2303drive.h>
#include "MapFloat.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

RMCS2303 rmcs;
byte slave_id1=1;
byte slave_id2=2;
byte slave_id3=3;
byte slave_id4=4;
//int INP_CONTROL_MODE=257;           
//int PP_gain=32;
//int PI_gain=16;
//int VF_gain=32;
//int LPR=334;
//int acceleration=5000;
//int speed=8000;

long int r_enc;
long int l_enc;

ros::NodeHandle nh;

//SoftwareSerial myserial(2,3);

geometry_msgs::Twist msg;
//std_msgs::Float32 l_v;
//std_msgs::Float32 r_v;
std_msgs::Int32 lwheel;
std_msgs::Int32 rwheel;

//ros::Publisher lvel("lvel", &l_v);
//ros::Publisher rvel("rvel", &r_v);
ros::Publisher left_ticks("left_ticks",&lwheel);
ros::Publisher right_ticks("right_ticks",&rwheel);

double wheel_rad = 0.0625, wheel_sep = 0.300;
double w_r=0, w_l=0;
double speed_ang;
double speed_lin;
double leftPWM;
double rightPWM;

void messageCb( const geometry_msgs::Twist& msg)
    {

//lwheel.data = ((rmcs.Position_Feedback(slave_id1)+rmcs.Position_Feedback(slave_id2))/2);
//rwheel.data = ((rmcs.Position_Feedback(slave_id3)+rmcs.Position_Feedback(slave_id4))/2);

  speed_lin = max(min(msg.linear.x, 1.0f), -1.0f);
  speed_ang = max(min(msg.angular.z, 1.0f), -1.0f);
  
//  speed_ang = msg.angular.z;
//  speed_lin = msg.linear.x;

  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  if(w_r==0)
{
  rightPWM=0;//or break right motor
  rmcs.Disable_Digital_Mode(slave_id1,0);
  rmcs.Disable_Digital_Mode(slave_id2,0);
  rmcs.Disable_Digital_Mode(slave_id3,0);
  rmcs.Disable_Digital_Mode(slave_id4,0);
//  rmcs.Disable_Digital_Mode(slave_id2,1);
//  rmcs.Disable_Digital_Mode(slave_id2,1);
//  rmcs.Brake_Motor(slave_id1,0);
//  rmcs.Brake_Motor(slave_id2,0);
//
}
  else
  rightPWM = mapFloat(fabs(w_r),0.0,18.0,1500,17200);
  
  if(w_l==0){
  leftPWM=0; // or break left motor
  rmcs.Disable_Digital_Mode(slave_id1,0);
  rmcs.Disable_Digital_Mode(slave_id2,0);
  rmcs.Disable_Digital_Mode(slave_id3,0);
  rmcs.Disable_Digital_Mode(slave_id4,0);
//  rmcs.Disable_Digital_Mode(slave_id2,1);
//  rmcs.Disable_Digital_Mode(slave_id2,1);

//  rmcs.Brake_Motor(slave_id1,0);
//  rmcs.Brake_Motor(slave_id2,0);
  }
  
  else
  leftPWM = mapFloat(fabs(w_l),0.0,18.0,1500,17200);

  rmcs.Speed(slave_id1,rightPWM);
  rmcs.Speed(slave_id2,rightPWM);
  rmcs.Speed(slave_id3,leftPWM);
  rmcs.Speed(slave_id4,leftPWM); 

if(w_r>0 && w_l>0){
  
// rmcs.Speed(slave_id1,rightPWM);
// rmcs.Speed(slave_id2,rightPWM);
// rmcs.Speed(slave_id3,leftPWM); 
// rmcs.Speed(slave_id4,leftPWM);         // forward
 rmcs.Enable_Digital_Mode(slave_id1,1);
 rmcs.Enable_Digital_Mode(slave_id2,1);
 rmcs.Enable_Digital_Mode(slave_id3,0);
 rmcs.Enable_Digital_Mode(slave_id4,0);
  
}

else if(w_r<0 && w_l<0){
  
// rmcs.Speed(slave_id1,rightPWM);
// rmcs.Speed(slave_id2,rightPWM);
// rmcs.Speed(slave_id3,leftPWM);
// rmcs.Speed(slave_id4,leftPWM);       // backwards
 rmcs.Enable_Digital_Mode(slave_id1,0);
 rmcs.Enable_Digital_Mode(slave_id2,0);
 rmcs.Enable_Digital_Mode(slave_id3,1);
 rmcs.Enable_Digital_Mode(slave_id4,1);
  
}
else if(w_r>0 && w_l<0){
  
// rmcs.Speed(slave_id1,rightPWM);
// rmcs.Speed(slave_id2,rightPWM);
// rmcs.Speed(slave_id3,leftPWM);
// rmcs.Speed(slave_id4,leftPWM);      //Left
 rmcs.Enable_Digital_Mode(slave_id1,1);
 rmcs.Enable_Digital_Mode(slave_id2,1);
 rmcs.Enable_Digital_Mode(slave_id3,1);
 rmcs.Enable_Digital_Mode(slave_id4,1);
  
}

else if(w_r<0 && w_l>0){
  
// rmcs.Speed(slave_id1,rightPWM);
// rmcs.Speed(slave_id2,rightPWM);
// rmcs.Speed(slave_id3,leftPWM); 
// rmcs.Speed(slave_id4,leftPWM);          //Right
 rmcs.Enable_Digital_Mode(slave_id1,0);
 rmcs.Enable_Digital_Mode(slave_id2,0);
 rmcs.Enable_Digital_Mode(slave_id3,0);
 rmcs.Enable_Digital_Mode(slave_id4,0);
  
}

else {
  rmcs.Brake_Motor(slave_id1,0);
  rmcs.Brake_Motor(slave_id2,0);
  rmcs.Brake_Motor(slave_id3,0);
  rmcs.Brake_Motor(slave_id4,0);
  rmcs.Brake_Motor(slave_id1,1);
  rmcs.Brake_Motor(slave_id2,1);
  rmcs.Brake_Motor(slave_id3,1);
  rmcs.Brake_Motor(slave_id4,1);
}
  
//  r_v.data = rightPWM;
//  l_v.data = leftPWM;


  

}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );


void setup() {
rmcs.Serial_selection(0); 
rmcs.Serial0(9600);
rmcs.begin(&Serial1,9600);

   //rmcs.WRITE_PARAMETER(slave_id1,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    //Uncomment to write parameters to drive. Comment to ignore.
   //rmcs.READ_PARAMETER(slave_id1);
   //rmcs.WRITE_PARAMETER(slave_id2,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    //Uncomment to write parameters to drive. Comment to ignore.
   //rmcs.READ_PARAMETER(slave_id2);
 nh.initNode();
 nh.subscribe(sub);

 
// nh.advertise(rvel);
// nh.advertise(lvel);

nh.advertise(left_ticks);
nh.advertise(right_ticks);
}
 

void loop() {
//lvel.publish(&l_v);
//rvel.publish(&r_v);

//rwheel.data = ((rmcs.Position_Feedback(slave_id1)+rmcs.Position_Feedback(slave_id2))/2);
//lwheel.data = -((rmcs.Position_Feedback(slave_id3)+rmcs.Position_Feedback(slave_id4))/2);
//lwheel.data = l_enc;
//rwheel.data = r_enc; 
lwheel.data = rmcs.Position_Feedback(slave_id4);
rwheel.data = -rmcs.Position_Feedback(slave_id2);

left_ticks.publish(&lwheel);
right_ticks.publish(&rwheel);
 nh.spinOnce();
}
