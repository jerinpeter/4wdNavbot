#include <ros.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include "MapFloat.h"

RMCS2303 rmcs;  // creation of motor driver object
// slave ids to be set on the motor driver refer to the manual in the reference section
byte slave_id1 = 1;
byte slave_id2 = 2;
byte slave_id3 = 3;
byte slave_id4 = 4;

ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist

std_msgs::Int32 lwheel;  // for storing left encoder value
std_msgs::Int32 rwheel;  // for storing right encoder value

// Publisher object with topic names left_ticks and right_ticks for publishing Enc Values
ros::Publisher left_ticks("left_ticks", &lwheel);
ros::Publisher right_ticks("right_ticks", &rwheel);
// Make sure to specify the correct values here
//*******************************************
double wheel_rad = 0.0625, wheel_sep = 0.300;  // wheel radius and wheel sepration in meters.
//******************************************
double w_r = 0, w_l = 0;
double speed_ang;
double speed_lin;
double leftPWM;
double rightPWM;

void messageCb(const geometry_msgs::Twist& msg)  // cmd_vel callback function definition
{
  speed_lin = max(min(msg.linear.x, 1.0f), -1.0f);   // limits the linear x value from -1 to 1
  speed_ang = max(min(msg.angular.z, 1.0f), -1.0f);  // limits the angular z value from -1 to 1

  // Kinematic equation for finding the left and right velocities
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));

  if (w_r == 0)
  {
    rightPWM = 0;
    rmcs.Disable_Digital_Mode(slave_id1,0);
    rmcs.Disable_Digital_Mode(slave_id2,0);  // if right motor velocity is zero set right pwm to zero and disabling motors
    rmcs.Disable_Digital_Mode(slave_id3,0);
    rmcs.Disable_Digital_Mode(slave_id4,0);
  }
  else
    rightPWM = mapFloat(fabs(w_r), 0.0, 18.0, 1500,17200);  // mapping the right wheel velocity with respect to Motor PWM values

  if (w_l == 0)
  {
    leftPWM = 0;
    rmcs.Disable_Digital_Mode(slave_id1,0);
    rmcs.Disable_Digital_Mode(slave_id2,0);  // if left motor velocity is zero set left pwm to zero and disabling motors
    rmcs.Disable_Digital_Mode(slave_id3,0);
    rmcs.Disable_Digital_Mode(slave_id4,0);
  }

  else
    leftPWM = mapFloat(fabs(w_l), 0.0, 18.0, 1500,
                       17200);  // mapping the right wheel velocity with respect to Motor PWM values

  rmcs.Speed(slave_id1,rightPWM);
  rmcs.Speed(slave_id2,rightPWM);
  rmcs.Speed(slave_id3,leftPWM);
  rmcs.Speed(slave_id4,leftPWM);

  if (w_r > 0 && w_l > 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,1);
    rmcs.Enable_Digital_Mode(slave_id2,1);  // forward condition
    rmcs.Enable_Digital_Mode(slave_id3,0);
    rmcs.Enable_Digital_Mode(slave_id4,0);
  }

  else if (w_r < 0 && w_l < 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,0);
    rmcs.Enable_Digital_Mode(slave_id2,0);  // backward condition
    rmcs.Enable_Digital_Mode(slave_id3,1);
    rmcs.Enable_Digital_Mode(slave_id4,1);
  }
  else if (w_r > 0 && w_l < 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,1);
    rmcs.Enable_Digital_Mode(slave_id2,1);  // Leftward condition
    rmcs.Enable_Digital_Mode(slave_id3,1);
    rmcs.Enable_Digital_Mode(slave_id4,1);
  }

  else if (w_r < 0 && w_l > 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,0);
    rmcs.Enable_Digital_Mode(slave_id2,0);  // rightward condition
    rmcs.Enable_Digital_Mode(slave_id3,0);
    rmcs.Enable_Digital_Mode(slave_id4,0);
  }

  else
  {
    rmcs.Brake_Motor(slave_id1,0);
    rmcs.Brake_Motor(slave_id2,0);
    rmcs.Brake_Motor(slave_id3,0);
    rmcs.Brake_Motor(slave_id4,0);  // if none of the above break the motors both in clockwise n anti-clockwise direction
    rmcs.Brake_Motor(slave_id1,1);
    rmcs.Brake_Motor(slave_id2,1);
    rmcs.Brake_Motor(slave_id3,1);
    rmcs.Brake_Motor(slave_id4,1);
  }
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  // creation of subscriber object sub for recieving the cmd_vel

void setup()
{
  rmcs.Serial_selection(0);  // 0 -> for Harware serial tx1 rx1 of arduino mega
  rmcs.Serial0(9600);
  rmcs.begin(&Serial3, 9600);

  nh.initNode();      // initialzing the node handle object
  nh.subscribe(sub);  // subscribing to cmd vel with sub object

  nh.advertise(left_ticks);   // advertise the left_ticks topic
  nh.advertise(right_ticks);  // advertise the left_ticks topic
}

void loop()
{
  lwheel.data =
      rmcs.Position_Feedback(slave_id4);  // the function reads the encoder value from the motor with slave id 4
  rwheel.data =
      -rmcs.Position_Feedback(slave_id2);  // the function reads the encoder value from the motor with slave id 4

  left_ticks.publish(&lwheel);   // publish left enc values
  right_ticks.publish(&rwheel);  // publish right enc values
  nh.spinOnce();
}
