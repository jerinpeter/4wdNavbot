#include<TMCStepper.h>
#include <AccelStepper.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "MapFloat.h"

#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); 

ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist


double speed_x;
double speed_y;
double speed_rot;

double linear_vel_x_mins_;
double linear_vel_y_mins_;
double angular_vel_z_mins_;
double tangential_vel_;

double x_rpm_;
double y_rpm_;
double tan_rpm_;

double circumference_ = 0.314;

double rpm_motor_fl;
double rpm_motor_fr;
double rpm_motor_br;
double rpm_motor_bl;


#define FR_EN_PIN           A8 // Enable
#define FR_DIR_PIN          48 // Direction
#define FR_STEP_PIN         46 // Step
//#define SW_RX            A11 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            D42 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define FR_SERIAL_PORT Serial1 // Arduino mega Hw serial pins Tx=18 and Rx=19 


#define FL_EN_PIN           A8 // Enable
#define FL_DIR_PIN          48 // Direction
#define FL_STEP_PIN         46 // Step
//#define SW_RX            A11 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            D42 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define FL_SERIAL_PORT Serial2 // Arduino mega Hw serial pins Tx=18 and Rx=19


#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

TMC2209Stepper FR_driver(&FR_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper FL_driver(&FL_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

constexpr uint32_t steps_per_m = 1911; // steps needed for robot to move 1m

AccelStepper FR_stepper = AccelStepper(FR_stepper.DRIVER, FR_STEP_PIN, FR_DIR_PIN);
AccelStepper FL_stepper = AccelStepper(FL_stepper.DRIVER, FL_STEP_PIN, FL_DIR_PIN);



void messageCb(const geometry_msgs::Twist& msg)  // cmd_vel callback function definition
 {

  
  speed_x = max(min(msg.linear.x, 1.0f), -1.0f);   
  speed_y = max(min(msg.linear.y, 1.0f), -1.0f); 
  speed_rot = max(min(msg.angular.z, 1.0f), -1.0f); 

  linear_vel_x_mins_ = speed_x * 60;
  linear_vel_y_mins_ = speed_y * 60;
  angular_vel_z_mins_ = speed_rot * 60;
  
  tangential_vel_ = angular_vel_z_mins_ * 0.37;  // 0.37 -> base width

  x_rpm_ = linear_vel_x_mins_ / circumference_;
  y_rpm_ = linear_vel_y_mins_ / circumference_;
  tan_rpm_ = tangential_vel_ / circumference_;

  rpm_motor_fl = x_rpm_ - y_rpm_ - tan_rpm_;    //front-left motor
   
  rpm_motor_bl = x_rpm_ + y_rpm_ - tan_rpm_;     //back-left motor

  rpm_motor_fr = x_rpm_ + y_rpm_ + tan_rpm_;     //front-right motor
  
  rpm_motor_br = x_rpm_ - y_rpm_ + tan_rpm_;     //back-right motor
  

if((rpm_motor_fl) && (rpm_motor_bl) && (rpm_motor_fr) && (rpm_motor_br) == 0)
{

    FR_stepper.setSpeed(0);
    FL_stepper.setSpeed(0);
    
}
  else
  {
    FR_stepper.setSpeed(rpm_motor_fr*100);                 // steps are set here 
    FL_stepper.setSpeed(rpm_motor_fl*100);    
  }
  
  if((rpm_motor_fl) && (rpm_motor_bl) && (rpm_motor_fr) && (rpm_motor_br) > 0)   // the steps calculated are executed with if conditions
{
 

    FR_stepper.runSpeed();
    FL_stepper.runSpeed();
}

  else if((rpm_motor_fl) && (rpm_motor_bl) && (rpm_motor_fr) && (rpm_motor_br) < 0)
{
 

    FR_stepper.runSpeed();
    FL_stepper.runSpeed();
}


 }


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  

void setup() {
     
     nh.initNode();      // initialzing the node handle object
     nh.subscribe(sub);  // subscribing to cmd vel with sub object
    
    FR_SERIAL_PORT.begin(115200);      // HW UART drivers
    //driver.begin();             // Initiate pins and registeries
    FR_driver.toff(5);                 // Enables driver in software
    FR_driver.rms_current(900);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    FR_driver.en_spreadCycle(true);   // Toggle spreadCycle on TMC2208/2209/2224
    //driver.pwm_autoscale(1);
    FR_driver.intpol(true); //1/256 microstep interpolation
    FR_driver.microsteps(1); //Microsteps. 1 = full step
    
    FR_stepper.setMaxSpeed(0.5*steps_per_m); //0.5m/s
    FR_stepper.setAcceleration(0.1*steps_per_m); // 0.1m/s^2
    FR_stepper.setEnablePin(FR_EN_PIN);
    FR_stepper.setPinsInverted(false, false, true);
    FR_stepper.enableOutputs();

    FL_SERIAL_PORT.begin(115200);      // HW UART drivers
    //driver.begin();             // Initiate pins and registeries
    FL_driver.toff(5);                 // Enables driver in software
    FL_driver.rms_current(900);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    FL_driver.en_spreadCycle(true);   // Toggle spreadCycle on TMC2208/2209/2224
    //driver.pwm_autoscale(1);
    FL_driver.intpol(true); //1/256 microstep interpolation
    FL_driver.microsteps(1); //Microsteps. 1 = full step
    
    FL_stepper.setMaxSpeed(0.5*steps_per_m); //0.5m/s
    FL_stepper.setAcceleration(0.1*steps_per_m); // 0.1m/s^2
    FL_stepper.setEnablePin(FL_EN_PIN);
    FL_stepper.setPinsInverted(false, false, true);
    FL_stepper.enableOutputs();

    
}

void loop() {
    nh.spinOnce();
//    stepper.setSpeed(0.5*steps_per_m);
//    stepper.runSpeed();
}
