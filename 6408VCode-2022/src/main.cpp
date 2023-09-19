/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//3 things: check if it goes straight and if I use coast, does the units get more accurate. Make the robot holding things while moving.

#include "vex.h"

using namespace vex;

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// MogoLift             motor         16              
// RightLift            motor         18              
// LeftLift             motor         15              
// RightFront           motor         19              
// LeftFront            motor         20              
// RightBack            motor         11              
// LeftBack             motor         13              
// Intake               motor         14              
// ---- END VEXCODE CONFIGURED DEVICES ----

// A global instance of competition
competition Competition;
// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor MogoLift = motor(PORT9, ratio36_1, false);
motor RightLift = motor(PORT18, ratio36_1, false);
motor LeftLift = motor(PORT15, ratio36_1, false);
motor RightFront = motor(PORT11, ratio18_1, false);
motor LeftFront = motor(PORT13, ratio18_1, true);
motor RightBack = motor(PORT19, ratio18_1, false);
motor LeftBack = motor(PORT20, ratio18_1, true);
motor Intake = motor(PORT14, ratio18_1, false);
pneumatics Clamp = pneumatics(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT4);
//bumper BumperBack = bumper(Brain.ThreeWirePort.H);
sonar RangeFinderLeft = sonar(Brain.ThreeWirePort.E);
sonar RangeFinderRight = sonar(Brain.ThreeWirePort.C);
limit LimitFront = limit(Brain.ThreeWirePort.B);

//Sonar output to three wire port C and input to Port D

// define your global instances of motors and other devices here

/////////////////////////////////////////////
// Section: GLOBAL CONSTANTS and VARIABLES
///////////////////////////////////////////

// WHEEL constant
const double WHEEL_DIAMETER = 3.25; // size of the wheel
const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER; // this is the circumference for our drivetrain wheel

// ALIGNING constants
const double ALIGN_SPEED = 31; // high speed to bang the wall
const double ALIGN_TIME = 0.65; // keep banging the wall for 0.6 second

// STRAIGHT variables
double FAST_VELOCITY = 50; //35//the top straight speed (used when going 15in <= distance)
double MEDIUM_VELOCITY = 33; //30// medium speed (used when going 3 in <= distance <= 15 in)
double SLOW_VELOCITY = 21; //21// slow speed (used when going distance < 3in)
double STIME_VELOCITY = 31; // speed when doing StraightTime

// SMOOTH CONSTANTS and variables
bool SMOOTH_FLAG = false; // do somoth acceleration to FAST_VELOCITY or not
double STOP_VELOCITY = 25; // the velocity to decelerate to when doing SMOOTH
double TIME_DELTA = 0.005; // time between changing velocity when doing smooth
double VELOCITY_DELTA = 0.73; // how much to change the velocity

// CONSTANTS for checking when to stop (could add ultrasonic range finder for this)
const int CHECK_DISTANCE = 30000;
const int CHECK_TIME = 30004;
vex::timer gyro_straight_timer;

// TURN constants
const double FAST_TURN_SPEED = 23;
const double SLOW_TURN_SPEED = 10;
const double START_SLOW_TURN = 30;
const double STOP_SLOW_TURN = 5;
const double TURN_SPEED_SLOPE = (FAST_TURN_SPEED - SLOW_TURN_SPEED)/(START_SLOW_TURN - STOP_SLOW_TURN);

//SCORING constants
double SCORE_TIME = 675;
const double CENTER_SCORE_POWER = 90;
const double CENTER_SCORE_TIME = 1000;

// Lift constants
const double UP_SPEED = 100;
const double DOWN_SPEED = 80;
const int NUM_OF_POS = 3;
const int LIFT_HEIGHTS[NUM_OF_POS] = {0, 650}; // lift positions from lowest to highest
float RightLiftPos = RightLift.rotation(deg); 


// GYRO values
// GLOBAL VARIABLE for intended heading
double heading = 0;

// The higher the correction factor, the stronger the correction. The less, the smoother.
const double CORRECTION_FACTOR = 0.40;

// Ultrasonic Range Finder
float RightDistance = 0;
float LeftDistance = 0;
bool ActivateSonar = false;

//////////////////////////////////////////////////////////////////////
// Section: Motor Setup, Inertial Setup
/////////////////////////////////////////////////////////////////////////
// This function sets the braketypes and torque for the motors, and resets their rotation.
void SetupMotors() {
  LeftFront.setStopping(brakeType::brake);
  LeftFront.resetRotation();
  LeftBack.setStopping(brakeType::brake);
  LeftBack.resetRotation();
  RightFront.setStopping(brakeType::brake);
  RightFront.resetRotation();
  RightBack.setStopping(brakeType::brake);
  RightBack.resetRotation();

  Intake.setMaxTorque(85, percentUnits::pct);
  Intake.resetRotation();
  Intake.setStopping(brakeType::coast);

  RightLift.setMaxTorque(100, percentUnits::pct);
  RightLift.resetRotation();
  RightLift.setStopping(brakeType::hold);
  LeftLift.setMaxTorque(100, percentUnits::pct);
  LeftLift.resetRotation();
  LeftLift.setStopping(brakeType::hold);

  MogoLift.setMaxTorque(100, percentUnits::pct);
  MogoLift.resetRotation();
  MogoLift.setStopping(brakeType::hold);
}

int CheckRightSonar() {
  while(true) {
    RightDistance = RangeFinderRight.distance(distanceUnits::in);

    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Distance inches: %.2f in", RightDistance);
    wait(200,msec);
    Brain.Screen.clearScreen();
    wait(5,msec);
  }
  return RightDistance;
}

int CheckLeftSonar() {
  while(true) {
    LeftDistance = RangeFinderLeft.distance(distanceUnits::in);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Distance inches: %.2f in", LeftDistance);
    wait(200,msec);
    Brain.Screen.clearScreen();
    wait(5,msec);
  }
  return LeftDistance;
}

//Table
/*Actual Distance (included sensor)          Sensor Distance
            5.25                                  5.65
            4.75                                  5.00

*/

// This function calibrates the inertial sensor by calling the "calibrate" and
// using the "isCalibrating" method to wait for calibration to complete.
void SetupInertial() {
// Calibrate the Inertial sensor
  Inertial.calibrate();
  Brain.Screen.print("Calibrating inertial");
  vex::task::sleep(4000);
  while (Inertial.isCalibrating()) {
    vex::task::sleep(100);
  }

// Calculate the drift and show to the user
  Inertial.setHeading(0, degrees);
  Inertial.setRotation(0, degrees);
  for (int i = 0; i < 5; i++) {
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("heading = %.31f, rotation = %.31f", Inertial.heading(degrees), Inertial.rotation(degrees));
    vex::task::sleep(1000);
  }
  double drift = (Inertial.rotation(degrees) / 5.0);
  Brain.Screen.setCursor (4, 1);
  Brain.Screen.print("DRIFT = %.31f", drift);
  vex::task::sleep(3000);
  Brain.Screen.clearScreen();
}

////////////////////////////////////////////////////////////////
//Ultrasonic Range Finder
///////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Section: Drive Train for Auton/Skills based on the Gyro/Inertial
// StraightTime, BackwardAlign
// ResetHeading, GetRotation, PrintGyro
// GyroTurn,
// InitializeCheckStop, CheckStop
// GyroStraight
// GyroStraightTime, GyroStraightDistance
/////////////////////////////////////////////////////////////
// StraightTime goes straight at a given tim at a given velocity
// Parameters:
//  - velocity: specifies how fast in terms of %
//  - seconds: how long the straight goes for
void StraightTime(double velocity, double seconds) {
  LeftFront.spin(fwd, velocity, pct);
  LeftBack.spin(fwd,velocity,pct);
  RightBack.spin(fwd,velocity,pct);
  RightFront.spin(fwd,velocity,pct);

  vex::task::sleep(int(seconds * 1000));

  // stop all motors
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
}

// Resets inertial heading variable to 0
void ResetHeading() {
  heading = 0;
  Inertial.resetRotation();
}

// Scaling by 1.009
// We got this number by rotating the robot 90 degrees and checking the inertial reading
double GetRotation() {
  return 1.009 * Inertial.rotation(degrees);
}

// Prints gyro stats for debugging
void PrintGyro() {
  Brain.Screen.print("Print gyro");
  for (int i = 0; i < 5; i ++) {
    Brain.Screen.setCursor(i + 6, 1);
    Brain.Screen.print("h: %.10f rotation: %.10f yaw = %.11f getR: %.1f", heading, Inertial.rotation(), Inertial.yaw(), GetRotation());
    vex::task::sleep(2000);
  }
  Brain.Screen.clearScreen();
}

// Turns based on inertial sensor.
// At the end, updates the heading variable
// Parameter:
// - turn_degrees: how much the robot turns in terms of degrees
void GyroTurn(double turn_degrees) {
  double target_heading = heading + turn_degrees;
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print(
      "Start h: %.10f, turnDeg: %.10f targetH: %.10f r: %.10f yaw: %.10f getR: %.1f", heading, 
      turn_degrees, target_heading, Inertial.rotation(), Inertial.yaw(), GetRotation());
  if (turn_degrees > 0) { //turn right
    // Turn fast until we get to within 30 degrees of target
    while (GetRotation() < target_heading - 30) {
      Brain.Screen.setCursor(4,1);
      Brain.Screen.print("r: %.10f y: %.10f getR: %.10f", Inertial.rotation(), Inertial.yaw(), GetRotation());
      
      RightFront.spin(directionType::rev,FAST_TURN_SPEED,pct);
      LeftFront.spin(directionType::fwd,FAST_TURN_SPEED,pct);
      RightBack.spin(directionType::rev,FAST_TURN_SPEED,pct);
      LeftBack.spin(directionType::fwd,FAST_TURN_SPEED,pct);
      vex::task::sleep(10);
    }
    // Overturns 3.0 degrees. At speed 10, by the time gyro returns 90 degrees,
    // robot would've overturned 3.5 degrees.
    while (GetRotation() < target_heading - 3.0) {
      Brain.Screen.setCursor(4, 1);
      Brain.Screen.print("r: %.10f y: %.10f getR: %.10f", Inertial.rotation(), Inertial.yaw(), GetRotation());

      RightFront.spin(directionType::rev,SLOW_TURN_SPEED,pct);
      LeftFront.spin(directionType::fwd,SLOW_TURN_SPEED,pct);
      RightBack.spin(directionType::rev,SLOW_TURN_SPEED,pct);
      LeftBack.spin(directionType::fwd,SLOW_TURN_SPEED,pct);
      vex::task::sleep(10);
    }
  } else if (turn_degrees < 0) { //turn left

    //Turn fast until we get to within 30 degrees of target
    while (GetRotation() > target_heading + 30) {
      Brain.Screen.setCursor(4,1);
      Brain.Screen.print("r: %.10f y: %.10f getR: %.10f", Inertial.rotation(), Inertial.yaw(), GetRotation());
      
      RightFront.spin(directionType::fwd,FAST_TURN_SPEED,pct);
      LeftFront.spin(directionType::rev,FAST_TURN_SPEED,pct);
      RightBack.spin(directionType::fwd,FAST_TURN_SPEED,pct);
      LeftBack.spin(directionType::rev,FAST_TURN_SPEED,pct);
      vex::task::sleep(10);
    }
    // Overturns 3.0 degrees. At speed 10, by the time gyro returns 90 degrees,
    // robot would've overturned 3.5 degrees.
    while (GetRotation() > target_heading + 3.0) {
      Brain.Screen.setCursor(4, 1);
      Brain.Screen.print("r: %.10f y: %.10f getR: %.10f", Inertial.rotation(), Inertial.yaw(), GetRotation());

      RightFront.spin(directionType::fwd,SLOW_TURN_SPEED,pct);
      LeftFront.spin(directionType::rev,SLOW_TURN_SPEED,pct);
      RightBack.spin(directionType::fwd,SLOW_TURN_SPEED,pct);
      LeftBack.spin(directionType::rev,SLOW_TURN_SPEED,pct);
      vex::task::sleep(10);
    }
  }   
  RightFront.stop(brakeType::brake);
  RightBack.stop(brakeType::brake);
  LeftBack.stop(brakeType::brake);
  LeftFront.stop(brakeType::brake);
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("FIN r: %.10f y: %.10f getR: %.10f", Inertial.rotation(), Inertial.yaw(), GetRotation());

  //Update the heading
  heading = target_heading;
}

/*
// This is an improved turn, where it tries to turn within 1 degree
// and also wiggles back and forth until 1 degree.
// Parameter:
// - turn_degrees: how much the robot turns in terms of degrees
void NewGyroTurn(double turn_degrees) {
  double target_heading = heading + turn_degrees;
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Start h: %.10f, turnDeg: %.10f targetH: %.10f r: %.10f yaw: %.10f getR: %.1f", heading, 
      turn_degrees, target_heading, Inertial.rotation(), Inertial.yaw(), GetRotation());
  double error = GetRotation() - target_heading;
  double prev_error = error;
  double abs_error = fabs(error);
  int wiggle_count = 0;
  while (abs_error > 1 && wiggle_count < 4) {
    double turn_speed;
    // decrease speed smoothly
    if (abs_error > START_SLOW_TURN) {
      turn_speed = FAST_TURN_SPEED;
    }
    else if (abs_error < STOP_SLOW_TURN) {
      turn_speed = SLOW_TURN_SPEED;
    } else {
      turn_speed = SLOW_TURN_SPEED + TURN_SPEED_SLOPE*(abs_error - STOP_SLOW_TURN); //for 5-30
    }
    // rotation
    if (error > 0) {
      // Turn left/clockwise if error > 0
      RightFront.spin(directionType::rev, turn_speed, velocityUnits::pct);
      LeftFront.spin(directionType::fwd, turn_speed, velocityUnits::pct);
      RightBack.spin(directionType::rev, turn_speed, velocityUnits::pct);
      LeftBack.spin(directionType::fwd, turn_speed, velocityUnits::pct);
    }
    vex::task::sleep(TIME_DELTA*1000);
    prev_error = error;
    error = GetRotation() - target_heading;
    abs_error = fabs(error);
    if ((error > 0 && prev_error < 0) || (error < 0 && prev_error >0)) {
      wiggle_count++;
    }
  }
  RightFront.stop(brakeType::brake);
  LeftFront.stop(brakeType::brake);
  RightBack.stop(brakeType::brake);
  LeftBack.stop(brakeType::brake);
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("FIN r: r: %.10f y: %.10f getR: %.10f", Inertial.rotation(), Inertial.yaw(), GetRotation());
  //Update the heading
  heading = heading + turn_degrees;  
}
*/

// Return inches traveled given degrees motor has turned
// Parameter:
// - motor_degrees: the number of degrees the drive motor has turned
double GetInchesFromDegrees(double motor_degrees) {
  // First translate motor_degrees to wheel_degrees by multiplying 5.0/3.0
  // as it is geared 3:5
  // Then given the wheel_degrees, divide by 360.0 to get number of rotations
  // and multiple circumference
  return (motor_degrees * (5.0/3.0) * (WHEEL_CIRCUMFERENCE / 360.0));
}

// Compute the top velocity we can have if robot needs to have a given stop_distance
// Use 2ad = vf^2 - vo^2 =>
//     vf = sqrt (2ad + vo^2)
double ComputeTopVelocity(double stop_distance) {
  return (sqrt((2 * (VELOCITY_DELTA / TIME_DELTA) * stop_distance) / 0.5 + STOP_VELOCITY * STOP_VELOCITY));
}

// Computes the "stopping" distance in inches to decelerate from top_velocity to STOP_VELOCITY
// Use d = time * average velocity
double ComputeStopDistance(double top_velocity) {
// stop time is (delta velocity) / acceleration
  double stop_time = (top_velocity - STOP_VELOCITY) / (VELOCITY_DELTA/TIME_DELTA);
// stop_distance is stop_time * average velocity
// the average velocity in motor power is translated to inches/sec by multiplying 0.5
// based on the experiment that for our robot inches/sec = 0.5 * motor_power
  return (((top_velocity + STOP_VELOCITY)/2.0) * stop_time *0.5);
}

// Initialize the objects needed for checking the stopping conditions
// Parameter:
// -stop_method: the type of stopping condition we are checking
void InitializeCheckStop(int stop_method) {
  if (stop_method == CHECK_DISTANCE) {
    LeftFront.setRotation(0, rotationUnits::deg);
  } else if (stop_method == CHECK_TIME) {
    gyro_straight_timer.clear();
  }
}

// This code checks the stopping type of the straight
// Parameters:
// - stop_method is the type of stopping condition we are checking (e.g., CHECK_DISTANCE, CHECK_TIME)
// - val is the value to check
// - output & distance_traveled is how far the robot has traveled
bool CheckStop(int stop_method, double val, double distance_traveled) {
  if (stop_method == CHECK_DISTANCE) {
    distance_traveled = GetInchesFromDegrees(fabs(LeftFront.rotation(rotationUnits::deg)));
    return (distance_traveled >= fabs(val));
  } else if (stop_method == CHECK_TIME) {
    return (gyro_straight_timer.time(timeUnits::sec) >= val);
  }
  return true;
}

// This code makes the robot go straight and autocorrects.
// Parameters:
//  - speed: how fast the robot moves
//  - stop_method
//  - stop_val is the value to check
void GyroStraight(double speed, int stop_method, double stop_val) { //(double speed, int stop_method, double stop_val, bool avoid_right, bool avoid_left)
// Speed will indicate if going forward (positive) or backward (negative)
  bool is_forward = (speed > 0);
// After recording if forward/backward, make speed positive
  speed = fabs(speed);

  InitializeCheckStop(stop_method);

  //Smooth variables
  double top_velocity = speed;
  double total_distance = stop_val;
  double distance_traveled = 0.0, stop_distance = 0.0;
  bool do_smooth = (SMOOTH_FLAG && (stop_method == CHECK_DISTANCE) && (top_velocity >= FAST_VELOCITY));

// Recalculate and decrease the top_velocity
  if (do_smooth) {
    stop_distance = ComputeStopDistance(top_velocity);
    if (2*stop_distance > total_distance) {
      stop_distance = total_distance/2.0;
      top_velocity = ComputeTopVelocity(stop_distance);
    }
  }

while (!CheckStop(stop_method, stop_val, distance_traveled)) {
  // Straighten code
  // Counter clockwise is negative, clockwise is positive. So if robot
  // has veered right, Gyro.value will be bigger than heading. Error will be positive
  // when veered left, error will be negative.
  double error = (GetRotation() - heading) / 180.0;
  double speed_correction = error * speed * CORRECTION_FACTOR; // error * speed * CORRECTION_FACTOR;
  if (is_forward) {
    // When going forward, if error is positive, right motors should spin faster
    // Example 1:
    // speed = 30, heading = 0, Inertial.rotation() = 4.5   robot is pointing right
    // error = 4.5/90 = 0.05, speed_correction = 0.05 * 30 * 0.7 = 1.05
    // Example 2:
    // speed = 10, heading = 0 Inertial.rotation() = -9  robot is pointing left
    // error = -9/90 = -0.1 speed_correction = -0.1 * 10 * 0.7 = -0.7
    
    if (ActivateSonar && (0 <= RightDistance <= 8)) {
      RightFront.spin(fwd, speed, velocityUnits::pct);
      LeftFront.spin(fwd, speed + (speed_correction*10), velocityUnits::pct);
      RightBack.spin(fwd, speed, velocityUnits::pct);
      LeftBack.spin(fwd, speed + (speed_correction*10), velocityUnits::pct);
    }
    else if (!ActivateSonar || (RightDistance > 8)) {
      LeftFront.spin(fwd, speed, velocityUnits::pct);
      RightFront.spin(fwd, speed + (speed_correction*15), velocityUnits::pct);
      LeftBack.spin(fwd, speed , velocityUnits::pct);
      RightBack.spin(fwd, speed + (speed_correction*15), velocityUnits::pct);
    }

    if (ActivateSonar && (0 <= LeftDistance <= 8)) {
      RightFront.spin(fwd, speed + (speed_correction*10), velocityUnits::pct);
      LeftFront.spin(fwd, speed, velocityUnits::pct);
      RightBack.spin(fwd, speed + (speed_correction*10), velocityUnits::pct);
      LeftBack.spin(fwd, speed, velocityUnits::pct);
    }
    else if (!ActivateSonar || (LeftDistance > 8)) {
      LeftFront.spin(fwd, speed, velocityUnits::pct);
      RightFront.spin(fwd, speed + (speed_correction*15), velocityUnits::pct);
      LeftBack.spin(fwd, speed , velocityUnits::pct);
      RightBack.spin(fwd, speed + (speed_correction*15), velocityUnits::pct);
    }

  } else {
    // When going backward, if error is positive (front is pointing to the right) left motors should spin faster
    LeftFront.spin(directionType::rev, speed, velocityUnits::pct);
    RightFront.spin(directionType::rev, speed, velocityUnits::pct);
    LeftBack.spin(directionType::rev, speed, velocityUnits::pct);
    RightBack.spin(directionType::rev, speed, velocityUnits::pct);
    
  }

  // smooth deceleration and acceleration code
  if (do_smooth) {
    if (((total_distance - distance_traveled) <= stop_distance) && (speed > STOP_VELOCITY)) {
      // If we are in stop_distance, slow down
      speed -= VELOCITY_DELTA;
    } else if (speed < top_velocity) {
      // Speed up if we haven't achieved top_velocity
      speed += VELOCITY_DELTA;
    }
  }
  task::sleep(TIME_DELTA*1000);
}
if (do_smooth) {
  LeftFront.stop(brakeType::coast);
  RightFront.stop(brakeType::coast);
  RightBack.stop(brakeType::coast);
  LeftBack.stop(brakeType::coast);
} else {
  LeftFront.stop(brake);
  RightFront.stop(brake);
  LeftBack.stop(brake);
  RightBack.stop(brake);
  }

}

// This function makes the robot go straight for a specified amount of time
// Parameters:
// - speed: how fast the robot goes in terms of %
// - time_constraint: how long it goes straight for
void GyroStraightTime(double speed, double time_constraint) {
  GyroStraight(speed, CHECK_TIME, time_constraint);
}

// This function makes the robot go straight until it reaches a specified distance
//Parameters:
// -speed: how fast robot goes %
// -inches: how far robot goes
void GyroStraightDistance(double speed, double inches) {
  GyroStraight(speed, CHECK_DISTANCE, inches);
}

/////////////////////////////////////////////////////////////////
// Section: Methods for turning on/off the Intake and Mobile Goal
////////////////////////////////////////
void IntakeOn() {
  Intake.spin(directionType::fwd, 80, velocityUnits::pct);
}

void IntakeOff() {
  Intake.stop();
}

void MogoLiftOut(bool state) {
  MogoLift.rotateFor(0.95, vex::rotationUnits::rev, 100, vex::velocityUnits::pct, state);
  vex::task::sleep(200);
  MogoLift.stop(hold);
}

void MogoLiftIn(bool state) {
  MogoLift.rotateFor(-0.725, vex::rotationUnits::rev, 80, vex::velocityUnits::pct, state);
  vex::task::sleep(200);
  MogoLift.stop(hold);
}



void LiftUp(int degree, bool state1, bool state2) {
  RightLift.rotateFor(-degree, rotationUnits::deg, 100, velocityUnits::pct, state1); //165
  LeftLift.rotateFor(degree, rotationUnits::deg, 100, velocityUnits::pct, state2);
  vex::task::sleep(200);
  RightLift.stop(hold);
  LeftLift.stop(hold);
}

void LiftDown(int degree, bool state1, bool state2) {
  RightLift.rotateFor(degree, rotationUnits::deg, 80, velocityUnits::pct, state1);
  LeftLift.rotateFor(-degree, rotationUnits::deg, 80, velocityUnits::pct, state2);
  vex::task::sleep(200);
  RightLift.stop(hold);
  LeftLift.stop(hold);
}

// Sleeps if doSleep is true
void SleepCheck(bool doSleep) {
  if (doSleep) {
    vex::task::sleep(6000);
  }
}

// Sleeps
void SleepCheck() {
  SleepCheck(true);
}

//Boolean values for mogo lift and lift
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;


/////////////////////////////////////////////////////////////////
//Sensors
////////////////////////////////////////////////////////////////

//Pneumatics
void togglePneumatics () {
  if(Clamp.value() == 0) {
    Clamp.open();
  } else {
    Clamp.close();
  }
}

//////////////////////////////////////////////////////////////////////
//Limit Switch
//////////////////////////////////////////////////////////////////////
bool LimitValue = true;
int LimitCheck() {
  while(true) {
    if(LimitFront.pressing() == 0 && LimitValue == true) {
      Clamp.close();
      LimitValue = false;
    }
    else if(LimitFront.pressing() == 1 && LimitValue == true) {
      Clamp.open();
      LimitValue = false;
    }
    else if(!LimitValue) {
      LimitValue = true;
    //set the toggle so that the button will not be pressed twice
    }
  }
  wait(20,msec);
  return 0;
}

//////////////////////////////////////////////////////////////////////
///Ultrasonic rangefinder
//////////////////////////////////////////////////////////////////////

//Ultrasonic rangefinder: usable range: 1.5” (3.0cm) and 115” (300cm)
//The distance that is measured is from the base of ultrasonic range finder
//Use to check Ultrasonic Range Finder if it works


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  SetupInertial();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your ofwn robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  vex::task SonarRight(CheckRightSonar);
  vex::task SonarLeft(CheckLeftSonar);

  //Notes
  /*Activate ultrasonic: ActivateSonar = true; for grabbing goal accurately
  Deactivate ultrasonic: ActivateSonar = false;
  GyroTurn(15) //positive to the right, negative to the left
  GyroStraightDistance(MEDIUM_VELOCITY,64.00); FAST_VELOCITY, MEDIUM_VELOCITY, SLOW_VELOCITY) Distances in inches
  LiftDown(10,false, false); // 165 degrees is the platform level. Finish the action, then move on: true, true. Multitask: false, false
  LiftDown(10,false, false);
  */

  //Start off on the left side, grab the yellow mobile goal
  ActivateSonar = true; // activate ultrasonic
  Clamp.open();
  LiftDown(10,false, false);
  GyroStraightDistance(FAST_VELOCITY,65.00);
  Clamp.close();
  
  vex::task::sleep(200);
  ActivateSonar = false;
  GyroTurn(15);
  LiftUp(290, false, true); //maximum height
  GyroStraightDistance(MEDIUM_VELOCITY,64.00);

  //Place the yellow mobile goal onto the opposite platform. Straighten using the platform
  LiftDown(150,false,true); //160 is limit
  Clamp.open(); //release the mobile goal
  GyroStraightDistance(-FAST_VELOCITY,5.00); //add minus for moving straight backwards
  LiftUp(40, false, false); //165 //Fix this
  GyroStraightDistance(-MEDIUM_VELOCITY,5.00);

  //Turn 140 degrees to the right, pushes the middle yellow mobile goal to homezone
  GyroTurn(140); //facing 5 degree
  LiftDown(190, false, true);
  ActivateSonar = true;
  GyroStraightDistance(MEDIUM_VELOCITY,53.00);
  ActivateSonar = false;
  GyroStraightDistance(-SLOW_VELOCITY,5.00); // back up


  //Turn left 130 degree, pushes the right yellow mobile goal to opposite zone
  GyroTurn(-95); //facing 130 degree
  vex::task::sleep(200);
  ActivateSonar = true;
  GyroStraightDistance(FAST_VELOCITY, 20.00);
  ActivateSonar = false;
  GyroTurn(-25);
  GyroStraightDistance(FAST_VELOCITY,40.00);
  vex::task::sleep(100);

  //Turn left 110 degree,
  GyroStraightDistance(-MEDIUM_VELOCITY,5.00); //back up
  GyroTurn(-77); //facing 240 degree
  vex::task::sleep(200);
  ActivateSonar = true;
  GyroStraightDistance(FAST_VELOCITY,38.00); //grab the opposite alliance goal near the platform
  ActivateSonar = false;
  Clamp.close();
  vex::task::sleep(200);

//20in
  //Back up a lot and turn to face the home platform
  GyroStraightDistance(-MEDIUM_VELOCITY,14.00);
  LiftUp(290, false, true);
  GyroTurn(-140); //facing 340 degree
  vex::task::sleep(200);
  GyroStraightDistance(MEDIUM_VELOCITY,67.00);
  GyroTurn(15);
  GyroStraightDistance(MEDIUM_VELOCITY,20.00);

  //Place the mobile goal down onto the platform
  LiftDown(160,false,true); //160 is limit
  Clamp.open();
  GyroStraightDistance(-FAST_VELOCITY,5.00);
  LiftUp(50, false, false); //165 //Fix this;;
  GyroStraightDistance(-MEDIUM_VELOCITY,5.00);

  //Turn left to grab the opposite alliance goal
  LiftDown(175, false, false);
  GyroTurn(-95); //facing 0 degree
  vex::task::sleep(200);
  GyroStraightDistance(FAST_VELOCITY,43.00);
  Clamp.close();

 //Turn facing the opposite platform and goes toward there
  GyroTurn(-120);
  LiftUp(290, false, true);
  GyroStraightDistance(MEDIUM_VELOCITY,88.00);

  //Place the mobile goal down onto the platform
  LiftDown(160,false,true); //165 is limit
  Clamp.open();
  GyroStraightDistance(-FAST_VELOCITY,5.00);
  LiftUp(50, false, false); //165 //Fix this
  GyroStraightDistance(-MEDIUM_VELOCITY,5.00);


/*
  //Face toward the left mobile goal and grab the goal
  GyroTurn(-90);
  GyroStraightDistance(FAST_VELOCITY,50.00);
  Clamp.close();

  //Face toward the home platform and place mobile goal on it
  GyroTurn(-135);
  LiftUp(290, false, true);
  vex::task::sleep(200);
  GyroStraightDistance(MEDIUM_VELOCITY,108.00);

  //Place the mobile goal down onto the platform
  LiftDown(150,false,true); //165 is limit
  Clamp.open();
  GyroStraightDistance(-MEDIUM_VELOCITY,6.00);
  LiftUp(40, false, false); //165 //Fix this
  GyroStraightDistance(-MEDIUM_VELOCITY,2.00); //Hands are still lift up
*/




/* Old autonomous


//go to goal
LeftLift.rotateFor(0.14, vex::rotationUnits::rev, 100, vex::velocityUnits::pct, false);
RightLift.rotateFor(-0.14, vex::rotationUnits::rev, 100, vex::velocityUnits::pct, true);
RightFront.rotateFor(2.8, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
RightBack.rotateFor(2.8, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
LeftFront.rotateFor(2.8, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
LeftBack.rotateFor(2.8, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, true);

wait(300, msec);

//grab goal
RightFront.rotateFor(0.2, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
RightBack.rotateFor(0.2, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
LeftFront.rotateFor(0.2, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
LeftBack.rotateFor(0.2, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, true);
Clamp.close();


wait(200, msec);



//return

RightFront.rotateFor(-2, vex::rotationUnits::rev, 30, vex::velocityUnits::pct, false);
RightBack.rotateFor(-2, vex::rotationUnits::rev, 30, vex::velocityUnits::pct, false);
LeftFront.rotateFor(-2, vex::rotationUnits::rev, 30, vex::velocityUnits::pct, false);
LeftBack.rotateFor(-2, vex::rotationUnits::rev, 30, vex::velocityUnits::pct, true);
Clamp.open();


//drop goal for easier maneuvering


wait(300, msec);

//reverse 
RightFront.rotateFor(-0.5, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
RightBack.rotateFor(-0.5, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
LeftFront.rotateFor(-0.5, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
LeftBack.rotateFor(-0.5, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, true);

wait(300, msec);

//rotate robot fully to allign properly with second mogo
MogoLift.rotateFor(0.95, vex::rotationUnits::rev, 100, vex::velocityUnits::pct, false);
RightFront.rotateFor(0.6, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
RightBack.rotateFor(0.6, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
LeftFront.rotateFor(-0.6, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
LeftBack.rotateFor(-0.6, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, true);

wait(200, msec);

//towards goal and lower mogo lift


LeftFront.rotateFor(-1.2, vex::rotationUnits::rev, 60, vex::velocityUnits::pct, false);
LeftBack.rotateFor(-1.2, vex::rotationUnits::rev, 60, vex::velocityUnits::pct, false);
RightFront.rotateFor(-1.2, vex::rotationUnits::rev, 60, vex::velocityUnits::pct, false);
RightBack.rotateFor(-1.2, vex::rotationUnits::rev, 60, vex::velocityUnits::pct, true);
MogoLift.rotateFor(-0.725, vex::rotationUnits::rev, 100, vex::velocityUnits::pct, true);
wait(200, msec);

//grab mogo and lift it while also lifting 4bar for the possibility of matchloading
//LeftLift.rotateFor(0.7, vex::rotationUnits::rev, 100, vex::velocityUnits::pct, false);
//RightLift.rotateFor(-0.7, vex::rotationUnits::rev, 100, vex::velocityUnits::pct, false);

LeftFront.rotateFor(0.7, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
LeftBack.rotateFor(0.7, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
RightFront.rotateFor(0.7, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, false);
RightBack.rotateFor(0.7, vex::rotationUnits::rev, 70, vex::velocityUnits::pct, true);

wait(20, msec);

//backup and put rings in mobile goal
Intake.rotateFor(10, vex::rotationUnits::rev, 50, vex::velocityUnits::pct, false);
*/
}  

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// Intake moving forward code
bool intakeForward = false;
void ToggleIntakeForward() {
  intakeForward = !intakeForward;
  if (intakeForward) {
    Intake.spin(forward);
  } else {
  Intake.stop();
  }
}

// Intake moving forward code
bool intakeReverse = false;
void ToggleIntakeReverse() {
  intakeReverse = !intakeReverse;
  if (intakeReverse) {
    Intake.spin(reverse);
  } else {
    Intake.stop();
  }
}




  /*
  bool CircularMotion = false;
  void toggleClamp () {
      CircularMotion = !CircularMotion;
  }
  */

void usercontrol(void){

  vex::task ValueSonarRight(CheckRightSonar);
  vex::task ValueSonarLeft(CheckLeftSonar);

  Controller1.ButtonB.pressed(ToggleIntakeForward);
  Controller1.ButtonY.pressed(ToggleIntakeReverse);

  //vex::task BackCheck(CheckBack);

  Controller1.ButtonA.pressed(togglePneumatics);
/*Controller1.ButtonA.pressed(toggleClamp);
   while (1) {
     if(CircularMotion == true){
      Clamp.open();
    }
    else if (CircularMotion == false){
      Clamp.close();
    }
*/
  // User control code here, inside the loop
 
  //sigmoid map values
  int sigmoid_map[255] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100,
  -100, -100, -100, -100, -99,  -99,  -99,  -99,  -99,  -99,  -99,  -99,
  -99,  -99,  -99,  -99,  -99,  -99,  -99,  -99,  -99,  -99,  -99,  -99,
  -99,  -98,  -98,  -98,  -98,  -98,  -98,  -97,  -97,  -97,  -96,  -96,
  -96,  -95,  -95,  -94,  -94,  -93,  -92,  -92,  -91,  -90,  -89,  -88,
  -86,  -85,  -84,  -82,  -80,  -79,  -77,  -75,  -73,  -70,  -68,  -66,
  -63,  -61,  -58,  -55,  -52,  -50,  -47,  -44,  -41,  -39,  -36,  -34,
  -31,  -29,  -27,  -24,  -22,  -21,  -19,  -17,  -16,  -14,  -13,  -12,
  -10,  -9,   -8,   -8,   -7,   -6,   -5,   -5,   -4,   -4,   -3,   -3,
  -3,   -2,   -2,   -2,   -2,   -1,   -1,   -1,   -1,   -1,   -1,   -1,
  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    1,    1,    1,    1,    1,    1,    1,    2,    2,
  2,    2,    3,    3,    3,    4,    4,    5,    5,    6,    7,    8,
  8,    9,    10,   12,   13,   14,   16,   17,   19,   21,   22,   24,
  27,   29,   31,   34,   36,   39,   41,   44,   47,   50,   52,   55,
  58,   61,   63,   66,   68,   70,   73,   75,   77,   79,   80,   82,
  84,   85,   86,   88,   89,   90,   91,   92,   92,   93,   94,   94,
  95,   95,   96,   96,   96,   97,   97,   97,   98,   98,   98,   98,
  98,   98,   99,   99,   99,   99,   99,   99,   99,   99,   99,   99,
  99,   99,   99,   99,   99,   99,   99,   99,   99,   99,   99,   100,
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,
  100,  100,  100
  };


  //drivetrain
    
    
    
    /*RightFront.spin(directionType::fwd, Controller1.Axis3.value() - Controller1.Axis1.value(), percentUnits::pct);
    RightBack.spin(directionType::fwd, Controller1.Axis3.value() - Controller1.Axis1.value(), percentUnits::pct);
    LeftFront.spin(directionType::fwd,Controller1.Axis3.value() + Controller1.Axis1.value(), percentUnits::pct);
    LeftBack.spin(directionType::fwd,Controller1.Axis3.value() + Controller1.Axis1.value(), percentUnits::pct);*/
        

    
  
  //speed control
  Intake.setVelocity(80, percent);
  RightFront.setVelocity(40, percent);
  LeftFront.setVelocity(40, percent);
  RightBack.setVelocity(40, percent);
  LeftBack.setVelocity(40, percent);

  //the brakes
  MogoLift.setStopping(brakeType::hold);
  RightLift.setStopping(brakeType::hold);
  LeftLift.setStopping(brakeType::hold);
    

    
      //BELOW: Intake Toggle Code
    
  bool togglebEnabled = false; // two-choice toggle, so we use bool
  bool buttonbPressed = false; // IGNORE, logic variable
  
  while (true){
    
    /*

    bool buttonB = Controller1.ButtonB.pressing();

    if (buttonB && !buttonbPressed){
      buttonbPressed = true; 
      togglebEnabled = !togglebEnabled; 
    }
    else if (!buttonB) buttonbPressed = false;
    
    ////////////////////////////////////////////////////////////////////
    // Code For toggle Enabled or Disabled
    
   

    if(togglebEnabled){
      // Do another thing
      Intake.spin(forward);
      RightFront.spin(directionType::fwd, sigmoid_map[Controller1.Axis3.value() + 127] - sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
      RightBack.spin(directionType::fwd, sigmoid_map[Controller1.Axis3.value() + 127] - sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
      LeftFront.spin(directionType::fwd,sigmoid_map[Controller1.Axis3.value() +127] + sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
      LeftBack.spin(directionType::fwd,sigmoid_map[Controller1.Axis3.value() +127] + sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
      }


    else{
      // Do initial thing
      Intake.stop();
      RightFront.spin(directionType::fwd, sigmoid_map[Controller1.Axis3.value() + 127] - sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
      RightBack.spin(directionType::fwd, sigmoid_map[Controller1.Axis3.value() + 127] - sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
      LeftFront.spin(directionType::fwd, sigmoid_map[Controller1.Axis3.value() + 127] + sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
      LeftBack.spin(directionType::fwd, sigmoid_map[Controller1.Axis3.value() + 127] + sigmoid_map[Controller1.Axis1.value() +127], percentUnits::pct);
    }

    */


    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("r: %.10f y: %.10f getR: %.10f", Inertial.rotation(), Inertial.yaw(), GetRotation());

   //End of intake toggle code
   


  
  //Lift
  // check the ButtonL1/ButtonL2 status to control RightLift
  if (Controller1.ButtonL2.pressing()) {
    RightLift.spin(forward);
    LeftLift.spin(reverse);
    Controller1LeftShoulderControlMotorsStopped = false;
  } else if (Controller1.ButtonL1.pressing()) {
    RightLift.spin(reverse);
    LeftLift.spin(forward);
    Controller1LeftShoulderControlMotorsStopped = false;
  } else if (!Controller1LeftShoulderControlMotorsStopped) {
    RightLift.stop();
    LeftLift.stop();
    // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
    Controller1LeftShoulderControlMotorsStopped = true;
  }
  // check the ButtonR1/ButtonR2 status to control MogoLift
  if (Controller1.ButtonR1.pressing()) {
    MogoLift.spin(reverse);
    Controller1RightShoulderControlMotorsStopped = false;
  } else if (Controller1.ButtonR2.pressing()) {
    MogoLift.spin(forward);
    Controller1RightShoulderControlMotorsStopped = false;
  } else if (!Controller1RightShoulderControlMotorsStopped) {
    MogoLift.stop();
    // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
    Controller1RightShoulderControlMotorsStopped = true;
  }  

  //Old Lift Code  
   /* if (Controller1.ButtonL2.pressing()) {
    LeftLift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    RightLift.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  } else if (Controller1.ButtonL1.pressing()) {
    LeftLift.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    RightLift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  }*/

  //crappy old pneumatics - doid not work
    /*
    Controller1.ButtonA.pressed(toggleClamp);
    while(true){
    if(CircularMotion == true){
      Clamp.open();
    }
    else if(CircularMotion == false) {
      Clamp.close();
    }
  }
  */

  wait(20, msec);


  }

  
     
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

   
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function


  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

