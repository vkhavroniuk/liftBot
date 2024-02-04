/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       OLEG KHAVRONIUK                                           */
/*    Created:      9/19/2023, 6:22:50 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "PID.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1 = controller(primary);
motor MotorLF = motor(PORT20, ratio18_1, true); // reversed 
motor MotorLB = motor(PORT19, ratio18_1, true); // reversed
motor MotorRF = motor(PORT11, ratio18_1, false); // forward direction
motor MotorRB = motor(PORT12, ratio18_1, false); // forward direction
motor Shooter = motor(PORT4, ratio36_1, true);

motor LIntake = motor(PORT1, ratio18_1, false);
motor RIntake = motor(PORT2, ratio18_1, false);
motor Arm = motor(PORT8, ratio36_1, false);
motor Lift = motor(PORT3, ratio36_1, false);
inertial DaInertial = inertial(PORT10);

limit catalimit = limit(Brain.ThreeWirePort.D);
limit liftlimit = limit(Brain.ThreeWirePort.H);
digital_out leftWing = digital_out(Brain.ThreeWirePort.A);
digital_out rightWing = digital_out(Brain.ThreeWirePort.B);
digital_out ratchet = digital_out(Brain.ThreeWirePort.C);

motor_group LeftMotors = motor_group(MotorLF, MotorLB);
motor_group RightMotors = motor_group(MotorRF, MotorRB);

// Global Variables & Constants
bool ShootButtonPressed = false;
bool WingButtonPressed = false;
bool ArmButtonPressed = false;
bool RWingButtonPressed = false;
bool RTE = false;
float velocity_control = 1;
PID drivePID;
PID turnPID;
PID straightPID;
  // 1 revolution = ~26cm
  // 60/36 Gear Ratio
  // One Rev is 360 degrees and 25.9207*(60/36) = 43.20cm
  // cm to degree ration is 360/43.20 = 8.334
float const WHEEL_DIAMETER = 8.3; // cm
float const WHEEL_CIRC = WHEEL_DIAMETER * 3.14;
float const GEAR_RATIO = 1.67;
float const DEGREE_PER_CM = 360 / (WHEEL_CIRC * GEAR_RATIO);

bool isArmOpen(){
  int encoderValue = Arm.position(deg);
  if(encoderValue > 40){
    return true;
  }
  return false;
}


bool isRightWOpen(){
  if(rightWing.value()){
    return true;
  }
  return false;
}


bool isLeftWOpen(){
  if(leftWing.value()){
    return true;
  }
  return false;
}


bool LiftInRange(void){
  if(Lift.position(deg) > 830){
    return false;
    //260 degrees for elevation
    //320 degrees for cool shooting
  }
  return true;
}


void push(int time){
 LeftMotors.setVelocity(65,pct);
 RightMotors.setVelocity(65,pct);
 RightMotors.spin(reverse);
 LeftMotors.spin(reverse);
 wait(time,sec);
 LeftMotors.stop();
 RightMotors.stop();
}


void event_Wings(void){
  if(!isLeftWOpen())
  {
    leftWing.set(true);
  }
  else
  {
    leftWing.set(false);
  }
  if(!isRightWOpen())
  {
    rightWing.set(true);
  }
  else
  {
    rightWing.set(false);
  }
}

void event_RightWing(void){
  if(!isRightWOpen())
  {
    rightWing.set(true);
  }
  else
  {
    rightWing.set(false);
  }
}

void elevate(void){
  if(!RTE){
    ratchet.set(0);
    Lift.setBrake(hold);
    Lift.spinTo(825,deg,false);
    RTE = true;
  }
  else if(RTE){
    ratchet.set(1);
    Lift.setBrake(hold);
    Lift.spinTo(240,deg,true);
    RTE = false;
  }
}


int catastop(){
  Shooter.setVelocity(20.0, percent);
  Shooter.spin(reverse);
  waitUntil(catalimit.value());
  Shooter.stop(hold);
  return 0;
}

void event_liftdown(void){
  if (Lift.position(deg) > 8) {
  Lift.setVelocity(50.0, percent);
  Lift.spin(reverse);
  }
}

void event_liftup(void){
  Lift.setVelocity(50.0, percent);
  Lift.spin(forward);
  waitUntil((!Controller1.ButtonUp.pressing()) || !LiftInRange());
  Lift.stop(hold);
}

void event_Catapult(void){
      if (!ShootButtonPressed) {
        Shooter.setVelocity(80.0, percent);
        Lift.spinTo(320,deg,false);
        Shooter.spin(reverse);
        ShootButtonPressed = true;
      }
      else {
        catastop();
        event_liftdown();
        ShootButtonPressed = false;
      };
};

void event_Intake(void){
  LIntake.setVelocity(100.0, percent);
  RIntake.setVelocity(100.0, percent);
  LIntake.spin(forward);
  RIntake.spin(reverse);
  waitUntil((!Controller1.ButtonR1.pressing()));
  LIntake.stop();
  RIntake.stop();
}

void event_Outake(void){
  LIntake.setVelocity(100.0, percent);
  RIntake.setVelocity(100.0, percent);
  LIntake.spin(reverse);
  RIntake.spin(forward);
  waitUntil((!Controller1.ButtonR2.pressing()));
  LIntake.stop();
  RIntake.stop();
}

void event_Arm(void){
    if (!isArmOpen()) {
      Arm.setStopping(coast);
    	Arm.setVelocity(70.0, percent);
      Arm.spinTo(75,deg, true); //was 170
      Arm.stop(hold);
    }
    else {
      Arm.setVelocity(70.0, percent);
      Arm.setStopping(coast);
      Arm.spinTo(30,deg,true);
      Arm.stop();
    }
}


void turn_right(int DegreesToTurn, float VelocityMin=2, float VelocityMax=12) {
  float speed;
  float currentDegrees;
  setPIDmin(turnPID, VelocityMin);
  setPIDmax(turnPID, VelocityMax);
  DaInertial.resetRotation();
  do {
    wait(20, msec);
    currentDegrees = DaInertial.rotation();
    speed = calculatePID(turnPID,DegreesToTurn, currentDegrees);
    RightMotors.spin(reverse, speed, volt);
    LeftMotors.spin(forward, speed, volt);
  } while(DegreesToTurn - currentDegrees > 0);
  RightMotors.stop(hold);
  LeftMotors.stop(hold);
  resetPID(turnPID);
}

void turn_left(int DegreesToTurn, float VelocityMin=2, float VelocityMax=12) {
  float speed;
  float currentDegrees;
  DaInertial.resetRotation();
  setPIDmin(turnPID, VelocityMin);
  setPIDmax(turnPID, VelocityMax);
  do {
    wait(20, msec);
    currentDegrees = fabs(DaInertial.rotation());
    speed = calculatePID(turnPID, DegreesToTurn, currentDegrees);
    RightMotors.spin(forward, speed, volt);
    LeftMotors.spin(reverse, speed, volt);
  } while(DegreesToTurn - currentDegrees > 0);
  RightMotors.stop(hold);
  LeftMotors.stop(hold);
  resetPID(turnPID);
}

void drive_forward(int distanceToDrive, float VelocityMin=2, float VelocityMax=12){
  float yaw;
  double correction;
  float currentDegree;
  double speed;
  float degreeToDrive = DEGREE_PER_CM * distanceToDrive;
  RightMotors.resetPosition();
  LeftMotors.resetPosition();
  DaInertial.resetRotation();
  setPIDmax(drivePID, VelocityMax);
  setPIDmin(drivePID, VelocityMin);
  do {
    wait(20,msec);
    currentDegree = (RightMotors.position(deg) + LeftMotors.position(deg)) / 2;
    speed = calculatePID(drivePID, degreeToDrive, currentDegree);
    yaw = DaInertial.rotation();
    correction = calculatePID(straightPID, 0, yaw);

    RightMotors.spin(forward, speed - correction, volt);
    LeftMotors.spin(forward, speed + correction, volt);
  } while(degreeToDrive - currentDegree > 3);
  RightMotors.stop(brake);
  LeftMotors.stop(brake);
  resetPID(drivePID);
  resetPID(straightPID);
}

void drive_backward(int distanceToDrive, float VelocityMin=2, float VelocityMax=12){
  float yaw;
  double correction;
  float currentDegree;
  float speed;
  float degreeToDrive = DEGREE_PER_CM * distanceToDrive;
  RightMotors.resetPosition();
  LeftMotors.resetPosition();
  DaInertial.resetRotation();
  setPIDmax(drivePID, VelocityMax);
  setPIDmin(drivePID, VelocityMin);

  do {
    wait(20,msec);
    currentDegree = (fabs(RightMotors.position(deg)) + fabs(LeftMotors.position(deg))) / 2;
    speed = calculatePID(drivePID, degreeToDrive, currentDegree);
    yaw = DaInertial.rotation();
    correction = calculatePID(straightPID, 0, yaw);
    LeftMotors.spin(reverse, speed - correction, volt);
    RightMotors.spin(reverse, speed + correction, volt);
  } while(degreeToDrive - currentDegree > 3);

  LeftMotors.stop(brake);
  RightMotors.stop(brake);
  resetPID(drivePID);
  resetPID(straightPID);
}

void outake_off(void){
  LIntake.stop();
  RIntake.stop();
}

void outake_on(void){
  LIntake.setVelocity(100, pct);
  RIntake.setVelocity(100, pct);
  LIntake.spin(reverse);
  RIntake.spin(forward);
}

void intake_on(void){
  LIntake.setVelocity(100, pct);
  RIntake.setVelocity(100, pct);
  LIntake.spin(forward);
  RIntake.spin(reverse);
}



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
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  DaInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial Sensor");
  while (DaInertial.isCalibrating())
  {
    wait(20, msec);
  };
  Brain.Screen.clearScreen();
}


int limit_switch_lift() {
  while (true) {

  while (true) {
    if (liftlimit.value()) {
      Lift.stop(hold);
      Lift.resetPosition();
      break;
    }
  }
      waitUntil(Lift.position(deg) > 9);
  }
  return 0;
}

int velocity_control_function(){
  while(true){
    velocity_control = 1;
    if(Lift.position(deg) > 300)
    {
      velocity_control = .6;
    }
    if(Lift.position(deg) > 700)
    {
      velocity_control = .4;
    }
    wait(500, msec);
  }
  return 0;
}

int ShowMeInfo(){
  while (true) {
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("Inertial Heading");
    Brain.Screen.setCursor(5,1);
    Brain.Screen.print(DaInertial.heading(degrees));
    Brain.Screen.setCursor(6,1);
    Brain.Screen.print("Inertial Rotation");
    Brain.Screen.setCursor(7,1);
    Brain.Screen.print(DaInertial.rotation(degrees)); 
    Brain.Screen.setCursor(8,1);
    Brain.Screen.print(" Motors Left/Right/Arm");
    Brain.Screen.setCursor(9,1);
    Brain.Screen.print(LeftMotors.position(rotationUnits::deg)); 
    Brain.Screen.setCursor(10,1);
    Brain.Screen.print(RightMotors.position(rotationUnits::deg));
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print(Lift.position(rotationUnits::deg));

    wait(25, msec);
  } 
  return 0;
}

void auto_own(void){
 
}


void auto_opposite(void){


}

void skills()
{


}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                        */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  //auto_opposite();
  //auto_own();
  //skills();
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

void usercontrol(void) {
  wait(15, msec);
  // User control code here, inside the loop
  Controller1.ButtonX.pressed(elevate);
  task VelocityController(velocity_control_function);
  while (1) {
    RightMotors.setVelocity((Controller1.Axis3.position() - Controller1.Axis1.position()) * velocity_control, percent);
    LeftMotors.setVelocity((Controller1.Axis1.position() + Controller1.Axis3.position()) * velocity_control, percent);
    RightMotors.spin(forward);
    LeftMotors.spin(forward);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  ratchet.set(0);
  Lift.resetPosition();
  task CataS(catastop);
  task LiftStopTask(limit_switch_lift);

  Controller1.ButtonL1.pressed(event_Catapult);
  Controller1.ButtonL2.pressed(event_Wings);
  Controller1.ButtonR2.pressed(event_Outake);
  Controller1.ButtonR1.pressed(event_Intake);
  Controller1.ButtonB.pressed(event_Arm);
  Controller1.ButtonRight.pressed(event_RightWing);
  Controller1.ButtonDown.pressed(event_liftdown);
  Controller1.ButtonUp.pressed(event_liftup);

  initPID(drivePID, 0.02, 0.001, 0.01, 40, 2, 10);
  initPID(turnPID, 0.08, 0.009, 0.05, 5, 2, 10);
  initPID(straightPID, 0.15, 0, 0.4, 1, -3, 3);

  ShootButtonPressed = false;
  WingButtonPressed = false;

  vex::task MyTask(ShowMeInfo);


  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
