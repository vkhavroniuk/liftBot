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
motor Arm = motor(PORT16, ratio36_1, false);
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
 LeftMotors.setVelocity(70,pct);
 RightMotors.setVelocity(70,pct);
 RightMotors.spin(reverse);
 LeftMotors.spin(reverse);
 wait(time,msec);
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

void event_LeftWing(void){
  if(!isLeftWOpen())
  {
    leftWing.set(true);
  }
  else
  {
    leftWing.set(false);
  }
}

void elevate(void){
  if(!RTE){
    ratchet.set(1);
    Lift.setBrake(hold);
    Lift.spinTo(825,deg,false);
    RTE = true;
  }
  else if(RTE){
    Lift.setBrake(hold);
    Lift.setVelocity(70.0, percent);
    Lift.spinTo(420,deg,true);
    RTE = false;
    ratchet.set(0);
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
        Shooter.setVelocity(90.0, percent);//was 80
        MotorLB.setBrake(hold);
        MotorLF.setBrake(hold);
        MotorRF.setBrake(hold);
        MotorRB.setBrake(hold);
        Lift.spinTo(320,deg,false);
        Shooter.spin(reverse);
        ShootButtonPressed = true;
      }
      else {
        catastop();
        MotorLB.setBrake(coast);
        MotorLF.setBrake(coast);
        MotorRF.setBrake(coast);
        MotorRB.setBrake(coast);
        event_liftdown();
        ShootButtonPressed = false;       
      };
};

void outake_off(void){
  LIntake.stop();
  RIntake.stop();
}

void outake_on(int spd = 100){
  LIntake.setVelocity(spd, pct);
  RIntake.setVelocity(spd, pct);
  LIntake.spin(reverse);
  RIntake.spin(forward);
}

void intake_on(void){
  LIntake.setVelocity(100, pct);
  RIntake.setVelocity(100, pct);
  LIntake.spin(forward);
  RIntake.spin(reverse);
}

void event_Intake(void){
  intake_on();
  waitUntil((!Controller1.ButtonR1.pressing()));
  outake_off();
}

void event_Outake(void){
  outake_on();
  waitUntil((!Controller1.ButtonR2.pressing()));
  outake_off();
}
void Arm_Move(void){
  Arm.setVelocity(80, pct);
  Arm.setMaxTorque(100, pct);
  Arm.setBrake(hold);//coast before
  Arm.spinToPosition(185, deg, true);//was 175
  Arm.stop();
}

void Arm_Move_back(void){
  Arm.setVelocity(80, pct);
  Arm.setMaxTorque(100, pct);
  Arm.setBrake(coast);
  Arm.spinToPosition(20, deg, true);
  Arm.stop();
}

void event_Arm(void){
    if (!isArmOpen() && Lift.position(deg) < 30) {
      Arm.setStopping(coast);
    	Arm.setVelocity(50.0, percent);
      Arm.setMaxTorque(100,pct);
      Arm.spinTo(150,deg, true); //was 170
      Arm.stop(hold);
    }
    else if (Lift.position(deg) < 30) {
      Arm.setVelocity(50.0, percent);
      Arm.setStopping(coast);
      Arm.spinTo(20,deg,true);
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
    MotorLB.spin(reverse, speed - correction, volt);
    MotorLF.spin(reverse, speed - correction, volt);
    MotorRB.spin(reverse, speed + correction, volt);
    MotorRF.spin(reverse, speed + correction, volt);
    //LeftMotors.spin(reverse, speed - correction, volt);
    //RightMotors.spin(reverse, speed + correction, volt);
  } while(degreeToDrive - currentDegree > 3);

  LeftMotors.stop(brake);
  RightMotors.stop(brake);
  resetPID(drivePID);
  resetPID(straightPID);
}


void parking(void)
{
  drive_backward(50,4,7);
  wait(20,msec);
  turn_left(50,4,7);
  wait(20,msec);
  drive_backward(2,4,5);
  wait(20,msec);
  event_RightWing();
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
  
  waitUntil(Lift.position(deg) > 9);

  while (true) {
    if (liftlimit.value()) {
      Lift.stop(hold);
      wait(200, msec);
      Lift.resetPosition();
      RTE = false;
      break;
    }
  }
  }
  return 0;
}

int velocity_control_function(){
  while(true){
    
    if(Lift.position(deg) > 700)
    {
      velocity_control = .4;
    }
    else if(Lift.position(deg) > 300)
    {
      velocity_control = .6;
    }
    else{
      velocity_control = 1;
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
    Brain.Screen.print("Lift");
    Brain.Screen.setCursor(9,1);
    Brain.Screen.print(Lift.position(rotationUnits::deg)); 
    Brain.Screen.setCursor(10,1);
    Brain.Screen.print(Arm.position(rotationUnits::deg));
    Brain.Screen.setCursor(11,1);
    Brain.Screen.print(velocity_control);

    wait(25, msec);
  } 
  return 0;
}

void auto_own(void){
  int speedMin = 3;
  int speedMax = 7;
  int turnSpeedMin = 2;
  int turnSpeedMax = 6;
  drive_backward(105, speedMin, 9);
  wait(20, msec);  
  turn_right(86,turnSpeedMin,turnSpeedMax);
  outake_on();
  wait(300, msec);
  drive_forward(3, 5, 5);
  outake_off();
  wait(20, msec);
  drive_backward(5, 6, 6);
  wait(20, msec);
  turn_right(90,turnSpeedMin,turnSpeedMax);
  wait(20, msec);
  drive_backward(47, speedMin, speedMax);
  wait(20, msec);
  turn_right(45,turnSpeedMin,turnSpeedMax);
  wait(20, msec);
  drive_backward(64, speedMin, speedMax);   
  wait(20, msec);
  Arm_Move();
  wait(40, msec);
  drive_forward(25,4,7);
  wait(20, msec);
  turn_left(90,5,7);
  wait(40, msec);
  Arm_Move_back();
  wait(20, msec);
  drive_backward(40, 3, 7);
  turn_left(47,turnSpeedMin,turnSpeedMax);
  Arm.setBrake(hold);
  Arm.spinToPosition(90, deg, false);
  drive_backward(45,5,7);
  wait(20, msec);
  drive_forward(3,4,7);
}


void auto_opposite(void){
  int speedMin = 2;
  int speedMax = 7;
  int turnSpeedMin = 2;
  int turnSpeedMax = 6;
  // driving to the goal
  drive_backward(101, 4, 7);//was 103
  wait(20, msec);
  turn_left(86, 3, turnSpeedMax);
  wait(20, msec);
  //spitting out triball
  outake_on();
  wait(20,msec); 
  //getting first triball
  drive_backward(35, 3, speedMax);
  outake_off();
  wait(20, msec);
  turn_right(143, 3, turnSpeedMax);
  wait(20, msec);
  intake_on();
  drive_forward(10,5,speedMax);
  wait(40, msec);
  //getting second triball

  drive_backward(10,3,speedMax);
  wait(20, msec);
  turn_left(155, 3, turnSpeedMax);
  wait(20, msec);
  outake_on(70);
  wait(300, msec); 
  //getting third triball
  turn_left(120, turnSpeedMin, turnSpeedMax);
  wait(20, msec);
  intake_on();
  drive_forward(20,5,speedMax);
  wait(20, msec);
  drive_backward(13,5,speedMax);
  wait(20, msec); 
  turn_right(115, turnSpeedMin, turnSpeedMax);
  outake_on(70);
  wait(300, msec);
  //scoring
  turn_left(110,turnSpeedMin,turnSpeedMax);
  wait(20, msec); 
  outake_off();
  drive_forward(10,speedMin,speedMax);
  wait(20, msec); 
  turn_left(48,speedMin,speedMax); 
  wait(20, msec);
  event_Wings();
  wait(20, msec);
  drive_backward(75,7,9);
}

void skills()
{
  parking();
  //event_Catapult();
  wait(3000, msec);
  //event_Catapult();
  vex::task CataS(catastop);
  wait(1000, msec);
  event_RightWing();
  wait(20, msec);
  drive_forward(3,4,5);
  wait(20, msec);
  turn_right(50,4,7);
  wait(20, msec);
  drive_forward(45,5,7);
  wait(20, msec);
  turn_left(33,3,7);
  wait(20, msec);
  drive_forward(200,5,7);
  wait(20, msec);
  turn_right(50,4,7);
  event_LeftWing();
  wait(20, msec);
  drive_backward(80,5,7);
  wait(20, msec);
  event_LeftWing();
  wait(20, msec);
  drive_backward(20,5,7);//was 35
  wait(20, msec);
  turn_right(100,4,7);
  wait(20, msec);
  event_Wings();
  wait(20, msec);
  push(1500);
  wait(20, msec);
  event_Wings();
  wait(20, msec);
  drive_forward(70,5,7);
  wait(20, msec);
  turn_right(90,4,7);
  wait(20, msec);
  drive_forward(65,5,7);//was 70
  wait(20, msec);
  turn_left(45,3,7);
  wait(20, msec);
  event_Wings();
  push(1500);
  /*
  /drive_forward(5,5,7);
  wait(20, msec);
  turn_right(15,4,7);
  wait(20, msec);
  drive_forward(50,5,7);
  */
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
  skills();
  //opposite doesn't have arm
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
  vex::task CataS(catastop);
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
  // deactivate ratchet
  ratchet.set(1);

  // reset variables and encoders
  Lift.resetPosition();
  Arm.resetPosition();

  ShootButtonPressed = false;
  WingButtonPressed = false;
  RTE = false;

  // run tasks 
  vex::task MyTask(ShowMeInfo);
  //vex::task CataS(catastop);
  vex::task LiftStopTask(limit_switch_lift);

  // assign buttons
  Controller1.ButtonY.pressed(parking);
  Controller1.ButtonX.pressed(elevate);
  Controller1.ButtonL1.pressed(event_Catapult);
  Controller1.ButtonL2.pressed(event_Wings);
  Controller1.ButtonR2.pressed(event_Outake);
  Controller1.ButtonR1.pressed(event_Intake);
  Controller1.ButtonB.pressed(event_Arm);
  Controller1.ButtonRight.pressed(event_RightWing);
  Controller1.ButtonDown.pressed(event_liftdown);
  Controller1.ButtonUp.pressed(event_liftup);


  // init auton pid
  initPID(drivePID, 0.02, 0.001, 0.01, 40, 2, 10);
  initPID(turnPID, 0.08, 0.009, 0.05, 5, 2, 10);
  initPID(straightPID, 0.15, 0, 0.4, 1, -3, 3);


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
// change PID motor start to single motor (test in robotics lab)