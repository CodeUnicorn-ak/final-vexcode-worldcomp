/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Main controller
controller Controller1 = controller(primary);

// Base 4 motor - standard drive
motor LB = motor(PORT6, ratio18_1, true);
motor RB = motor(PORT4, ratio18_1, false);
motor LF = motor(PORT2, ratio18_1, false);
motor RF = motor(PORT3, ratio18_1, true);

// Intake Motors
motor Rintake = motor(PORT17, ratio18_1, true);
motor Lintake = motor(PORT18, ratio18_1, false);

// Roller Motors
motor BRoller = motor(PORT5, ratio18_1, false);
motor FRoller = motor(PORT12, ratio18_1, false);

// Create motor group so that we can create Drive train
// Left hand side two motors
vex::motor_group leftDrive(LF, LB);

// Right hand side two motors
vex::motor_group rightDrive(RF, RB);

// Define Drivetrain using 2 motor groups
drivetrain Drivetrain =
    drivetrain(leftDrive, rightDrive, 319.19, 270, 25, mm, 1);

// Controller Deadzone
void conDeadZone() {
  float leftJoystick = Controller1.Axis3.position(pct);
  float rightJoystick = Controller1.Axis2.position(pct);
  float Joystick =
      (Controller1.Axis3.position(pct) + Controller1.Axis2.position(pct)) / 2;
  if (fabs(leftJoystick) < 5) {
    leftJoystick = 0;
  }
  if (fabs(rightJoystick) < 5) {
    rightJoystick = 0;
  }
  // Self Correcting Drive
  if (fabs(rightJoystick - leftJoystick) < 45) {
    rightJoystick = Joystick;
    leftJoystick = Joystick;
  }

  // Controller Command
  leftDrive.setVelocity(leftJoystick, pct);
  rightDrive.setVelocity(rightJoystick, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
}
//
void stopBase() {
  leftDrive.stop(brakeType::coast);
  rightDrive.stop(brakeType::coast);
  leftDrive.stop(brakeType::coast);
  rightDrive.stop(brakeType::coast);
}
void drive(int velo, int distance) {
  Drivetrain.setDriveVelocity(velo, percent);
  Drivetrain.driveFor(distance, inches);
}
void turn(int velo, int degree) {
  Drivetrain.setTurnVelocity(velo, percent);
  Drivetrain.turnFor(degree, degrees);
}
void velo(int velo) {
  FRoller.setVelocity(velo, pct);
  BRoller.setVelocity(velo, pct);
  Lintake.setVelocity(velo, pct);
  Rintake.setVelocity(velo, pct);
}
void sleep(int time) { vex::task::sleep(time); }
 void temp(){
 
  Brain.Screen.printAt(2, 40 ,"Bottom Roller %f " , FRoller.temperature(percentUnits::pct));
   Brain.Screen.printAt(2, 60 ,"Top Roller %f " , BRoller.temperature(percentUnits::pct));
   Brain.Screen.printAt(2, 80 ,"LB  %f " , LB.temperature(percentUnits::pct));
   Brain.Screen.printAt(2, 100 ,"LF  %f " , LF.temperature(percentUnits::pct));
   Brain.Screen.printAt(2, 120 ,"RF  %f " , RF.temperature(percentUnits::pct));
   Brain.Screen.printAt(2, 140 ,"RB  %f " , RB.temperature(percentUnits::pct));
   Brain.Screen.printAt(2, 160 ,"Rintake %f " , Rintake.temperature(percentUnits::pct));
   Brain.Screen.printAt(2, 180 ,"Lintake %f " , Lintake.temperature(percentUnits::pct));
   
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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // reset postitons of motors
  leftDrive.resetPosition();
  leftDrive.resetRotation();
  rightDrive.resetPosition();
  rightDrive.resetRotation();
  Rintake.resetPosition();
  Lintake.resetRotation();
  Rintake.resetPosition();
  Lintake.resetRotation();
  BRoller.resetPosition();
  FRoller.resetRotation();
  BRoller.resetPosition();
  FRoller.resetRotation();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // flip-out
  drive(30, 3);
  // drive(10, -7);
  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  Lintake.setVelocity(70, pct);
  Rintake.setVelocity(70, pct);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  wait(500, msec);
  // drive fwd
  drive(25, 11);

  Drivetrain.stop();

  Drivetrain.setTurnVelocity(20, pct);
  Drivetrain.turnFor(-84, degrees);

  Drivetrain.stop();

  drive(40, 20);
  // Drivetrain.turnFor(90, degrees);

  // drive(40, 10);

  Lintake.setVelocity(100, pct);
  Rintake.setVelocity(100, pct);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  // turn left 90
  Drivetrain.setTurnVelocity(10, percent);
  Drivetrain.turnFor(-36, degrees);

  Lintake.stop();
  Rintake.stop();

  rightDrive.setVelocity(50, pct);
  leftDrive.setVelocity(50, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(2000, msec);
  rightDrive.stop();
  leftDrive.stop();

  // put in ball
  FRoller.setVelocity(100, pct);
  BRoller.setVelocity(100, pct);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  // drive(50, 3);
  wait(1000, msec);
  BRoller.stop();
  FRoller.stop();

  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(2300, msec);
  rightDrive.stop();
  leftDrive.stop();

  Drivetrain.setTurnVelocity(20, percent);
  Drivetrain.turnFor(125, degrees);

  BRoller.stop();
  FRoller.stop();
  // turn back
  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(2550, msec);
  rightDrive.stop();
  leftDrive.stop();

  // moving ball up
  FRoller.setVelocity(55, pct);
  BRoller.setVelocity(55, pct);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  Lintake.setVelocity(100, pct);
  Rintake.setVelocity(100, pct);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  wait(400, msec);
  BRoller.stop();
  FRoller.stop();

  // moving to next ball
  Lintake.setVelocity(70, pct);
  Rintake.setVelocity(70, pct);
  drive(25, 55);
  drive(20, -10);

  Drivetrain.setTurnVelocity(20, percent);
  Drivetrain.turnFor(-88, degrees);

  Lintake.stop();
  Rintake.stop();

  // move towards tower
  rightDrive.setVelocity(30, pct);
  leftDrive.setVelocity(30, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(2000, msec);
  rightDrive.stop();
  leftDrive.stop();

  FRoller.setVelocity(100, pct);
  BRoller.setVelocity(100, pct);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  Lintake.setVelocity(5, pct);
  Rintake.setVelocity(5, pct);
  Lintake.spin(directionType::rev);
  Rintake.spin(directionType::rev);
  wait(800, msec);
  BRoller.stop();
  FRoller.stop();
  drive(20, -7);
  Lintake.stop();
  Rintake.stop();

  FRoller.setVelocity(100, pct);
  BRoller.setVelocity(100, pct);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  Lintake.setVelocity(100, pct);
  Rintake.setVelocity(100, pct);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  wait(300, msec);
  BRoller.stop();
  FRoller.stop();
  Lintake.stop();
  Rintake.stop();

  drive(20, 7);

  FRoller.setVelocity(100, pct);
  BRoller.setVelocity(100, pct);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  wait(2000, msec);
  BRoller.stop();
  FRoller.stop();

  drive(20, -16);
  Drivetrain.setTurnVelocity(20, pct);
  Drivetrain.turnFor(85, degrees);

  Drivetrain.stop();

  Lintake.setVelocity(75, pct);
  Rintake.setVelocity(75, pct);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  wait(500, msec);

  // drive fwd
  drive(20, 40);
  Lintake.stop();
  Rintake.stop();
  rightDrive.setVelocity(30, pct);
  leftDrive.setVelocity(30, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  Lintake.setVelocity(75, pct);
  Rintake.setVelocity(75, pct);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  drive(30, -17);
  Drivetrain.setTurnVelocity(25, pct);
  Drivetrain.turnFor(-40, degrees);

  FRoller.setVelocity(45, pct);
  BRoller.setVelocity(45, pct);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  Lintake.setVelocity(100, pct);
  Rintake.setVelocity(100, pct);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  wait(400, msec);
  BRoller.stop();
  FRoller.stop();
 
Lintake.stop();
Rintake.stop();

  rightDrive.setVelocity(25, pct);
  leftDrive.setVelocity(25, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(2300, msec);
  rightDrive.stop();
  leftDrive.stop();

  FRoller.setVelocity(100, pct);
  BRoller.setVelocity(100, pct);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  // drive(50, 3);
  wait(1500, msec);
  BRoller.stop();
  FRoller.stop();

  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(2300, msec);
  rightDrive.stop();
  leftDrive.stop();


//bop bop
  Drivetrain.setTurnVelocity(70, pct);
  Drivetrain.turnFor(142, degrees);

Lintake.setVelocity(100, pct);
  Rintake.setVelocity(100, pct);
  Lintake.spin(directionType::rev);
  Rintake.spin(directionType::rev);

  rightDrive.setVelocity(100, pct);
  leftDrive.setVelocity(100, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  rightDrive.setVelocity(100, pct);
  leftDrive.setVelocity(100, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  rightDrive.setVelocity(100, pct);
  leftDrive.setVelocity(100, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  rightDrive.setVelocity(100, pct);
  leftDrive.setVelocity(100, pct);
  rightDrive.spin(directionType::fwd);
  leftDrive.spin(directionType::fwd);
  wait(1500, msec);
  rightDrive.stop();
  leftDrive.stop();

  rightDrive.setVelocity(15, pct);
  leftDrive.setVelocity(15, pct);
  rightDrive.spin(directionType::rev);
  leftDrive.spin(directionType::rev);
  wait(3000, msec);
  rightDrive.stop();
  leftDrive.stop();
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
    //bool freeze2 = false;
  while (1) {
    conDeadZone();
     temp();


    // Controller controls
    while (Controller1.ButtonY.pressing()) {
      //freeze2 = true;
      Lintake.setVelocity(100, pct);
      Rintake.setVelocity(100, pct);
      Lintake.spin(directionType::fwd);
      Rintake.spin(directionType::fwd);
    }
    while (Controller1.ButtonR1.pressing()) {
      //freeze2 = true;
      Lintake.setVelocity(100, pct);
      Rintake.setVelocity(100, pct);
      FRoller.setVelocity(25, pct);

      Lintake.spin(directionType::fwd);
      Rintake.spin(directionType::fwd);
      FRoller.spin(directionType::fwd);
    
    }

   while (Controller1.ButtonUp.pressing()) {
      //freeze2 = true;
      Lintake.setVelocity(50, pct);
      Rintake.setVelocity(50, pct);
  
      Lintake.spin(directionType::rev);
      Rintake.spin(directionType::rev);
    
    }
   
   while (Controller1.ButtonR2.pressing()) {
      //freeze2 = true;
      Lintake.setVelocity(100, pct);
      Rintake.setVelocity(100, pct);
      FRoller.setVelocity(100, pct);
      BRoller.setVelocity(100, pct);
      Lintake.spin(directionType::fwd);
      Rintake.spin(directionType::fwd);
      FRoller.spin(directionType::fwd);
      BRoller.spin(directionType::fwd);
    }
     while (Controller1.ButtonL1.pressing()) {
      //freeze = true;
      FRoller.setVelocity(100, pct);
      BRoller.setVelocity(100, pct);
      FRoller.spin(directionType::fwd);
      BRoller.spin(directionType::fwd);
    }
    while (Controller1.ButtonDown.pressing()) {
      //freeze = true;
      FRoller.setVelocity(65, pct);
      BRoller.setVelocity(65, pct);
      FRoller.spin(directionType::fwd);
      BRoller.spin(directionType::fwd);
    }

    while (Controller1.ButtonL2.pressing()) {
     // freeze2 = true;
      Lintake.setVelocity(100, pct);
      Rintake.setVelocity(100, pct);
      FRoller.setVelocity(100, pct);
      BRoller.setVelocity(100, pct);
      Lintake.spin(directionType::fwd);
      Rintake.spin(directionType::fwd);
      FRoller.spin(directionType::fwd);
      BRoller.spin(directionType::rev);
    }

   while (Controller1.ButtonX.pressing()) {
      //freeze2 = true;
      Lintake.setVelocity(25, pct);
      Rintake.setVelocity(25, pct);
      FRoller.setVelocity(25, pct);
      BRoller.setVelocity(25, pct);
      Lintake.spin(directionType::rev);
      Rintake.spin(directionType::rev);
      FRoller.spin(directionType::rev);
      BRoller.spin(directionType::rev);
    }
       while (Controller1.ButtonA.pressing()) {
       //freeze2 = true;
      Lintake.setVelocity(25, pct);
      Rintake.setVelocity(25, pct);
      FRoller.setVelocity(25, pct);
      BRoller.setVelocity(25, pct);
      Lintake.spin(directionType::rev);
      Rintake.spin(directionType::rev);
    }
     while(Controller1.ButtonR1.pressing()){
       //freeze = false;
      Lintake.setVelocity(100, pct);
      Rintake.setVelocity(100, pct);
      FRoller.setVelocity(25, pct);
    
      Lintake.spin(directionType::fwd);
      Rintake.spin(directionType::fwd);
      FRoller.spin(directionType::fwd);
    }

    while(Controller1.ButtonB.pressing()){
       //freeze = false;
      Lintake.stop(brakeType::hold);
      Rintake.stop(brakeType::hold);
      BRoller.stop(brakeType::hold);
      FRoller.stop(brakeType::hold);
    }

     
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
