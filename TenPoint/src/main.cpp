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
  if (fabs(rightJoystick - leftJoystick) < 40) {
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
  leftDrive.stop(brakeType::hold);
  rightDrive.stop(brakeType::hold);
  leftDrive.stop(brakeType::hold);
  rightDrive.stop(brakeType::hold);
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
void sleep(int time){
vex::task::sleep(time);

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
  //flip-out and intake
  velo(100);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  //move forward to intake 1st ball plus preload
  drive(50, 40);
  Rintake.stop(brakeType::hold);
  Lintake.stop(brakeType::hold);
  BRoller.stop(brakeType::hold);
  FRoller.stop(brakeType::hold);
  /*
  //end of stage 1
  //turn left 90
  turn(25, -90);
  //move forward
  drive(50, 12);
  //turn left 45
  turn(25, -45);
  //move forward towards tower
  drive(30, 40);
//put ball in
  velo(100);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  wait(3000, msec);
  Rintake.stop(brakeType::hold);
  Lintake.stop(brakeType::hold);
  BRoller.stop(brakeType::hold);
  FRoller.stop(brakeType::hold);
  //move backward to get into position 
  drive(45, -20);
  //turn towards wall
  turn(25, 225);
  //move backward towards wall and realign
  drive(25, -30);
  wait(1000, msec);
  //move towards next balls
  drive(50, 52);
  //turn towards 2 new balls
  turn(25, 90);
  //drive forward while intaking
  velo(50);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  drive(50,54);
  //turn left
  turn(25,-90);
  //move towards tower and put balls in
  drive(35, 45);
  wait(2000, msec);
  Rintake.stop(brakeType::hold);
  Lintake.stop(brakeType::hold);
  BRoller.stop(brakeType::hold);
  FRoller.stop(brakeType::hold);
  //drive backwards
  drive(25, -12);
  //turn right towards tower
  turn(25, 90);
  //move forward to tower
  drive(50, 50);
  //45 degree turn left
  turn(25, -45);
  //move forward and put ball in
  velo(100);
  Lintake.spin(directionType::fwd);
  Rintake.spin(directionType::fwd);
  BRoller.spin(directionType::fwd);
  FRoller.spin(directionType::fwd);
  drive(30,15);
  wait(2000,msec);
  Rintake.stop(brakeType::hold);
  Lintake.stop(brakeType::hold);
  BRoller.stop(brakeType::hold);
  FRoller.stop(brakeType::hold);
  //drive back
  drive(30, -30);
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

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    conDeadZone();
    /* Limit switch command
    bool freeze;
*/

    // Controller controls
    while (Controller1.ButtonR1.pressing()) {
      // freeze = true;
      Lintake.setVelocity(100, pct);
      Rintake.setVelocity(100, pct);
      Lintake.spin(directionType::fwd);
      Rintake.spin(directionType::fwd);
    }

    while (Controller1.ButtonR2.pressing()) {
      // freeze = true;
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
      // freeze = false;
      FRoller.setVelocity(100, pct);
      BRoller.setVelocity(100, pct);
      FRoller.spin(directionType::fwd);
      BRoller.spin(directionType::fwd);
    }
    while (Controller1.ButtonL2.pressing()) {
      // freeze = false;
      Lintake.setVelocity(100, pct);
      Rintake.setVelocity(100, pct);
      FRoller.setVelocity(100, pct);
      BRoller.setVelocity(100, pct);
      Lintake.spin(directionType::fwd);
      Rintake.spin(directionType::fwd);
      FRoller.spin(directionType::fwd);
      BRoller.spin(directionType::rev);
    }
    while (Controller1.ButtonB.pressing()) {
      Lintake.stop(brakeType::hold);
      Rintake.stop(brakeType::hold);
      BRoller.stop(brakeType::hold);
      FRoller.stop(brakeType::hold);
    }
    while (Controller1.ButtonX.pressing()) {
      // freeze = false;
      Lintake.setVelocity(25, pct);
      Rintake.setVelocity(25, pct);
      FRoller.setVelocity(25, pct);
      BRoller.setVelocity(25, pct);
      Lintake.spin(directionType::rev);
      Rintake.spin(directionType::rev);
      FRoller.spin(directionType::rev);
      BRoller.spin(directionType::rev);
    }
    /* while (freeze * LimitSwitchA.pressing()) {
         wait(130, msec);
         BRoller.stop(brakeType::hold);
         FRoller.stop(brakeType::hold);
     }


     */
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
