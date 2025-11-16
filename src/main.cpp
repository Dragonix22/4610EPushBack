
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Student                                                   */
/*    Created:      9/21/2025, 10:24:47 AM                                    */
/*    Description:  V5 project                                                */
/*                /                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <string>
using namespace vex;
using std::string;
// A global instance of competition
competition Competition;
vex::brain Brain;

// define your global instances of motors and other devices here
motor leftFrontTop = motor(PORT1,false);
motor leftFrontBottom = motor(PORT5,true);
motor leftBack = motor(PORT11,true);
motor rightFrontTop = motor(PORT13,true);
motor rightFrontBottom = motor(PORT6,false);
motor rightBack = motor(PORT17,false);
motor intakeStage1 = motor(PORT2,true);
motor intakeStage2 = motor(PORT4,true);
digital_out wings = digital_out(Brain.ThreeWirePort.A);
digital_out adjust = digital_out(Brain.ThreeWirePort.B);
digital_out tongue = digital_out(Brain.ThreeWirePort.C);
bool wingState = false;
bool adjustState = false;
bool tongueState = false;
inertial inert = inertial(PORT7);//7 on actual
bool autonStarted = false;
int currAuton = 0;
controller controller1 = controller();
bool s1IntakeOn = false;
bool s2IntakeOn=false;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


double inchesToDegrees(double inches){
    const double PI = 3.14159265358979323;
    const double wheelDiamater = 3.25;
    return(0.25*((inches*360.0)/(PI*wheelDiamater)));
}

void driveForwardProp(double distance){
    leftFrontTop.resetPosition();
    leftFrontBottom.resetPosition();
    leftBack.resetPosition();
    distance = inchesToDegrees(distance);
    while(leftFrontTop.position(degrees) < distance){
        double speed = (distance + leftFrontTop.position(degrees))/2;
        leftFrontTop.spin(forward,speed,percent);
        leftFrontBottom.spin(forward,speed,percent);
        leftBack.spin(forward,speed,percent);
        rightFrontTop.spin(forward,speed,percent);
        rightFrontBottom.spin(forward,speed,percent);
        rightBack.spin(forward,speed,percent);
    }

    leftFrontTop.stop(hold);
    leftFrontBottom.stop(hold);
    leftBack.stop(hold);
    rightFrontTop.stop(hold);
    rightFrontBottom.stop(hold);
    rightBack.stop(hold);
}

void driveReverseProp(double distance){
    leftFrontTop.resetPosition();
    leftFrontBottom.resetPosition();
    leftBack.resetPosition();
    distance = inchesToDegrees(distance);
    while(leftFrontTop.position(degrees) > -distance){

        double speed = (distance - leftFrontTop.position(degrees))/2;
        leftFrontTop.spin(reverse,speed,percent);
        leftFrontBottom.spin(reverse,speed,percent);
        leftBack.spin(reverse,speed,percent);
        rightFrontTop.spin(reverse,speed,percent);
        rightFrontBottom.spin(reverse,speed,percent);
        rightBack.spin(reverse,speed,percent);
    }

    leftFrontTop.stop(brake);
    leftFrontBottom.stop(brake);
    leftBack.stop(brake);
    rightFrontTop.stop(brake);
    rightFrontBottom.stop(brake);
    rightBack.stop(brake);

}


void turnLeftProp(double distance){
    inert.resetRotation();
    while(inert.rotation(degrees)>=-distance+5){

        double speed = (distance + inert.rotation(degrees))/4;
        leftFrontTop.spin(reverse,speed,percent);
        leftFrontBottom.spin(reverse,speed,percent);
        leftBack.spin(reverse,speed,percent);
        rightFrontTop.spin(forward,speed,percent);
        rightFrontBottom.spin(forward,speed,percent);
        rightBack.spin(forward,speed,percent);
    }

    leftFrontTop.stop(coast);
    leftFrontBottom.stop(coast);
    leftBack.stop(coast);
    rightFrontTop.stop(coast);
    rightFrontBottom.stop(coast);
    rightBack.stop(coast);

}

void turnRightProp(double distance){
    inert.resetRotation();
    while(inert.rotation(degrees) <= distance-5){
        double speed = (distance - inert.rotation(degrees))/4;
        leftFrontTop.spin(forward,speed,percent);
        leftFrontBottom.spin(forward,speed,percent);
        leftBack.spin(forward,speed,percent);
        rightFrontTop.spin(reverse,speed,percent);
        rightFrontBottom.spin(reverse,speed,percent);
        rightBack.spin(reverse,speed,percent);
    }

    leftFrontTop.stop(brake);
    leftFrontBottom.stop(brake);
    leftBack.stop(brake);
    rightFrontTop.stop(brake);
    rightFrontBottom.stop(brake);
    rightBack.stop(brake);

}

void redLeft(){
    adjust.set(true);    
    intakeStage1.spin(forward);
    driveForwardProp(32);
    turnRightProp(60);    
    driveForwardProp(16);
    intakeStage2.spin(forward);
    wait(2,sec);
    intakeStage2.stop(hold);
    driveReverseProp(60);
    turnRightProp(135);
    driveForwardProp(20);
    tongue.set(true);
    wait(2,sec);
    tongue.set(false);
    adjust.set(true);
    driveReverseProp(40);
    intakeStage2.spin(forward);
}

void blueLeft(){
   driveForwardProp(4);
   driveReverseProp(4);


}
void redRight(){
    adjust.set(true);    
    intakeStage1.spin(forward);
    driveForwardProp(32);
    turnLeftProp(60);    
    driveForwardProp(16);
    intakeStage2.spin(forward);
    wait(2,sec);
    intakeStage2.stop(hold);
    driveReverseProp(60);
    turnLeftProp(135);
    driveForwardProp(20);
    tongue.set(true);
    wait(2,sec);
    tongue.set(false);
    adjust.set(true);
    driveReverseProp(40);
    intakeStage2.spin(forward);
}


void blueRight(){

}

void skillsAuton(){

}

void pre_auton(void) {
    // Clear screen and set font
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono60);

    // Calibrate inertial
    inert.calibrate();
    Brain.Screen.printAt(5, 30, "Calibrating inertial...");
    while(inert.isCalibrating()) wait(100, msec); // wait for calibration
    Brain.Screen.clearScreen();

    // Define auton names
    const int NUM_AUTONS = 5;
    string autonNames[NUM_AUTONS] = {
        "Blue Left",
        "Red Left",
        "Blue Right",
        "Red Right",
        "Skills Auton"
    };

    // Define button positions (x, y, width, height)
    struct Button { int x, y, w, h; };
    Button autonButtons[NUM_AUTONS] = {
        {270, 10, 80, 80},   // Blue Left
        {355, 10, 80, 80},   // Red Left
        {270, 95, 80, 80},   // Blue Right
        {355, 95, 80, 80},   // Red Right
        {270, 180, 165, 80}  // Skills Auton (bigger button)
    };

    bool selected = false;

    while(!selected){
        // Draw buttons dynamically
        for(int i=0; i<NUM_AUTONS; i++){
            if(i == 0 || i == 2) Brain.Screen.setFillColor(blue);
            else if(i == 1 || i == 3) Brain.Screen.setFillColor(red);
            else Brain.Screen.setFillColor(green); // Skills auton

            Brain.Screen.drawRectangle(autonButtons[i].x, autonButtons[i].y,
                                       autonButtons[i].w, autonButtons[i].h);

            // Draw text
            Brain.Screen.setFillColor(black);
            Brain.Screen.setFont(mono30);
            Brain.Screen.printAt(autonButtons[i].x + 10, autonButtons[i].y + 50, autonNames[i].c_str());
        }

        // Display currently selected auton
        Brain.Screen.setFillColor(black);
        Brain.Screen.setFont(mono30);
        Brain.Screen.printAt(5, 170, "SELECTED AUTON:");
        Brain.Screen.printAt(5, 210, "                     "); // clear line
        Brain.Screen.printAt(5, 210, autonNames[currAuton].c_str());

        // Check for screen touch
        if(Brain.Screen.pressing()){
            while(Brain.Screen.pressing()) wait(10,msec);

            int mx = Brain.Screen.xPosition();
            int my = Brain.Screen.yPosition();

            for(int i=0; i<NUM_AUTONS; i++){
                if(mx >= autonButtons[i].x && mx <= autonButtons[i].x + autonButtons[i].w &&
                   my >= autonButtons[i].y && my <= autonButtons[i].y + autonButtons[i].h){
                    currAuton = i;
                    selected = true;
                    autonStarted = true;
                    break;
                }
            }
        }

        wait(50,msec);
    }

    // Final confirmation
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono60);
    Brain.Screen.printAt(50, 100, "Auton Selected:");
    Brain.Screen.printAt(50, 150, autonNames[currAuton].c_str());
    wait(2, sec); // brief pause before auton starts
}


/*
void pre_auton(void) {
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(vex::fontType::mono60);
    inert.calibrate();
    Brain.Screen.printAt(5,30,"calibrating inert:");
    wait(3,sec);
    Brain.Screen.clearScreen();
}
    */
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
    blueLeft();
    /*switch(currAuton) {
        case 0: blueLeft(); break;
        case 1: redLeft(); break;
        case 2: blueRight(); break;
        case 3: redRight(); break;
        case 4: skillsAuton(); break; // NEW AUTON
        default: break;
    }
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




void wingMananger(){
    wingState = true;
    while(1){
        if(controller1.ButtonX.pressing()){
            while(controller1.ButtonX.pressing()){
                wait(2,msec);
            }
            wingState=!wingState;
        }
        wings.set(wingState);
    }
}

void adjustMananger(){
    adjustState = true;
    while(1){
        if(controller1.ButtonA.pressing()){
            while(controller1.ButtonA.pressing()){
                wait(2,msec);
            }
            adjustState=!adjustState;
        }
        adjust.set(adjustState);
    }
}

void tongueManager(){
    tongueState = true;
    while(1){
        if(controller1.ButtonB.pressing()){
            while(controller1.ButtonB.pressing()){
                wait(2,msec);
            }
            tongueState=!tongueState;
        }
        tongue.set(tongueState);
    }
} 

void intakeManager(){
    s1IntakeOn = true;
    s2IntakeOn=true;
    while(1){
        if(controller1.ButtonR1.pressing()){
            while(controller1.ButtonR1.pressing()){
                wait(10, msec);
            }
            s1IntakeOn=!s1IntakeOn;
            
        }

        if(!s1IntakeOn){
            intakeStage1.spin(forward,100,pct);
        }else if(controller1.ButtonR2.pressing()){
            intakeStage1.spin(reverse, 100, pct);
        }else{
            intakeStage1.stop();
        } 

        if(controller1.ButtonL1.pressing()){
            intakeStage2.spin(forward,100,pct);
        }else if(controller1.ButtonL2.pressing()){
            intakeStage2.spin(reverse, 100, pct);
        }else{
            intakeStage2.stop();
        }

    }
}

void driveManager(){
    while(1) {
        int raw3 = controller1.Axis3.position();
        int raw1 = controller1.Axis1.position();
        //ded zone
        const int deadZone=5;
        if(abs(raw3)<=deadZone) raw3=0;
        if(abs(raw1)<=deadZone) raw1=0;


        //logarithmic drive (127 signed int )
        double axis3 = 0.0;
        double axis1 = 0.0;

        if (raw3!=0) axis3=(raw3>0 ? 1.0: -1.0)*((raw3*raw3)/127.0);
        if (raw1!=0) axis1=(raw1>0 ? 1.0: -1.0)*((raw1*raw1)/127.0);

        leftBack.spin(forward, axis3 + axis1, pct);
        leftFrontBottom.spin(forward, axis3 + axis1, pct);
        leftFrontTop.spin(forward, axis3 + axis1, pct);
        rightBack.spin(forward,axis3 - axis1, pct);
        rightFrontBottom.spin(forward,axis3 - axis1, pct);
        rightFrontTop.spin(forward, axis3 - axis1, pct);
        wait(20,msec);
    }
}

void motorDegreeManagers(){
    while(1){
        Brain.Screen.printAt(5,10,"%d",leftFrontBottom.position(degrees));
        Brain.Screen.printAt(5,25,"%d",leftBack.position(degrees));
        Brain.Screen.printAt(5,40,"%d",leftFrontTop.position(degrees));
        Brain.Screen.printAt(5,55,"%d",rightBack.position(degrees));
        Brain.Screen.printAt(5,60,"%d",rightFrontBottom.position(degrees));
        Brain.Screen.printAt(5,75,"%d",rightFrontTop.position(degrees));

    }
}
void usercontrol(void) {
    //Brain.Screen.printAt(5,30,"Driving");
    thread p(motorDegreeManagers);
    thread i(intakeManager);
    thread w(wingMananger);
    thread a(adjustMananger);
    thread d(driveManager);
    wings.set(false);
    adjust.set(false);
    // User control code here, inside the loop
    //Brain.Screen.printAt(10,50,"not aadityas password: 264859");
    //Brain.Screen.printAt( 10, 50, "Aaditya's Password: 264 859" );
    //while(1){

    //}
  

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
