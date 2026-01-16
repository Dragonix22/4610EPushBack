
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       4610E                                                     */
/*    Created:      9/21/2025, 10:24:47 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <string>
#include <algorithm>

using namespace vex;
using std::string;
// A global instance of competition
competition Competition;
vex::brain Brain;

// define your global instances of motors and other devices here
motor leftFrontTop = motor(PORT1,ratio36_1, false);
motor leftFrontBottom = motor(PORT5,ratio36_1, true);
motor leftBack = motor(PORT11,ratio36_1, true);
motor rightFrontTop = motor(PORT6,ratio36_1, true);
motor rightFrontBottom = motor(PORT10,ratio36_1, false);
motor rightBack = motor(PORT17,ratio36_1, false);
motor intakeStage1 = motor(PORT2,ratio36_1, true);
motor intakeStage2 = motor(PORT4,ratio36_1, true);
digital_out wing = digital_out(Brain.ThreeWirePort.A);
digital_out adjust = digital_out(Brain.ThreeWirePort.B);
digital_out tongue = digital_out(Brain.ThreeWirePort.C);
bool wingState = false;
bool adjustState = false;
bool tongueState = false;
inertial inert = inertial(PORT21);
controller controller1 = controller();
bool s1IntakeOn = false;
bool s2IntakeOn=false;
bumper aligner = bumper(Brain.ThreeWirePort.H);
double k_p_drive = 0.15;
double k_p_turn = 0.15;


int autonPage=0;
bool autonStarted = false;
enum AutonID {
    BLUE_LEFT,
    RED_LEFT,
    BLUE_RIGHT,
    RED_RIGHT,
    SKILLS,
    TWO_INCH,
    DEBUG,
    PARK,
    SOLO_AWP
};

int currAuton = BLUE_LEFT;


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
    return(inches * 8.125);
    //const double PI = 3.14159265358979323;
    //const double wheelDiamater = 3;
    //return 0.75 * ((inches*360.0)/(PI*wheelDiamater));//0.75 gear ratio 36:48 
}

void driveForwardProp(double distance,double minSpeed=40, double maxSpeed=40){
    leftFrontTop.resetPosition();
    leftFrontBottom.resetPosition();
    leftBack.resetPosition();
    rightFrontTop.resetPosition();
    rightFrontBottom.resetPosition();
    rightBack.resetPosition();
    double target = inchesToDegrees(distance);
    double k_p = k_p_drive;
    double position = (leftFrontTop.position(deg) + leftFrontBottom.position(deg) + leftBack.position(deg) + rightFrontTop.position(deg) + rightFrontBottom.position(deg) + rightBack.position(deg)) / 6.0;
    double error = target - position;
    printf("drive forward: %f\n", target);
    while(fabs(error)>8){
        position = (leftFrontTop.position(deg) + leftFrontBottom.position(deg) + leftBack.position(deg) + rightFrontTop.position(deg) + rightFrontBottom.position(deg) + rightBack.position(deg)) / 6.0;
        error = target - position;

        double speed = k_p * error;

        if (speed > 0) speed = std::max(speed, minSpeed);
        else if (speed < 0) speed = std::min(speed, -minSpeed);

        speed = std::min(std::max(speed, -maxSpeed), maxSpeed);

        printf("Error: %f\n", error);
        printf("Position: %f\n", position); 

        leftFrontTop.spin(forward,speed,percent);
        leftFrontBottom.spin(forward,speed,percent);
        leftBack.spin(forward,speed,percent);
        rightFrontTop.spin(forward,speed,percent);
        rightFrontBottom.spin(forward,speed,percent);
        rightBack.spin(forward,speed,percent);

        wait(10,msec);
    }
    
    leftFrontTop.stop(hold);
    leftFrontBottom.stop(hold);
    leftBack.stop(hold);
    rightFrontTop.stop(hold);
    rightFrontBottom.stop(hold);
    rightBack.stop(hold);
    
}

void driveReverseProp(double distance,double minSpeed=40, double maxSpeed=40){
    leftFrontTop.resetPosition();
    leftFrontBottom.resetPosition();
    leftBack.resetPosition();
    double target = inchesToDegrees(distance);
    double k_p = k_p_drive;

    double position = leftFrontTop.position(degrees);
    double error = target + position;
    printf("drive reverse: %f\n", target);
    while(fabs(error)>8){
        position = leftFrontTop.position(degrees);
        error = target - position;

        double speed = k_p * error;
        //speed = std::min(std::max(speed, -100.0), 100.0);

        if (speed > 0) speed = std::max(speed, minSpeed);
        else if (speed < 0) speed = std::min(speed, -minSpeed);

        speed = std::min(std::max(speed, -maxSpeed), maxSpeed);

        printf("Error: %f\n", error);
        printf("Position: %f\n", position); 
        leftFrontTop.spin(reverse,speed,percent);
        leftFrontBottom.spin(reverse,speed,percent);
        leftBack.spin(reverse,speed,percent);
        rightFrontTop.spin(reverse,speed,percent);
        rightFrontBottom.spin(reverse,speed,percent);
        rightBack.spin(reverse,speed,percent);
        wait(10, msec);

    }
    leftFrontTop.stop(hold);
    leftFrontBottom.stop(hold);
    leftBack.stop(hold);
    rightFrontTop.stop(hold);
    rightFrontBottom.stop(hold);
    rightBack.stop(hold);
}


void turnLeftProp(double degreesTarget, double minSpeed=5, double maxSpeed=50) {
    inert.resetRotation();
    double k_p = k_p_turn;

    double current = inert.rotation(degrees);
    double error = degreesTarget - current;
    printf("turn left: %f\n", degreesTarget);
    while (fabs(error)>10) {
        current = inert.rotation(degrees);
        error = degreesTarget + current;

        double speed = k_p * error;
        
        if (speed > 0) speed = std::max(speed, minSpeed);
        else if (speed < 0) speed = std::min(speed, -minSpeed);

        speed = std::min(std::max(speed, -maxSpeed), maxSpeed);
        printf("Error: %f\n", error);

        leftFrontTop.spin(reverse, speed, pct);
        leftFrontBottom.spin(reverse, speed, pct);
        leftBack.spin(reverse, speed, pct);
        rightFrontTop.spin(forward, speed, pct);
        rightFrontBottom.spin(forward, speed, pct);
        rightBack.spin(forward, speed, pct);

        wait(10, msec);
    }

    leftFrontTop.stop(hold);
    leftFrontBottom.stop(hold);
    leftBack.stop(hold);
    rightFrontTop.stop(hold);
    rightFrontBottom.stop(hold);
    rightBack.stop(hold);
}

void turnRightProp(double degreesTarget, double minSpeed=5, double maxSpeed=50) {
    inert.resetRotation();
    double k_p = k_p_turn;

    double current = inert.rotation(degrees);
    double error = degreesTarget - current; 
    printf("turn right: %f\n", degreesTarget);
    while (fabs(error)>10) {
        Brain.Screen.printAt(30,30,"Err: %f", error);
        current = inert.rotation(degrees);
        error = degreesTarget - current; 

        double speed = k_p * error+1;

        if (speed > 0) speed = std::max(speed, minSpeed);
        else if (speed < 0) speed = std::min(speed, -minSpeed);        
        speed = std::min(std::max(speed, -maxSpeed), maxSpeed);


        printf("Error: %f\n", error);


        leftFrontTop.spin(forward, speed, percent);
        leftFrontBottom.spin(forward, speed, percent);
        leftBack.spin(forward, speed, percent);
        rightFrontTop.spin(reverse, speed, percent);
        rightFrontBottom.spin(reverse, speed, percent);
        rightBack.spin(reverse, speed, percent);
        wait(10,msec);
    }

    leftFrontTop.stop(hold);
    leftFrontBottom.stop(hold);
    leftBack.stop(hold);
    rightFrontTop.stop(hold);
    rightFrontBottom.stop(hold);
    rightBack.stop(hold);

}

//each tile is 24 inches
//diagonal of tile is sqrt(24^2 + 24^2) = 33.94 inches
//diagonal of two by 1 tile is sqrt(48^2 + 24^2) = 53.67 inches
//robot db length is 15 inches

//simple skills auto

void skillsAuton(){
    tongue.set(false);//idk if flipped or not
    adjust.set(true);
    //intakeStage1.spin(forward);
    driveForwardProp(40,30);
    turnRightProp(90);

    tongue.set(true);
    intakeStage1.spin(forward);
    driveForwardProp(2);
    wait(2,sec);

    driveReverseProp(4);
    adjust.set(true);
    intakeStage2.spin(forward);
    wait(2,sec);
    adjust.set(false);
    tongue.set(false);
    intakeStage2.stop(hold);

    driveForwardProp(4);
    turnRightProp(90); 

    driveForwardProp(40,30);
    turnLeftProp(90);
    tongue.set(true);
    intakeStage1.spin(forward);
    driveForwardProp(2);
    wait(2,sec);

    driveReverseProp(4);
    adjust.set(true);
    intakeStage2.spin(forward);
    wait(2,sec);
    adjust.set(false);
    tongue.set(false);
    intakeStage2.stop(hold);

    //drive to other side of field and either repeat or empty both loaders
}


void soloAWP(){
    //lock in for this
}


void parkAuton(){
    tongue.set(false);
    intakeStage1.spin(forward);
    driveForwardProp(7,30);
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
    redLeft();
}
void redRight(){
    tongue.set(false);
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
    redRight();
}

void debug(){
    driveForwardProp(2);
}

void pre_auton(void) {

    tongue.set(true);
    wing.set(true);
    adjust.set(true);

    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono30);

    // Inertial calibration
    inert.calibrate();
    Brain.Screen.printAt(5, 30, "Calibrating inertial...");
    while (inert.isCalibrating()) wait(100, msec);
    Brain.Screen.clearScreen();

    struct Button { int x, y, w, h; };

    Button pageButton = {10, 10, 120, 60};
    autonPage = 0;

    // ---------- MATCH AUTONS ----------
    const int NUM_MATCH = 4;
    string matchNames[NUM_MATCH] = {
        "Blue L", "Red L", "Blue R", "Red R"
    };

    AutonID matchIDs[NUM_MATCH] = {
        BLUE_LEFT, RED_LEFT, BLUE_RIGHT, RED_RIGHT
    };

    Button matchButtons[NUM_MATCH] = {
        {270, 10, 80, 80},
        {355, 10, 80, 80},
        {270, 95, 80, 80},
        {355, 95, 80, 80}
    };

    // ---------- SKILLS / UTIL ----------
    const int NUM_SKILLS = 4;
    string skillNames[NUM_SKILLS] = {
        "Skills", "Debug", "Solo AWP", "Park"
    };

    AutonID skillIDs[NUM_SKILLS] = {
        SKILLS, DEBUG, SOLO_AWP, PARK
    };

    Button skillButtons[NUM_SKILLS] = {
        {270, 10, 80, 80},
        {355, 10, 80, 80},
        {270, 95, 80, 80},
        {355, 95, 80, 80}
    };

    bool selected = false;

    while (!selected) {
        Brain.Screen.clearScreen();

        // ----- PAGE BUTTON -----
        Brain.Screen.setFillColor({200, 200, 200});
        Brain.Screen.drawRectangle(
            pageButton.x, pageButton.y,
            pageButton.w, pageButton.h
        );

        Brain.Screen.setFillColor(black);
        Brain.Screen.printAt(
            pageButton.x + 15,
            pageButton.y + 40,
            autonPage == 0 ? "MATCH" : "SKILLS"
        );

        // ----- DRAW BUTTONS -----
        if (autonPage == 0) {
            for (int i = 0; i < NUM_MATCH; i++) {
                Brain.Screen.setFillColor((i == 0 || i == 2) ? blue : red);
                Brain.Screen.drawRectangle(
                    matchButtons[i].x, matchButtons[i].y,
                    matchButtons[i].w, matchButtons[i].h
                );
                Brain.Screen.setFillColor(black);
                Brain.Screen.printAt(
                    matchButtons[i].x + 5,
                    matchButtons[i].y + 50,
                    matchNames[i].c_str()
                );
            }
        } else {
            for (int i = 0; i < NUM_SKILLS; i++) {
                Brain.Screen.setFillColor(green);
                Brain.Screen.drawRectangle(
                    skillButtons[i].x, skillButtons[i].y,
                    skillButtons[i].w, skillButtons[i].h
                );
                Brain.Screen.setFillColor(black);
                Brain.Screen.printAt(
                    skillButtons[i].x + 5,
                    skillButtons[i].y + 50,
                    skillNames[i].c_str()
                );
            }
        }

        // ----- TOUCH -----
        if (Brain.Screen.pressing()) {
            while (Brain.Screen.pressing()) wait(10, msec);
            int mx = Brain.Screen.xPosition();
            int my = Brain.Screen.yPosition();

            // Page toggle
            if (mx >= pageButton.x && mx <= pageButton.x + pageButton.w &&
                my >= pageButton.y && my <= pageButton.y + pageButton.h) {
                autonPage = !autonPage;
                continue;
            }

            // Match selection
            if (autonPage == 0) {
                for (int i = 0; i < NUM_MATCH; i++) {
                    if (mx >= matchButtons[i].x &&
                        mx <= matchButtons[i].x + matchButtons[i].w &&
                        my >= matchButtons[i].y &&
                        my <= matchButtons[i].y + matchButtons[i].h) {
                        currAuton = matchIDs[i];
                        selected = true;
                    }
                }
            }
            // Skills selection
            else {
                for (int i = 0; i < NUM_SKILLS; i++) {
                    if (mx >= skillButtons[i].x &&
                        mx <= skillButtons[i].x + skillButtons[i].w &&
                        my >= skillButtons[i].y &&
                        my <= skillButtons[i].y + skillButtons[i].h) {
                        currAuton = skillIDs[i];
                        selected = true;
                    }
                }
            }
        }

        wait(50, msec);
    }

    // ----- CONFIRM -----
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono60);
    Brain.Screen.printAt(40, 120, "Auton Selected");
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
//override auton to always run 1 auton for debug purposes
/*
void autonomous(void) {
    skillsAuton();
}


*/

void autonomous(void) {
    switch(currAuton) {
        case SKILLS: skillsAuton(); break;
        case DEBUG: debug(); break;
        case SOLO_AWP: soloAWP(); break;
        case PARK: parkAuton(); break;
        
        default: break;
    }
    
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
        wing.set(wingState);
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

void tongueManager() {
    bool tongueState = false;  
    while (true) {

        if (controller1.ButtonB.pressing()) {

            while (controller1.ButtonB.pressing()) {
                wait(5, msec);
            }

            tongueState = !tongueState;   
            tongue.set(tongueState);    
        }

        wait(10, msec);
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

        wait(10,msec);
    }
}


double logDrive(double raw) {
    const double deadzone = 10.0;
    const double maxInput = 127.0;
    const double curve = 5.0; 
    if (fabs(raw) < deadzone) return 0.0;
    double sign = (raw > 0) ? 1.0 : -1.0;
    double x = (fabs(raw) - deadzone) / (maxInput - deadzone);
    double scaled = log10(1.0 + curve * x) / log10(1.0 + curve);
    return sign * scaled * maxInput;
}
//helper functionto clamp values for drive code
double clamp(double v, double minV, double maxV) {
    return fmin(fmax(v, minV), maxV);
}


void driveManager(){
    while(1) {
        int raw3 = controller1.Axis3.position();
        int raw1 = controller1.Axis1.position(); 
        double axis3 = logDrive(raw3);
        double axis1 = logDrive(raw1);

        double turnBoost=1.5;
        axis1*=turnBoost;

        double left = clamp(axis3 + axis1, -127, 127);
        double right = clamp(axis3 - axis1, -127, 127);

        leftBack.spin(forward, left, pct);
        leftFrontBottom.spin(forward, left, pct);
        leftFrontTop.spin(forward, left, pct);
        rightBack.spin(forward, right, pct);
        rightFrontBottom.spin(forward, right, pct);
        rightFrontTop.spin(forward, right, pct);

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

void alignerManager(){
    controller1.rumble("._."); //._.
}

void usercontrol(void) {
    //Brain.Screen.printAt(5,30,"Driving");
    //thread p(motorDegreeManagers);
    thread i(intakeManager);
    thread w(wingMananger);
    thread a(adjustMananger);
    thread d(driveManager);
    thread t(tongueManager);
    wing.set(false);
    adjust.set(false);
    aligner.pressed(alignerManager);

    // User control code here, inside the loop
    //Brain.Screen.printAt(10,50,"not aadityas password: 264859");
    //Brain.Screen.printAt( 10, 50, "Aaditya's Password: 264 859" );
    //while(1){}
  

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
