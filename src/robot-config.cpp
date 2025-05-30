#include "vex.h"

using namespace vex;


// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
controller Controller1;

gps GPS = gps(PORT5, -88.9, -125, mm, 270);
digital_out arm = digital_out(Brain.ThreeWirePort.B);
line Arm_Limit = line(Brain.ThreeWirePort.H);


      //// Lift Motor Layout ////
////////////////  |    ////////////////
//L1//  //L2//    |    //R2//  //R1//
//L3//  //L4//    |    //R4//  //R3//
////////////////       ////////////////

// Left Motors 
motor LiftL1 = motor(PORT4,ratio6_1,true);  // A
motor LiftL2 = motor(PORT1,ratio6_1,false); // B
motor LiftL3 = motor(PORT3,ratio6_1,false); // C
motor LiftL4 = motor(PORT2,ratio6_1,true);  // D
// Right Motors
motor LiftR1 = motor(PORT8,ratio6_1,false); //
motor LiftR2 = motor(PORT9,ratio6_1,true);
motor LiftR3 = motor(PORT7,ratio6_1,true);
motor LiftR4 = motor(PORT10,ratio6_1,false);

motor_group Lift_Winch = motor_group(LiftL1, LiftL2, LiftL3, LiftL4,
                                     LiftR1, LiftR2, LiftR3, LiftR4);


void vexcodeInit( void ) 
{

  chassis.Gyro.calibrate();
  while (chassis.Gyro.isCalibrating()) 
  {
    wait(25, msec);
  }

}

