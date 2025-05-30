/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//

#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
#include "JAR-Template/odom.h"
#include "JAR-Template/drive.h"
#include "JAR-Template/util.h"
#include "JAR-Template/PID.h"
#include "autons.h"

#include "ai_jetson.h"
#include "ai_robot_link.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

extern Drive chassis;
extern ai::jetson jetson_comms;
extern ai::robot_link link;

extern int dashboardTask( void );