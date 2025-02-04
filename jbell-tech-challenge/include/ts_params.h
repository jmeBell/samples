#ifndef TS_PARAMS_H_
#define TS_PARAMS_H_

#include "ros/ros.h"

// the limits of where the turtle can be sent
// turtlesim units
double MIN_X = 0.01;
double MAX_X = 11.07;
double MIN_Y = 0.01;
double MAX_Y = 11.07;
double MIN_THETA = -M_PI;
double MAX_THETA = M_PI;

// number of turtle poses received before following waypoints again
// 125 is approximately 2 seconds of delay
int WAIT_AFTER_KEYS = 125;

// path planning and following
double POS_TOLERANCE = 0.1;
double ANGLE_TOLERANCE = 0.01;
double POS_GAIN = 2.0;
double ANGLE_GAIN = 2.0;

void getParams(ros::NodeHandle ndHndl)
{
  if (ndHndl.hasParam("MIN_X"))
  {
    ndHndl.getParam("MIN_X", MIN_X);
  }
  if (ndHndl.hasParam("MAX_X"))
  {
    ndHndl.getParam("MAX_X", MAX_X);
  }
  if (ndHndl.hasParam("MIN_Y"))
  {
    ndHndl.getParam("MIN_Y", MIN_Y);
  }
  if (ndHndl.hasParam("MAX_Y"))
  {
    ndHndl.getParam("MAX_Y", MAX_Y);
  }
  if (ndHndl.hasParam("MIN_THETA"))
  {
    ndHndl.getParam("MIN_THETA", MIN_THETA);
  }
  if (ndHndl.hasParam("MAX_THETA"))
  {
    ndHndl.getParam("MAX_THETA", MAX_THETA);
  }
  if (ndHndl.hasParam("WAIT_AFTER_KEYS"))
  {
    ndHndl.getParam("WAIT_AFTER_KEYS", WAIT_AFTER_KEYS);
  }
  if (ndHndl.hasParam("POS_TOLERANCE"))
  {
    ndHndl.getParam("POS_TOLERANCE", POS_TOLERANCE);
  }
  if (ndHndl.hasParam("ANGLE_TOLERANCE"))
  {
    ndHndl.getParam("ANGLE_TOLERANCE", ANGLE_TOLERANCE);
  }
  if (ndHndl.hasParam("POS_GAIN"))
  {
    ndHndl.getParam("POS_GAIN", POS_GAIN);
  }
  if (ndHndl.hasParam("ANGLE_GAIN"))
  {
    ndHndl.getParam("ANGLE_GAIN", POS_GAIN);
  }
}

#endif /* TS_PARAMS_H_ */
