#ifndef AIM_GO_AIM_H_
#define AIM_GO_AIM_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

geometry_msgs::Twist aimGoAim(turtlesim::Pose start, turtlesim::Pose end, double distance_tolerance, double angle_tolerance)
{
  geometry_msgs::Twist cmd;

  // calculate the distance to target
  double dist_to_go = sqrt(pow(end.x-start.x, 2) + pow(end.y-start.y, 2));

  // if at target, turn to target pose
  if(dist_to_go < distance_tolerance)
  {
    // use raw difference here, can apply gain after return
    cmd.angular.z = end.theta - start.theta;
  }
  else
  {
    // if not at target, check if bot is facing it
    double angle_error = atan2(end.y-start.y, end.x-start.x) - start.theta;
    if(angle_error > M_PI)
    {
      angle_error -= (2.0*M_PI);
    }
    else if(angle_error < -M_PI)
    {
      angle_error += (2.0*M_PI);
    }

    // continuously adjust angle to target
    cmd.angular.z = angle_error;

    // only move forward once (roughly) pointing the right direction
    if(fabs(angle_error) < angle_tolerance)
    {
      cmd.linear.x = dist_to_go;
    }

  }
  return cmd;
}

#endif /* AIM_GO_AIM_H_ */
