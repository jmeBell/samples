#include "ros/ros.h"
#include "ts_params.h"
#include "aim_go_aim.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

class TurtleSimMove
{
  public:
  TurtleSimMove():
    wp_follow_countdown(-1)
  {
    // set up interfaces
    keyboard_sub = nh.subscribe("/keyboard", 1, &TurtleSimMove::keyboardCb, this);
    waypoint_sub = nh.subscribe("/waypoint", 1, &TurtleSimMove::waypointCb, this);
    turtlepose_sub = nh.subscribe("/turtle1/pose", 1, &TurtleSimMove::turtleposeCb, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
  }

  void keyboardCb(const geometry_msgs::TwistConstPtr& msg)
  {
    // keyboard messages blocks waypoint following
    // wp_follow_countdown decrements for each turtle pose
    wp_follow_countdown = WAIT_AFTER_KEYS;

    // keyboard messages pass-through
    // (mux with waypoint following)
    cmd_vel_pub.publish(*msg);
  }

  void waypointCb(const turtlesim::PoseConstPtr& msg)
  {
    ROS_INFO("waypoint received %f %f %f\n", msg->x, msg->y, msg->theta);
    // check that waypoint is feasible and keep
    if(MIN_X < msg->x && msg->x < MAX_X &&
       MIN_Y < msg->y && msg->y < MAX_Y &&
       MIN_THETA < msg->theta && msg->theta < MAX_THETA)
    {
      waypoint = *msg;
      ROS_INFO("waypoint in bounds");
    }
    else
    {
      ROS_INFO("waypoint rejected: out of bounds: %f - %f", MIN_X, MAX_X);
    }
  }

  void turtleposeCb(const turtlesim::PoseConstPtr& msg)
  {
    wp_follow_countdown--;
    if(wp_follow_countdown < 0)
    {
      //ROS_INFO("calculating waypoint following");
      auto follow_cmd = aimGoAim(*msg, waypoint, POS_TOLERANCE, ANGLE_TOLERANCE);
      follow_cmd.linear.x *= POS_GAIN;
      follow_cmd.angular.z *= ANGLE_GAIN;
      cmd_vel_pub.publish(follow_cmd);
    }
  }

  private:
  int wp_follow_countdown;
  turtlesim::Pose waypoint;
  ros::NodeHandle nh;
  ros::Subscriber keyboard_sub, waypoint_sub, turtlepose_sub;
  ros::Publisher cmd_vel_pub;
};//End of class TurtleSimMove


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlesim_move");
  ros::NodeHandle nh("~");
  getParams(nh);
  TurtleSimMove moveTurtleObject;
  ros::spin();
  return 0;
}
