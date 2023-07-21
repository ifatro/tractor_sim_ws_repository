#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "geographic_msgs/GeoPoint.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include <thread>
#include <mutex>
#include <tf/tf.h>
#include "utils.h"

class path_planning
{
public:
  ros::Publisher robot_target_path_pub;
  ros::Subscriber target_setpoint_sub;
  ros::Subscriber robot_estimated_location_sub;
  ros::Subscriber robot_in_turn_sub;

  //===================================
  //	Interface
  //===================================
  path_planning(ros::NodeHandle *node_handle, geographic_msgs::GeoPoint robot_init_location)
  {
    mRobot_init_location = robot_init_location;
    robot_target_path_pub = node_handle->advertise<geometry_msgs::Pose2D>("path_planning/robot_target_path", 10);
    target_setpoint_sub = node_handle->subscribe("target_location", 10, &path_planning::target_location_callback, this);
    robot_estimated_location_sub = node_handle->subscribe("robot_estimated_location", 10, &path_planning::robot_estimated_location_callback, this);
    robot_in_turn_sub = node_handle->subscribe("robot_control/in_turn", 10, &path_planning::robot_in_turn_callback, this);
    std::thread publishing{&path_planning::mainLoop, this};
    publishing.join();
  }

  bool onInit()
  {
    mIsAlive = true;
    mCount = 0;
    mNewTargetSetpoint = true;
    mIn_turn = false;
    return mIsAlive;
  }

private:
  //===================================
  //	Member Functions
  //===================================
  void mainLoop()
  {
    ros::Rate rate(10);

    while (ros::ok())
    {
      ros::spinOnce();
      if (mNewTargetSetpoint || mIn_turn)
      {
        mNewTargetSetpoint = false;
        double bearing = calc_bearing(mRobot_init_location.latitude, mRobot_init_location.longitude, mTarget_location.latitude, mTarget_location.longitude);
        double distance = calc_distance(mRobot_init_location.latitude, mRobot_init_location.longitude, mTarget_location.latitude, mTarget_location.longitude);
        geometry_msgs::Pose2D robot2target_pose;
        robot2target_pose.theta = bearing;
        robot2target_pose.x = distance; //*cos(robot2target_pose.theta);
        robot2target_pose.y = distance; //*sin(robot2target_pose.theta);
        robot_target_path_pub.publish(robot2target_pose);
      }

      rate.sleep();
    }
  }

  //***********************************
  // Callback functions
  //***********************************

  void target_location_callback(const geographic_msgs::GeoPoint &msg)
  {
    mTarget_location = msg;
    mNewTargetSetpoint = true;
  }

  void robot_estimated_location_callback(const sensor_msgs::NavSatFix &msg)
  {
    mRobot_init_location.latitude = msg.latitude;
    mRobot_init_location.longitude = msg.longitude;
  }

  void robot_in_turn_callback(const std_msgs::Bool &msg)
  {
    mIn_turn = msg.data;
  }


private:
  //===================================
  //	Member Variables
  //===================================
  bool mIsAlive;
  bool mNewTargetSetpoint;
  int mCount;
  geographic_msgs::GeoPoint mRobot_init_location;
  geographic_msgs::GeoPoint mTarget_location;
  bool mIn_turn;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_path_planning_node");
  ros::NodeHandle node_handle;

  // robot initial position:
  geographic_msgs::GeoPoint robot_init_location;
  robot_init_location.latitude = INITIAL_ROBOT_LAT;
  robot_init_location.longitude = INITIAL_ROBOT_LON;
  path_planning tractor_path_planning = path_planning(&node_handle, robot_init_location);
  bool isAlive = tractor_path_planning.onInit();

  ros::spin();
  return 0;
}