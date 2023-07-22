
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "math.h"
#include <thread>
#include <mutex>
#include <tf/tf.h>
#include "utils.h"

class sim_ground_truth
{
public:
  ros::Publisher robot_location_pub;
  ros::Subscriber ground_truth_sub;
  ros::Publisher robot_yaw_ground_truth_pub;
  //===================================
  //	Interface
  //===================================
  sim_ground_truth(ros::NodeHandle *node_handle)
  {
    robot_location_pub = node_handle->advertise<sensor_msgs::NavSatFix>("robot_location", 10);
    robot_yaw_ground_truth_pub = node_handle->advertise<std_msgs::Float64>("robot_yaw_ground_truth", 10);
    ground_truth_sub = node_handle->subscribe("ground_truth/state", 10, &sim_ground_truth::ground_truth_callback, this);
    std::thread publishing{&sim_ground_truth::mainLoop, this};
    publishing.join();
  }
  bool onInit()
  {
    mIsAlive = true;
    mBearing = 0;
    mDistance = 0;
    mYaw_groungTruth = 0;
    return mIsAlive;
  }

private:
  //===================================
  //	Member Variables
  //===================================

  bool mIsAlive;
  double mBearing;
  double mDistance;
  double mYaw_groungTruth;
  geographic_msgs::GeoPoint mRobot_true_location;

  void mainLoop()
  {
    ros::Rate rate(10);

    while (ros::ok())
    {
      ros::spinOnce();

      // calculate ground truth of the robot location from an initial position:

      mRobot_true_location = calc_ikun(INITIAL_ROBOT_LAT, INITIAL_ROBOT_LON, mBearing, mDistance);
      sensor_msgs::NavSatFix msg;
      msg.status.status = 1;
      msg.header.stamp = ros::Time::now();
      msg.position_covariance_type = int(0);
      msg.latitude = mRobot_true_location.latitude;
      msg.longitude = mRobot_true_location.longitude;
      msg.position_covariance[0] = mBearing;
      msg.position_covariance[1] = mYaw_groungTruth;
      robot_location_pub.publish(msg);
      rate.sleep();
    }
  }
  //===================================
  //	Member Functions
  //===================================

  //***********************************
  // Callback functions
  //***********************************
  void ground_truth_callback(const nav_msgs::Odometry &msg)
  {
    mBearing = atan2(-msg.pose.pose.position.y, msg.pose.pose.position.x);
    mDistance = sqrt(pow(msg.pose.pose.position.x, 2) + pow(msg.pose.pose.position.y, 2));
    mYaw_groungTruth = (tf::getYaw(msg.pose.pose.orientation)) * RAD2DEG;
    std_msgs::Float64 robot_yaw;
    robot_yaw.data = mYaw_groungTruth;
    robot_yaw_ground_truth_pub.publish(robot_yaw);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_ground_truth_node");
  ros::NodeHandle node_handle;
  sim_ground_truth tractor_sim_ground_truth = sim_ground_truth(&node_handle);
  bool isAlive = tractor_sim_ground_truth.onInit();
  ros::spin();
  return 0;
}