#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geographic_msgs/GeoPoint.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <thread>
#include <mutex>
#include <tf/tf.h>
#include "utils.h"

class robot_navigation
{
public:
  ros::Publisher robot_est_location_pub;
  ros::Publisher robot_bearing_pub;
  ros::Publisher robot_odometry_pub;
  ros::Subscriber imu_sub;
  ros::Subscriber ground_truth_sub;
  ros::Subscriber in_turn_sub;
  //===================================
  //	Interface
  //===================================
  robot_navigation(ros::NodeHandle *node_handle)
  {
    robot_est_location_pub = node_handle->advertise<sensor_msgs::NavSatFix>("robot_estimated_location", 10);
    imu_sub = node_handle->subscribe("imu", 10, &robot_navigation::imu_callback, this);
    // temporary till IMU is fixed////////////////////////////////////////////////////////
    ground_truth_sub = node_handle->subscribe("ground_truth/state", 10, &robot_navigation::imu_from_ground_truth_callback, this);
    /////////////////////////////////////////////////////////////////////////////////////
    in_turn_sub = node_handle->subscribe("robot_control/in_turn", 10, &robot_navigation::in_turn_callback, this);
    robot_bearing_pub = node_handle->advertise<std_msgs::Float64>("robot_navigation/robot_bearing_imu", 10);
    robot_odometry_pub = node_handle->advertise<nav_msgs::Odometry>("robot_navigation/robot_odometry", 10);
    std::thread publishing{&robot_navigation::mainLoop, this};
    publishing.join();
  }

  bool onInit()
  {
    mIsAlive = true;
    mCount = 0;
    mImuTimeprev = ros::Time::now().toSec();
    mYawmeasImu = 0;
    mXprev = 0;
    mYprev = 0;
    mZprev = 0;
    mDistance_est = 0;
    mX_est = 0;
    mY_est = 0;
    m_vX_imu_meas = 0;
    m_vY_imu_meas = 0;
    m_in_turn = false;

    return mIsAlive;
  }
  private:
    //===================================
  //	Member Variables
  //===================================

  bool mIsAlive;
  int mCount;
  double mImuTimeprev;
  double mYawmeasImu;
  double mXprev;
  double mYprev;
  double mZprev;
  double mDistance_est;
  double mX_est;
  double mY_est;
  double m_vX_imu_meas;
  double m_vY_imu_meas;
  double m_in_turn;
  geographic_msgs::GeoPoint mRobot_est_location;

  void mainLoop()
  {
    ros::Rate rate(100);

    while (ros::ok())
    {
      ros::spinOnce();
      sensor_msgs::NavSatFix msg;
      msg.status.status = 1;
      msg.header.stamp = ros::Time::now();
      msg.position_covariance_type = int(0);
      double Bearing1 = atan2(-mY_est, mX_est);
      mRobot_est_location = calc_ikun(INITIAL_ROBOT_LAT, INITIAL_ROBOT_LON, Bearing1, mDistance_est);
      msg.status.status = 1;
      msg.header.stamp = ros::Time::now();
      msg.position_covariance_type = int(0);
      msg.latitude = mRobot_est_location.latitude;
      msg.longitude = mRobot_est_location.longitude;
      robot_est_location_pub.publish(msg);
      rate.sleep();
    }
  }

  //===================================
  //	Member Functions
  //===================================

  //***********************************
  // Callback functions
  //***********************************
  void imu_from_ground_truth_callback(const nav_msgs::Odometry &msg)
  {

    mDistance_est = sqrt(pow(msg.pose.pose.position.x, 2) + pow(msg.pose.pose.position.y, 2));
    mX_est = msg.pose.pose.position.x;
    mY_est = msg.pose.pose.position.y;
    m_vX_imu_meas = msg.twist.twist.linear.x;
    m_vY_imu_meas = msg.twist.twist.linear.y;
  }

  void imu_callback(const sensor_msgs::Imu &msg)
  {

    double dt = msg.header.stamp.toSec() - mImuTimeprev;
    mYawmeasImu = (tf::getYaw(msg.orientation)) * RAD2DEG;
    std_msgs::Float64 pub_msg;
    pub_msg.data = mYawmeasImu;
    robot_bearing_pub.publish(pub_msg);

    nav_msgs::Odometry imu_extract;
    imu_extract.header.frame_id = "world";
    imu_extract.header.stamp = ros::Time::now();
    imu_extract.pose.pose.orientation = msg.orientation;
    imu_extract.twist.twist.linear.x =  m_vX_imu_meas;
    imu_extract.twist.twist.linear.y =  m_vY_imu_meas;
    imu_extract.twist.twist.linear.z = 0;
    imu_extract.pose.pose.position.x = mXprev + dt * imu_extract.twist.twist.linear.x;
    imu_extract.pose.pose.position.y = mYprev + dt * imu_extract.twist.twist.linear.y;
    imu_extract.pose.pose.position.z = 0;
    imu_extract.twist.twist.angular.x = msg.angular_velocity.x;
    imu_extract.twist.twist.angular.y = msg.angular_velocity.y;
    imu_extract.twist.twist.angular.z = msg.angular_velocity.z;
    imu_extract.pose.covariance[0] = dt;
    imu_extract.twist.covariance[0] = 0;
    mXprev = imu_extract.pose.pose.position.x;
    mYprev = imu_extract.pose.pose.position.y;
    mZprev = imu_extract.pose.pose.position.z;

    ////////////////////////////////////////
    // temporary (because the IMU is very noisy in acceleration)
    imu_extract.pose.pose.position.x = mX_est;
    imu_extract.pose.pose.position.y = mY_est;
    ////////////////////////////////////////
    mImuTimeprev = msg.header.stamp.toSec();
    robot_odometry_pub.publish(imu_extract);


 
  }

  void in_turn_callback(const std_msgs::Bool &msg)
  {
    m_in_turn = msg.data;
  }
  //===================================
  //	Member Variables
  //===================================

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_navigation_node");
  ros::NodeHandle node_handle;
  robot_navigation tractor_robot_navigation = robot_navigation(&node_handle);
  bool isAlive = tractor_robot_navigation.onInit();
  ros::spin();
  return 0;
}