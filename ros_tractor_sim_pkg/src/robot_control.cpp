#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <thread>
#include <mutex>

class robot_control
{
public:

  ros::Publisher left_wheel_2_velocity_controller_command_pub;
  ros::Publisher left_wheel_velocity_controller_command_pub;
  ros::Publisher right_wheel_2_velocity_controller_command_pub;
  ros::Publisher right_wheel_velocity_controller_command_pub;
  ros::Publisher in_turn_pub;

  ros::Subscriber robot_path_sub;
  ros::Subscriber robot_bearing_meas_sub;
  ros::Subscriber robot_odometry_sub;

  ros::Publisher curr_dist_pub;
  ros::Publisher cmd_dist_pub;
  ros::Publisher dt_pub;

  //===================================
  //	Interface
  //===================================
  robot_control(ros::NodeHandle *node_handle)
  {


    if(!node_handle->getParam("/MAX_TOURQE",MAX_TOURQE )) MAX_TOURQE=70.0;
    if(!node_handle->getParam("/MIN_TOURQE",MIN_TOURQE )) MIN_TOURQE=25.0;
    if(!node_handle->getParam("/ACCELERATION_TIME",ACCELERATION_TIME )) ACCELERATION_TIME=10.0;
    if(!node_handle->getParam("/BEARING_ERROR",BEARING_ERROR )) BEARING_ERROR=1.0;
    if(!node_handle->getParam("/DISTANCE_ERROR",DISTANCE_ERROR )) DISTANCE_ERROR=1.0;
    if(!node_handle->getParam("/DECELERATION_DISTANCE",DECELERATION_DISTANCE )) DECELERATION_DISTANCE=30.0;
    if(!node_handle->getParam("/MIN_TOURQE_DEACCELERATIN",MIN_TOURQE_DEACCELERATIN )) MIN_TOURQE_DEACCELERATIN=10.0;
    
    ANGULAR_ACCELERATION = (MAX_TOURQE - MIN_TOURQE) / ACCELERATION_TIME;



    left_wheel_2_velocity_controller_command_pub = node_handle->advertise<std_msgs::Float64>("my_robot_model/left_wheel_2_hinge_joint_velocity_controller/command", 10);
    left_wheel_velocity_controller_command_pub = node_handle->advertise<std_msgs::Float64>("my_robot_model/left_wheel_hinge_joint_velocity_controller/command", 10);
    right_wheel_2_velocity_controller_command_pub = node_handle->advertise<std_msgs::Float64>("my_robot_model/right_wheel_2_hinge_joint_velocity_controller/command", 10);
    right_wheel_velocity_controller_command_pub = node_handle->advertise<std_msgs::Float64>("my_robot_model/right_wheel_hinge_joint_velocity_controller/command", 10);
    curr_dist_pub = node_handle->advertise<std_msgs::Float64>("robot_control/curr_dist", 10);
    cmd_dist_pub = node_handle->advertise<std_msgs::Float64>("robot_control/cmd_dist", 10);
    dt_pub = node_handle->advertise<std_msgs::Float64>("robot_control/time_from_start_driving_ahead", 10);
    in_turn_pub = node_handle->advertise<std_msgs::Bool>("robot_control/in_turn", 10);

    // from path planning node distance and bearing to the target:
    robot_path_sub = node_handle->subscribe("path_planning/robot_target_path", 10, &robot_control::path_callback, this);
    // from navigation node measured bearing of robot (from IMU):
    robot_bearing_meas_sub = node_handle->subscribe("robot_navigation/robot_bearing_imu", 10, &robot_control::bearing_imu_callback, this);
    // from navigation node measured odometry (x,y,z) of robot (from IMU extract):
    robot_odometry_sub = node_handle->subscribe("robot_navigation/robot_odometry", 10, &robot_control::odometry_imu_callback, this);
    std::thread publishing{&robot_control::mainLoop, this};
    publishing.join();
  }

  bool onInit()
  {
    mIsAlive = true;
    mCount = 0;
    mBearing_cmd = 0;
    mBearing_cmdPrev = -1;
    mBearing_imu_meas = 0;
    mStartDrivingAhead = false;
    mXinit = 0;
    mYinit = 0;
    mX = 0;
    mY = 0;
    mDistance_cmd.data = 0;
    mIn_turn.data = false;
    mWheelRadius = 0.05;
    mTimeAtAheadDriveStart = ros::Time::now();

    return mIsAlive;
    SAMPLE_TIME=0.033;
    mLastVel=0;
  }

private:
  //===================================
  //	Member Variables
  //===================================

  double mBearing_cmd;
  std_msgs::Float64 mDistance_cmd;
  std_msgs::Float64 mCurr_distance;
  std_msgs::Bool mIn_turn;
  double mBearing_cmdPrev;
  double mBearing_imu_meas;
  double mXinit;
  double mYinit;
  double mX;
  double mY;
  std_msgs::Float64 mVel_cmd_left, mVel_cmd_right;
  float mLastVel;
  bool mIsAlive;
  bool mStartDrivingAhead;
  int mCount;
  double mWheelRadius;
  ros::Time mTimeAtAheadDriveStart;
  double  MAX_TOURQE ;
  double  MIN_TOURQE ;
  double  ACCELERATION_TIME ;
  double  BEARING_ERROR;
  double  DISTANCE_ERROR;
  double  DECELERATION_DISTANCE;
  double MIN_TOURQE_DEACCELERATIN;
  double  SAMPLE_TIME;
  double  ANGULAR_ACCELERATION ;

  void mainLoop()
  {
    ros::Rate rate(30);
    while (ros::ok())
    {
      ros::spinOnce();
      mCurr_distance.data = sqrt(pow((mX - mXinit), 2) + pow((mY - mYinit), 2));
      curr_dist_pub.publish(mCurr_distance);
      cmd_dist_pub.publish(mDistance_cmd);

      // spin around till bearing measurement = bearing command

      if (abs(mBearing_cmd - mBearing_cmdPrev) > BEARING_ERROR)
      {
        // a new target point has arrived from the path planning:

        mStartDrivingAhead = false;
      }

      if (abs(fmod(mBearing_cmd - mBearing_imu_meas, 360)) > BEARING_ERROR && !mStartDrivingAhead) // turn to bearing angle
      {
        mCount = 0;
        mIn_turn.data = true;
        calcVelCmdTurn();
      }

      else // start driving ahead
      {

        if (mCount == 0)
        {
          mTimeAtAheadDriveStart = ros::Time::now();
        }
        mCount++;
        mIn_turn.data = false;
        mStartDrivingAhead = true;
        calcVelCmdDrivingAhead();
      }

      // wheels control:

      left_wheel_2_velocity_controller_command_pub.publish(mVel_cmd_left);
      left_wheel_velocity_controller_command_pub.publish(mVel_cmd_left);
      right_wheel_2_velocity_controller_command_pub.publish(mVel_cmd_right);
      right_wheel_velocity_controller_command_pub.publish(mVel_cmd_right);
      in_turn_pub.publish(mIn_turn);
      rate.sleep();
    }
  }

  //===================================
  //	Member Functions
  //===================================
  void calcVelCmdDrivingAhead()
  {

    double dt = ros::Time::now().toSec() - mTimeAtAheadDriveStart.toSec();

    if (mCurr_distance.data + DISTANCE_ERROR < mDistance_cmd.data) //drive while ditance to target >1 [meter]
    {
      if (mCurr_distance.data +  DECELERATION_DISTANCE < mDistance_cmd.data) //max torque while distance to target > 5[meter]
      {

        if (dt <= ACCELERATION_TIME) //acceleration
        {
          mVel_cmd_left.data = std::min(MIN_TOURQE+ ANGULAR_ACCELERATION * dt, MAX_TOURQE);
          mVel_cmd_right.data = std::min(MIN_TOURQE+ ANGULAR_ACCELERATION * dt, MAX_TOURQE);
        }
        else //constant torque
        {
          mVel_cmd_left.data = MAX_TOURQE;
          mVel_cmd_right.data = MAX_TOURQE;
        }
      }
      else  //DEACCELERATIN
      {
        mVel_cmd_left.data =std::max(mVel_cmd_left.data - 2*ANGULAR_ACCELERATION  * 0.03, MIN_TOURQE);
        mVel_cmd_right.data =std::max(mVel_cmd_right.data - 2*ANGULAR_ACCELERATION  * 0.03, MIN_TOURQE);

      }
    }
    else //stop
    {
      mVel_cmd_left.data = 0;
      mVel_cmd_right.data = 0;
    }
    std_msgs::Float64 y;
    y.data = mVel_cmd_left.data;
    dt_pub.publish(y);
    //mLastVel=
  }

  void calcVelCmdTurn()
  {
    /*deceleration*/

    if (abs(mVel_cmd_left.data) > MIN_TOURQE)
    {
      mVel_cmd_left.data = mVel_cmd_left.data - 2*ANGULAR_ACCELERATION  * 0.03;
      mVel_cmd_right.data = mVel_cmd_left.data -2*ANGULAR_ACCELERATION  * 0.03;

    }
    else  // turn to the bearing angle command
    {


      if (abs(mBearing_cmd - mBearing_imu_meas) < 180)
      {
        if (mBearing_cmd > mBearing_imu_meas)
        {

          mVel_cmd_left.data = -MIN_TOURQE;
          mVel_cmd_right.data = MIN_TOURQE;
        }
        else
        {
          mVel_cmd_left.data = MIN_TOURQE;
          mVel_cmd_right.data = -MIN_TOURQE;
        }
      }

      else
      {

        if (mBearing_cmd > mBearing_imu_meas)
        {

          mVel_cmd_left.data = MIN_TOURQE;
          mVel_cmd_right.data = -MIN_TOURQE;
        }
        else
        {
          mVel_cmd_left.data = -MIN_TOURQE;
          mVel_cmd_right.data = MIN_TOURQE;
        }
      }
    }
  }

  //***********************************
  // Callback functions
  //***********************************

  void path_callback(const geometry_msgs::Pose2D &msg)
  {
    if (msg.x <= 500) // legal command
    {
      mBearing_cmdPrev = mBearing_cmd;
      // float64 x
      // float64 y
      // float64 theta
      // path planning  bearing command
      mBearing_cmd = msg.theta;
      mDistance_cmd.data = msg.x;
      mXinit = mX;
      mYinit = mY;
    }
    else
    {
      mBearing_cmd = mBearing_imu_meas;
      mDistance_cmd.data = 0;
    }
  }

  void bearing_imu_callback(const std_msgs::Float64 &msg)
  {
    // imu bearing measurement
    mBearing_imu_meas = msg.data;
  }

  void odometry_imu_callback(const nav_msgs::Odometry &msg)
  {
    mX = msg.pose.pose.position.x;
    mY = msg.pose.pose.position.y;
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_control_node");
  ros::NodeHandle node_handle;
  robot_control tractor_robot_control = robot_control(&node_handle);
  bool isAlive = tractor_robot_control.onInit();
  ros::spin();
  return 0;
}