// Includes_____________________________________________________________________
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tfMessage.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include <sstream>                                  // stringstream
#include <math.h>                                   // atan2
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>             // Required for matrix inverse calcs
#include <eigen3/Eigen/Geometry>          // Required for matrix definitions

#define _USE_MATH_DEFINES

// Setup namespaces_____________________________________________________________
using namespace std;


// Globals______________________________________________________________________
const int _loop_rate = 5;       // rate for the main loop to cycle through in Hz


// Code_________________________________________________________________________



template <typename Scalar> inline void
print4x4Matrix (Eigen::Matrix<Scalar, 4, 4>& matrix)
{
  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
  printf ("\n");
}



void
CalculateRpyFromMatrix(Eigen::Matrix<double, 3, 3>& mat,
                       double& roll, double& pitch, double& yaw)
{
  roll = atan2(mat(2,1),mat(2,2));
  pitch = atan2(-mat(2,0), sqrt(pow(mat(2,1),2) + pow(mat(2,2),2)));
  yaw = atan2(mat(1,0), mat(0,0));
}



class DataSelector
{
  public:
    // Default constructor
    DataSelector()
    {
      use_imu = true;

      // Set up the publishers and subscribers
      base_odom_publisher   = nh.advertise<nav_msgs::Odometry>("odometry/base", 1000);
      orientation_publisher = nh.advertise<sensor_msgs::Imu>("orientation/selected", 1000);
      imu_subscriber        = nh.subscribe("microstrain_imu/data/enu", 1,
                                           &DataSelector::imu_callback, this);
      prism_odom_subscriber = nh.subscribe("odometry/prism", 1,
                                           &DataSelector::odom_callback, this);
    }



    void imu_callback(const sensor_msgs::Imu& imu_data)
    {
      // Use IMU data to determine if the robot is moving and set the use_imu flag
      // TODO

      latest_imu_msg = imu_data;

      // Apply a dirty hack to the yaw to get it to align with the SX10 world frame
      Eigen::Quaternion<double> q1(imu_data.orientation.w,
                                   imu_data.orientation.x,
                                   imu_data.orientation.y,
                                   imu_data.orientation.z);
      Eigen::Matrix3d R;
      R = q1.toRotationMatrix();
      double angle = 70.0 * (M_PI/180.0);
      Eigen::Matrix3d R_offset;
      R_offset <<  cos(angle), sin(angle), 0,
                  -sin(angle), cos(angle), 0,
                   0,          0,          1;
      R = R * R_offset;
      Eigen::Quaterniond q2(R);
      latest_imu_msg.orientation.w = q2.w();
      latest_imu_msg.orientation.x = q2.x();
      latest_imu_msg.orientation.y = q2.y();
      latest_imu_msg.orientation.z = q2.z();

      // Print the yaw to confirm that the hack worked
      double roll, pitch, yaw, yaw_to_print;
      CalculateRpyFromMatrix(R, roll, pitch, yaw);
      yaw_to_print = yaw;
      yaw_to_print = yaw_to_print * (180 / M_PI);                   // Convert to degrees
      if (yaw_to_print < 0) { yaw_to_print = 360 + yaw_to_print;}   // Scale to 0->360
      double yaw_true = yaw_to_print;
      yaw_true = 360.0 - yaw_true;                                  // Reverse direction
      yaw_true = yaw_true + 90.0;                                   // So 0 is north
      if (yaw_true > 360) { yaw_true = yaw_true - 360;}   // Scale to 0->360
      cout << "Yaw (REP103): " << setprecision(4) << yaw_to_print
           << " (True N): " << yaw_true << endl;

      // Re-publish the IMU data IF the robot is moving
      if (use_imu) { orientation_publisher.publish(latest_imu_msg); }
    }



    void odom_callback(const nav_msgs::Odometry& odom_data)
    {
      sensor_msgs::Imu imu_data = latest_imu_msg;
      Eigen::Matrix4d prism_pose = Eigen::Matrix4d::Identity();

      // Get the orientation of the prism from the IMU
      Eigen::Matrix3d R;
      Eigen::Quaternion<double> q1(imu_data.orientation.w,
                                   imu_data.orientation.x,
                                   imu_data.orientation.y,
                                   imu_data.orientation.z);
      R = q1.toRotationMatrix();
      prism_pose.block<3,3>(0,0) = R;

      // Get the pose of the prism from the SX10 data
      prism_pose(0,3) = odom_data.pose.pose.position.x;
      prism_pose(1,3) = odom_data.pose.pose.position.y;
      prism_pose(2,3) = odom_data.pose.pose.position.z;

      // Get the transform from prism to base
      Eigen::Matrix4d prism_to_base = Eigen::Matrix4d::Identity();
      prism_to_base << 1, 0, 0, -0.108,
                       0, 1, 0, -0.1515,
                       0, 0, 1, -0.6513,
                       0, 0, 0, 1;

      // Calculate the pose of the base
      Eigen::Matrix4d base_pose = Eigen::Matrix4d::Identity();
      base_pose = prism_pose * prism_to_base;

      // Convert to ROS format and publish
      nav_msgs::Odometry output_msg;
      output_msg.header.frame_id = "map";
      output_msg.child_frame_id = "base_link";
      output_msg.header.stamp = ros::Time::now();
      output_msg.pose.pose.position.x = base_pose(0,3);
      output_msg.pose.pose.position.y = base_pose(1,3);
      output_msg.pose.pose.position.z = base_pose(2,3);
      Eigen::Quaternion<double> q2(base_pose.block<3,3>(0,0));
      output_msg.pose.pose.orientation.w = q2.w();
      output_msg.pose.pose.orientation.x = q2.x();
      output_msg.pose.pose.orientation.y = q2.y();
      output_msg.pose.pose.orientation.z = q2.z();
      base_odom_publisher.publish(output_msg);
    }



  private:
    ros::NodeHandle nh;
    ros::Publisher base_odom_publisher;
    ros::Publisher orientation_publisher;
    ros::Subscriber imu_subscriber;
    ros::Subscriber prism_odom_subscriber;
    ros::Subscriber sx10_orientation_subscriber;

    // Class variables
    bool use_imu;
    sensor_msgs::Imu latest_imu_msg;

}; // End of class DataSelector



int main(int argc, char **argv)
{
  // Set up ROS node, including publishers and subscribers
  ros::init(argc, argv, "sensor_data_selector");
  ROS_INFO("Setting up sensor_data_selector");

  DataSelector selector1;

  // Sit here waiting, allowing the callbacks to run and only ending when
  // the node is shut down with "Ctrl+C"
  ros::spin();
  return 0;
}
