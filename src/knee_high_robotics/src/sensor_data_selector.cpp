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


// Setup namespaces_____________________________________________________________
using namespace std;


// Globals______________________________________________________________________
const int _loop_rate = 5;       // rate for the main loop to cycle through in Hz


// Code_________________________________________________________________________


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

    }


    void odom_callback(const nav_msgs::Odometry& odom_data)
    {

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


}; // End of class DataSelector


int main(int argc, char **argv)
{
  // Set up ROS node, including publishers and subscribers
  ros::init(argc, argv, "sensor_data_selector");
  ROS_INFO("Setting up sensor_data_selector");
  ros::NodeHandle nh;
  ros::Rate loop_rate(_loop_rate);

  // Set up Publishers and subscribers
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("odometry/measured", 1000);



  // Enter into a loop, consistently polling TA for positions, only exit
  // when "Ctrl + C" is pressed
  while (ros::ok())
  {

    // spinOnce makes sure the callbacks run, sleep makes the loop run at 1 Hz
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Sit here waiting, allowing the callbacks to run and only ending when
  // the node is shut down with "Ctrl+C"
  ros::spin();

  return 0;
}
