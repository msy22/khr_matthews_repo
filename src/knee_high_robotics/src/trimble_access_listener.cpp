// Includes____________________________________________________________________________
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tfMessage.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include <sstream>

// Setup namespaces_____________________________________________________________
using namespace std;

// Globals____________________________________________________________________________
const int _port_num = 2000;
const int _loop_rate = 1;       // rate for the main loop to cycle through in Hz


// Code____________________________________________________________________________


//void ParseTrimbleAccessMessage (string input_ta_msg,
//                                nav_msgs::Odometry& output_odom)
//{

//}




int main(int argc, char **argv)
{
  // Set up ROS node, including publishers and subscribers
  ros::init(argc, argv, "trimble_access_listener");
  ROS_INFO("Setting up trimble_access_listener");
  ros::NodeHandle nh;
  ros::Rate loop_rate(_loop_rate);
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("odometry/measured", 1000);
  
  string latest_ta_position;
  nav_msgs::Odometry latest_odom_msg;

  // Enter into a loop, consistently polling TA for positions, only exits
  // when "Ctrl + C" is pressed
  while (ros::ok())
  {
    /* TODO
     * Poll or wait for TA to return the next position estimate as a string
     *
     */

    // Publish the odometry
    ROS_INFO("Spinning at 1Hz, publishing a blank odometry msgs");
    odom_publisher.publish(latest_odom_msg);

    // These functions make sure this while loop runs at 10 Hz, and allows
    // callbacks to run
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Sit here waiting, allowing the callbacks to run and only ending when
  // the node is shut down with "Ctrl+C"
  ros::spin();

  return 0;
}
