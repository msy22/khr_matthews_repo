// Includes_____________________________________________________________________
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tfMessage.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include <sys/socket.h>                             // socket
#include <netinet/in.h>                             // Required for TCP comms
#include <sstream>                                  // stringstream
#include <boost/algorithm/string/trim.hpp>          // trim
#include <math.h>                                   // atan2

// Custom libraries we've written for the Hackathon
#include <knee_high_robotics/tcp_client_class.h>

// Setup namespaces_____________________________________________________________
using namespace std;

// Defines______________________________________________________________________
#define PORT_INPUT 2000
#define PORT_OUTPUT 2001
#define IP_WAFFLE "192.168.20"
#define IP_VM "192.168.1.5"
#define IP_LAPTOP "192.168.1.1"
#define IP_XYZ_ROUTER "192.168.1.1"
#define IP_MATTS_LAPTOP "192.168.2.20"
#define IP_JACKAL "192.168.2.22"
#define BUFF_LENGTH 1024

// Globals______________________________________________________________________
const int _loop_rate = 1;       // rate for the main loop to cycle through in Hz
const float _heading_from_position_threshold = 0.05; // 5 cm


// Code_________________________________________________________________________


// Takes in a string of comma separated numbers and returns a vector of floats
void SplitStringOfNumbers (string string_to_be_split,
                           vector<float>& split_string)
{
  string substr;
  stringstream ss(string_to_be_split);

  while(ss.good())
  {
    getline(ss, substr, ',');
    boost::algorithm::trim(substr);                   // Remove ALL whitespaces
    split_string.push_back(atof(substr.c_str()));     // converts to float
  }
}



/* Takes in two nav_msgs::Odometry messages and assigns a heading to the
 * second input, based on the difference in position of the two. The heading
 * has to conform to ROS REP 103: http://www.ros.org/reps/rep-0103.html
 * I.e. East = 0 degrees and yaw increases CCW
 */
void CalculateHeadingFromPosition (nav_msgs::Odometry& latest_odom,
                                   nav_msgs::Odometry& prev_odom,
                                   float heading)
{
  // Calculate the difference between current and previous position
  float delta_x = latest_odom.pose.pose.position.x -
                  prev_odom.pose.pose.position.x;
  float delta_y = latest_odom.pose.pose.position.y -
                  prev_odom.pose.pose.position.y;
  float dist_moved = sqrt(pow(2, delta_x) + pow(delta_y,2));

  // if the robot has moved far enough, use the difference to calculate the
  // heading, if not assume the robot has the same heading as previous position
  if (dist_moved > _heading_from_position_threshold)
  {
    heading = atan2(delta_y, delta_x);
  }
  else
  {
    ROS_WARN("Robot didn't move far enough, copying previous orientation");
    heading = tf::getYaw(prev_odom.pose.pose.orientation);
  }
}



/* Takes in a comma-separated string "x,y,yaw" and turns this into a standard
 * ROS odometry message formatted according to this documentation:
 * http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
 * Note that this
 */
void ParseTrimbleAccessMessage (string input_ta_msg,
                                nav_msgs::Odometry& latest_odom,
                                nav_msgs::Odometry& prev_odom)
{
  /* TODO:
   * Check this is a position measurement and not a command message. For now the
   * code implicitly assumes it is a measurement and contains floats in the
   * format "x,y,yaw"
   */

  ROS_INFO("Received position from Trimble Access: %s", input_ta_msg.c_str());

  // split the message from one string into a vector of floats
  vector<float> split_message;
  SplitStringOfNumbers(input_ta_msg, split_message);

  // Fill out header details
  latest_odom.header.frame_id = "odom";
  latest_odom.child_frame_id = "base_link";
  latest_odom.header.stamp = ros::Time::now();

  // Fill out XYZ coordinate details
  latest_odom.pose.pose.position.x = split_message[0];
  latest_odom.pose.pose.position.y = split_message[1];
  latest_odom.pose.pose.position.z = 0.0;

  // Fill out the orientation details based on the difference in position
  // between this position and the last position
  float heading;
  CalculateHeadingFromPosition(latest_odom, prev_odom, heading);
  tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,heading);
  latest_odom.pose.pose.orientation.x = quat[0];
  latest_odom.pose.pose.orientation.y = quat[1];
  latest_odom.pose.pose.orientation.z = quat[2];
  latest_odom.pose.pose.orientation.w = quat[3];
}


void TestTcpClientWithGoogle ()
{
  ROS_INFO("Testing TCP interaction by pinging Google.com");

  //connect to host
  tcp_client c;
  c.conn("google.com" , 80);

  //send some data
  c.send_data("GET / HTTP/1.1\r\n\r\n");

  //receive and echo reply
  cout<<"----------------------------\n\n";
  cout<<c.receive(1024);
  cout<<"\n\n----------------------------\n\n";
}



int main(int argc, char **argv)
{
  // Set up ROS node, including publishers and subscribers
  ros::init(argc, argv, "trimble_access_listener");
  ROS_INFO("Setting up trimble_access_listener");
  ros::NodeHandle nh;
  ros::Rate loop_rate(_loop_rate);
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("odometry/measured", 1000);

  string latest_ta_position;
  nav_msgs::Odometry prev_odom_msg;
  nav_msgs::Odometry latest_odom_msg;

  // Test the TCP client by pinging google, then create a TCP connection to
  // Trimble Access
  TestTcpClientWithGoogle();
  // Note: the "conn" function will hang if no connection can be made
//  tcp_client trimble_tcp_client;
//  trimble_tcp_client.conn(IP_LAPTOP, PORT_INPUT);

  // Enter into a loop, consistently polling TA for positions, only exits
  // when "Ctrl + C" is pressed
  while (ros::ok())
  {
    // Get a message from Trimble Access
    //latest_ta_position = trimble_tcp_client.receive(BUFF_LENGTH);
    //ParseTrimbleAccessMessage(latest_ta_position, latest_odom_msg, prev_odom_msg);

    // Publish the odometry
    ROS_INFO("Spinning at 1Hz, publishing a blank odometry msgs");
    odom_publisher.publish(latest_odom_msg);

    /* TODO:
     * Publish the /odom -> /base_link transform
     * I'm not 100% sure if move_base actually needs the tf transform between
     * frames as well as the odometry.
     */

    // spinOnce makes sure the callbacks run, sleep makes the loop run at 1 Hz
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Sit here waiting, allowing the callbacks to run and only ending when
  // the node is shut down with "Ctrl+C"
  ros::spin();

  return 0;
}
