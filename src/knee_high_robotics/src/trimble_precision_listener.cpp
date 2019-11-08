// Includes_____________________________________________________________________
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tfMessage.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <sys/socket.h>                             // socket
#include <netinet/in.h>                             // Required for TCP comms
#include <sstream>                                  // stringstream
#include <boost/algorithm/string/trim.hpp>          // trim
#include <math.h>                                   // atan2
#include <iostream>
#include <fstream>

// Custom libraries we've written for the Hackathon
#include <knee_high_robotics/tcp_client_class.h>

// Setup namespaces_____________________________________________________________
using namespace std;

// Defines______________________________________________________________________
#define PORT_INPUT 2000
#define PORT_OUTPUT 2001
#define IP_WAFFLE "192.168.20"
#define IP_VM "192.168.1.5"
#define IP_LAPTOP "172.16.13.35"         // Address of laptop running the TA server default: 172.16.12.39"
#define IP_XYZ_ROUTER "192.168.1.1"
#define IP_MATTS_LAPTOP "192.168.2.20"  // Address of Linux laptop
#define IP_JACKAL "192.168.2.22"        // WiFi address of Jackal
#define BUFF_LENGTH 512


// Globals______________________________________________________________________
const int _loop_rate = 5;       // rate for the main loop to cycle through in Hz
const float _heading_from_position_threshold = 0.05; // 5 cm


// Code_________________________________________________________________________



// Takes in a string of comma separated numbers and returns a vector of floats
void SplitStringOfNumbers (string string_to_be_split,
                           vector<float>& split_string)
{
  string delimiter = ",";
  size_t pos = 0;
  string substr;

  while ((pos = string_to_be_split.find(delimiter)) != string::npos)
  {
      substr = string_to_be_split.substr(0, pos);
      boost::algorithm::trim(substr);                  // Remove ALL whitespaces
      string_to_be_split.erase(0, pos + delimiter.length());
      split_string.push_back(atof(substr.c_str()));
  }
  split_string.push_back(atof(string_to_be_split.c_str()));
}



void ConstructOdometryMessage(string input_str,
                              nav_msgs::Odometry& output_msg)
{
  vector<float> input_data;
  SplitStringOfNumbers(input_str, input_data);

  // Fill out message details
  output_msg.header.frame_id = "map";
  output_msg.child_frame_id = "aux_gps"; // dirty hack because the prism is where the antenna was
  output_msg.header.stamp = ros::Time::now();
  output_msg.pose.pose.position.y = input_data[1];
  output_msg.pose.pose.position.x = input_data[2];
  output_msg.pose.pose.position.z = input_data[3];
  output_msg.pose.pose.orientation.w = 1.0;
}



void HackOdometryMessage(string input_str,
                              nav_msgs::Odometry& output_msg)
{
  vector<float> input_data;
  SplitStringOfNumbers(input_str, input_data);

  // Fill out message details
  output_msg.header.frame_id = "map";
  output_msg.child_frame_id = "aux_gps"; // dirty hack because the prism is where the antenna was
  output_msg.header.stamp = ros::Time::now();
  output_msg.pose.pose.position.y = input_data[0];
  output_msg.pose.pose.position.x = input_data[1];
  output_msg.pose.pose.position.z = 0.658; // hard-coded height of prism on robot
  output_msg.pose.pose.orientation.w = 1.0;
}



void ConstructGoalMessage(string input_str,
                          geometry_msgs::PoseStamped& output_msg)
{
  vector<float> input_data;
  SplitStringOfNumbers(input_str, input_data);

  // Fill out header details
  output_msg.header.stamp = ros::Time::now();
  output_msg.header.frame_id = "map";
  output_msg.pose.position.y = input_data[1];
  output_msg.pose.position.x = input_data[2];
  output_msg.pose.position.z = input_data[3];
  output_msg.pose.orientation.w = 1.0;
}



/* Takes in a comma-separated string that may contain multiple messages, and
 * outputs the latest valid message of each type. For now we explicitly assume
 * that every message is complete.
 */
void ParseTrimblePrecisionMessage (string input_tp_msg,
                                   string& latest_goal,
                                   string& latest_position,
                                   string& latest_orientation)
{
  /* Split the message into semicolon separated messages. Any incomplete
   * messages in the back end of the input string will automatically be ignored */
  vector<string> messages;
  string delimiter = ";";
  size_t pos = 0;
  string substr;

  while ((pos = input_tp_msg.find(delimiter)) != string::npos)
  {
    substr = input_tp_msg.substr(0, pos+1);
    boost::algorithm::trim(substr);
    messages.push_back(substr);
    input_tp_msg.erase(0, pos + delimiter.length());
  }

  /* Allocate messages to the right strings. This method inherently means that
     early messages are overwritten by later ones */
  string cmd_type;
  for (int i=0; i < messages.size(); i++)
  {
    cmd_type = messages[i][0];
    if (cmd_type == "0") {latest_goal = messages[i];}
    if (cmd_type == "1") {latest_position = messages[i];}
    if (cmd_type == "2") {latest_orientation = messages[i];}
  }
}


void basicParser(string input_tp_msg,
                 string& latest_position)
{
  vector<string> elements;
  string delimiter = ",";
  size_t pos = 0;
  string substr;

  while ((pos = input_tp_msg.find(delimiter)) != string::npos)
  {
    substr = input_tp_msg.substr(0, pos+1);
    boost::algorithm::trim(substr);
    elements.push_back(substr);
    input_tp_msg.erase(0, pos + delimiter.length());
  }
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
  // Set up ROS node
  ros::init(argc, argv, "trimble_precison_listener");
  ROS_INFO("Setting up trimble_precision_listener");
  ros::NodeHandle nh;
  ros::Rate loop_rate(_loop_rate);

  // Set up Publishers and subscribers
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("odometry/prism", 1000);
  ros::Publisher goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

  // Wait so the publishers have time to become established
  ros::Duration(0.5).sleep();

  // Set up various containers for ROS messages
  string latest_tp_message, latest_goal, latest_position, latest_orientation,
         previous_position, previous_goal, previous_orientation;
  nav_msgs::Odometry latest_odom_msg;
  geometry_msgs::PoseStamped latest_goal_msg;

  // Test the TCP client by pinging google (i.e. uncomment to debug tcp comms)
  //TestTcpClientWithGoogle();

  // Create the TCP socket to listen to positions from Trimble Access
  // Note: the "conn" function will hang if no connection can be made
  tcp_client trimble_tcp_client;
  trimble_tcp_client.conn(IP_LAPTOP, PORT_INPUT);

  // FOR DEBUGGING
  //string test_str = "1,10.000,20.000,5;0,10.000,20.000,5;";

  // Enter into a loop, consistently polling TA for positions, only exit
  // when "Ctrl + C" is pressed
  while (ros::ok())
  {
    // Get a message from Trimble Precision
    latest_tp_message.clear();
    latest_tp_message = trimble_tcp_client.receive(BUFF_LENGTH);
    ROS_INFO("Received TP message: %s", latest_tp_message.c_str());

    // code to make this work
    HackOdometryMessage(latest_tp_message, latest_odom_msg);
    odom_publisher.publish(latest_odom_msg);

    // Parse the string to extract the relevent messages
//    ParseTrimblePrecisionMessage(latest_tp_message, latest_goal,
//                                 latest_position, latest_orientation);

    // Publish the latest messages IF AND ONLY IF they are new
    if (latest_goal.compare(previous_goal) != 0)
    {
      ConstructGoalMessage(latest_goal, latest_goal_msg);
      goal_publisher.publish(latest_goal_msg);
      previous_goal = latest_goal;
    }
    if (latest_position.compare(previous_position) != 0)
    {
      ConstructOdometryMessage(latest_position, latest_odom_msg);
      odom_publisher.publish(latest_odom_msg);
      previous_position = latest_position;
    }
    if (latest_orientation.compare(previous_orientation) != 0)
    {
      previous_orientation = latest_orientation;
    }

    // spinOnce makes sure the callbacks run, sleep makes the loop run at 1 Hz
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Sit here waiting, allowing the callbacks to run and only ending when
  // the node is shut down with "Ctrl+C"
  ros::spin();
  return 0;
}
