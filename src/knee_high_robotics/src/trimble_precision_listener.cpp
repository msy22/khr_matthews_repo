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
#define IP_LAPTOP "172.16.12.39"
#define IP_XYZ_ROUTER "192.168.1.1"
#define IP_MATTS_LAPTOP "192.168.2.20"
#define IP_JACKAL "192.168.2.22"
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


/* Reads in a string of characters which will contain several messages
 */
void GetLastCompleteMessage (string& str_in,
                             string& str_out)
{
  string delimiter = ";";
  size_t pos = 0;
  string substr;
  //cout << "str_in: " << str_in << endl;

  while ((pos = str_in.find(delimiter)) != string::npos)
  {
      substr = str_in.substr(0, pos);
      boost::algorithm::trim(substr);                  // Remove ALL whitespaces
      //cout << substr << endl;
      str_in.erase(0, pos + delimiter.length());
  }

  // We should now have the last complete string before the last delimiter
  str_out = substr.c_str();
}



/* Takes in a comma-separated string "x,y,yaw" and turns this into a standard
 * ROS odometry message formatted according to this documentation:
 * http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
 */
void ParseTrimblePrecisionMessage (string input_ta_msg,
                                   string& latest_goal,
                                   string& latest_position,
                                   string& latest_orientation)
{
  /* TODO:
   * Check this is a position measurement and not a command message. For now the
   * code implicitly assumes it is a measurement and contains floats in the
   * format "x,y,yaw"
   */

  //ROS_INFO("Received position from Trimble Access: %s", input_ta_msg.c_str());

  // split the message from one string into a vector of floats
  vector<float> split_message;
  string last_ta_msg;
  //cout << "input_ta_msg: " << input_ta_msg << endl;
  GetLastCompleteMessage(input_ta_msg, last_ta_msg);
  //cout << processed_ta_msg << endl;
  SplitStringOfNumbers(last_ta_msg, split_message);
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
  ros::init(argc, argv, "trimble_precison_listener");
  ROS_INFO("Setting up trimble_precision_listener");
  ros::NodeHandle nh;
  ros::Rate loop_rate(_loop_rate);

  // Set up various containers for ROS messages
  string latest_tp_message, latest_goal, latest_position, latest_orientation,
         previous_position, previous_goal, previous_orientation;

  // Test the TCP client by pinging google (i.e. uncomment to debug tcp comms)
  //TestTcpClientWithGoogle();

  // Create the TCP socket to listen to positions from Trimble Access
  // Note: the "conn" function will hang if no connection can be made
  tcp_client trimble_tcp_client;
  trimble_tcp_client.conn(IP_LAPTOP, PORT_INPUT);

  // Enter into a loop, consistently polling TA for positions, only exit
  // when "Ctrl + C" is pressed
  while (ros::ok())
  {
    // Get a message from Trimble Precision
    latest_tp_message.clear();
    latest_tp_message = trimble_tcp_client.receive(BUFF_LENGTH);
    //ROS_INFO("Received TP message: %s", latest_tp_message);

    // Parse the string to extract the relevent messages
    ParseTrimblePrecisionMessage(latest_tp_message, latest_goal,
                                 latest_position, latest_orientation);

    // Publish the latest messages IF AND ONLY IF the are new
    if (latest_goal != previous_goal)
    {
      // Publish goal
      previous_goal = latest_goal;
    }
    if (latest_position != previous_position)
    {
      // Publish position
      previous_position = latest_position;
    }
    if (latest_orientation != previous_orientation)
    {
      // Publish orientation
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
