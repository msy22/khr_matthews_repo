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
#define BUFF_LENGTH

// Globals______________________________________________________________________
const int _port_num = 2000;
const int _loop_rate = 1;       // rate for the main loop to cycle through in Hz


// Code_________________________________________________________________________


//void ParseTrimbleAccessMessage (string input_ta_msg,
//                                nav_msgs::Odometry& output_odom)
//{

//}


/* Takes in two nav_msgs::Odometry messages and assigns a heading to the
 * second input, based on the difference in position of the two
 *
 */
void CalculateHeadingFromPosition ()
{



  /* TODO
   * Apply a threshold and return
   *
   */
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
  tcp_client trimble_tcp_client;
  trimble_tcp_client.conn(IP_LAPTOP, PORT_INPUT);

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
