// Includes_____________________________________________________________________
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>
#include <string>                             // stod
#include <cstdio>
#include <cstring>

// Setup namespaces_____________________________________________________________
using namespace std;


typedef basic_ifstream<char> ifstream;


// Global Constants_____________________________________________________________
string _default_waypoint_filepath = "/home/matt/KHR_matthews_repo";
string _default_waypoint_filename = "testWaypoints.csv";
bool _tool_actuated = false;
bool _job_completed = false;


// Code_________________________________________________________________________


class Waypoint
{
  public:
    // Identifying details
    int id;
    string name;
    double time_stamp;

    // Coordinates, both measured UTM coordinates and XYZ equivalent in map frame
    double x;
    double y;
    double z;
    double yaw;
};


// Makes two fake waypoints directly in front of the robot to test move_base
//void CreateDummyWaypoints (vector<Waypoint>& waypoint_list_to_fill)
//{
//  Waypoint wp1 (new Waypoint());
//  Waypoint wp2 (new Waypoint());
//  wp1.id = 1; wp1.x = 1;
//  wp2.id = 2; wp2.x = 2;
//}


// This callback works like an interrupt, and will be executed every time the
// /tool_actuated rostopic is updated
void CallbackToolActuated (std_msgs::Bool tool_msg)
{
  _tool_actuated = tool_msg.data;
}



// This callback works like an interrupt, and will be executed every time the
// /job_completed rostopic is updated
void CallbackJobCompleted(std_msgs::Bool job_msg)
{
  _job_completed = job_msg.data;
}



//void LoadWaypointList (vector<Waypoint>& waypoint_list_to_fill,
//                       string filepath_to_waypoints)
//{
//    ROS_INFO("Loading waypoints from: %s", filepath_to_waypoints.c_str());
//    const int x_id = 2;
//    const int y_id = 3;
//    const int yaw = 4;
//    const int tStamp_id = 1;

//    ifstream in(filepath_to_waypoints);
//    if (!in.is_open())
//    {
//        ROS_ERROR("Error loading file\n");
//        return -1;
//    }

//    int n_lines = 0;
//    string line;
//    vector<string> vec;

//    //Always get the line -> so while advances
//    getline(in, line);
//    //Tokenize the line
//    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
//    boost::char_separator<char> sep(s.c_str());
//    tokenizer tokens(line, sep);

//    //Assign tokens to a string vector
//    vec.clear();
//    vec.assign(tokens.begin(), tokens.end());

//    cout << vec.size()<<endl;
//    cout << line << endl;

//    while (in.eof() != 1 )
//    {

//        float X, Y, yaw;
//        float tStamp;

//        getline(in, line);

//            //cout << n_scan<<',' <<(int) atof(vec[scan_id-1].c_str())<<','<< in.eof()  << endl;
//        //void lines could be accientaly present, in case continue
//        if (line.size() == 0)
//        {
//            cout<<"void line"<<endl;
//            continue;
//        }

//        //Tokenize the line
//        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
//        boost::char_separator<char> sep(s.c_str());
//        tokenizer tokens(line, sep);

//        //Assign tokens to a string vector
//        vec.clear();
//        vec.assign(tokens.begin(), tokens.end());

//        Waypoint newPoint;
//        //Create the point
//        newPoint.x = atof(vec[x_id-1].c_str());
//        newPoint.y = atof(vec[y_id-1].c_str());
//        newPoint.yaw = atof(vec[yaw_id-1].c_str());
//        tStamp = atof(vec[tStamp_id-1].c_str());
//        waypoint_list_to_fill(newPoint);
//        cout<<"timeNow"<<tStamp<<endl;
//    }
//}

//void NavigateRobotToWaypoints (vector<Waypoint>& waypoint_list,
//                               MoveBaseClient& ac,
//                               ros::Publisher& tool_publisher)
//{
//  // Iterate through waypoints in list, one by one
//  move_base_msgs::MoveBaseGoal goal;
//  for (int i=0; i < waypoints_list.size(); i++)
//  {
//    // Create move goal
//    wp = waypoints_list[i];
//    goal.target_pose.header.frame_id = "map";
//    goal.target_pose.header.stamp = ros::Time::now();
//    goal.target_pose.pose.position.x = wp.x;
//    goal.target_pose.pose.position.y = wp.y;
//    goal.target_pose.pose.orientation.w = 1.0;

//    // Send goal to move_base
//    ROS_INFO("Sending goal " + wp.id);
//    ac.sendGoal(goal);

//    // Wait for the robot to get there then do something
//    ac.waitForResult();
//    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//    {
//      ROS_INFO("Hooray, successfully reached waypoint " + wp.id);
//      // Now send the signal to actuate the tool
//      std_msgs::Bool actuate_tool; actuate_tool.data = true;
//      tool_publisher.publish(actuate_tool);
//      /* TODO
//       * Now maybe wait for the bool to be set to false by node 3
//       * (tool_actuator) before continuing to the next waypoint
//       *
//       * Proceed IAOI /tool_actuated == False
//       */
//    }
//    else
//    {
//      ROS_INFO("Failed to reach waypoint " + wp.id + " for some reason");
//      /* TODO
//       * Deal with the failure appropriately (re-send or move to next)
//       */
//    }
//  }
//}



int main(int argc, char **argv)
{
  // Setup ROS node, including publishers and subscribers
  ROS_INFO("Setting up robot_navigator node...");
  ros::init(argc, argv, "robot_navigator");
  ros::NodeHandle nh;
  ros::Publisher tool_publisher   = nh.advertise<std_msgs::Bool>("tool_actuated", 10);
  ros::Publisher job_publisher    = nh.advertise<std_msgs::Bool>("job_completed", 10);
  ros::Subscriber tool_subscriber = nh.subscribe("tool_actuated", 10, CallbackToolActuated);
  ros::Subscriber job_subscriber  = nh.subscribe("job_completed", 10, CallbackJobCompleted);

  // Read waypoints into a list
  vector<Waypoint> waypoints_list;
//  LoadWaypointList(waypoints_list, _default_waypoint_filepath + _default_waypoint_filename);

//  // Set up SimpleActionServer/Client
//  MoveBaseClient ac("move_base", true);
//  while(!ac.waitForServer(ros::Duration(10.0)))    // Waits 10s before giving up
//  {
//    ROS_INFO("Waiting for the move_base action server to come up");
//  }

//  // Iterate through waypoints in list, one by one
//  NavigateRobotToWaypoints(waypoints_list, ac, tool_publisher);

//  // Report success
//  _job_completed = true;
//  std_msgs::Bool job_msg;
//  job_msg.data = _job_completed;
//  job_publisher.publish(job_msg);

  // Return to home (origin frame)
  /* TODO
   * Send a move goal to make the robot move back to the start of the waypoint
   * list
   */

  // have this here just to check the callbacks are cycling
  ROS_INFO("Spinning until Ctrl+C'd");
  ros::spin();

  ROS_INFO("Yay, reached the end of the robots trajectory, job done!");
  return 0;
}
