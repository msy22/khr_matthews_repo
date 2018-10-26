// Includes_____________________________________________________________________
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <string>                             // stod
#include <cstdio>
#include <cstring>


//extra includes
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <string>
#include <iostream>
//#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud_conversion.h>
//#include <pcl_ros/transforms.h>

//#include <pcl/io/ply_io.h>
//#include <pcl/io/pcd_io.h>

// Setup namespaces_____________________________________________________________
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Global Constants_____________________________________________________________
const string _default_waypoint_filepath = "./src/robot_navigator/";
//const std::string &in_filename = {"/catkin_ws/LOAM_ROS_testing/move.csv"};
const string _default_waypoint_filename = "wayPoints.csv";
bool _tool_actuated = false;
bool _job_completed = false;


// Code_________________________________________________________________________


class Waypoint
{
  public:
    // Identifying details
    int id;
    string name;

    // Coordinates, both measured UTM coordinates and XYZ equivalent in map frame
    double x;
    double y;
    double yaw;
};



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



void LoadWaypointList (vector<Waypoint>& waypoint_list_to_fill, string filepath_to_waypoints)
{


    const int x_id = 2;
    const int y_id = 3;
    const int yaw_id = 4;
    const int tStamp_id = 1;
    cout<<filepath_to_waypoints<<endl;
    ifstream in(filepath_to_waypoints.c_str());
    if (!in.is_open())
    {
        cout<<"Error loading file\n";
        //return -1;
    }

    int n_lines = 0;

    string line;
    vector< string > vec;

    //Always get the line -> so while advances
    getline(in, line);
    cout << line << endl;
    getchar();
    while (in.eof() != 1 )
    {

        float X, Y, yaw;
        float tStamp;
        Waypoint newPoint;
        //Create the point
        cout<<line<<endl;
        getline(in,line,',');
        tStamp = atof(line.c_str());//atof(vec[tStamp_id-1].c_str());
        getline(in,line,',');
        newPoint.x = atof(line.c_str());//atof(vec[x_id-1].c_str());
        getline(in,line,',') ;
        newPoint.y = atof(line.c_str());//atof(vec[y_id-1].c_str());
        getline(in,line);
        newPoint.yaw = atof(line.c_str());//atof(vec[yaw_id-1].c_str());

        waypoint_list_to_fill.push_back(newPoint);
        cout<<"timeNow : "<<tStamp<<", X : "<< newPoint.x<<", Y : "<<newPoint.y<<", yaw : "<<newPoint.yaw<<endl;

    }
}



void NavigateRobotToWaypoints (vector<Waypoint>& waypoints_list,
                               MoveBaseClient& ac,
                               ros::Publisher& tool_publisher)
{
  // Iterate through waypoints in list, one by one
  move_base_msgs::MoveBaseGoal goal;
  for (int i=0; i < waypoints_list.size(); i++)
  {
    // Create move goal
    Waypoint wp;
    wp = waypoints_list[i];
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = wp.x;
    goal.target_pose.pose.position.y = wp.y;

    tf::Quaternion goal_quat = tf::createQuaternionFromRPY(0,0,wp.yaw);
    goal.target_pose.pose.orientation.x = goal_quat[0];
    goal.target_pose.pose.orientation.y = goal_quat[1];
    goal.target_pose.pose.orientation.z = goal_quat[2];
    goal.target_pose.pose.orientation.w = goal_quat[3];

    // Send goal to move_base
    //ROS_INFO("Sending goal " + wp.id);
    ac.sendGoal(goal);

    // Wait for the robot to get there then do something
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      //ROS_INFO("Hooray, successfully reached waypoint " + wp.id);
      // Now send the signal to actuate the tool
      std_msgs::Bool actuate_tool; actuate_tool.data = true;
      tool_publisher.publish(actuate_tool);
      /* TODO
       * Now maybe wait for the bool to be set to false by node 3
       * (tool_actuator) before continuing to the next waypoint
       */
    }
    else
    {
      //ROS_INFO("Failed to reach waypoint " + wp.id + " for some reason");
      /* TODO
       * Deal with the failure appropriately (re-send or move to next)
       */
    }
  }
}



int main(int argc, char **argv)
{
  // Setup ROS node, including publishers and subscribers
  ros::init(argc, argv, "robot_navigator");
  ros::NodeHandle nh("~");
  ros::Publisher tool_publisher   = nh.advertise<std_msgs::Bool>("tool_actuated", 10);
  ros::Publisher job_publisher    = nh.advertise<std_msgs::Bool>("job_completed", 10);
  ros::Subscriber tool_subscriber = nh.subscribe("tool_actuated", 10, CallbackToolActuated);
  ros::Subscriber job_subscriber  = nh.subscribe("job_completed", 10, CallbackJobCompleted);

  // Read waypoints into a list
  vector<Waypoint> waypoints_list;
  LoadWaypointList(waypoints_list, _default_waypoint_filepath + _default_waypoint_filename);

  // Set up SimpleActionServer/Client
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(10.0)))    // Waits 10s before giving up
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //// Iterate through waypoints in list, one by one
  NavigateRobotToWaypoints(waypoints_list, ac, tool_publisher);

  // Report success
  _job_completed = true;
  std_msgs::Bool job_msg;
  job_msg.data = _job_completed;
  job_publisher.publish(job_msg);

  // Return to home (origin frame)
  /* TODO
   * Send a move goal to make the robot move back to the start of the waypoint
   * list
   */

  return 0;
}
