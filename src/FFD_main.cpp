
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <sstream>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <cstdlib> 
#include <ctime> 
#include "FFD.h"

#include "../laser_geometry-kinetic-devel/include/laser_geometry/laser_geometry.h"

using std::vector;
using Eigen::Vector2f;
using FFD::Contour;
using FFD::FrontierDB;
using sensor_msgs::convertPointCloud2ToPointCloud;
using sensor_msgs::convertPointCloudToPointCloud2;


laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud laser_in_map;
sensor_msgs::PointCloud2 laser_in_map_pc2;
geometry_msgs::TransformStamped robot_transform;

Contour* contour;
FrontierDB* f_database;
nav_msgs::Odometry odom_msg;
nav_msgs::OccupancyGrid global_map;
geometry_msgs::PoseStamped goal_msg;
visualization_msgs::Marker marker;
tf2_ros::TransformListener* listener; // Odom listener
tf::TransformListener* listener2; // Laser listener
tf2_ros::Buffer* tfBuffer_;
int frontier_i;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool initial_map_FLAG = false; 

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){

    try{
        odom_msg = *msg;
        ROS_INFO("ODOM: [%f]", (*msg).header.stamp.toSec());
    }
    catch(ros::Exception &e ){
        ROS_ERROR("Error occured: %s ", e.what());
    }

}

void SendGoaltoMoveBase(int frontier_i ){

//////         Handle Movebase Action       //////

    //End exploration if there are no more frontiers. 
    if(f_database->frontier_goals.size() == 0)
			return;
    
    // Fill out move base msg publisher.
    move_base_msgs::MoveBaseGoal goal_pub;
	goal_pub.target_pose.header.frame_id = "map";
	goal_pub.target_pose.header.stamp = ros::Time::now();
	
     // If timeout or no action pick random frontier goal. 
    bool at_target = false;
	int attempts = 0;	
    while(!at_target && attempts < 2) 
    {
		if(attempts >= 0)
        {
			frontier_i = (rand() % f_database->frontier_goals.size());
			at_target = false;
		}
			attempts++;
    }

    goal_pub.target_pose.pose.position.x = f_database->frontier_goals[frontier_i][0] - 0.1;
    goal_pub.target_pose.pose.position.y = f_database->frontier_goals[frontier_i][1];
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    goal_pub.target_pose.pose.orientation = odom_quat;
    ROS_INFO("Navigating to: x: %f y: %f", goal_pub.target_pose.pose.position.x, goal_pub.target_pose.pose.position.y);
    
    // Initaiate move base client
    MoveBaseClient ac("move_base", true);
    
    //If move base server not up wait untill it is. 
    while(!ac.waitForServer(ros::Duration(15.0)))
    {
				ROS_INFO("Is the move_base action server up? ");
	}
    
    //Publishes nav goal marker.
    ac.sendGoal(goal_pub);
    marker = f_database->PublishNavGoal(goal_pub);
	
    //Time to wait for robot to travel to goal before setting new path. 
    ac.waitForResult(ros::Duration(10.0));
    
    //If the robot reaches given goal, rotate in circle and publush new goal. 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
		at_target = true;
		ROS_INFO("The base moved to %f,%f", goal_pub.target_pose.pose.position.x, goal_pub.target_pose.pose.position.y);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(3.14);
		goal_pub.target_pose.pose.orientation = odom_quat;
		marker = f_database->PublishNavGoal(goal_pub);
        ac.sendGoal(goal_pub);
		ac.waitForResult();
	}

}

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    
    if(initial_map_FLAG == true)
    {
    
        // Transform laserscan to pointcloud. 
        ros::Time t = ros::Time(0); 
        listener2->waitForTransform("/scan", "/map", t, ros::Duration(3));
        projector_.transformLaserScanToPointCloud("map",*msg,laser_in_map,*listener2);
        
        // Get transform of robot pose to map frame. 
        robot_transform = tfBuffer_->lookupTransform("base_link","map",ros::Time::now(),ros::Duration(3));
        
         //////         FFD         //////
    
        // Generate a list of contour points (set resolution of line) from laser scan points
        contour->GenerateContour( laser_in_map );
    
        // Return active area as ------- from current contour
        contour->UpdateActiveArea( odom_msg, laser_in_map, robot_transform );
    
        // Appends new frontiers from contour
        f_database->ExtractNewFrontier(*contour, global_map);
        
        f_database->MaintainFrontiers(*contour, global_map); 

        // Frontier is a list of points, returns the index of the frontier with shortest distace with respect to the robot position. 
        frontier_i = f_database->UpdateClosestFrontierAverage(*contour);
        SendGoaltoMoveBase(frontier_i);

    ROS_INFO("LASER: [%f]", (*msg).header.stamp.toSec());
    
    }
}

void OccupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    if (initial_map_FLAG == false)
    {
        initial_map_FLAG = true;
    }
    
    try{
        
        global_map = *msg;
        ROS_INFO("Occupancy: [%f]", (*msg).header.stamp.toSec());
    }
    
    catch(ros::Exception &e ){
        ROS_ERROR("Error occured: %s ", e.what());
    }
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "FFD");
    ros::NodeHandle n;
    contour = new Contour(&n);
    f_database = new FrontierDB(&n);
    
    tfBuffer_ = new tf2_ros::Buffer; 
    listener = new tf2_ros::TransformListener(*tfBuffer_);
    listener2 = new tf::TransformListener();
    

    ros::Subscriber odom_sub = n.subscribe("/odom", 1, OdomCallback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, LaserCallback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, OccupancyMapCallback);
    //ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "waypoint_marker", 0 );
    ros::Rate loop_rate(10);
    
    while (ros::ok()){
        
        ros::spinOnce();
        //Publish Calculated Goal Message to Rviz
        //goal_pub.publish(goal_msg);
        vis_pub.publish( marker );
        loop_rate.sleep();
    }
    delete contour;
    return 0;

}
