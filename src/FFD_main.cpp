//TODO
//Make main look like navigation_main structure
//Edit function signature so they follow google guidelines 


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/LaserScan.h"
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

tf2_ros::TransformListener* listener; // Odom listener
tf::TransformListener* listener2; // Laser listener
tf2_ros::Buffer* tfBuffer_;

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

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(initial_map_FLAG == true)
    {
    // Transform laserscan to pointcloud. 
    listener2->waitForTransform("/base_laser", "/map", ros::Time::now(), ros::Duration(3.0));
    projector_.transformLaserScanToPointCloud("map",*msg,laser_in_map,*listener2);
    //std::cout << "X = :" << laser_in_map.points[0].x; 
    
    // Get transform of robot pose to map frame. 
    robot_transform = tfBuffer_->lookupTransform("base_link","map",ros::Time::now(),ros::Duration(5.0));
    
    // Generate a list of contour points (set resolution of line) from laser scan points
    contour->GenerateContour( laser_in_map );
    
    // Return active area as ------- from current contour
    contour->UpdateActiveArea( odom_msg, laser_in_map, robot_transform );
    
    // Appends new frontiers from contour
    f_database->ExtractNewFrontier(*contour, global_map); //Somtimes this segfaults need to find out why.... timing issue
    f_database->MaintainFrontiers(*contour, global_map); 
    //f_database->UpdateClosestFrontierAverage(*contour);

    // Publish closest frontier waypoint to robot.
    //vector<float> robot_pos = f_database->GetCalculatedWaypoint(*contour);
    //goal_msg = f_database->PublishClosestFrontierAsNavGoal(robot_pos);
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
    
    tfBuffer_ = new tf2_ros::Buffer ; 
    listener = new tf2_ros::TransformListener(*tfBuffer_);
    listener2 = new tf::TransformListener();
    
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, OdomCallback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, LaserCallback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, OccupancyMapCallback);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Rate loop_rate(10);
    
    while (ros::ok()){
        
        ros::spinOnce();
        //Publish Calculated Goal Message to Rviz
        goal_pub.publish(goal_msg);
        loop_rate.sleep();
    }
    delete contour;
    return 0;

}
