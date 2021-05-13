///////////////////////////////////////////////////////////////////////////////
//      Title     : FFD.h
//      Project   : fast_frontier_detection
//      Created   : 01/15/21
//      Author    : Adrian Abeyta
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "fast_frontier_detection/contour.h"
#include "math/math_util.h" 
#include "ros/ros_helpers.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>  
#include <numeric>  
#include <cmath>
#include <cfloat>
#include "visualization_msgs/Marker.h"
#include "move_base_msgs/MoveBaseAction.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using nav_msgs::OccupancyGrid;

namespace FFD{

Contour::Contour(ros::NodeHandle* n) :
    resolution_(0.05) 
    {
        contour_.header.frame_id = "/map";
        contour_pub_ = n->advertise<sensor_msgs::PointCloud> ("contour", 1);
        
    }


void Contour::GenerateContour(const sensor_msgs::PointCloud& laser_coordinates){

    contour_.points.clear();

    for (int i = 0; i < laser_coordinates.points.size() - 1; ++i)
    {
        const float point1_x = laser_coordinates.points[i].x;
        const float point1_y = laser_coordinates.points[i].y;
        const float point2_x = laser_coordinates.points[i+1].x;
        const float point2_y = laser_coordinates.points[i+1].y;

        line2f segment(point1_x,point1_y,point2_x,point2_y);
        SampleLine(segment);
    }
    contour_pub_.publish(contour_);
    return;
}

void Contour::SampleLine(const line2f line){

    const float delta_x = line.p1.x() - line.p0.x();
    const float delta_y = line.p1.y() - line.p0.y();

    const float x_range = fabs(delta_x);
    const float y_range = fabs(delta_y); 

    const float line_length = sqrt(x_range*x_range + y_range*y_range);
    const float line_slope = (delta_y/delta_x);
    
    // X step is always positive because of the square root, if statement later accounts for it.
    const float x_step = sqrt((resolution_*resolution_)/( 1 + (line_slope*line_slope) )); 
    
    for (int i = 0; i*x_step<x_range; ++i)
    {
        geometry_msgs::Point32 point;
        
        if(line.p0.x() > line.p1.x())
        {

            point.x = line.p0.x() - i*x_step;  // Create i*x_step variable 
            point.y = line.p0.y() - i*x_step*line_slope;
        }
        else
        {
            point.x = line.p0.x() + i*x_step;  // Create i*x_step variable 
            point.y = line.p0.y() + i*x_step*line_slope;
        }

        point.z = 0.00;
        
        contour_.points.push_back(point);
    }
    return;
}

void Contour::UpdateActiveArea( const nav_msgs::Odometry& msg , const sensor_msgs::PointCloud& laser_coordinates,  geometry_msgs::TransformStamped robot_transform )
{
    
    // Calulate robot pose in map frame given transformation.
    const float robot_x = robot_transform.transform.translation.x; 
    const float robot_y = robot_transform.transform.translation.y;
    
    //Update Private Variable
    robot_pos_ = { robot_x, robot_y };

    float xmin = laser_coordinates.points[0].x;
    float xmax = laser_coordinates.points[0].x;
    float ymin = laser_coordinates.points[0].y;
    float ymax = laser_coordinates.points[0].y;

     //Find max and min laser values from robot frame.
     for ( const auto& point: laser_coordinates.points )
     {
         xmax = std::max(xmax,point.x);
         ymax = std::max(ymax,point.y);
         xmin = std::min(xmin,point.x);
         ymin = std::min(ymin,point.y);
     }

    //  Clear old area add new one
     active_area_.clear();
     active_area_.push_back(xmin);
     active_area_.push_back(xmax);
     active_area_.push_back(ymin);
     active_area_.push_back(ymax);

}


sensor_msgs::PointCloud Contour::GetContour()const{
    return contour_;
}
std::vector<float> Contour::GetActiveArea()const{
    return active_area_;
}
std::vector<float> Contour::GetRobotPosition()const{

   return robot_pos_;
}

} // End of FrontierDB Class 

    
