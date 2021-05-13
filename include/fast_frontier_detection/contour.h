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
#include <vector>
#include <list>
#include "sensor_msgs/PointCloud.h"
#include "eigen3/Eigen/Dense"
#include "math/geometry.h"
#include "math/line2d.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "ros/ros_helpers.h"
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"

#ifndef CONTOURS_H
#define CONTOURS_H

namespace ros {
  class NodeHandle;
}

namespace FFD{

    class Contour {
      public:
       
        //Default Constructor
        Contour( ros::NodeHandle* n );
        
        //Generate a list of contour points (set resolution of line) from laser scan points
        void GenerateContour( const sensor_msgs::PointCloud& laser_coordinates );
        
        //Generate a vector of points sampled from line and appends to contour
        void SampleLine( const geometry::line2f line );
        
        //Return active area from current contour
        void UpdateActiveArea( const nav_msgs::Odometry& msg , const sensor_msgs::PointCloud& laser_coordinates,  geometry_msgs::TransformStamped robot_transform );
        
        //Returns contour data
        sensor_msgs::PointCloud GetContour()const;
        
        //Returns bounds for active area
        std::vector<float> GetActiveArea()const;
        
        //Returns robot position
        std::vector<float> GetRobotPosition()const;
      
      private:
        
        // Robot position
        std::vector<float> robot_pos_;
        
        sensor_msgs::PointCloud contour_; //Only one contour in the entire program
        
        const float resolution_; //m : line sampling
        
         std::vector<float> active_area_;

        ros::Publisher contour_pub_; 
        
        
    };

}
#endif