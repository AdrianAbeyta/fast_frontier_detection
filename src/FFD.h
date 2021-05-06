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
#include "../shared/math/geometry.h"
#include "../shared/math/line2d.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "move_base_msgs/MoveBaseAction.h"
using nav_msgs::OccupancyGrid;


#ifndef CONTOURS_H
#define CONTOURS_H

namespace ros {
  class NodeHandle;
}

namespace FFD{

    // Describes a frontier
    struct frontier {
        sensor_msgs::PointCloud msg;
    };
    // Holds a vector of frontiers for processing
    struct frontier_vector{
        std::vector<frontier> frontiers;
    };


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
        sensor_msgs::PointCloud GetContour();
        //Returns bounds for active area
        std::vector<float> GetActiveArea();
        //Returns robot position
        std::vector<float> GetRobotPosition();
      
      private:
        // Robot position
        std::vector<float> robot_pos_;
        sensor_msgs::PointCloud contour_; //Only one contour in the entire program
        const float resolution_; //m : line sampling
        std::vector<float> active_area_; // order of values is (xmin, xmax, ymin, ymax)
    };

    class FrontierDB {
      public:
        
        // Default contructor
        FrontierDB( ros::NodeHandle* n );

        // Appends new frontiers from contour
        void ExtractNewFrontier( Contour& c, const nav_msgs::OccupancyGrid& g );
        bool IsCellFrontier( const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell );
        bool InGrid(const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell  ) const;
        
        // Performs maintenance step
        void MaintainFrontiers( Contour& c , const nav_msgs::OccupancyGrid& graph);
        
        // Clears the new frontiers db. 
        void ClearNewFrontier();
        bool FrontierIsEmpty(frontier frontier);
        
        // Combines the new and past frontier points within the active area. 
        void MergeFrontiers();
        
        // Frontier is a list of points, the robot goal is the average of the frontiers points. The closest average is the frontier average to go. 
        int UpdateClosestFrontierAverage(Contour& c);

        // Vizualize waypoint.
        visualization_msgs::Marker PublishNavGoal( move_base_msgs::MoveBaseGoal goal_msg );

        // Inialized value for determining closest frontier waypoint from froniters list. 
        float closest_frontier_distance_ = 100000.0;
        
        // Current frontier points 
        std::vector<std::vector<float>> frontier_goals; 
        
        private:
        
        //Current list of frontiers.
        frontier_vector frontier_DB;
        
        //Current list of newly detected frontiers. 
        frontier_vector new_frontiers;
        
    };

}
#endif
