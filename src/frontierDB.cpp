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

#include "fast_frontier_detection/frontierDB.h"
#include "math/math_util.h" 
#include "ros/ros_helpers.h"
#include <sensor_msgs/PointCloud.h>
//#include "geometry_msgs/Point32.h"
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
using FFD::Frontier;
using FFD::Frontier_Vector;
using FFD::FrontierDB;

namespace FFD{

FrontierDB::FrontierDB(ros::NodeHandle* n) :
    frontier_DB_(),
    new_frontiers_()
    {
        frontier_pub_ = n->advertise<sensor_msgs::PointCloud> ("latest_frontiers", 1);
    }

void FrontierDB::ExtractNewFrontier(Contour& c, const nav_msgs::OccupancyGrid& g){
    const sensor_msgs::PointCloud contour = c.GetContour();
    bool last_pt_frontier = false;
    Frontier f;
    f.msg.header.frame_id = "/map";

    for (auto& point : contour.points)
    {
        const float x = point.x;
        const float y = point.y;
        const int x_cell = ((x - g.info.origin.position.x) / g.info.resolution);
        const int y_cell = ((y - g.info.origin.position.y) / g.info.resolution);
        
        //Remove extreme points from frontier 
        if( !isinf(x) && !isinf(y) && !isnan(x) && !isnan(y)  ) 
        {
            const bool is_frontier = IsCellFrontier(g,x_cell,y_cell);

        //Start populating a new frontier
            if (last_pt_frontier == false && is_frontier)
            { 
                f.msg.points.push_back(point);
                last_pt_frontier = true;
            }
        //Continue populating a current frontier
            else if(last_pt_frontier == true && is_frontier)
            {
                
                f.msg.points.push_back(point);
            }
            //Save populated frontier, clear and look for new starting point of a new frontier
            else if(last_pt_frontier == true && !is_frontier && !FrontierIsEmpty(f) )
            {
                new_frontiers_.frontiers.push_back(f);
                last_pt_frontier = false;
                f.msg.points.clear();
            }
        }
    }

    // Combine all frontier pointclouds for visualization. 
    Frontier viz_frontiers;
    viz_frontiers.msg.header.frame_id = "/map";
    
    for (auto& frontier:new_frontiers_.frontiers)
    {
        for (auto& point:frontier.msg.points)
        {
            viz_frontiers.msg.points.push_back(point);
        }
    }

    frontier_pub_.publish(viz_frontiers.msg);
    viz_frontiers.msg.points.clear();
    
    return;
}

bool FrontierDB::IsCellFrontier(const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell)
{
    const int x_upper = g.info.width ;
    const int y_upper = g.info.height;

    //Is center cell in 3x3 unknown space? -1
    if(g.data[x_cell+y_cell*g.info.width] == 0) 
    {
          // Find all surrounding cells by adding +/- 1 to col and row 
        for ( int col = x_cell-1; col <= x_cell+1; ++col)
        {
            for ( int row = y_cell-1; row <= y_cell+1; ++row)
            { 

                // If the cell given is not center and its within the grid.
                if ( !((col == x_cell) && (row == y_cell)) && InGrid(g,col, row) )
                {
                    int map_loc = col+row*g.info.width;
                    // Is any one of the surrounding cells open space? 0
                    if( col != x_cell && row != y_cell && g.data[map_loc] == -1)
                    {
                        return true;
                    }
                }
            }
        }
    } 

return false;
}

bool FrontierDB::InGrid( const nav_msgs::OccupancyGrid& g ,const int col, const int row  ) const
{

    const int rows = g.info.width ; 
    const int cols = g.info.height;

    if( row < 0 || col < 0 || row >= rows ||col >= cols) 
    {
      return false;  
    }
  
 return true;
}


void FrontierDB::MaintainFrontiers( const Contour& c, const nav_msgs::OccupancyGrid& graph)
{
    //Get active area
    std::vector<float> active_area = c.GetActiveArea(); //Xmin,Xmax,Ymin,Ymax // 

    // Remove frontierDB points in the active area
    
    // Check if between x min and x max and if beween y min and y max
    for(unsigned int x = active_area[0]; x<=active_area[1]; ++x)
    {
       
        for(unsigned int y = active_area[2]; y<active_area[3]; ++y)
        {

            // Calulate cell position.
            const int x_cell = ((x - graph.info.origin.position.x) / graph.info.resolution);
            const int y_cell = ((y - graph.info.origin.position.y) / graph.info.resolution);
            
            if ( IsCellFrontier(graph,x_cell,y_cell) )
            {   
                // split the current frontier into two partial frontiers
                int Enables_f = -1;
                int Enables_p = -1;

                for (int i = 0; i < frontier_DB_.frontiers.size(); ++i)
                {
                    for (int j = 0; j < frontier_DB_.frontiers[i].msg.points.size(); ++j) 
                    {
                        
                        if( frontier_DB_.frontiers[i].msg.points[j].x == x && frontier_DB_.frontiers[i].msg.points[j].y == y )
                        {
                            Enables_f = i;
                            Enables_p = j;
                        }
                    }//j
                }//i
                
                if(Enables_f == -1 || Enables_p == -1)
                    continue;

                Frontier f1;
                Frontier f2;
                
                //remove f from frontiersDB, add f1 f2 to frontiersDB
                for( int i=0; i<=Enables_p; i++)
                {
                    f1.msg.points.push_back( frontier_DB_.frontiers[Enables_f].msg.points[i] );
                }
                
                for( int i=Enables_p+1; i<frontier_DB_.frontiers[Enables_f].msg.points.size(); i++)
                {
                    f2.msg.points.push_back( frontier_DB_.frontiers[Enables_f].msg.points[i] );
                }
                
                frontier_DB_.frontiers.erase( frontier_DB_.frontiers.begin() + Enables_f );
            }//if p is a frontier
        }//for x
    }//for y
    

    // Update frontier_goal_choices pts
    frontier_goals.clear();
    float sum_x = 0.0;
    float sum_y = 0.0;
   
    for( const auto& frontier:new_frontiers_.frontiers)  // Dont use auto 
    {
        
        for ( const auto& point:frontier.msg.points)
        { 
             if ( isfinite(point.x) && isfinite(point.y) && !isnan(point.x) && !isnan(point.y))
            {
                vector<float> frontier_average_pt = {point.x,point.y};
                frontier_goals.push_back(frontier_average_pt);

            }
        }   
    }

    // Merge New Frontiers with existing frontiers. 
    MergeFrontiers();
 
    // Clear the new frontiers db
    ClearNewFrontier();

}

 
bool FrontierDB::FrontierIsEmpty (const Frontier &frontier ) const
{   
   return frontier.msg.points.empty();
}

void FrontierDB::ClearNewFrontier()
{
    
    for( unsigned int i=0; i< frontier_DB_.frontiers.size(); i++)
   {
       if( frontier_DB_.frontiers[i].msg.points.size() == 0 )
       {
           frontier_DB_.frontiers.erase(frontier_DB_.frontiers.begin() + i);
       }
   } 

return;
}

void FrontierDB::MergeFrontiers()
{
    //Storing new detected frontiers
    int ActiveArea[ 1000 ][ 1000 ];
    for (unsigned int i = 0; i < frontier_DB_.frontiers.size(); ++i)
    {
      for (unsigned int j = 0; j < frontier_DB_.frontiers[i].msg.points.size(); ++j)
      {
          int x = frontier_DB_.frontiers[i].msg.points[j].x;
          int y = frontier_DB_.frontiers[i].msg.points[j].y;
          ActiveArea[x][y] = i;
      }
    }
 
    for( unsigned int i=0; i<new_frontiers_.frontiers.size(); ++i)
    {
        Frontier f = new_frontiers_.frontiers[i];
        bool overlap = 0;
 
        for(unsigned int j=0; j< f.msg.points.size(); ++j)
        {
           int x = f.msg.points[j].x;
           int y = f.msg.points[j].y;
          
           if( ActiveArea[x][y] != 0 ) //overlap
           {
               int exists = ActiveArea[x][y];
               //merge f and exists
               for( int merged=0; merged<f.msg.points.size(); ++merged)
               {
                   frontier_DB_.frontiers[exists].msg.points.push_back(f.msg.points[merged]); 
               }
 
               new_frontiers_.frontiers[i].msg.points.clear(); 
      
               overlap = 1;
               break;
           }
        }//for j
 
        if ( overlap == 0)
        {
            frontier_DB_.frontiers.push_back(f);
        }
 
    } //for i

}

int FrontierDB::UpdateClosestFrontierAverage( Contour& c )
{

    std::vector<float> robot_pos =  c.GetRobotPosition();
    

    float distance_to_pt = 0.0;
    int frontier_i = 0;
    float closest_frontier_distance = 100000.0;

    for(int i = 0; i< frontier_goals.size(); i++ )
    {   
            distance_to_pt = sqrt(pow(frontier_goals[i][0]-robot_pos[0],2) + pow(frontier_goals[i][1]-robot_pos[1],2));
                
            //Get shortest distance to point. 
            if ( distance_to_pt  > 0.6 && distance_to_pt <= closest_frontier_distance ) 
            {
                //set goal and update distance. 
                closest_frontier_distance  = distance_to_pt;
                frontier_i  = i;
            }
        
        ROS_INFO("Closest frontier distance: %f", closest_frontier_distance);
        
        return frontier_i;    
    } 

}


visualization_msgs::Marker FrontierDB::PublishNavGoal( move_base_msgs::MoveBaseGoal goal_msg )
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = goal_msg.target_pose.header.frame_id;
    marker.header.stamp = goal_msg.target_pose.header.stamp;
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal_msg.target_pose.pose.position.x;
    marker.pose.position.y = goal_msg.target_pose.pose.position.y;
    marker.pose.position.z = goal_msg.target_pose.pose.position.z;
    marker.pose.orientation.x = goal_msg.target_pose.pose.orientation.x;
    marker.pose.orientation.y = goal_msg.target_pose.pose.orientation.y;
    marker.pose.orientation.z = goal_msg.target_pose.pose.orientation.z;
    marker.pose.orientation.w = goal_msg.target_pose.pose.orientation.w;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;
}

} // End of FrontierDB Class 

    
