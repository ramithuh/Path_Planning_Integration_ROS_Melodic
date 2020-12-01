// Including general libraries
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <set>
#include <pluginlib/class_list_macros.h>

// Including ROS specific libraries
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// To accomodate for moving base
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <angles/angles.h>

// To accomodate sensory input
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

// Navigation messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

// Costmap transform
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// To get costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

// Defining the whole thing in a namespace
using namespace std;
using std::string;

#ifndef PathPlanners_ROS
#define PathPlanners_ROS

struct vertex{  //Stores a vertex along with k1,k2 Costs

    int x,y;
    float k1;
    float k2;
    vertex(int,int,float,float);
    vertex():x(0),y(0),k1(0),k2(0){}
    
};

// Structure to store the cells in a data type
struct cells{
    vertex currentCell;
    float fCost;
};      



namespace PathPlanners_all{
  
class PathPlannersROS : public nav_core::BaseGlobalPlanner {
public:
    PathPlannersROS();
    PathPlannersROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // Overriden classes from interface nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
    
    // Base variables for Planners
    float originX;float originY;float resolution;
    double step_size_, min_dist_from_robot_;
    bool initialized_;
    int width;int height;
    
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    // Base functions for Planners
    void getCoordinate (float& x, float& y);
    void convertToCoordinate(int index, float& x, float& y);
    void mapToWorld(double mx, double my, double& wx, double& wy);
    void add_open(multiset<cells> & OPL, vertex neighborCell, vertex goalCell, float g_score[1000][1000],int n);
   
    bool validate(float x, float y);
    bool isValid(vertex startCell,vertex goalCell); 
    bool isFree(vertex CellID); //returns true if the cell is Free

    vertex convertToCellVertex (float x, float y);
    int    convertToCellIndex (vertex current);
    int getIndex(int i,int j){return (i*width)+j;}
    int getRow(int index){return index/width;}
    int getCol(int index){return index%width;}
    
    float getMoveCost(vertex CellID1, vertex CellID2);

    vector<vertex> getNeighbour (vertex CellID);
    vector<int> PathFinder(vertex startCell, vertex goalCell);
    vector<int> AStar(vertex startCell, vertex goalCell, float g_score[1000][1000]);
    vector<int> Dijkstra(int startCell, int goalCell, float g_score[1000][1000]);
    vector<int> BFS(int startCell, int goalCell, float g_score[1000][1000]);
    vector<int> constructPath(vertex startCell, vertex goalCell, float g_score[1000][1000]);
    
    float heuristic(vertex cellID, vertex goalCell, int n){
        int x1 = goalCell.x;
        int y1 = goalCell.y;
        int x2 = cellID.x;
        int y2 = cellID.y;

        int dx = abs(x2-x1) ; int dy = abs(y2-y1);
        // Manhattan Heuristic
        if(n==1)
            return dx + dy;
        // Euclidean Heuristic
        else if(n==2)
            return sqrt(dx*dx + dy*dy);

        // Chebyshev Heuristic
        else if(n==3)
            return max(dx,dy);

        // Octile Heuristic
        else
            return dx+dy+(sqrt(2)-2)*min (dx,dy);
    }
};
};
#endif
