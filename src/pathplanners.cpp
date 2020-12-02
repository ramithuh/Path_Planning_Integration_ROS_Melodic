/*####################*/
// Author => Ramith Hettiarachchi < im@ramith.fyi >
// CSIRO Data61
/*####################*/

#include "pathplanners.h"
#include<iostream>
#include <queue>
#include <limits>
#include <fstream>
#include <cmath>
#include <sstream>
#include <chrono>
#include <thread>
#include <random>

#define FRAMES_PER_SECOND 5

double Inf = INT_MAX;
using namespace std;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds


vertex::vertex(int p_x,int p_y,float p_k1, float p_k2):x{p_x},y{p_y},k1{p_k1},k2{p_k2}{}
//Goal & start
vertex s_goal(25,25,0,0);
vertex s_start(0,0,0,0);
vertex s_last(0,0,0,0);
double km = 0;

//constraints 
int grid_s_x = 50;
int grid_s_y = 50;

#define size_grid_s_x 500
#define size_grid_s_y 500

bool enabled = 0; //autorun

double rhs[size_grid_s_x+1][size_grid_s_y+1];
double   g[size_grid_s_x+1][size_grid_s_y+1];

bool GRID[size_grid_s_x+1][size_grid_s_y+1];
bool PATH[size_grid_s_x+1][size_grid_s_y+1];

bool   Ukey[size_grid_s_x+1][size_grid_s_y+1];
double Ukey_k1[size_grid_s_x+1][size_grid_s_y+1];
double Ukey_k2[size_grid_s_x+1][size_grid_s_y+1];


m_priority_queue U;

queue<vertex> changed_nodes;


PLUGINLIB_EXPORT_CLASS(PathPlanners_all::PathPlannersROS, nav_core::BaseGlobalPlanner);

int mapSize;
bool* OGM;
static const float INFINIT_COST = INT_MAX;
float infinity = std::numeric_limits< float >::infinity();

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end){
	
	timespec temp;
	
	if ((end.tv_nsec-start.tv_nsec)<0){
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	}
	
	else{
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}


inline vector <int> getNeighbour (int CellID);


namespace PathPlanners_all
{
PathPlannersROS::PathPlannersROS(){}
PathPlannersROS::PathPlannersROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros){initialize(name, costmap_ros);}


bool PathPlannersROS::isVertexEqual(vertex v1,vertex v2){
    if(v1.x == v2.x && v1.y == v2.y){
        return 1;
    }
    return 0;
}



void PathPlannersROS::showpq(m_priority_queue gq){
    m_priority_queue g = gq;
    while (!g.empty()) {
        vertex c_v = g.top();

        cout << '\t' <<c_v.x<<","<<c_v.y<<"("<<c_v.k1<<","<<c_v.k2<<")"<<"   " ;
        g.pop();
    }
    cout << '\n';
}

double PathPlannersROS::h(vertex s1,vertex s2){   
    //heuristic function
    return sqrt(pow((s1.x-s2.x),2) + pow((s1.y-s2.y),2));
}

bool PathPlannersROS::isInQueue(vertex s){
    if(Ukey[s.x][s.y]==1){
        return 1;
    }
    return 0;
}

void PathPlannersROS::pushToQueue(vertex s){
    Ukey[s.x][s.y] = 1;
    Ukey_k1[s.x][s.y] = s.k1;
    Ukey_k2[s.x][s.y] = s.k2;
    U.push(s);
}

bool PathPlannersROS::isCostLower(vertex b, vertex a){   
    if(a.k1 > b.k1){
        return 1;
    }else if(a.k1 == b.k1){
        if(a.k2 > b.k2)return 1;
        else return 0;
    }else return 0;
}

vertex PathPlannersROS::CalculateKey(vertex s){
    //cout<<"✨";
    if(s.x < 0 || s.x > grid_s_x || s.y < 0 || s.y > grid_s_y){
        s.k1 = Inf;
        s.k2 = Inf;
        return s;
    }


    double k1  = min(g[s.x][s.y],rhs[s.x][s.y]) + h(s_start,s) + km;
    double k2  = min(g[s.x][s.y],rhs[s.x][s.y]);

    s.k1 = k1;
    s.k2 = k2;
    return s;
}

double PathPlannersROS::edgecost(vertex a,vertex b){
    bool blocked = GRID[a.x][a.y] + GRID[b.x][b.y];

    if(blocked > 0){
        return Inf;
    }else{
        return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
    }

}

double PathPlannersROS::cg_cost(vertex a,vertex b){
    bool blocked = GRID[a.x][a.y] + GRID[b.x][b.y];

    if(blocked > 0){
        return Inf;
    }else{
        double cost =  sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) ) + g[b.x][b.y];
        if(cost >= Inf){
            return Inf;
        }else return cost;
    }

}

void PathPlannersROS::UpdateVertex(vertex u){
    //cout<<"✨";
    if(u.x < 0 || u.x > grid_s_x || u.y < 0 || u.y > grid_s_y){
        return;
    }


    if(!isVertexEqual(u,s_goal)){
        double c1,c2,c3,c4,c5,c6,c7,c8;

        if(u.y+1 > grid_s_y)c1 = Inf;
        else c1 = cg_cost(u,vertex(u.x,u.y+1,0,0));

        if(u.x+1 > grid_s_x)c2 = Inf;
        else c2 = cg_cost(u,vertex(u.x+1,u.y,0,0));

        if(u.y-1 < 0) c3 = Inf;
        else c3 = cg_cost(u,vertex(u.x,u.y-1,0,0));

        if(u.x-1 < 0) c4 = Inf;
        else c4 = cg_cost(u,vertex(u.x-1,u.y,0,0));

        if(u.x-1 < 0 || u.y - 1 < 0) c5 = Inf;
        else c5 = cg_cost(u,vertex(u.x-1,u.y-1,0,0));

        if(u.x-1 < 0 || u.y + 1 > grid_s_y) c6 = Inf;
        else c6 = cg_cost(u,vertex(u.x-1,u.y+1,0,0));

        if(u.x + 1 > grid_s_x || u.y - 1 < 0) c7 = Inf;
        else c7 = cg_cost(u,vertex(u.x+1,u.y-1,0,0));

        if(u.x + 1 > grid_s_x || u.y + 1 > grid_s_y) c8 = Inf;
        else c8 = cg_cost(u,vertex(u.x+1,u.y+1,0,0));

        rhs[u.x][u.y] = min(min(min(c3,c4),min(c1,c2)),min(min(c7,c8),min(c5,c6)));
    }
    u = CalculateKey(u);
    if(isInQueue(u)){
        Ukey[u.x][u.y] = 0; //remove from Priority Queue
    } 
    if(rhs[u.x][u.y]!=g[u.x][u.y]){
        pushToQueue(u);
    }
}

bool PathPlannersROS::isGhost(vertex s){
    if(Ukey[s.x][s.y]==1 && Ukey_k1[s.x][s.y]==s.k1 && Ukey_k2[s.x][s.y]==s.k2){
        return 0;
    }
    return 1;
}

void PathPlannersROS::pop(){
    vertex s = U.top();
    Ukey[s.x][s.y]=0;
    U.pop();
}

vertex PathPlannersROS::TopKey(){
    if(U.size()==0)return vertex(0,0,Inf,Inf);

    vertex temp = U.top();

    while(isGhost(temp)){
        pop(); //pop unwanted ones
        if(U.size()==0) return vertex(0,0,Inf,Inf);
        temp = U.top();
    }
    return temp; //return top most vertex which isn't a ghost
}

void PathPlannersROS::ComputeShortestPath(){

    while(isCostLower(TopKey(),CalculateKey(s_start)) ||
        rhs[s_start.x][s_start.y] != g[s_start.x][s_start.y])
    {

        //cout<<"🍀 => "<<U.size();
        vertex k_old = TopKey(); 
        pop();
        vertex u     = k_old;
        //cout<<" <= Selected "<<u.x<<","<<u.y<<endl;
        //cout<<k_old.k1<<","<<k_old.k2<<endl;

        if(k_old.k1 == Inf ) { //break if path doesn't exist
            enabled = 0;
            return;
        }
        if(isCostLower(k_old,CalculateKey(u))){
            u = CalculateKey(u);
            //cout<<"🔥";
            pushToQueue(u);
        }else if(g[u.x][u.y] > rhs[u.x][u.y]){
            g[u.x][u.y] = rhs[u.x][u.y];
            //cout<<" => g[u.x][u.y] > rhs[u.x][u.y]"<<endl;
            UpdateVertex(vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(vertex(u.x-1 ,u.y  ,0,0));

            UpdateVertex(vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y+1,0,0));
        }else{
            g[u.x][u.y] = Inf;
            //cout<<" => else"<<endl;

            UpdateVertex(vertex(u.x   ,u.y  ,0,0));

            UpdateVertex(vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(vertex(u.x-1 ,u.y  ,0,0));

            UpdateVertex(vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y+1,0,0));
        }



    }

}
void PathPlannersROS::fillGRID(){
    string line;
    ifstream textfile("in.in");

    int i = 0;
    while (getline (textfile, line)) {
        std::stringstream ss(line);

        int j=0;
        for (int x; ss >> x;j++) {
            GRID[j][i]=x;
            if (ss.peek() == ',')
                ss.ignore();
        }
        i++;
    }
    textfile.close();
}

void PathPlannersROS::fillGRID_(bool random=0){

    if(random){
        for(int i=0;i <= grid_s_x;i++)
            for(int j=0;j<= grid_s_y;j++)
                GRID[i][j] = rand() & 1;
    }else{
        for(int i=0;i <= grid_s_x;i++)
            for(int j=0;j<= grid_s_y;j++)
                GRID[i][j] = 0;
    }
    GRID[s_goal.x ][s_goal.y] = 0;
    GRID[s_start.x][s_start.y] = 0;
    
}
void PathPlannersROS::initialize(vertex startCell,vertex goalCell){
    s_start = startCell;
    s_last  = s_start;
    s_goal  = goalCell;

    while(U.size()){
        U.pop();
    }

    km = 0;
    for(int i=0;i <= grid_s_x;i++)
        for(int j=0;j<= grid_s_y;j++){
            rhs[i][j] = Inf;
              g[i][j] = Inf;
            PATH[i][j]= 0;
        }
    rhs[s_goal.x][s_goal.y] = 0;
}

void PathPlannersROS::Traverse(vertex pos){
    if(pos.x < 0 || pos.x > grid_s_x || pos.y < 0 || pos.y > grid_s_y){
      return;
    }
    PATH[pos.x][pos.y] = 1;
}

int PathPlannersROS::indexofSmallestElement(double array[]){
    int index = 0;

    for(int i = 1; i < 8; i++){
        if(array[i] < array[index])
            index = i;              
    }

    return index;
}
int  moves[8][2] = {{-1,0},{0,-1},{1,0},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};

double PathPlannersROS::step_cost(int x,int y){
    if(x < 0 || x > grid_s_x || y < 0 || y > grid_s_y){
        return Inf;
    }else return g[x][y];
}

int PathPlannersROS::onestep(){

    if(s_start.x==s_goal.x && s_start.y==s_goal.y)return -1;

    double arr[8] = {};

    for(int i=0; i<8; i++)
        arr[i] = step_cost(s_start.x + moves[i][0],s_start.y + moves[i][1]) + edgecost(s_start,vertex(s_start.x + moves[i][0],s_start.y + moves[i][1],0,0));
    //cout<<" ✨ "<<s_start.x<<","<<s_start.y<<endl;

    int min_index = indexofSmallestElement(arr); //arg min 

    s_start.x = s_start.x + moves[min_index][0];
    s_start.y = s_start.y + moves[min_index][1];
     
    Traverse(s_start); //move to start

	/*
    //scan graph for changed costs...
    if(changed_nodes.size()>0){ //if any edge costs changed

        km = km + h(s_last,s_start);

        s_last = vertex(s_start.x,s_start.y,0,0);

        while(changed_nodes.size()>0){ //for all directed edges (u,v) with changed edge costs
            vertex current = changed_nodes.front();
            UpdateVertex(current);
            changed_nodes.pop();
        }
       
        ComputeShortestPath();
    }*/
    return 1;


}
vector<int> PathPlannersROS::DStarLite(int startCell,int goalCell){
	

    vertex startCell_(getRow(startCell),getCol(startCell),0,0);
    vertex  goalCell_(getRow(goalCell ),getCol(goalCell),0,0);

	
    initialize(startCell_,goalCell_);
    
    cout<<"Successfully loaded GRID"<<endl;
    s_goal = CalculateKey(s_goal);
    cout<<"going to push to queue...";
    pushToQueue(s_goal);
    cout<<"going to compute shortest path...";
    ComputeShortestPath();

    cout<<"Successfully Computed Cost => \n";

    
    cout<<g[s_start.x][s_start.y]<<endl;

    vector<int> bestPath;
    vector<int> emptyPath;

    if(g[s_start.x][s_start.y]!=Inf){
        while(onestep()>0){
            bestPath.insert(bestPath.begin()+bestPath.size(),getIndex(s_start.x,s_start.y));
        }
    }else{
		cout << "Path not found!" << endl;
		return emptyPath;
    }
	//vector<int> bestPath;
	cout<<"returining path "<<endl;
    return bestPath;
}


void PathPlannersROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	if (!initialized_){
		costmap_ros_ = costmap_ros;
		costmap_ = costmap_ros_->getCostmap();

		originX = costmap_->getOriginX();
		originY = costmap_->getOriginY();

		width = costmap_->getSizeInCellsX();
		height = costmap_->getSizeInCellsY();
		resolution = costmap_->getResolution();
		mapSize = width*height;

		grid_s_x = width; ////
		grid_s_y = height;////

		OGM = new bool [mapSize]; 
		for (unsigned int iy = 0; iy < height; iy++){
			for (unsigned int ix = 0; ix < width; ix++){
				unsigned int cost = static_cast<int>(costmap_->getCost(ix,iy));
				
				if (cost <= 150){
					OGM[iy*width+ix]=true;
					// cout <<"Traversable"<< ix<<","<<iy<<"   cost:"<<cost<<endl;
					GRID[ix][iy] = false; ////
				}

				else{
					OGM[iy*width+ix]=false;
					GRID[ix][iy] = true; ////
					// cout <<"Obstacle"<< ix<<","<<iy<<"   cost:"<<cost<<endl;
				}
			}
		}
		ROS_INFO("Jump Point Search initialized successfully");
		initialized_ = true;
	}
	else
    	ROS_WARN("Planner already initialized");
}



bool PathPlannersROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
	if (!initialized_){
		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
		return false;
	}

	ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
	plan.clear();

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
		costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
		return false;
	}

	tf::Stamped < tf::Pose > goal_tf;
	tf::Stamped < tf::Pose > start_tf;

	poseStampedMsgToTF(goal, goal_tf);
	poseStampedMsgToTF(start, start_tf);

	float startX = start.pose.position.x;
	float startY = start.pose.position.y;

	float goalX = goal.pose.position.x;
	float goalY = goal.pose.position.y;

	getCoordinate(startX, startY);
	getCoordinate(goalX, goalY);

	int startCell;
	int goalCell;

	if (validate(startX, startY) && validate(goalX, goalY)){
		startCell = convertToCellIndex(startX, startY);
		goalCell = convertToCellIndex(goalX, goalY);
	}

	else{
		ROS_WARN("the start or goal is out of the map");
		return false;
	}

	if (isValid(startCell, goalCell)){
		vector<int> bestPath;
		bestPath.clear();
		bestPath = PathFinder(startCell, goalCell);
		if(bestPath.size()>0){
			for (int i = 0; i < bestPath.size(); i++){
				float x = 0.0;
				float y = 0.0;
				int index = bestPath[i];
				convertToCoordinate(index, x, y);
				geometry_msgs::PoseStamped pose = goal;

				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;

				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;

				plan.push_back(pose);
			}

			float path_length = 0.0;
			std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
			geometry_msgs::PoseStamped last_pose;
			last_pose = *it;
			it++;

			for (; it!=plan.end();++it){
				path_length += hypot((*it).pose.position.x - last_pose.pose.position.x, (*it).pose.position.y - last_pose.pose.position.y );
				last_pose = *it;
			}

			cout <<"The global path length: "<< path_length<< " meters"<<endl;
			return true;
		}
		else{
			ROS_WARN("The planner failed to find a path, choose other goal position");
			return false;
		}
	}
	
	else{
		ROS_WARN("Not valid start or goal");
		return false;
	}
}

void PathPlannersROS::getCoordinate(float& x, float& y){
	x = x - originX;
	y = y - originY;
}

int PathPlannersROS::convertToCellIndex(float x, float y){
	int cellIndex;
	float newX = x / resolution;
	float newY = y / resolution;
	cellIndex = getIndex(newY, newX);
	return cellIndex;
}

void PathPlannersROS::convertToCoordinate(int index, float& x, float& y){
	x = getCol(index) * resolution;
	y = getRow(index) * resolution;
	x = x + originX;
	y = y + originY;
}

bool PathPlannersROS::validate(float x, float y){
	bool valid = true;
	if (x > (width * resolution) || y > (height * resolution))
		valid = false;
	return valid;
}

void PathPlannersROS::mapToWorld(double mx, double my, double& wx, double& wy){
	costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
	wx = costmap->getOriginX() + mx * resolution;
	wy = costmap->getOriginY() + my * resolution;
}

vector<int> PathPlannersROS::PathFinder(int startCell, int goalCell){
	vector<int> bestPath;
	float g_score [mapSize];
	for (uint i=0; i<mapSize; i++)
		g_score[i]=infinity;

	timespec time1, time2;

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	
	//bestPath=AStar(startCell, goalCell,  g_score);
	bestPath=DStarLite(startCell, goalCell);

	// bestPath=Dijkstra(startCell, goalCell,  g_score);
	// bestPath=BFS(startCell, goalCell,  g_score);
	// bestPath=JPS(startCell,goalCell,g_score);

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

	cout<<" Time taken to generate path= " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;

	return bestPath;
}

vector<int> PathPlannersROS::AStar(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell]+heuristic(startCell,goalCell,1);
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				add_open(OPL, neighborCells[i], goalCell, g_score, 1); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}

vector<int> PathPlannersROS::Dijkstra(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell];
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				add_open(OPL, neighborCells[i], goalCell, g_score, 0); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}

vector<int> PathPlannersROS::BFS(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell];
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+1;
				add_open(OPL, neighborCells[i], goalCell, g_score, 0); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}



vector<int> PathPlannersROS::constructPath(int startCell, int goalCell,float g_score[])
{
	vector<int> bestPath;
	vector<int> path;

	path.insert(path.begin()+bestPath.size(), goalCell);
	int currentCell=goalCell;

	while(currentCell!=startCell){ 
		vector <int> neighborCells;
		neighborCells=getNeighbour(currentCell);

		vector <float> gScoresNeighbors;
		for(uint i=0; i<neighborCells.size(); i++)
			gScoresNeighbors.push_back(g_score[neighborCells[i]]);
		
		int posMinGScore=distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
		currentCell=neighborCells[posMinGScore];

		path.insert(path.begin()+path.size(), currentCell);
	}

	for(uint i=0; i<path.size(); i++)
		bestPath.insert(bestPath.begin()+bestPath.size(), path[path.size()-(i+1)]);

	return bestPath;
}

void PathPlannersROS::add_open(multiset<cells> & OPL, int neighborCell, int goalCell, float g_score[] ,int n){
	cells CP;
	CP.currentCell=neighborCell;
	if (n==1)
		CP.fCost=g_score[neighborCell]+heuristic(neighborCell,goalCell,1);
	else
		CP.fCost=g_score[neighborCell];
	OPL.insert(CP);
}

vector <int> PathPlannersROS::getNeighbour (int CellID){
	int rowID=getRow(CellID);
	int colID=getCol(CellID);
	int neighborIndex;
	vector <int>  freeNeighborCells;

	for (int i=-1;i<=1;i++)
		for (int j=-1; j<=1;j++){
			if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
				neighborIndex = getIndex(rowID+i,colID+j);
				if(isFree(neighborIndex) )
					freeNeighborCells.push_back(neighborIndex);
			}
		}

	return  freeNeighborCells;
}

bool PathPlannersROS::isValid(int startCell,int goalCell){ 
	bool isvalid=true;
	
	bool isFreeStartCell=isFree(startCell);
	bool isFreeGoalCell=isFree(goalCell);
	
	if (startCell==goalCell){
		cout << "The Start and the Goal cells are the same..." << endl; 
		isvalid = false;
	}
	
	else{
		
		if(!isFreeStartCell && !isFreeGoalCell){
			cout << "The start and the goal cells are obstacle positions..." << endl;
			isvalid = false;
		}
		
		else{
			if(!isFreeStartCell){
				cout << "The start is an obstacle..." << endl;
				isvalid = false;
			}
			else{
				if(!isFreeGoalCell){
					cout << "The goal cell is an obstacle..." << endl;
					isvalid = false;
				}
				else{
					if (getNeighbour(goalCell).size()==0){
						cout << "The goal cell is encountred by obstacles... "<< endl;
						isvalid = false;
					}
					else{
						if(getNeighbour(startCell).size()==0){
							cout << "The start cell is encountred by obstacles... "<< endl;
							isvalid = false;
						}
					}
				}
			}
		}
	}

return isvalid;
}

float  PathPlannersROS::getMoveCost(int CellID1, int CellID2){
	int i1=0,i2=0,j1=0,j2=0;

	i1=getRow(CellID1);
	j1=getCol(CellID1);
	i2=getRow(CellID2);
	j2=getCol(CellID2);

	float moveCost=INFINIT_COST;
	if(i1!=i2 && j1!=j2)
		moveCost=1.4;
	else
		moveCost=1;
	return moveCost;
} 

bool  PathPlannersROS::isFree(int CellID){
	return OGM[CellID];
} 
};

bool operator<(cells const &c1, cells const &c2){return c1.fCost < c2.fCost;}
