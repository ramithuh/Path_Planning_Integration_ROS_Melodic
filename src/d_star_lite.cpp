/*####################*/
// Author => Ramith Hettiarachchi < im@ramith.fyi >
// CSIRO Data61
/*####################*/

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

struct vertex{  //Stores a vertex along with k1,k2 Costs
    int x,y;
    float k1;
    float k2;
    vertex(int,int,float,float);
    vertex():x(0),y(0),k1(0),k2(0){}
};
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

int getIndex(int i,int j){return (i*grid_s_x)+j;}
int getRow(int index){return index/grid_s_x;}
int getCol(int index){return index%grid_s_x;}


struct compare{ //Custom Comparison Function
    bool operator()(const vertex & a, const vertex & b){   
            if(a.k1 > b.k1){
                return 1;
            }else if((a.k1 == b.k1)){
                if(a.k2 > b.k2)return 1;
                else return 0;
            }else return 0;
    }
};

bool isVertexEqual(vertex v1,vertex v2){
    if(v1.x == v2.x && v1.y == v2.y){
        return 1;
    }
    return 0;
}

typedef priority_queue<vertex, vector<vertex>, compare > m_priority_queue; //Min Priority Queue
m_priority_queue U;

queue<vertex> changed_nodes;

void showpq(m_priority_queue gq){
    m_priority_queue g = gq;
    while (!g.empty()) {
        vertex c_v = g.top();

        cout << '\t' <<c_v.x<<","<<c_v.y<<"("<<c_v.k1<<","<<c_v.k2<<")"<<"   " ;
        g.pop();
    }
    cout << '\n';
}

double h(vertex s1,vertex s2){   
    //heuristic function
    return sqrt(pow((s1.x-s2.x),2) + pow((s1.y-s2.y),2));
}

bool isInQueue(vertex s){
    if(Ukey[s.x][s.y]==1){
        return 1;
    }
    return 0;
}

void pushToQueue(vertex s){
    Ukey[s.x][s.y] = 1;
    Ukey_k1[s.x][s.y] = s.k1;
    Ukey_k2[s.x][s.y] = s.k2;
    U.push(s);
}

bool isCostLower(vertex b, vertex a){   
    if(a.k1 > b.k1){
        return 1;
    }else if(a.k1 == b.k1){
        if(a.k2 > b.k2)return 1;
        else return 0;
    }else return 0;
}

vertex CalculateKey(vertex s){
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

double edgecost(vertex a,vertex b){
    bool blocked = GRID[a.x][a.y] + GRID[b.x][b.y];

    if(blocked > 0){
        return Inf;
    }else{
        return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
    }

}

double cg_cost(vertex a,vertex b){
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

void UpdateVertex(vertex u){
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

bool isGhost(vertex s){
    if(Ukey[s.x][s.y]==1 && Ukey_k1[s.x][s.y]==s.k1 && Ukey_k2[s.x][s.y]==s.k2){
        return 0;
    }
    return 1;
}

void pop(){
    vertex s = U.top();
    Ukey[s.x][s.y]=0;
    U.pop();
}

vertex TopKey(){
    if(U.size()==0)return vertex(0,0,Inf,Inf);

    vertex temp = U.top();

    while(isGhost(temp)){
        pop(); //pop unwanted ones
        if(U.size()==0) return vertex(0,0,Inf,Inf);
        temp = U.top();
    }
    return temp; //return top most vertex which isn't a ghost
}

void ComputeShortestPath(){

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
void fillGRID(){
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

void fillGRID_(bool random=0){

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
void initialize(vertex startCell,vertex goalCell){
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

void Traverse(vertex pos){
    if(pos.x < 0 || pos.x > grid_s_x || pos.y < 0 || pos.y > grid_s_y){
      return;
    }
    PATH[pos.x][pos.y] = 1;
}

int indexofSmallestElement(double array[]){
    int index = 0;

    for(int i = 1; i < 8; i++){
        if(array[i] < array[index])
            index = i;              
    }

    return index;
}
int  moves[8][2] = {{-1,0},{0,-1},{1,0},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};

double step_cost(int x,int y){
    if(x < 0 || x > grid_s_x || y < 0 || y > grid_s_y){
        return Inf;
    }else return g[x][y];
}

int onestep(){

    if(s_start.x==s_goal.x && s_start.y==s_goal.y)return -1;

    double arr[8] = {};

    for(int i=0; i<8; i++)
        arr[i] = step_cost(s_start.x + moves[i][0],s_start.y + moves[i][1]) + edgecost(s_start,vertex(s_start.x + moves[i][0],s_start.y + moves[i][1],0,0));
    cout<<" ✨ "<<s_start.x<<","<<s_start.y<<endl;

    int min_index = indexofSmallestElement(arr); //arg min 

    s_start.x = s_start.x + moves[min_index][0];
    s_start.y = s_start.y + moves[min_index][1];
     
    Traverse(s_start); //move to start

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
    }
    return 1;


}
vector<int> dStarLite(int startCell,int goalCell){
    vertex startCell_(getRow(startCell),getCol(startCell),0,0);
    vertex  goalCell_(getRow(goalCell ),getCol(goalCell),0,0);

    initialize(startCell_,goalCell_);
    
    cout<<"Successfully loaded GRID"<<endl;

    for(int k=0;k<20;k++){
        for(int m=0;m<20;m++){
            cout<<GRID[k][m]<<" ";
        } 
        cout<<endl;
    }
    cout<<"Priority Queue Size = "<<U.size()<<endl;

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
        while(onestep()){
            bestPath.insert(bestPath.begin()+bestPath.size(), getIndex(s_start.x,s_start.y));
        }
    }else{
		cout << "❓ Path not found!" << endl;
		return emptyPath;
    }

    return bestPath;
}


int main(){

}