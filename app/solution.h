#ifndef CODECRAFT_SOLUTION_H
#define CODECRAFT_SOLUTION_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <stdio.h>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace chrono;

using Workbench = struct {
    //using Ptr = shared_ptr<Workbench>;
    float x=0., y=0.;
    int type=0, time_left=-1 , sources_status=0 , product_status=0, idx=-1;
};

class Robot{
    public:
     using Ptr = shared_ptr<Robot>;
     int workbench_near=-1, thing_carry=0;
     float time_coef = 1., collision_coef=1., angular_speed=0., \
            linear_speed_x=0., linear_speed_y=0., \
            face_where=0., x=0., y=0.; // same order as definitions given by task instructions

     int idx=-1, target_id=-1;
     float target_distance=99.;
     bool busy = false;
     // some actions can be done
     inline void Rotate(float& x);
     inline void Forward(float& x);
     inline void Buy();
     inline void Sell();
     inline void Destroy();

     
};

class Solution{

    public:
     
     Solution();
     ~Solution();
     void InitMap();  // init 100*100 map
     bool GetFrameInfo(); // get information of workbenches and robots per frame
     void AssaignTasks(); // try to get a plan for each robot and print it

     void MoveRobot2Workbench(int& id_robo,int& id_wb);
     void CalculateDistance(int& id_robo,int& id_wb);
     void SetTarget(int& id_robo);
     void KeepRobotWait(int& id_robo);


    private:
     vector<vector<int>> map_;
     int current_frame_=0 ,current_money_=0 ,nums_workbench_=0;
     vector<Robot::Ptr> robots_; // store 4 robots
     vector<Workbench> workbenches_; // store all workbenches
     
};

#endif