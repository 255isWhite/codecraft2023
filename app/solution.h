#ifndef CODECRAFT_SOLUTION_H
#define CODECRAFT_SOLUTION_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <stdio.h>
#include <chrono>

using namespace std;
using namespace chrono;

using Workbench = struct {
    float x=0., y=0.;
    int time_left=-1 , sources_status=0 , product_status=0;
};

using Robot = struct{
    int workbench_near=-1, thing_carry=0;
    float time_coef = 1., collision_coef=1., angular_speed=0., \
            linear_speed_x=0., linear_speed_y=0., \
            face_where=0., x=0., y=0.;
};

class Solution{

    public:
     Solution();
     ~Solution();
     void InitMap();
     bool GetFrameInfo();
     void AssaignTasks();
     void PublishOrders();

    private:
     vector<vector<int>> map_;
     int current_frame_=0 ,current_money_=0 ,nums_workbench_=0;
     vector<Robot> robots_;
     vector<Workbench> workbenches_;
     bool know_num_workbenches_=false;
     
};

#endif