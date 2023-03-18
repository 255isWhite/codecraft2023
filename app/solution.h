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
#include <ctime>
#include <set>

using namespace std;
using namespace chrono;

using Workbench = struct {
    //using Ptr = shared_ptr<Workbench>;
    float x=0., y=0.;
    int type=0, time_left=-1 , sources_status=0 , product_status=0, idx=-1;
    bool product_been_ordered=false;
    int source_deliver_status = 0;// only 4,5,6,7 workbenches have this , present in bit;

};

class Robot{
    public:
     using Ptr = shared_ptr<Robot>;
     int workbench_near=-1, thing_carry=0;
     float time_coef = 1., collision_coef=1., angular_speed=0., \
            linear_speed_x=0., linear_speed_y=0., \
            face_where=0., x=0., y=0.; // same order as definitions given by task instructions

     int idx=-1, target_id=-1, target_type=-1;
     float target_distance=99.;
     bool busy = false;
     int wait_time = 0; // if wait too long , just destroy wahtever carried

     float virtual_force = 0., virtual_speed = 0., virtual_angle = 0.;

     // some actions can be done
     inline void Rotate(float& x);
     inline void Forward(float& x);
     inline void Buy();
     inline void Sell();
     inline void Destroy();

     inline void ResetTarget(){
        target_id = -1;
        target_type = -1;
     }

     inline bool HasTarget(){
        return target_type>0;
     }

     
};

class Solution{

    public:
     
     Solution();
     ~Solution();
     void InitMap();  // init 100*100 map
     bool GetFrameInfo(); // get information of workbenches and robots per frame
     void AssignTasks(); // try to get a plan for each robot and print it

     void MoveRobot2Target(const int& id_robo);
     float CalculateDistance(const float& x1,const float& y1,const float& x2,const float& y2);
     float CalculateAngle(const float& x1,const float& y1,const float& x2,const float& y2);
     void SetTarget(const int& id_robo);
     void KeepRobotWait(const int& id_robo);
     void SelectNearestWorkbench(float& dis_emerge ,float& dis_now, int type,const int& id_robo, int& source_type);
     bool CheckProduct(const int& id_robo);
     void FindNearestSameWorkbench(const int& type,const int id, int& id_robo);
     void DetectLazyRobot();
     bool SearchThisTypeReadyWorkbench(int type,const int& id_robo);
     void ComputeVirtualForce(const int& id_robo);
     bool CheckTargetSourceStatus(const int& id_robo);


    private:
     vector<vector<int>> map_;
     int current_frame_=0 ,current_money_=0 ,nums_workbench_=0, rand_num_=0, rand_force_ = 0;
     vector<Robot::Ptr> robots_; // store 4 robots
     vector<vector<Workbench>> workbenches_; // store all workbenches
     vector<int> wb_count_;
     float k_gravity_ = 1.0, k_obstacle_ = 10.0, collision_dis_ = 1.5, gravity_dis_ = 3.0;
     
};

#endif