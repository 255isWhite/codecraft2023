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
     
     // APF
     float virtual_force = 0., virtual_speed = 0., virtual_angle = 0.;

     // DWA
     float min_speed=-2 , max_speed=6;
     float a = 0., alpha = 0.;
     float max_yawrate=M_PI;
     float radius = 0.45;
     float local_speed = 0., local_yawrate=0.;

     // others
     float nearest_pair_dis = 99.;


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

     void CalculateAcceleration(){
        if(thing_carry){
            radius = 0.53;
            float m = 20*M_PI*radius*radius;
            a = 250/m;
            float J = 0.5*m*radius*radius;
            alpha = 50/J;
        } else {
            radius = 0.45;
            float m = 20*M_PI*radius*radius;
            a = 250/m;
            float J = 0.5*m*radius*radius;
            alpha = 50/J;
        }
     }
     
};

class Solution{

    public:
     
     Solution();
     ~Solution();
     void InitMap();  // init 100*100 map
     bool GetFrameInfo(); // get information of workbenches and robots per frame
     void AssignTasks(); // try to get a plan for each robot and print it
     void PairWorkbenches();

     void MoveRobot2Target(const int& id_robo);
     float CalculateDistance(const float& x1,const float& y1,const float& x2,const float& y2);
     float CalculateAngle(const float& x1,const float& y1,const float& x2,const float& y2);
     void SetTarget(const int& id_robo);
     void KeepRobotWait(const int& id_robo);
     void SelectNearestWorkbench4567(float& dis_emerge ,float& dis_now, int type,const int& id_robo, int& source_type);
     void SelectNearestWorkbench123(float& dis_now, int type,const int& id_robo);
     bool CheckProduct(const int& id_robo);
     int FindNearestWorkbench(const int& type, const int& id_robo, float& dis);
     void DetectLazyRobot();
     bool SearchThisTypeReadyWorkbench(int type,const int& id_robo);
     void ComputeVirtualForce(const int& id_robo);
     bool CheckTargetSourceStatus(const int& id_robo);
     void FindBetterTarget(const int& id_robo);
     void PreventCollision(const int& id_robo ,float& angle, float view_field, float turn_angle,float distance, float radius);
     void SetTarget(const int& id_robo, int type, int id);
     bool DWAcomputing(const int& id_robo);
     bool DetectCollision(const int& id_robo, float& angle, float view_field, float distance, float radius);
     void SwitchTarget(const int& id_robo);
     int FindMostEmergencyWB7(int type);

    private:
     vector<vector<int>> map_;
     int current_frame_=0 ,current_money_=0 ,nums_workbench_=0, rand_num_=0, rand_force_ = 0;
     vector<Robot::Ptr> robots_; // store 4 robots
     vector<vector<Workbench>> workbenches_; // store all workbenches
     vector<int> wb_count_;
     float k_gravity_ = 1.0, k_obstacle_ = 10.0, collision_dis_ = 1.5, gravity_dis_ = 3.0;
     // DWA settings
     float dT_ = 0.02; // 20 ms
     float velocity_resolution_=0.1, yawrate_resolution_=0.1;
     
};

#endif