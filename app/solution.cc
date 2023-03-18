#pragma once
#include "solution.h"

Solution::Solution(){
    cerr<<"[Construction]\n";
}

Solution::~Solution(){
    cerr<<"[Destruction]\n";
}

void Solution::InitMap(){
    string line;
    while (getline(cin,line)) {    
        //cerr<<line<<endl;
        if (line[0] == 'O' && line[1] == 'K') break;
        vector<int> line_map;
        for(auto& c: line){
            if(c=='.') line_map.push_back(0);
            else if(c=='A') line_map.push_back(10);
            else line_map.push_back(c-'0');
        }
        map_.push_back(line_map);
    }
    cout<<"OK";
    fflush(stdout);
}

bool Solution::GetFrameInfo(){
    wb_count_.clear();
    wb_count_.resize(10,0);
    auto start = high_resolution_clock::now();
    string line,word;
    int count = 0;
    while(getline(cin,line)){
        //cerr<<line<<endl;
        if(0==count){
            stringstream ss(line);
            while (ss>>word)
            {
                int x = stoi(word);
                if(x<1e4) current_frame_ = x;
                else current_money_ = x;
            }
        } else if(1==count){
            stringstream ss(line);
            while (ss>>word)
            {
                int x = stoi(word);
                nums_workbench_ = x;
            }
        } else if(count <= (1+nums_workbench_)){
            stringstream ss(line);
            int idata[4],inum;
            if(ss>>inum) idata[0]=inum;
            float fdata[2],fnum;
            for(int i=0;i<2;++i){
                if(ss>>fnum) fdata[i]=fnum;
            }
            for(int i=1;i<4;++i){
                if(ss>>inum) idata[i]=inum;
            }
            if(current_frame_ == 1 && count==2){
                workbenches_.resize(10,vector<Workbench>());
            }
            if(current_frame_ == 1){
                Workbench wbench;
                wbench.x = fdata[0];
                wbench.y = fdata[1];
                wbench.type = idata[0];
                wbench.time_left = idata[1];
                wbench.sources_status = idata[2];
                wbench.product_status = idata[3];
                wbench.idx = (count-2);

                workbenches_[wbench.type].emplace_back(wbench);
            } else {
                int type = idata[0];
                auto wbench = &workbenches_[type][wb_count_[type]++];
                wbench->time_left = idata[1];
                wbench->sources_status = idata[2];
                wbench->product_status = idata[3];
            }

            // cerr<<idata[0]<<" ";
            // for(auto& i:fdata){cerr<<i<<" ";}
            // for(int i=1;i<4;++i){cerr<<idata[i]<<" ";}
            // cerr<<endl;
        } else if(count <= (5+nums_workbench_)){
            stringstream ss(line);
            int idata[2],inum;
            for(int i=0;i<2;++i){
                if(ss>>inum) idata[i]=inum;
            }
            float fdata[8],fnum;
            for(int i=0;i<8;++i){
                if(ss>>fnum) fdata[i]=fnum;
            }
            if(current_frame_ == 1 && count==(2+nums_workbench_)){
                robots_.resize(4);
                for_each(begin(robots_),end(robots_),[](Robot::Ptr& pt){pt = make_shared<Robot>();});
            }
            auto rb = robots_[count-2-nums_workbench_];
            rb->workbench_near = idata[0];
            rb->thing_carry = idata[1];
            rb->time_coef = fdata[0];
            rb->collision_coef = fdata[1];
            rb->angular_speed = fdata[2];
            rb->linear_speed_x = fdata[3];
            rb->linear_speed_y = fdata[4];
            rb->face_where = fdata[5];
            rb->x = fdata[6];
            rb->y = fdata[7];
            rb->idx = (count-2-nums_workbench_);
            // for(auto& i:idata){cerr<<i<<" ";}
            // for(auto& i:fdata){cerr<<i<<" ";}
            // cerr<<endl;
        } else if(line[0]=='O' && line[1]=='K'){
            auto end = high_resolution_clock::now();
            auto cost = duration_cast<duration<double,ratio<1,1000>>>(end - start);
            //cerr<<"get info cost "<<cost.count()<<" ms\n";
            return true;
        }
        ++count;
    }

    // getline(cin,line);
    // cerr<<line;
    //cerr<<line<<endl;
}

void Solution::AssignTasks(){
    cout<<current_frame_<<endl; // no need to change
    //do something
    DetectLazyRobot();
    for(int i=0;i<4;++i){
        auto r = robots_[i];
        if(r->busy){
            if(r->workbench_near == workbenches_[r->target_type][r->target_id].idx){
                int thing_type = r->thing_carry;
                if(thing_type == 0){
                    if(current_frame_>8750) continue;
                    r->Buy();
                    r->busy = false;
                    workbenches_[r->target_type][r->target_id].product_been_ordered = false;
                    r->ResetTarget();
                } else {
                    if(1&(workbenches_[r->target_type][r->target_id].sources_status>>thing_type)){
                        r->busy = false;
                        r->ResetTarget();
                    } else {
                        r->Sell();
                        r->busy = false;
                        r->ResetTarget();
                    }
                }
            } else {
                if(CheckTargetSourceStatus(i)) 
                    MoveRobot2Target(i);
            }
        } else {
            SetTarget(i);
            MoveRobot2Target(i);
        }
    }
    cout<<"OK"<<endl; // no need to change
    fflush(stdout); // no need to change
}

bool Solution::CheckTargetSourceStatus(const int& id_robo){
    auto r = robots_[id_robo];
    if(1&(workbenches_[r->target_type][r->target_id].sources_status>>r->thing_carry)){
        r->busy = false;
        r->ResetTarget();
        return false;
    }
    return true;
}

void Solution::ComputeVirtualForce(const int& id_robo){
    auto rb = robots_[id_robo];
    if(rb->target_type==-1){
        rb->virtual_force = 0;
    } else {
        auto rb = robots_[id_robo];
        auto wb = &workbenches_[rb->target_type][rb->target_id];
        float angle = CalculateAngle(rb->x,rb->y,wb->x,wb->y);
        float distance = CalculateDistance(rb->x,rb->y,wb->x,wb->y);
        float gravity_force = k_gravity_ * distance;
        if(distance < gravity_dis_) gravity_force = gravity_force/distance*gravity_dis_;
        float total_force_x = gravity_force * cos(angle);
        float total_force_y = gravity_force * sin(angle);
        //cerr<<"id "<<id_robo<<" gravity "<<gravity_force<<endl;
        //cerr<<"id "<<id_robo<<" virtual angle before"<<atan2(total_force_y,total_force_x)<<endl;

        for(auto& robot:robots_){
            if(robot->idx==id_robo) continue;
            int sign = (rand_force_++%2)?1:1;
            float x1 = rb->x+cos(angle)*1 , y1= rb->y+sin(angle)*1;
            float estimate_dis1 =  CalculateDistance(x1,y1,robot->x,robot->y);
            if(estimate_dis1 < 0.5){
                angle += sign*M_PI/2;
                break;
            }
            float x3 = rb->x+cos(angle)*3 , y3= rb->y+sin(angle)*3;
            float estimate_dis3 =  CalculateDistance(x3,y3,robot->x,robot->y);
            if(estimate_dis3 < 0.5){
                angle += sign*M_PI/3;
                break;
            }
            float x5 = rb->x+cos(angle)*5 , y5= rb->y+sin(angle)*5;
            float estimate_dis5 =  CalculateDistance(x5,y5,robot->x,robot->y);
            if(estimate_dis5 < 0.5){
                angle += sign*M_PI/6;
                break;
            }

            // other robots give forces

            // float dis_between = CalculateDistance(rb->x,rb->y,robot->x,robot->y);
            // if(dis_between > collision_dis_) continue;
            // // float angle_away = CalculateAngle(robot->x,robot->y,rb->x,rb->y);
            // float angle_away = angle+M_PI_2;
            // if(angle_away>M_PI) angle_away-=2*M_PI;
            // float force = k_obstacle_ * (1.0/dis_between - 1.0/collision_dis_)*pow(1.0/dis_between,2);
            // // cerr<<"id "<<id_robo<<" ot force "<<force<<endl;
            // float x_force = force * cos(angle_away);
            // float y_force = force * sin(angle_away);
            // total_force_x += x_force;
            // total_force_y += y_force;
        }

        if(angle>M_PI) angle-=2*M_PI;
        if(angle<-M_PI) angle+=2*M_PI;

        //rb->virtual_angle = atan2(total_force_y,total_force_x);
        rb->virtual_angle = angle;
        //cerr<<"id "<<id_robo<<" virtual angle after"<<rb->virtual_angle<<endl;
    }

}

void Solution::MoveRobot2Target(const int& id_robo){

    //if(id_robo==2 || id_robo==3) return;

    if(robots_[id_robo]->target_id == -1){
        KeepRobotWait(id_robo);
        return;
    }

    ComputeVirtualForce(id_robo);
    auto rb = robots_[id_robo];
    auto wb = &workbenches_[rb->target_type][rb->target_id];

    // calculate rotation speed
    float border_theta = 0.;
    if(rb->thing_carry) border_theta = 0.2448;
    else border_theta = 0.1222;

    float toward_where= rb->virtual_angle;
    float diff = rb->face_where-toward_where;
    // cerr<<"id "<<id_robo<<" where angle "<<rb->face_where<<endl;
    float abs_diff = abs(diff);
    float sign = diff<0?1.:-1.;
    if(abs_diff>M_PI){
        sign*=-1;
        abs_diff = (2*M_PI-abs_diff);
    }
    float rot_speed = 0.;

    if(abs_diff > border_theta+0.05){
        rot_speed = sign * M_PI;
    } else if(abs_diff > 0.01745){
        rot_speed = sign * 0.15;
    } else {
        rot_speed = sign * 0.02;
    }
    //cerr<<"id "<<id_robo<<" rot "<<rot_speed<<endl;

    float speed = 0.;
    // calculate linear speed
    rb->target_distance = CalculateDistance(rb->x,rb->y,wb->x,wb->y);
    float border = 0.;
    if(rb->thing_carry) border = 1.2708;
    else border = 0.9163;
    double dis = rb->target_distance;
    border += 0.2;
    speed = min(6.,exp(dis));
    speed = (dis<0.4?0.:speed);

    if(abs(rot_speed)>3) speed=0;

    // pub movement actions
    rb->Forward(speed);
    rb->Rotate(rot_speed);
}

float Solution::CalculateDistance(const float& x1,const float& y1,const float& x2,const float& y2){
    double distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    return distance;
}

float Solution::CalculateAngle(const float& x1,const float& y1,const float& x2,const float& y2){
    float toward_where= atan2(y2-y1,x2-x1);
    return toward_where;
}

void Solution::KeepRobotWait(const int& id_robo){
    float zero = 0.;
    robots_[id_robo]->Forward(zero);
    robots_[id_robo]->Rotate(zero);
}

void Solution::SetTarget(const int& id_robo){
    if(robots_[id_robo]->thing_carry){
        // situation 1: things on robot, unbusy, wants know where to go
        int type_carry = robots_[id_robo]->thing_carry;
        float dis = 99., dis_emerge=99.;
        if(type_carry == 7){
            SelectNearestWorkbench(dis_emerge,dis,8,id_robo,type_carry);
            SelectNearestWorkbench(dis_emerge,dis,9,id_robo,type_carry);
        } else if(type_carry==4 || type_carry==5 || type_carry==6){
            if(workbenches_[7].size()==0) SelectNearestWorkbench(dis_emerge, dis,9,id_robo,type_carry);
            SelectNearestWorkbench(dis_emerge,dis,7,id_robo,type_carry);
        } else if(type_carry==1){
            SelectNearestWorkbench(dis_emerge,dis,4,id_robo,type_carry);
            SelectNearestWorkbench(dis_emerge,dis,5,id_robo,type_carry);
        } else if(type_carry==2){
            SelectNearestWorkbench(dis_emerge,dis,4,id_robo,type_carry);
            SelectNearestWorkbench(dis_emerge,dis,6,id_robo,type_carry);
        } else if(type_carry==3){
            SelectNearestWorkbench(dis_emerge,dis,5,id_robo,type_carry);
            SelectNearestWorkbench(dis_emerge,dis,6,id_robo,type_carry);
            // int i = id_robo;
            // cerr<<"id "<<id_robo<<" target "<<robots_[i]->target_type<<","<<robots_[i]->target_id<<endl;
        } else cerr<<"ERROR carry "<<type_carry<<endl;
        
        if(robots_[id_robo]->target_id == -1)
            robots_[id_robo]->busy = false;
        else robots_[id_robo]->busy = true;
        //cerr<<"id "<<id_robo<<" tar "<<robots_[id_robo]->target_type<<endl;
    } else {
        bool has_target = CheckProduct(id_robo);
        if(!has_target){

        }
        //cerr<<"id "<<id_robo<<" tar "<<robots_[id_robo]->target_type<<endl;
        if(robots_[id_robo]->HasTarget()){
            robots_[id_robo]->busy = true;
        } else {
            KeepRobotWait(id_robo);
        }
    }
}

void Solution::SelectNearestWorkbench(float& dis_emerge, float& dis_now, int type,const int& id_robo, int& source_type){
    auto rb = robots_[id_robo];
    for(int i=0;i<workbenches_[type].size();++i){
        auto wb = workbenches_[type][i];
        if(1&(wb.sources_status>>source_type)) continue;
        if(type == 7){
            switch (source_type)
            {
            case 4:
                if((1&(wb.sources_status>>5)) && (1&(wb.sources_status>>6))){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            case 5:
                if((1&(wb.sources_status>>4)) && (1&(wb.sources_status>>6))){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            case 6:
                if((1&(wb.sources_status>>4)) && (1&(wb.sources_status>>5))){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            default:
                break;
            }
        }
        if(type == 6){
            switch (source_type)
            {
            case 2:
                if(1&(wb.sources_status>>3)){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            case 3:
                if(1&(wb.sources_status>>2)){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            default:
                break;
            }
        }
        if(type == 5){
            switch (source_type)
            {
            case 1:
                if(1&(wb.sources_status>>3)){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            case 3:
                if(1&(wb.sources_status>>1)){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            default:
                break;
            }
        }
        if(type == 4){
            switch (source_type)
            {
            case 1:
                if(1&(wb.sources_status>>2)){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            case 2:
                if(1&(wb.sources_status>>1)){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            default:
                break;
            }
        }
        if(dis_emerge < 99) continue;
        float temp_dis = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
        if(temp_dis < dis_now){
            dis_now = temp_dis;
            robots_[id_robo]->target_id = i;
            robots_[id_robo]->target_type = type;
        }
    }
}

bool Solution::CheckProduct(const int& id_robo){

    // 有7直接拿， 
    // 有4，5，6需要看看7缺不缺，缺就拿，根本没有工作台7的话就拿给9
    // 
    bool has_target = SearchThisTypeReadyWorkbench(7,id_robo);
    if(has_target) return true;

    for(int i=0;i<workbenches_[7].size();++i){
        for(int type=6;type>=4;--type){
            if(1&(workbenches_[7][i].sources_status>>type))
                continue;
            else{
                has_target = SearchThisTypeReadyWorkbench(type,id_robo);
            }
            if(has_target) break;
        }
        if(has_target) break;
    }
    if(has_target) return true;
    if(workbenches_[7].size()==0){
        for(int i=0;i<workbenches_[9].size();++i){
            for(int type=6;type>=4;--type){
                if(1&(workbenches_[9][i].sources_status>>type))
                    continue;
                else{
                    has_target = SearchThisTypeReadyWorkbench(type,id_robo);
                }
                if(has_target) break;
            }
            if(has_target) break;
        }
    }
    if(has_target) return true;
    //return false;

    rand_num_++;
    int target_type = rand_num_%3+1;
    has_target = SearchThisTypeReadyWorkbench(target_type,id_robo);
    if(has_target) return true;

}

bool Solution::SearchThisTypeReadyWorkbench(int type,const int& id_robo){
    float dis = 99.;
    int nearest_id = -1;
    auto rb = robots_[id_robo];
    for(int j=0;j<workbenches_[type].size();++j){
        auto wb = &workbenches_[type][j];
        if(wb->product_status && !wb->product_been_ordered){
            float temp_dis = CalculateDistance(rb->x,rb->y,wb->x,wb->y);
            // cerr<<"temp "<<temp_dis<<" type "<<type<<" id "<<j<<endl;
            if(temp_dis<dis){
                dis = temp_dis;
                nearest_id = j;
            }
        }
    }
    if(nearest_id != -1){
        auto wb = &workbenches_[type][nearest_id];
        wb->product_been_ordered = true;
        robots_[id_robo]->target_type = type;
        robots_[id_robo]->target_id = nearest_id; 
        return true;
    }
    return false;
}

void Solution::FindNearestSameWorkbench(const int& type,const int id, int& id_robo){
    float dis = 99.;
    float x1 = workbenches_[type][id].x, y1 = workbenches_[type][id].y;
    // more than 1 of this type workbench
    for(int i=0;i<workbenches_[type].size();++i){
        if(i==id) continue;
        float x2 = workbenches_[type][i].x , y2 = workbenches_[type][i].y;
        float temp_dis = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
        if(temp_dis < dis){
            dis = temp_dis;
            robots_[id_robo]->target_id = i;
        }
    }
    //cerr<<"id "<<robots_[id_robo]->target_id<<endl;
}

void Solution::DetectLazyRobot(){
    for(auto& rb:robots_){
        if(rb->linear_speed_x==0 && rb->linear_speed_y==0 && rb->angular_speed==0){
            rb->wait_time ++;
        } else rb->wait_time=0;
        if(rb->wait_time == 500)
            rb->Destroy();
    }
}

void Robot::Rotate(float& x){
    //cerr<<"rotate "<<idx<<" "<<x<<endl;
    cout<<"rotate "<<idx<<" "<<x<<endl;
}

void Robot::Forward(float& x){
    cout<<"forward "<<idx<<" "<<x<<endl;
}

void Robot::Sell(){
    cout<<"sell "<<idx<<endl;
}

void Robot::Buy(){
    cout<<"buy "<<idx<<endl;
}

void Robot::Destroy(){
    cout<<"destroy "<<idx<<endl;
}



    // ORIGINAL official demo

    // readUntilOK();
    // puts("OK");
    // fflush(stdout);
    // int frameID;
    // while (scanf("%d", &frameID) != EOF) {
    //     readUntilOK();
    //     printf("%d\n", frameID);
    //     int lineSpeed = 3;
    //     double angleSpeed = 1.5;
    //     for(int robotId = 0; robotId < 4; robotId++){
    //         printf("forward %d %d\n", robotId, lineSpeed);
    //         printf("rotate %d %f\n", robotId, angleSpeed);
    //     }
    //     printf("OK\n", frameID);
    //     fflush(stdout);
    // }
    // return 0;