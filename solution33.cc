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
            // if(nums_workbench_ == 18){ // special map4
            //     if( count==2){
            //         stringstream ss(line);
            //         count++;
            //         continue;
            //     }
            // }
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
    //cerr<<"stat "<<workbenches_[7][0].source_deliver_status<<endl;

    //do something
    DetectLazyRobot();
    for(int i=0;i<4;++i){
        auto r = robots_[i];
        if(r->busy){
            if(!CheckTargetSourceStatus(i)) continue;
            FindBetterTarget(i);
            //else if(nums_workbench_==18)SwitchTarget(i); // special for map3
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
                        auto rb = robots_[i];
                        //if(workbenches_[7].size()==1 && rb->target_type==7)
                        workbenches_[rb->target_type][rb->target_id].source_deliver_status &= ~(1<<rb->thing_carry);
                        r->busy = false;
                        r->ResetTarget();
                    }
                }
            } else {
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

void Solution::SwitchTarget(const int& id_robo){
    auto rb = robots_[id_robo];
    float dis_old = rb->target_distance;
    float type = rb->thing_carry;
    if(!rb->thing_carry){

        for(auto other_robot:robots_){
            if(other_robot->idx == id_robo) continue;
            if(other_robot->target_type == -1) continue;
            if(other_robot->thing_carry != type) continue ;

            float target_dis_other = other_robot->target_distance;
            int id_other = other_robot->target_id;
            int type_other = other_robot->target_type;
            float x = workbenches_[type_other][id_other].x;
            float y = workbenches_[type_other][id_other].y;
            float temp_dis = CalculateDistance(rb->x,rb->y,x,y);
            if(temp_dis < target_dis_other){
                other_robot->target_id = rb->target_id;
                other_robot->target_type = rb->target_type;
                rb->target_id = id_other;
                rb->target_type = type_other;
            }
        }
    } else {
        // SetTarget(id_robo);
        // if(target_type!=rb->target_type || target_id!=rb->target_id)
        //     workbenches_[target_type][target_id].source_deliver_status &= ~(1<<rb->thing_carry);
    }

}

void Solution::FindBetterTarget(const int& id_robo){
    auto rb = robots_[id_robo];
    float dis_old = rb->target_distance;
    float type_old = rb->target_type;
    float target_type = rb->target_type;
    float target_id = rb->target_id;
    if(!rb->thing_carry){
        for(auto other_robot:robots_){
            if(other_robot->idx == id_robo) continue;
            if(other_robot->thing_carry) continue ;
            if(other_robot->target_type<type_old) continue;
            float target_dis_other = other_robot->target_distance;
            int id_other = other_robot->target_id;
            int type_other = other_robot->target_type;
            float x = workbenches_[type_other][id_other].x;
            float y = workbenches_[type_other][id_other].y;
            float temp_dis = CalculateDistance(rb->x,rb->y,x,y);
            if(temp_dis < target_dis_other){
                other_robot->target_id = rb->target_id;
                other_robot->target_type = rb->target_type;
                rb->target_id = id_other;
                rb->target_type = type_other;
            }
        }
    } else {
        if(rb->thing_carry==1||rb->thing_carry==2||rb->thing_carry==3) return;
        if(rb->target_type!=9 && rb->target_type!=8) // 8,9 donnot care
        workbenches_[target_type][target_id].source_deliver_status &= ~(1<<rb->thing_carry);
        SetTarget(id_robo);
        if(target_type==rb->target_type && target_id==rb->target_id){
            if(rb->target_type!=9 && rb->target_type!=8) // 8,9 donnot care
            workbenches_[rb->target_type][rb->target_id].source_deliver_status |= (1<<rb->thing_carry);
        }
    }

}

bool Solution::CheckTargetSourceStatus(const int& id_robo){
    auto r = robots_[id_robo];
    if(1&(workbenches_[r->target_type][r->target_id].sources_status>>r->thing_carry)){
        r->busy = false;
        r->ResetTarget();
        return false;
    }
    if(workbenches_[7].size()==1 && (!r->thing_carry) &&(1&(workbenches_[7][0].source_deliver_status>>r->target_type))){
        r->busy = false;
        r->ResetTarget();
        return false;
    }

    // if(workbenches_[7].size()==1 && r->target_type==7){
    //     for(auto rb:robots_){
    //         if(rb->idx == r->idx)
    //             continue;
    //         if(rb->target_type==7){
    //             if(rb->thing_carry == r->target_type){
    //                 r->busy = false;
    //                 r->ResetTarget();
    //                 return false;
    //             }
    //         };
    //     }

    // }
    return true;
}

void Solution::ComputeVirtualForce(const int& id_robo){
    auto rb = robots_[id_robo];
    if(rb->target_type==-1){
        rb->virtual_force = 0;
        rb->virtual_angle = rb->face_where;
    } else {
        auto rb = robots_[id_robo];
        auto wb = &workbenches_[rb->target_type][rb->target_id];
        float angle = CalculateAngle(rb->x,rb->y,wb->x,wb->y);

        // DWAcomputing(id_robo);
        if(nums_workbench_==18){
            PreventCollision(id_robo,angle,M_PI/3,M_PI/4,2,1);
        } else PreventCollision(id_robo,angle,M_PI/3,M_PI/6,1.5,0.9);
        
        PreventCollision(id_robo,angle,M_PI/4,M_PI/6,3,0.9);
        PreventCollision(id_robo,angle,M_PI/12,M_PI/12,8,0.5);

        //rb->virtual_angle = atan2(total_force_y,total_force_x);
        rb->virtual_angle = angle;
        //cerr<<"id "<<id_robo<<" virtual angle after"<<rb->virtual_angle<<endl;
    }

}

bool Solution::DWAcomputing(const int& id_robo){
    auto rb = robots_[id_robo];
    rb->CalculateAcceleration();

    float current_speed = sqrt(pow(rb->linear_speed_x,2)+pow(rb->linear_speed_y,2));
    float current_yawrate = rb->angular_speed;

    float min_speed = max(current_speed-rb->a*dT_,rb->min_speed);
    float max_speed = min(current_speed+rb->a*dT_,rb->max_speed);
    float min_yawrate = max(current_yawrate-rb->alpha*dT_,-rb->max_yawrate);
    float max_yawrate = min(current_yawrate+rb->alpha*dT_,rb->max_yawrate);

    float min_cost = 1e6;
    for(float v=max_speed; v>=min_speed ; v-=velocity_resolution_){
        for(float r=min_yawrate; r<=max_yawrate; r+=yawrate_resolution_){
            if(abs(r)>3){
                if(rb->target_distance < 2)
                    if(v > 0.399 * M_PI) ;
                if(rb->x<2 || rb->x>48 || rb->y<2 || rb->y>48) 
                    if(v>2) continue;
            }
            float x = rb->x, y=rb->y, angle=rb->face_where;
            float temp_cost = 0, yaw_cost=0, col_cost=0, speed_cost=0;
            float min_dis = 10;
            for(float t=0;t<=1;t+=dT_){
                angle += r*dT_;
                x += v*dT_*cos(angle);
                y += v*dT_*sin(angle);

                for(auto& rot:robots_){
                    if(rot->idx == id_robo) continue;
                    float  rotR;
                    if(rot->thing_carry) float  rotR = 0.53;
                    else float rotR = 0.45;
                    float border = rotR+rb->radius+0.1;
                    float real_dis = CalculateDistance(rb->x,rb->y,rot->x,rot->y);
                    if(real_dis < border){
                        col_cost += 1e6;
                        break;
                    }
                    if(real_dis < min_dis) min_dis = real_dis;
                }
                col_cost = 1/min_dis;
                speed_cost = 1/v;
                // yaw_cost = fabs(current_yawrate-r);
                temp_cost = speed_cost + yaw_cost + col_cost;
                if(temp_cost<=min_cost){
                    min_cost = temp_cost;
                    rb->local_speed = v;
                    rb->local_yawrate = r;
                }
            }
        }
    }
    if(min_cost >= 1e6-1) {    cerr<<"here\n";return false;}

}

void Solution::PreventCollision(const int& id_robo, float& angle, float view_field, float turn_angle, float distance, float radius){
    auto rb = robots_[id_robo];
    float angle_left = angle+view_field;
    if(angle_left>M_PI) angle_left-=2*M_PI;
    float angle_right = angle-view_field;
    if(angle_right<(-M_PI)) angle_left+=2*M_PI;
    for(auto& robot:robots_){
        if(robot->idx==id_robo) continue;
        
        float x = rb->x+cos(angle)*distance , y= rb->y+sin(angle)*distance;
        float xl = rb->x+cos(angle_left)*distance , yl = rb->y+sin(angle_left)*distance;
        float xr = rb->x+cos(angle_right)*distance , yr = rb->y+sin(angle_right)*distance;

        float estimate_dis =  CalculateDistance(x,y,robot->x,robot->y);
        float estimate_disl =  CalculateDistance(xl,yl,robot->x,robot->y);
        float estimate_disr =  CalculateDistance(xr,yr,robot->x,robot->y);

        if(estimate_dis < radius){
            angle += turn_angle;
        }
        if(estimate_disl < radius){
            angle -= turn_angle/2;
        }
        if(estimate_disr < radius){
            angle += turn_angle/2;
        }
    }
    while(angle > M_PI) angle-=2*M_PI;
    while(angle < -M_PI) angle+=2*M_PI;
}

bool Solution::DetectCollision(const int& id_robo, float& angle, float view_field, float distance, float radius){
    auto rb = robots_[id_robo];
    float angle_left = angle+view_field;
    if(angle_left>M_PI) angle_left-=2*M_PI;
    float angle_right = angle-view_field;
    if(angle_right<(-M_PI)) angle_left+=2*M_PI;
    for(auto& robot:robots_){
        if(robot->idx==id_robo) continue;
        
        float x = rb->x+cos(angle)*distance , y= rb->y+sin(angle)*distance;
        float xl = rb->x+cos(angle_left)*distance , yl = rb->y+sin(angle_left)*distance;
        float xr = rb->x+cos(angle_right)*distance , yr = rb->y+sin(angle_right)*distance;

        float estimate_dis =  CalculateDistance(x,y,robot->x,robot->y);
        float estimate_disl =  CalculateDistance(xl,yl,robot->x,robot->y);
        float estimate_disr =  CalculateDistance(xr,yr,robot->x,robot->y);

        if(estimate_dis < radius){
            return true;
        }
        if(estimate_disl < radius){
            return true;
        }
        if(estimate_disr < radius){
            return true;
        }
    }

    return false;
}

void Solution::MoveRobot2Target(const int& id_robo){

    //if(id_robo==2 || id_robo==3 || id_robo==1) return;

    if(robots_[id_robo]->target_id == -1){
        KeepRobotWait(id_robo);
        return;
    }

    ComputeVirtualForce(id_robo);
    auto rb = robots_[id_robo];
    auto wb = &workbenches_[rb->target_type][rb->target_id];
    rb->target_distance = CalculateDistance(rb->x,rb->y,wb->x,wb->y);

    
    // if(DetectCollision(id_robo,rb->face_where,M_PI/3,2,1)){
    //     if(DWAcomputing(id_robo)){
    //         return;
    //     } else {
    //         //continue
    //     }
    // }

    

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
    float border = 0.;
    if(rb->thing_carry) border = 1.2708;
    else border = 0.9163;
    double dis = rb->target_distance;
    border += 0.2;
    //speed = min(6.,exp(dis)+1.75);
    if(nums_workbench_==43) speed = min(6.,exp(dis)+1.75);
    else if (nums_workbench_==25) speed = min(6.,exp(dis)+1.1);
    else if (nums_workbench_==50) speed = min(6.,exp(dis)+1.5);
    else if (nums_workbench_==18) speed = min(6.,exp(dis)+1.5);
    //if(dis<0.8 ) rot_speed>0.2?0.2:rot_speed;
    
    // speed = (dis<0.4?:speed);

    // if(dis<1){
    //     float abs_rot = min((double)abs(rot_speed),exp(-1*dis))+M_PI/6;
    //     rot_speed = sign*abs_rot;
    // }

    if(abs(rot_speed)>3){
        if(rb->target_distance < 2){
            if(nums_workbench_==43)speed = 0.379 * M_PI;
            else speed = 0.399 * M_PI;
        }
        if(rb->x<2 || rb->x>48 || rb->y<2 || rb->y>48){
            if(nums_workbench_==43) speed=0;
            else if (nums_workbench_==25) speed=-0.5;
            else if (nums_workbench_==50) speed = speed>2.1?2.1:speed;
            else if (nums_workbench_==18) speed = speed>2.4?2.4:speed;
        }

            
    }
    //if(nums_workbench_==18) {if(abs_diff>M_PI_2) {speed = 1/abs_diff+2.8;speed = speed>2.2?2.2:speed;}} // special for map3

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
            SelectNearestWorkbench4567(dis_emerge,dis,8,id_robo,type_carry);
            SelectNearestWorkbench4567(dis_emerge,dis,9,id_robo,type_carry);
        } else if(type_carry==4 || type_carry==5 || type_carry==6){
            if(workbenches_[7].size()==0) SelectNearestWorkbench4567(dis_emerge, dis,9,id_robo,type_carry);
            SelectNearestWorkbench4567(dis_emerge,dis,7,id_robo,type_carry);
        } else if(type_carry==1){
            if(nums_workbench_==18){
                if(!(1&(workbenches_[4][0].sources_status>>1)) && !(1&(workbenches_[4][0].source_deliver_status>>1))){ 
                    // if no source 1 and no robot to send 1
                    SelectNearestWorkbench4567(dis_emerge,dis,4,id_robo,type_carry);
                } else {
                    SelectNearestWorkbench4567(dis_emerge,dis,5,id_robo,type_carry);
                }
            } else {
                SelectNearestWorkbench4567(dis_emerge,dis,5,id_robo,type_carry);
                SelectNearestWorkbench4567(dis_emerge,dis,4,id_robo,type_carry);
            }
        } else if(type_carry==2){
            if(nums_workbench_==18){
                if(!(1&(workbenches_[4][0].sources_status>>2)) && !(1&(workbenches_[4][0].source_deliver_status>>2))){ 
                    // if no source 2 and no robot to send 2
                    SelectNearestWorkbench4567(dis_emerge,dis,4,id_robo,type_carry);
                } else {
                    SelectNearestWorkbench4567(dis_emerge,dis,6,id_robo,type_carry);
                }
            } else {
                if(0) SelectNearestWorkbench4567(dis_emerge,dis,6,id_robo,type_carry);
                else{
                    SelectNearestWorkbench4567(dis_emerge,dis,6,id_robo,type_carry);
                    SelectNearestWorkbench4567(dis_emerge,dis,4,id_robo,type_carry);
                }
            }
        } else if(type_carry==3){
                if(0) SelectNearestWorkbench4567(dis_emerge,dis,6,id_robo,type_carry);
                else{
                    SelectNearestWorkbench4567(dis_emerge,dis,6,id_robo,type_carry);
                    SelectNearestWorkbench4567(dis_emerge,dis,5,id_robo,type_carry);
                }
            // int i = id_robo;
            // cerr<<"id "<<id_robo<<" target "<<robots_[i]->target_type<<","<<robots_[i]->target_id<<endl;
        } else cerr<<"ERROR carry "<<type_carry<<endl;
        
        if(robots_[id_robo]->target_id == -1)
            robots_[id_robo]->busy = false;
        else {
            robots_[id_robo]->busy = true;
            auto rb = robots_[id_robo];
            // if(workbenches_[7].size()==1 && rb->target_type==7)
            if(rb->target_type!=9 && rb->target_type!=8) // 8,9 donnot care
            workbenches_[rb->target_type][rb->target_id].source_deliver_status |= (1<<rb->thing_carry);
        }
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

void Solution::SelectNearestWorkbench4567(float& dis_emerge, float& dis_now, int type,const int& id_robo, int& source_type){
    auto rb = robots_[id_robo];
    for(int i=0;i<workbenches_[type].size();++i){
        auto wb = workbenches_[type][i];
        if((1&(wb.sources_status>>source_type)) || (1&(wb.source_deliver_status>>source_type))) continue;
        if(type == 7){
            switch (source_type)
            {
            case 4:
                if((1&(wb.sources_status>>5)) || (1&(wb.sources_status>>6))){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if((1&(wb.sources_status>>5)) && (1&(wb.sources_status>>6))){
                        temp -= 100;
                    }
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            case 5:
                if((1&(wb.sources_status>>4)) || (1&(wb.sources_status>>6))){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if((1&(wb.sources_status>>4)) && (1&(wb.sources_status>>6))){
                        temp -= 100;
                    }
                    if(temp < dis_emerge){
                        dis_emerge = temp;
                        robots_[id_robo]->target_id = i;
                        robots_[id_robo]->target_type = type;  
                    }
                }
                break;
            case 6:
                if((1&(wb.sources_status>>4)) || (1&(wb.sources_status>>5))){
                    float temp = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
                    if((1&(wb.sources_status>>4)) && (1&(wb.sources_status>>5))){
                        temp -= 100;
                        // cerr<<"temp "<<temp<<endl;
                        // cerr<<"id "<<wb.idx<<endl;
                    }
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

void Solution::SelectNearestWorkbench123(float& dis_now, int type,const int& id_robo){
    auto rb = robots_[id_robo];
    for(int i=0;i<workbenches_[type].size();++i){
        auto wb = workbenches_[type][i];
        float temp_dis = CalculateDistance(rb->x,rb->y,wb.x,wb.y);
        if(temp_dis < dis_now){
            dis_now = temp_dis;
            robots_[id_robo]->target_id = i;
            robots_[id_robo]->target_type = type;
        }
    }   
}

int Solution::FindMostEmergencyWB7(int type){
    if(workbenches_[type].size()==1) return 0;
}

bool Solution::CheckProduct(const int& id_robo){


    // if(nums_workbench_==50 ){
    //     int ttarget_types= rand_num_%2+2;
    //     if(id_robo==0 || id_robo==2){
    //     bool oo = SearchThisTypeReadyWorkbench(ttarget_types,id_robo);
    //     if(oo) return true;
    //     }

    // }

    // 有7直接拿， 
    // 有4，5，6需要看看7缺不缺，缺就拿，根本没有工作台7的话就拿给9
    // 
    bool has_target = SearchThisTypeReadyWorkbench(7,id_robo);
    if(has_target) return true;

    if(workbenches_[7].size()==0){
        for(int type=6;type>=4;--type){
            has_target = SearchThisTypeReadyWorkbench(type,id_robo);
            if(has_target) break;
        }
    }
    if(has_target) return true;

    for(int i=0;i<workbenches_[7].size();++i){
        for(int type=4;type<=6;++type){
            if((1&(workbenches_[7][i].sources_status>>type)) || (1&(workbenches_[7][0].source_deliver_status>>type)))
                continue;
            // if(workbenches_[7].size()==1 && (1&(workbenches_[7][0].source_deliver_status>>type)))
            //     continue;
            has_target = SearchThisTypeReadyWorkbench(type,id_robo);
            if(has_target) break;
        }
        if(has_target) break;
    }
    if(has_target) return true;



    // if(nums_workbench_==50){
    //     // nearest
    //     float dis = 99;
    //     for(int i=1;i<3;++i){
    //         SelectNearestWorkbench123(dis,i,id_robo);
    //     }
    //     if(robots_[id_robo]->HasTarget()) return true;
    // }
    // // random
    rand_num_++;
    int target_types=0;
    if(nums_workbench_==50)target_types= rand_num_%2+2; // special for map3
    else target_types = rand_num_%3+1;
    has_target = SearchThisTypeReadyWorkbench(target_types,id_robo);
    if(has_target) return true;

    return false;


}

bool Solution::SearchThisTypeReadyWorkbench(int type,const int& id_robo){
    float dis = 99.;
    int nearest_id = -1;
    auto rb = robots_[id_robo];
    for(int j=0;j<workbenches_[type].size();++j){
        auto wb = &workbenches_[type][j];
        //if(wb->product_status && !wb->product_been_ordered){
        if(wb->product_status){
            if(nums_workbench_==43 || nums_workbench_==18){
                if(type!=1&&type!=2&&type!=3){
                    if( wb->product_been_ordered) continue;
                } // special map1
            } else if( wb->product_been_ordered) continue;
            float temp_dis = CalculateDistance(rb->x,rb->y,wb->x,wb->y);
            // cerr<<"temp "<<temp_dis<<" type "<<type<<" id "<<j<<endl;
            if(temp_dis<dis){
                dis = temp_dis;
                nearest_id = j;
            }
        }
    }
    if(nums_workbench_==50){
        if(type==1){
            nearest_id=2;
        } else if(type==2){
            nearest_id=1;
        } else if(type==3){
            nearest_id=2;
        }
    }
    if(nearest_id == -1) return false;
    SetTarget(id_robo,type,nearest_id);
    return true;
}

void Solution::SetTarget(const int& id_robo, int type, int id){
    if(id == -1) return;
    auto wb = &workbenches_[type][id];
    wb->product_been_ordered = true;
    robots_[id_robo]->target_type = type;
    robots_[id_robo]->target_id = id; 
}

int Solution::FindNearestWorkbench(const int& type, const int& id_robo,float& dis){
    auto rb = robots_[id_robo];
    float x=rb->x,y=rb->y;
    int idx = -1;

    // more than 1 of this type workbench
    for(int i=0;i<workbenches_[type].size();++i){
        float x2 = workbenches_[type][i].x , y2 = workbenches_[type][i].y;
        float temp_dis = sqrt(pow(x2-x,2)+pow(y2-y,2));
        int source = workbenches_[type][i].sources_status;
        if((source>0) && (source&(source-1)!=0)) continue;  // sources are full 
        if(temp_dis < dis){
            dis = temp_dis;
            idx = i;
        }
    }

    return idx;
    //cerr<<"id "<<robots_[id_robo]->target_id<<endl;
}

void Solution::DetectLazyRobot(){
    for(auto& rb:robots_){
        if(rb->linear_speed_x==0 && rb->linear_speed_y==0 && rb->angular_speed==0){
            rb->wait_time ++;
        } else rb->wait_time=0;
        if(rb->wait_time == 50)
            SetTarget(rb->idx);
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