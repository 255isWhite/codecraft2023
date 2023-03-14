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
        cerr<<line<<endl;
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
    wb_count.clear();
    wb_count.resize(10,0);
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
                auto wbench = &workbenches_[type][wb_count[type]++];
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
    for(int i=0;i<4;++i){
        auto r = robots_[i];
        if(r->busy){
            if(r->workbench_near == workbenches_[r->target_type][r->target_id].idx){
                int thing_type = r->thing_carry;
                if(thing_type == 0){
                    // if(workbenches_[r->target_type][r->target_id].product_status){
                    //     r->Buy();
                    //     r->busy = false;
                    //     r->ResetTarget();
                    // } else {
                    //     KeepRobotWait(i);
                    // }

                    r->Buy();
                    r->busy = false;
                    workbenches_[r->target_type][r->target_id].has_been_assigned = false;
                    r->ResetTarget();
                } else {
                    if(1&(workbenches_[r->target_type][r->target_id].sources_status>>thing_type)){
                        FindNearestSameWorkbench(r->target_type,r->target_id,i);
                        MoveRobot2Workbench(i,r->target_type,r->target_id);
                    } else {
                        r->Sell();
                        r->busy = false;
                        r->ResetTarget();
                    }
                }
            } else {
                MoveRobot2Workbench(i,r->target_type,r->target_id);
            }
        } else {
            SetTarget(i);
            cerr<<"id "<<i<<" target "<<robots_[i]->target_type<<","<<robots_[i]->target_id<<endl;
            MoveRobot2Workbench(i,r->target_type,r->target_id);
        }
    }
    cout<<"OK"<<endl; // no need to change
    fflush(stdout); // no need to change
}

void Solution::MoveRobot2Workbench(int& id_robo,int& type,int& id_wb){

    if(robots_[id_robo]->target_id == -1){
        KeepRobotWait(id_robo);
        return;
    }

    // calculate rotation speed
    float border_theta = 0.;
    if(robots_[id_robo]->thing_carry) border_theta = 0.2448;
    else border_theta = 0.1222;
    float x_r=robots_[id_robo]->x, y_r=robots_[id_robo]->y;
    float x_w=workbenches_[type][id_wb].x, y_w=workbenches_[type][id_wb].y;
    float toward_where= atan2(y_w-y_r,x_w-x_r);
    float diff = robots_[id_robo]->face_where-toward_where;
    float abs_diff = abs(diff);
    float sign = diff<0?1.:-1.;
    float rot_speed = 0.;

    if(abs_diff > border_theta){
        rot_speed = sign*M_PI;
    } else if(abs_diff > 0.01745){
        rot_speed = 0.15;
    } else {
        rot_speed = 0.;
    }

    float speed = 0.;
    // calculate linear speed
    robots_[id_robo]->target_distance = CalculateDistance(id_robo,type,id_wb);
    float border = 0.;
    if(robots_[id_robo]->thing_carry) border = 1.2708;
    else border = 0.9163;
    double dis = robots_[id_robo]->target_distance;
    speed = (dis<border?0.5:6.);
    speed = (dis<0.4?0.:speed);

    // pub movement actions
    robots_[id_robo]->Forward(speed);
    robots_[id_robo]->Rotate(rot_speed);
}

float Solution::CalculateDistance(int& id_robo,int type,int& id_wb){
    float x_r=robots_[id_robo]->x, y_r=robots_[id_robo]->y;
    float x_w=workbenches_[type][id_wb].x, y_w=workbenches_[type][id_wb].y;
    double distance = sqrt(pow(x_r-x_w,2)+pow(y_r-y_w,2));
    return distance;
}

void Solution::KeepRobotWait(int& id_robo){
    float zero = 0.;
    robots_[id_robo]->Forward(zero);
    robots_[id_robo]->Rotate(zero);
}

void Solution::SetTarget(int& id_robo){
    if(robots_[id_robo]->thing_carry){
        // situation 1: things on robot, unbusy, wants know where to go
        int type_carry = robots_[id_robo]->thing_carry;
        float dis = 99.;
        if(type_carry == 7){
            SelectNearestWorkbench(dis,8,id_robo);
            SelectNearestWorkbench(dis,9,id_robo);
        } else if(type_carry==4 || type_carry==5 || type_carry==6){
            SelectNearestWorkbench(dis,7,id_robo);
            SelectNearestWorkbench(dis,9,id_robo);
        } else if(type_carry==1){
            SelectNearestWorkbench(dis,4+id_robo%2,id_robo);
        } else if(type_carry==2){
            SelectNearestWorkbench(dis,4+2*(id_robo%2),id_robo);
        } else if(type_carry==3){
            SelectNearestWorkbench(dis,6-id_robo%2,id_robo);
        } else cerr<<"ERROR carry "<<type_carry<<endl;

        robots_[id_robo]->busy = true;
    } else {
        CheckProduct(id_robo);
        if(robots_[id_robo]->HasTarget()){
            robots_[id_robo]->busy = true;
        } else {
            KeepRobotWait(id_robo);
        }
    }
}

void Solution::SelectNearestWorkbench(float& dis_now, int type, int& id_robo){
    for(int i=0;i<workbenches_[type].size();++i){
        float temp_dis=99.;
        if(temp_dis = CalculateDistance(id_robo,8,i) < dis_now){
            dis_now = temp_dis;
            robots_[id_robo]->target_id = i;
            robots_[id_robo]->target_type = type;
        }
    }
}

void Solution::CheckProduct(int& id_robo){
    for(int i=7;i>3;--i){
        for(int j=0;j<workbenches_[i].size();++j){
            auto wb = &workbenches_[i][j];
            // cerr<<"status "<<wb.product_status<<" has "<<!wb.has_been_assigned<<endl;
            if(wb->product_status && !wb->has_been_assigned){
                wb->has_been_assigned = true;
                robots_[id_robo]->target_type = i;
                robots_[id_robo]->target_id = j; 
                return;
            }
        }
    }

    rand_num++;
    int target_type = rand_num%3+1;
    for(int j=0;j<workbenches_[target_type].size();++j){
        auto wb = &workbenches_[target_type][j];
        // cerr<<"status "<<wb.product_status<<" has "<<!wb.has_been_assigned<<endl;
        if(wb->product_status && !wb->has_been_assigned){
            wb->has_been_assigned = true;
            robots_[id_robo]->target_type = target_type;
            robots_[id_robo]->target_id = j; 
            return;
        }
    }

}

void Solution::FindNearestSameWorkbench(int& type, int& id, int& id_robo){
    float dis = 99.;
    float x1 = workbenches_[type][id].x, y1 = workbenches_[type][id].y;
    for(int i=0;i<workbenches_[type].size();++i){
        if(i==id) continue;
        float x2 = workbenches_[type][i].x, y2 = workbenches_[type][i].y;
        float temp_dis = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
        if(temp_dis < dis){
            dis = temp_dis;
            robots_[id_robo]->target_id = i;
        }
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