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
        } else if(1==count && current_frame_==1){
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
            if(current_frame_ == 1 && count==2) workbenches_.resize(nums_workbench_);
            auto wbench = &workbenches_[count-2];
            wbench->x = fdata[0];
            wbench->y = fdata[1];
            wbench->type = idata[0];
            wbench->time_left = idata[1];
            wbench->sources_status = idata[2];
            wbench->product_status = idata[3];
            wbench->idx = (count-2);
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

void Solution::AssaignTasks(){
    cout<<current_frame_<<endl; // no need to change
    //do something
    for(int i=0;i<4;++i){
        auto r = robots_[i];
        if(r->busy){

            if(r->target_distance<0.4){
                cerr<<"here2\n";
                int thing_type = r->thing_carry;
                if(thing_type == 0){
                    if(workbenches_[r->target_id].product_status){
                        r->Buy();
                        r->busy = false;
                    } else {
                        KeepRobotWait(i);
                    }
                } else {
                    if(1&(workbenches_[r->target_id].sources_status>>thing_type)){
                        KeepRobotWait(i);
                    } else {
                        r->Sell();
                        r->busy = false;
                    }
                }
            } else {
                MoveRobot2Workbench(i,r->target_id);
            }
        } else {
            r->busy = true;
            SetTarget(i);
            MoveRobot2Workbench(i,r->target_id);
        }
    }
    cout<<"OK"<<endl; // no need to change
    fflush(stdout); // no need to change
}

void Solution::MoveRobot2Workbench(int& id_robo,int& id_wb){
    // calculate linear speed
    float dis = robots_[id_robo]->target_distance;
    int target = robots_[id_robo]->target_id;
    int near = robots_[id_robo]->workbench_near;
    float speed = (near == target?0.:(min(dis-0.4,6.)));

    // calculate rotation speed
    float x_r=robots_[id_robo]->x, y_r=robots_[id_robo]->y;
    float x_w=workbenches_[id_wb].x, y_w=workbenches_[id_wb].y;
    float toward_where= atan2(y_w-y_r,x_w-x_r);
    if(id_robo==9){
        cerr<<"face "<<robots_[id_robo]->face_where<<endl;
        cerr<<"toward "<<toward_where<<endl;
    }
    float diff = robots_[id_robo]->face_where-toward_where;
    float sign = diff>0?-1.:1.;
    float rot_speed = 0.;
    if(abs(diff)<0.1){
        rot_speed = 0;
    } else {
        rot_speed = sign * diff * dis;
    }

    // pub movement actions
    robots_[id_robo]->Forward(speed);
    robots_[id_robo]->Rotate(rot_speed);
}

void Solution::CalculateDistance(int& id_robo,int& id_wb){
    float x_r=robots_[id_robo]->x, y_r=robots_[id_robo]->y;
    float x_w=workbenches_[id_wb].x, y_w=workbenches_[id_wb].y;
    double distance = sqrt(pow(x_r-x_w,2)+pow(y_r-y_w,2));
    robots_[id_robo]->target_distance = distance;
}

void Solution::KeepRobotWait(int& id_robo){
    float zero = 0.;
    robots_[id_robo]->Forward(zero);
    robots_[id_robo]->Rotate(zero);
}

void Solution::SetTarget(int& id_robo){
    robots_[id_robo]->target_id = 5;
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