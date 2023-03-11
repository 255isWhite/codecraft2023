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
        // cerr<<line<<endl;
        if(0==count){
            stringstream ss(line);
            while (ss>>word)
            {
                int x = stoi(word);
                if(x<1e4) current_frame_ = x;
                else current_money_ = x;
            }
        } else if(1==count && !know_num_workbenches_){
            stringstream ss(line);
            while (ss>>word)
            {
                int x = stoi(word);
                nums_workbench_ = x;
                know_num_workbenches_ = true;
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
            Workbench wbench;
            wbench.x = fdata[0];
            wbench.y = fdata[1];
            wbench.time_left = idata[0];
            wbench.sources_status = idata[1];
            wbench.product_status = idata[2];
            workbenches_.push_back(wbench);
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
            Robot rb;
            rb.workbench_near = idata[0];
            rb.thing_carry = idata[1];
            rb.time_coef = fdata[0];
            rb.collision_coef = fdata[1];
            rb.angular_speed = fdata[2];
            rb.linear_speed_x = fdata[3];
            rb.linear_speed_y = fdata[4];
            rb.face_where = fdata[5];
            rb.x = fdata[6];
            rb.y = fdata[7];
            robots_.push_back(rb);
            // for(auto& i:idata){cerr<<i<<" ";}
            // for(auto& i:fdata){cerr<<i<<" ";}
            // cerr<<endl;
        } else if(line[0]=='O' && line[1]=='K'){
            auto end = high_resolution_clock::now();
            auto cost = duration_cast<duration<double,ratio<1,1000>>>(end - start);
            cerr<<"get info cost "<<cost.count()<<" ms\n";
            return true;
        }
        ++count;
    }

    // getline(cin,line);
    // cerr<<line;
    //cerr<<line<<endl;
}

void Solution::AssaignTasks(){
    //do something
}

void Solution::PublishOrders(){
    cout<<current_frame_<<endl;
    for(int i=0;i<4;++i){
        cout<<"forward "<<i<<" 6"<<endl;
        cout<<"rotate "<<i<<" 3"<<endl;
    }
    cout<<"OK"<<endl;
}



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