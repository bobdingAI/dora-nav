extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}


#include"task_server.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>



bool task_server::onTaskCallRecvd(char* msg, void *dora_context)
{
    std::cout << "***********Task**************" << std::endl;
    Task_h *task = reinterpret_cast<Task_h *>(msg);
    std::cout << "I recv a mission: " << task->task_type << endl;

    switch (task->task_type)
    {
        case 1:
        {
            std::thread thread([this, task, dora_context]{
                this->Avoid_obstacle(task->s_start, task->s_end, task->info, dora_context);
            });
            thread.detach();
            break;
        }   
    }

    return true;
}


//****************************************接受消息**********************************************

void task_server::onCurPoseSDRecvd(char *msg)
{
    std::cout << "***********cur_pose**************" << std::endl;
    mtx_pose.lock();
    CurPose_h *cur_pose = reinterpret_cast<CurPose_h *>(msg);
    if (cur_pose != nullptr) 
    {  
        dora_cur_pose = *cur_pose;  
    } 
    else
    {
        std::cerr << "Invalid pointer: cur_pose is null." << std::endl;
    }
    mtx_pose.unlock();
}

void task_server::onStatMsgRecvd(char *msg)
{
    std::cout << "***********stat**************" << std::endl;

    mtx_stat.lock();
    VehicleStat_h *cur_stat = reinterpret_cast<VehicleStat_h *>(msg);
    if (cur_stat != nullptr) 
    {
        dora_stat_data = *cur_stat;
    }
    else
    {
        std::cerr << "Invalid pointer: cur_stat is null." << std::endl;
    }
    mtx_stat.unlock();
}

void task_server::onAttriRecvd(char *msg)
{
    std::cout << "***********Attri**************" << std::endl;
    mtx_road.lock();
    RoadAttri_h *road_msg = reinterpret_cast<RoadAttri_h *>(msg);
    if (road_msg != nullptr) 
    {
        dora_road_attri = *road_msg;
    }
    else
    {
        std::cerr << "Invalid pointer: road_msg is null." << std::endl;
    }
    mtx_road.unlock();
    
}

void task_server::onTargetRecvd(char *msg)
{
    mtx_object.lock();
    ObjectArray_h *target_msg = reinterpret_cast<ObjectArray_h *>(msg);
    if (target_msg != nullptr) 
    {
        dora_object_arry = *target_msg;
    }
    else
    {
        std::cerr << "Invalid pointer: target_msg is null." << std::endl;
    }
    mtx_object.unlock();
}

//*****************************************************************************

//***********************************加锁访问***********************************

inline CurPose_h task_server::get_Curpose_WithMutex()
{
    CurPose_h res;
    mtx_pose.lock();
    res = dora_cur_pose;
    mtx_pose.unlock();
    return res;
}

inline VehicleStat_h task_server::get_Stat_WithMutex()
{
    VehicleStat_h res;
    mtx_stat.lock();
    res = dora_stat_data;
    mtx_stat.unlock();
    return res;
}

inline RoadAttri_h task_server::get_RoadAttri_WithMutex()
{
    RoadAttri_h res;
    mtx_road.lock();
    res = dora_road_attri; 
    mtx_road.unlock();
    return res;
}

inline ObjectArray_h task_server::get_Object_WithMutex(){
    ObjectArray_h res;
    mtx_object.lock();
    res = dora_object_arry;
    mtx_object.unlock();
    return res;
}

//***************************************************************************************************************************


//*******************************************************车辆控制**************************************************************
/**
 * @brief 设置速度
 * @param 
 */
inline bool task_server::SetSpeed(bool enable,float speed,std::string source, void *dora_context){
    Controlsrv_h *srv = new Controlsrv_h;
    srv->type   = srv->Is_swith_speed;
    srv->enable = enable;
    srv->info   = speed;
    srv->source = source;

    Controlsrv_h * srv_out = srv;
    char * output_data = (char*)srv_out;
    std::string out_id = "SetSpeed_service";
    //std::cout<<json_string<<endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Controlsrv_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
    delete srv;

}

inline bool task_server::SetRoute(bool enabel,float d,float s, void* dora_context){
    Route_h *srv = new Route_h;
    srv->enable    = enabel;
    srv->target_s  = s;
    srv->target_d  = d;


    Route_h * srv_out = srv;
    char * output_data = (char*)srv_out;
    std::string out_id = "routing_service";
    //std::cout<<json_string<<endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Route_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
    delete srv;
}
 

inline bool task_server::SetStop(bool enable,float distance,std::string source, void* dora_context){
    Controlsrv_h *srv = new Controlsrv_h;
    srv->type   = srv->Is_stop;
    srv->enable = enable;
    srv->info   = distance;
    srv->source = source;
       
    Controlsrv_h * srv_out = srv;
    char * output_data = (char*)srv_out;
    std::string out_id = "SetStop_service";
    //std::cout<<json_string<<endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Controlsrv_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
    delete srv;
}

//***********************************************************************************************************

//***************************************************任务执行需要的功能函数**************************************
bool task_server::Is_have_target(int position,float s_cur,float front,float back,ObjectArray_h &objectArry)
{
    
    float road_w = get_RoadAttri_WithMutex().road_width;
    for(const auto &obj : objectArry.objs)
    {
        //整个车道
        if(position == 0){
            if(obj.s_pos>(s_cur-back) && obj.s_pos<(s_cur+front)) return true;
        }
        //左车道
        if(position == 1){
            if(obj.s_pos>(s_cur-back) && obj.s_pos<(s_cur+front)){
                if(obj.d_pos < -road_w/4) return true;
            }
        }
        //右车道
        if(position == 2){
            if(obj.s_pos>(s_cur-back) && obj.s_pos<(s_cur+front)){
                if(obj.d_pos > -road_w/4) return true;
            }
        }
    }
    return false;
}


//*****************************************************************************************************************


//*******************************************************任务表*****************************************************
/**
 * @brief 根据线程id为任务表添加任务
 * @param 
 */
void task_server::Add_task(std::string str){
    std::stringstream sin;
    sin << std::this_thread::get_id();
    std::string stid = sin.str();
    unsigned long long tid = std::stoull(stid);
    task_table[tid] = str;
}

/**
 * @brief 删除该线程对应的任务表中任务
 * @param 
 */
void task_server::Delete_task(){
    std::stringstream sin;
    sin << std::this_thread::get_id();
    std::string stid = sin.str();
    unsigned long long tid = std::stoull(stid);
    task_table.erase(tid);
}

//*************************************************************************************************************

void task_server::Avoid_obstacle(float s_start,float s_end,float info, void *dora_context){
    Add_task("Avoid_obstacle_mission");

    CurPose_h           cur_pose;
    ObjectArray_h       objsArry;
    VehicleStat_h       stat_data;
    float               road_w;

    const int rate = 10;   // 设定频率为 xx HZ
    const chrono::milliseconds interval((int)(1000/rate));

    do{
        this_thread::sleep_for(interval);
        cur_pose  = get_Curpose_WithMutex();
        stat_data = get_Stat_WithMutex();
        objsArry  = get_Object_WithMutex();
        road_w    = get_RoadAttri_WithMutex().road_width;

        if(cur_pose.s > 40 && cur_pose.s < 41){
            SetRoute(true, road_w/2, 12, dora_context);
            SetRoute(false,-road_w/2, 12, dora_context);
        }

        if(cur_pose.s > 50)
        {
            SetStop(true, 5, "Avoid_obstacle_mission", dora_context);
        }

        // if (objsArry.objs.size() >=1){
        //     for (const auto& obj : objsArry.objs){
        //        if ( obj.s_pos -cur_pose.s<=20&& obj.s_pos - cur_pose.s>=9){
        //          SetSpeed(true,5, "Avoid_obstacle", dora_context);
        //        }
        //     }
        // }

        // if(cur_pose.d < -road_w/4){ 
        //      if(!Is_have_target(2,cur_pose.s, 11, 6, objsArry) && stat_data.veh_speed>1)
        //      {
        //         SetRoute(true, road_w/25, 12, dora_context);
        //         std::cout << "change to left lane" << std::endl; 
        //     }
        //  }
        // if(cur_pose.d >= -road_w/4){ 
        //     // SetStop(false,5, "Avoid_obstacle_mission");
        //      if(Is_have_target(2,cur_pose.s, 13, 0,objsArry) &&
        //        !Is_have_target(1,cur_pose.s, 8, 8,objsArry) && stat_data.veh_speed>1){
        //             SetRoute(true,-road_w/3, 8, dora_context);
        //             std::cout << "change to right lane" << std::endl;
        //             this_thread::sleep_for(std::chrono::seconds(2));                                  
        //      }     
        // }     

    }while(cur_pose.s < s_end && cur_pose.s > s_start); 
    // SetRoute(false,-road_w/2, 12, dora_context);
    //SetSpeed(false, 4, "Avoid_obstacle", dora_context);

    Delete_task();
}


int run(void *dora_context)
{

    while (true)
    {

        void * event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            task_server core;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            std::cout << "Input Data length: " << data_len << std::endl;
            if(strcmp("cur_pose_all", data_id,12) == 0)
            {
                core.onCurPoseSDRecvd(data);
            }
            else if(strcmp("VehicleStat", data_id,11) == 0)
            {
                core.onStatMsgRecvd(data);
            }
            else if(strcmp("road_attri_msg", data_id,14) == 0)
            {
                core.onAttriRecvd(data);
            }
            else if(strcmp("task_exc_service", data_id),16 == 0)
            {
                core.onTaskCallRecvd(data, dora_context);
            }

        }

        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "task_server_exc" << std::endl;

    auto dora_context = init_dora_context_from_env();

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "END task_server_exc" << std::endl;

    return ret;
}


