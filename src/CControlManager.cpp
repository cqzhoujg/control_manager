/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2020-3-4
Description: CTrackRobot
**************************************************/

#include "CControlManager.h"

CControlManager::CControlManager():
    m_bWaitForDone(false),
    m_bManControlCmd(false),
    m_bManData(false),
    m_bPause(false),
    m_bReturn(false),
    m_bRectifyOdom(true),
    m_bMotorError(false),
    m_bMotorStatusError(false),
    m_bMotorBusy(false),
    m_bTaskReturning(false),
    m_bRecordResume(false),
    m_bPausedDone(false),
    m_dRobotSpeed(0.0),
    m_dPauseOdom(0.0),
    m_dPauseOffset(0.0),
    m_dPauseRecordOdom(0.0),
    m_nTransId(0),
    unMotorStatus(0),
    m_sTaskStatus("idle")
{
    ros::NodeHandle PublicNodeHandle;
    ros::NodeHandle PrivateNodeHandle("~");

    PrivateNodeHandle.param("precise_return", m_nPreciseReturn, 1);
    PrivateNodeHandle.param("print_level", m_sPrintLevel, std::string("debug"));
    PrivateNodeHandle.param("record_laser", m_nRecordLaser, 1);
    PrivateNodeHandle.param("first_filter_radius", m_d1stFilterRadius, 0.1);
    PrivateNodeHandle.param("first_filter_num", m_n1stFilterNum, 30);
    PrivateNodeHandle.param("second_filter_radius", m_d2ndFilterRadius, 0.01);
    PrivateNodeHandle.param("second_filter_num", m_n2ndFilterNum, 3);
    PrivateNodeHandle.param("detection_range", m_dDetectRange, 0.05);
    PrivateNodeHandle.param("adjust_mileage", m_nAdjustMileage, 0);
    PrivateNodeHandle.param("adjust_min_range", m_dAdjustMinRange, 0.02);
    PrivateNodeHandle.param("adjust_max_range", m_dAdjustMaxRange, 0.4);

    PublicNodeHandle.param("sub_cmd_topic", m_sSubCmdTopic, std::string("train_ctrl_cmd"));
    PublicNodeHandle.param("sub_manager_status_topic", m_sSubManagerStatusTopic, std::string("train_task_status"));
    PublicNodeHandle.param("sub_move_ack_topic", m_sSubMoveAckTopic, std::string("train_move_ack"));
    PublicNodeHandle.param("sub_pos_topic", m_sSubDataTopic, std::string("train_robot_position"));
    PublicNodeHandle.param("sub_motor_heart_beat_topic", m_sSubHeartBeatTopic, std::string("train_motor_heart_beat"));
    PublicNodeHandle.param("pub_manager_cmd_topic", m_sPubManagerCmdTopic, std::string("train_move_cmd"));
    PublicNodeHandle.param("pub_status_topic", m_sPubAckTopic, std::string("train_ctrl_ack"));
	PublicNodeHandle.param("pub_data_file_topic", m_sPubDataFileTopic, std::string("train_data_file"));
	PublicNodeHandle.param("pub_heart_beat_topic", m_sPubHearBeatTopic, std::string("train_robot_heart_beat"));

    ROS_INFO("[ros param] precise_return:%d", m_nPreciseReturn);
    ROS_INFO("[ros param] print_level:%s", m_sPrintLevel.c_str());
    ROS_INFO("[ros param] record_laser:%d", m_nRecordLaser);
    ROS_INFO("[ros param] first_filter_radius:%f", m_d1stFilterRadius);
    ROS_INFO("[ros param] first_filter_num:%d", m_n1stFilterNum);
	ROS_INFO("[ros param] second_filter_radius:%f", m_d2ndFilterRadius);
    ROS_INFO("[ros param] second_filter_num:%d", m_n2ndFilterNum);
    ROS_INFO("[ros param] detection_range:%f", m_dDetectRange);
    ROS_INFO("[ros param] adjust_mileage:%d", m_nAdjustMileage);
    ROS_INFO("[ros param] adjust_min_range:%f", m_dAdjustMinRange);
	ROS_INFO("[ros param] adjust_max_range:%f", m_dAdjustMaxRange);

    ROS_INFO("[ros param] sub_cmd_topic:%s", m_sSubCmdTopic.c_str());
    ROS_INFO("[ros param] sub_manager_status_topic:%s", m_sSubManagerStatusTopic.c_str());
    ROS_INFO("[ros param] sub_move_ack_topic:%s", m_sSubMoveAckTopic.c_str());
    ROS_INFO("[ros param] sub_pos_topic:%s", m_sSubDataTopic.c_str());
    ROS_INFO("[ros param] sub_heart_beat_topic:%s", m_sSubHeartBeatTopic.c_str());
    ROS_INFO("[ros param] pub_manager_cmd_topic:%s", m_sPubManagerCmdTopic.c_str());
    ROS_INFO("[ros param] pub_status_topic:%s", m_sPubAckTopic.c_str());
    ROS_INFO("[ros param] pub_data_file_topic:%s", m_sPubDataFileTopic.c_str());
    ROS_INFO("[ros param] pub_heart_beat_topic:%s", m_sPubHearBeatTopic.c_str());

    m_TaskRunCmd.cmd = "run";
    m_TaskRunCmd.move_type = ONLY_MOVE;
    m_TaskRunCmd.back = 0;
    m_TaskRunCmd.speed = 0.5;
    m_TaskRunCmd.distance = 0;

    double dTimeout = 60.0;
    custom_msgs::TrainRobotHeartBeat::ConstPtr MotorStatus = \
        ros::topic::waitForMessage<custom_msgs::TrainRobotHeartBeat>(m_sSubHeartBeatTopic, ros::Duration(dTimeout));

    if(MotorStatus == nullptr)
    {
        ROS_ERROR("[CControlManager] wait for msg robot_status timeout: %fs.",dTimeout);
        exit(-1);
    }
    timespec_get(&m_tMotorHeartTime, TIME_UTC);

    // Se the logging level manually to Debug, Info, Warn, Error
    ros::console::levels::Level printLevel = ros::console::levels::Info;
    if(m_sPrintLevel == "debug")
        printLevel = ros::console::levels::Debug;
    else if(m_sPrintLevel == "warn")
        printLevel = ros::console::levels::Warn;
    else if(m_sPrintLevel == "error")
        printLevel = ros::console::levels::Error;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, printLevel);

    m_CommandSub = PublicNodeHandle.subscribe<custom_msgs::TrainRobotControl>(m_sSubCmdTopic, 1, boost::bind(&CControlManager::CommandCallBack, this, _1));
    m_StatusSub = PublicNodeHandle.subscribe<custom_msgs::GeneralTopic>(m_sSubManagerStatusTopic, 1, boost::bind(&CControlManager::StatusCallBack, this, _1));
    m_DataSub = PublicNodeHandle.subscribe<custom_msgs::TrainRobotPosition>(m_sSubDataTopic, 10, boost::bind(&CControlManager::DataCallBack, this, _1));
    m_HeartBeatSub = PublicNodeHandle.subscribe<custom_msgs::TrainRobotHeartBeat>(m_sSubHeartBeatTopic, 10, boost::bind(&CControlManager::MotorHeartBeatCallBack, this, _1));
    m_MoveAckSub = PublicNodeHandle.subscribe<custom_msgs::TrainRobotControlAck>(m_sSubMoveAckTopic, 10, boost::bind(&CControlManager::MoveAckCallBack, this, _1));

    m_CommandPub = PublicNodeHandle.advertise<custom_msgs::TrainRobotControl>(m_sPubManagerCmdTopic, 10);
    m_CtrlAckPub = PublicNodeHandle.advertise<custom_msgs::TrainRobotControlAck>(m_sPubAckTopic, 10);
    m_DataFileNamePub = PublicNodeHandle.advertise<custom_msgs::GeneralTopic>(m_sPubDataFileTopic, 10);
    m_HeartBeatPub = PublicNodeHandle.advertise<custom_msgs::TrainRobotHeartBeat>(m_sPubHearBeatTopic, 10);

    m_OffsetCaller = PublicNodeHandle.serviceClient<custom_msgs::ControlService>("odom_offset");
    m_CurrentPosCaller = PublicNodeHandle.serviceClient<custom_msgs::CurrentPositionRequest>("current_position");

    try
    {
        m_pOdomManThread = new std::thread(std::bind(&CControlManager::TaskManThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CControlManager] malloc odom man thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pDataManThread = new std::thread(std::bind(&CControlManager::DataManThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CControlManager] malloc data man thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pReturnThread = new std::thread(std::bind(&CControlManager::ReturnThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CControlManager] malloc data man thread failed, %s", exception.what());
        exit(-1);
    }

    try
    {
        m_pMonitorThread = new std::thread(std::bind(&CControlManager::MonitorThreadFunc, this));
    }
    catch (std::bad_alloc &exception)
    {
        ROS_ERROR("[CControlManager] malloc monitor thread failed, %s", exception.what());
        exit(-1);
    }
}

/***********************************************************
Function: CControlManager::CommandCallBack
Description:前端web消息的回调函数
Input: const custom_msgs::TrainRobotControl::ConstPtr &Command, topic消息
Output: void
Others: void
************************************************************/
void CControlManager::CommandCallBack(const custom_msgs::TrainRobotControl::ConstPtr &Command)
{
    custom_msgs::TrainRobotControl Cmd = *Command;
    custom_msgs::TrainRobotControlAck Ack;
    Ack.trans_id = Command->trans_id;
    Ack.header.stamp = ros::Time::now();
    Ack.header.frame_id = Command->cmd;
    Ack.sender = "control_manager";
    Ack.ack = "succeed";
    m_sPresetFile = Cmd.preset_file;

    if(Cmd.cmd == "joy_run")
    {
        if(m_sTaskStatus == "forward" || m_bMotorError || m_sTaskStatus == "backward")
        {
            return;
        }
        m_CommandPub.publish(Cmd);
        return;
    }
    ROS_INFO("[CommandCallBack] cmd:%s, distance:%f, speed:%f, back:%d", Cmd.cmd.c_str(), Cmd.distance, Cmd.speed, Cmd.back);

    if(m_bMotorError)
    {
        Ack.ack = "failed";
        Ack.data = "motor error";
        ROS_WARN("[CommandCallBack] motor error");
    }
    else if(m_bMotorBusy && Cmd.cmd != "stop")
    {
        ROS_WARN("[CommandCallBack] the motor is busy, cmd:%s", Cmd.cmd.c_str());
        Ack.ack = "failed";
        Ack.data = "busy";
    }
    else if(m_bPause && Cmd.cmd != "start" && Cmd.cmd != "adjust_mileage" && Cmd.cmd != "return")
    {
        ROS_WARN("[CommandCallBack] the motor is paused, cmd:%s", Cmd.cmd.c_str());
        Ack.data = "paused";
        ROS_WARN("[CommandCallBack] paused");
    }
    else if((!m_bPause || !m_bPausedDone) && Cmd.cmd == "start")
    {
        ROS_WARN("[CommandCallBack] the motor is not paused, cmd:%s", Cmd.cmd.c_str());
        Ack.ack = "failed";
        Ack.data = "not paused";
    }
    else if(Cmd.cmd == "run")
    {
        if(m_bManControlCmd)
        {
            ROS_WARN("[CommandCallBack] control manager is busy, cmd:%s", Cmd.cmd.c_str());
            Ack.ack = "failed";
            Ack.data = "busy";
        }
        else
        {
            m_TaskRunCmd = Cmd;
            m_sPresetFile = Cmd.preset_file;
            m_dRobotSpeed = Cmd.speed;
            m_nTaskName = TASK_RUN;

            ExecuteTask();
        }

    }
    else if(Cmd.cmd == "alarm_run")
    {
        if(m_bManControlCmd)
        {
            ROS_WARN("[CommandCallBack] control manager is busy, cmd:%s", Cmd.cmd.c_str());
            Ack.ack = "failed";
            Ack.data = "busy";
        }
        else
        {
            m_AlarmRunCmd = Cmd;
            m_sPresetFile = Cmd.preset_file;
            m_dRobotSpeed = Cmd.speed;
            m_nTaskName = ALARM_RUN;
            ExecuteTask();
        }
    }
    else if(Cmd.cmd == "return")
    {
        if(m_bManControlCmd && !m_bPause)
        {
            ROS_WARN("[CommandCallBack] control manager is busy, cmd:%s", Cmd.cmd.c_str());
            Ack.ack = "failed";
            Ack.data = "busy";
        }
        else
        {
            ExecuteReturn();
            m_sTaskStatus = Cmd.cmd;
        }
    }
    else if(Cmd.cmd == "stop")
    {
        if(m_bPause)
        {
            ROS_WARN("[CommandCallBack] already paused, cmd:%s", Cmd.cmd.c_str());
            Ack.data = "paused";
        }
        else if(m_sTaskStatus != "forward" && m_sTaskStatus != "backward" && m_sTaskStatus != "return")
        {
            ROS_WARN("[CommandCallBack] no task to pause, cmd:%s", Cmd.cmd.c_str());
            Ack.ack = "failed";
            Ack.data = "no task to pause";
        }
        else
        {
            m_bPause = true;
            m_bPausedDone = false;
            ROS_INFO("[CommandCallBack] m_bPause:%d", m_bPause);
        }
    }
    else if(Cmd.cmd == "start")
    {
        m_bPause = false;
        m_nTaskName = Task_RESUME;
        ExecuteTask();
        m_sTaskStatus = Command->speed > 0 ? "forward" : "backward";
        ROS_INFO("[CommandCallBack] m_bPause:%d", m_bPause);
    }
    else if(Cmd.cmd == "adjust_mileage")
    {
        if(!PubMoveCmd(Cmd, Ack.data))
        {
            Ack.ack = "failed";
        }
    }
    else
    {
        ROS_ERROR("[CommandCallBack] unknown cmd:%s", Cmd.cmd.c_str());
        Ack.ack = "failed";
        Ack.data = "unknown cmd";
    }
    m_CtrlAckPub.publish(Ack);
}

/***********************************************************
Function: CControlManager::StatusCallBack
Description: 机器人状态执行函数
Input: const custom_msgs::TrainRobotControlAck::ConstPtr &Status, topic消息
Output: void
Others: void
************************************************************/
void CControlManager::StatusCallBack(const custom_msgs::GeneralTopic::ConstPtr &Status)
{
    if(Status->data == "task_done" || Status->data == "relocation_done")
    {
        m_cFileRw.CloseFile();

        if(m_bWaitForDone)
        {
            m_bWaitForDone = false;
        }
        if(m_bReturn)
        {
            m_bReturn = false;
        }

        m_bMotorError = false;
    }
}

/***********************************************************
Function: CControlManager::MotorHeartBeatCallBack
Description: track_robot心跳监听
Input: const custom_msgs::TrainRobotHeartBeat::ConstPtr &HeartBeat, topic消息
Output: void
Others: void
************************************************************/
void CControlManager::MotorHeartBeatCallBack(const custom_msgs::TrainRobotHeartBeat::ConstPtr &HeartBeat)
{
    m_HeartBeat = *HeartBeat;
    if(m_bPause)
    {
        m_sTaskStatus = "pause";
    }
    if ((HeartBeat->status & (uint8_t)0x03) > 0)
    {
        if(HeartBeat->status != unMotorStatus)
            ROS_WARN("[MotorHeartBeatCallBack] motor status error,%d",HeartBeat->status);
        m_bMotorError = true;
        m_bMotorStatusError = true;
        m_sTaskStatus = "error";
    }
    else if(m_bMotorStatusError)
    {
        m_sTaskStatus = "idle";
        m_bMotorError = false;
        m_bMotorStatusError = false;
    }
    m_bMotorBusy = abs(HeartBeat->velocity_x) > 0.0001;
    unMotorStatus = HeartBeat->status;
    timespec_get(&m_tMotorHeartTime, TIME_UTC);

    custom_msgs::TrainRobotHeartBeat NewHeartBeat = m_HeartBeat;

    NewHeartBeat.task_status = m_sTaskStatus;

    m_HeartBeatPub.publish(NewHeartBeat);
}

/***********************************************************
Function: CControlManager::MoveAckCallBack
Description: track_robot心跳监听
Input: const custom_msgs::TrainRobotControlAck::ConstPtr &MoveAck, topic消息
Output: void
Others: void
************************************************************/
void CControlManager::MoveAckCallBack(const custom_msgs::TrainRobotControlAck::ConstPtr &MoveAck)
{
    if(MoveAck->trans_id == m_nTransId)
    {
        m_MoveAckMsg = *MoveAck;
        m_MoveAckCondition.notify_all();
    }
}

/***********************************************************
Function: CControlManager::DataCallBack
Description: 机器人历程及激光数据消息回调函数
Input: const custom_msgs::TrainRobotPosition::ConstPtr &PosData, topic消息
Output: void
Others: void
************************************************************/
void CControlManager::DataCallBack(const custom_msgs::TrainRobotPosition::ConstPtr &PosData)
{
    if(m_sTaskStatus == "alarm_run")
    {
        custom_msgs::TrainRobotPosition PosDataTemp= *PosData;
        if(FilterAndDetect(PosDataTemp))
        {
            m_vAlarmPos.push_back(PosData->x);
        }
        return;
    }

    string sPositionData;
    int nCount = 0;
    if(m_bRecordResume)
        sPositionData = (to_string(PosData->x + m_dPauseRecordOdom)).append(":");
    else
        sPositionData = (to_string(PosData->x)).append(":");

    for(int i = 0; i < PosData->y.size(); i++)
    {
        if(nCount == 0)
        {
            sPositionData.append(to_string(PosData->y[i])).append(",").append(to_string(PosData->z[i]));
            nCount++;
        }
        else
        {
            sPositionData.append(",").append(to_string(PosData->y[i])).append(",").append(to_string(PosData->z[i]));
        }
    }

    nCount = 0;

    if(!PosData->laser_dis.empty() && !PosData->laser_angle.empty() && 1 == m_nRecordLaser)
    {
        sPositionData.append(":");

        for(int i = 0; i < PosData->laser_dis.size(); i++)
        {
            if(nCount == 0)
            {
                sPositionData.append(to_string(PosData->laser_dis[i])).append(",").append(to_string(PosData->laser_angle[i]));
                nCount++;
            }
            else
            {
                sPositionData.append(",").append(to_string(PosData->laser_dis[i])).append(",").append(to_string(PosData->laser_angle[i]));
            }
        }
    }
    sPositionData.append("\n");
    m_cFileRw.Output(sPositionData);
}

/***********************************************************
Function: CControlManager::FilterAndDetect
Description: 根据输入直线的y,z值,输出格式化字符串
Input: y[] z[]
Output: bool
Others: void
************************************************************/
bool CControlManager::FilterAndDetect(custom_msgs::CurrentPosition &yzData)
{
    vector<double> vdDataY;
    vector<double> vdDataZ;
    bool bRet = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = yzData.response.y.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width*cloud->height);
    string re_str;

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = 1.0;
        cloud->points[i].y = yzData.response.y[i];
        cloud->points[i].z = yzData.response.z[i];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr FirstFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(cloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d1stFilterRadius);     //设置半径为5cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n1stFilterNum); //设置查询点的邻域点集数小于5的删除
    radiusoutlier.filter(*FirstFilterCloud);

    //二次滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr SecondFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    radiusoutlier.setInputCloud(FirstFilterCloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d2ndFilterRadius);     //设置半径为1cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n2ndFilterNum); //设置查询点的邻域点集数小于3的删除
    radiusoutlier.filter(*SecondFilterCloud);

    for(auto point : SecondFilterCloud->points)
    {
        if(point.y < m_AlarmRunCmd.limit_dis/1000)
        {
            bRet = true;
        }
    }
    cloud.reset();
    FirstFilterCloud.reset();
    SecondFilterCloud.reset();

    return bRet;
}

/***********************************************************
Function: CControlManager::FilterAndDetect
Description: 根据输入直线的y,z值,输出格式化字符串
Input: y[] z[]
Output: bool
Others: void
************************************************************/
bool CControlManager::FilterAndDetect(custom_msgs::TrainRobotPosition &yzData)
{
    vector<double> vdDataY;
    vector<double> vdDataZ;
    bool bRet = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = yzData.y.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width*cloud->height);
    string re_str;

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = 1.0;
        cloud->points[i].y = yzData.y[i];
        cloud->points[i].z = yzData.z[i];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr FirstFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(cloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d1stFilterRadius);     //设置半径为5cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n1stFilterNum); //设置查询点的邻域点集数小于5的删除
    radiusoutlier.filter(*FirstFilterCloud);

    //二次滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr SecondFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    radiusoutlier.setInputCloud(FirstFilterCloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d2ndFilterRadius);     //设置半径为1cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n2ndFilterNum); //设置查询点的邻域点集数小于3的删除
    radiusoutlier.filter(*SecondFilterCloud);

    for(auto point : SecondFilterCloud->points)
    {
        if(point.y < m_AlarmRunCmd.limit_dis/1000)
        {
            bRet = true;
        }
    }
    cloud.reset();
    FirstFilterCloud.reset();
    SecondFilterCloud.reset();

    return bRet;
}

//added by btrmg for filter 2020.04.11
/***********************************************************
Function: CControlManager::FilterPoints
Description: 根据输入直线的y,z值,输出格式化字符串
Input: y[] z[]
Output: string
Others: void
************************************************************/
string CControlManager::FilterPoints(vector<double> y,vector<double > z)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = y.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width*cloud->height);
    string re_str;
//    int point_number = cloud->points.size();
//    ROS_INFO("the point size is:%d",point_number);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = 1.0;
        cloud->points[i].y = y[i];
        cloud->points[i].z = z[i];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr FirstFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(cloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d1stFilterRadius);     //设置半径为5cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n1stFilterNum); //设置查询点的邻域点集数小于5的删除
    radiusoutlier.filter(*FirstFilterCloud);
//    if(cloud_after_Radius->points.size() != cloud->points.size())
//    {
//        int filtered_point = cloud->points.size()-cloud_after_Radius->points.size();
//        ROS_INFO("filter %d points",filtered_point);
//    }

    //二次滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr SecondFilterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    radiusoutlier.setInputCloud(FirstFilterCloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(m_d2ndFilterRadius);     //设置半径为1cm的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(m_n2ndFilterNum); //设置查询点的邻域点集数小于3的删除
    radiusoutlier.filter(*SecondFilterCloud);

    int nCount = 0;
    for(size_t k=0; k<SecondFilterCloud->points.size(); k++)
    {
        if(nCount == 0)
        {
            re_str.append(to_string(SecondFilterCloud->points[k].y)).append(",").append(to_string(SecondFilterCloud->points[k].z));
            nCount++;
        }
        else
        {
            re_str.append(",").append(to_string(SecondFilterCloud->points[k].y)).append(",").append(to_string(SecondFilterCloud->points[k].z));
        }
    }
    cloud.reset();
    FirstFilterCloud.reset();
    SecondFilterCloud.reset();
    return  re_str;
}
//added end

/***********************************************************
Function: CControlManager::TaskManThreadFunc
Description: 前段下发移动指令的分发处理机制
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::TaskManThreadFunc()
{
    ROS_INFO("[TaskManThreadFunc] start");
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_CmdMutex);
        if (!m_bManControlCmd)
        {
            m_CmdCondition.wait(lock);
        }
        lock.unlock();

        if(ReadIniConfig(m_ConfigIni))
        {
            if(m_nTaskName == TASK_RUN)
            {
                TaskRunManage();
            }
            else if(m_nTaskName == Task_RESUME)
            {
                TaskResume();
            }
            else
            {
                AlarmRunManage();
            }
        }
        m_bManControlCmd = false;
    }
    ROS_INFO("[TaskManThreadFunc] end");
}

/***********************************************************
Function: CControlManager::WaitMotionDone
Description: 等待单步运动结束
Input: custom_msgs::TrainRobotControl &ControlCmd, 运动消息指令
Output: true or false
Others: void
************************************************************/
int CControlManager::WaitMotionDone(custom_msgs::TrainRobotControl &ControlCmd, bool bIgnorePause)
{
    if(bIgnorePause)
        ROS_INFO("[WaitMotionDone] bIgnorePause:%d", bIgnorePause);
    m_bWaitForDone = true;
    string sOutput;
    if(!PubMoveCmd(ControlCmd, sOutput))
    {
        ROS_ERROR("[WaitMotionDone] %s", sOutput.c_str());
        m_sTaskStatus = "error";
        return MOTION_ERROR;
    }
    double dTimeLen = abs(ControlCmd.distance / ControlCmd.speed) + 15;
    timespec StartTime, CurrentTime;
    long pollInterval;
    timespec_get(&StartTime, TIME_UTC);
    while(m_bWaitForDone)
    {
        if(m_bMotorError)
        {
            ROS_ERROR("[WaitMotionDone] motor status error");
            m_sTaskStatus = "error";
            return MOTION_ERROR;
        }

        if(m_bPause && !bIgnorePause)
        {
            custom_msgs::TrainRobotControl Cmd = m_TaskRunCmd;
            Cmd.cmd = "stop";
            ROS_INFO("[WaitMotionDone] send cmd: %s", Cmd.cmd.c_str());
            if(!PubMoveCmd(Cmd, sOutput))
            {
                ROS_ERROR("[WaitMotionDone] %s", sOutput.c_str());
                m_sTaskStatus = "error";
                return MOTION_ERROR;
            }

            //读取
            custom_msgs::CurrentPosition CurrentPos;
            CurrentPos.request.data = "control_manager";
            bool bResult = m_CurrentPosCaller.call(CurrentPos);
            if(!bResult || CurrentPos.response.data != "succeed")
            {
                ROS_WARN("[ReturnMotion] call current position service failed");
                return MOTION_ERROR;
            }
            m_dPauseOdom = CurrentPos.response.x;
            ROS_DEBUG("[ReturnMotion] pause odom: %f", m_dPauseOdom);
            m_bPausedDone = true;
            m_cFileRw.CloseFile();
            m_sTaskStatus = "pause";
            return MOTION_PAUSE;
        }

        timespec_get(&CurrentTime, TIME_UTC);
        pollInterval=((CurrentTime.tv_sec - StartTime.tv_sec)\
                                 + (CurrentTime.tv_nsec - StartTime.tv_nsec)) / 1000000000;
        if(pollInterval > dTimeLen)
        {
            ROS_ERROR("[WaitMotionDone] wait for done timeout error,pollInterval=%ld",pollInterval);
            m_sTaskStatus = "error";
            return MOTION_ERROR;
        }
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    this_thread::sleep_for(std::chrono::milliseconds(200));
    return MOTION_DONE;
}

/***********************************************************
Function: CControlManager::ReturnMotion
Description: 返回至出发点
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::ReturnMotion()
{
    ROS_INFO("[ReturnMotion] start.");
    m_bReturn = true;
    m_bTaskReturning = true;

    //根据INI文件对运动任务进行分割
    FileRectify dataRectify;
    double dOffset = 0.0, dStepDis;
    int nTotal = m_ConfigIni.nTotal;
    m_sPlatId = m_ConfigIni.sPlatId;

    if(0 == nTotal || 1 != m_nPreciseReturn)
    {
        EmptyPresetReturn();
        return;
    }
    else
    {
        vector<string> vsMileage = m_ConfigIni.vsMileage;
        double dTargetDis[nTotal];
        int nCount = 0;
        for(auto dDis : m_ConfigIni.dTargetDis)
        {
            dTargetDis[nCount] = dDis;
            nCount++;
        }
        m_sOriginPresetId = m_TaskRunCmd.speed >= 0 ? vsMileage[0] : vsMileage[nTotal-1];
        //读取
        custom_msgs::CurrentPosition CurrentPos;
        CurrentPos.request.data = "control_manager";
        bool bResult = m_CurrentPosCaller.call(CurrentPos);
        if(!bResult || CurrentPos.response.data != "succeed")
        {
            ROS_ERROR("[ReturnMotion] call current position service failed");
            m_sTaskStatus = "error";
            return;
        }
        double dCurrentOdom = CurrentPos.response.x;
        ROS_DEBUG("[ReturnMotion] dCurrentOdom:%f",dCurrentOdom);

        //计算到达目标定经过的预置位点数
        int nGetOffsetTimes = 0;
        double dNewTargetDis[nTotal];
        vector<string> vsPresetId;;

        int nNewTargetCount = 0;
        for(int i=0; i<nTotal; i++)
        {
            if((m_TaskRunCmd.speed >= 0 && dTargetDis[i] < dCurrentOdom) ||\
                (m_TaskRunCmd.speed < 0 && dTargetDis[i] > dCurrentOdom))
            {
                nGetOffsetTimes++;
                dNewTargetDis[nNewTargetCount] = dTargetDis[i];
                vsPresetId.push_back(vsMileage[i]);
                nNewTargetCount++;
            }
        }
        ROS_INFO("[ReturnMotion] nGetOffsetTimes=%d",nGetOffsetTimes);
        if(nGetOffsetTimes <= 1)
        {
            EmptyPresetReturn();
        }
        else if(nGetOffsetTimes > 1)
        {
            bool bLessPresetPoint = (abs(dTargetDis[nTotal-1] - m_TaskRunCmd.distance) > EPSINON) || \
                                    (abs(dTargetDis[0] - 0.0) > EPSINON);

            double dSpeed = abs(m_TaskRunCmd.speed);
            string sPresetId;
            if(m_TaskRunCmd.speed >=0)
            {
                dSpeed = dCurrentOdom >= 0 ? -dSpeed : dSpeed;
            }
            else
            {
                dSpeed = dCurrentOdom >= dTargetDis[nTotal-1] ? -dSpeed : dSpeed;
            }
            for(int i=0; i<nGetOffsetTimes; i++)
            {
                //下达单步运动后的指令
                int nPresetOrder = m_TaskRunCmd.speed < 0 ? i : nGetOffsetTimes-1-i;
                ROS_INFO("[ReturnMotion] nPresetOrder=%d",nPresetOrder);

                sPresetId = vsPresetId[nPresetOrder];

                custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
                ControlCmd.move_type = ONLY_MOVE;
                ControlCmd.back = 0;
                ControlCmd.speed = dSpeed;

                if(i == 0)
                {
                    ControlCmd.distance = abs(dNewTargetDis[nPresetOrder] - dCurrentOdom);
                }
                else
                {
                    if(ControlCmd.speed < 0)
                        dStepDis = abs(dNewTargetDis[nPresetOrder+1] - dNewTargetDis[nPresetOrder]);
                    else
                        dStepDis = abs(dNewTargetDis[nPresetOrder-1] - dNewTargetDis[nPresetOrder]);

                    ControlCmd.distance = dStepDis + dOffset;
                }

                //等待单步运动完成
                if(WaitMotionDone(ControlCmd) != MOTION_DONE)
                {
                    return;
                }

                if(i != nGetOffsetTimes-1)
                {
                    //请求位移偏差
                    if(!GetOffset(m_sPlatId, sPresetId, &dOffset))
                    {
                        ROS_ERROR("[ReturnMotion] get offset error");
                        return;
                    }
                    if(ControlCmd.speed > 0)
                    {
                        dOffset = -dOffset;
                    }
                }

                //实时矫正里程偏差
                if(1 == m_nAdjustMileage && abs(dOffset) > m_dAdjustMinRange  && abs(dOffset) < m_dAdjustMaxRange)
                {
                    if(bLessPresetPoint && 0 == i)
                    {
                        ROS_DEBUG("[ReturnMotion] less preset point and i is 0, do not adjust mileage");
                    }
                    else
                    {
                        custom_msgs::TrainRobotControl AdjustCmd = ControlCmd;
                        AdjustCmd.cmd = "adjust_mileage";
                        AdjustCmd.expected_mileage = int(AdjustCmd.distance * 1000);
                        AdjustCmd.actual_mileage = int((AdjustCmd.distance - dOffset) * 1000);
                        string sOutput;
                        if(!PubMoveCmd(AdjustCmd, sOutput))
                        {
                            ROS_ERROR("[ReturnMotion] %s", sOutput.c_str());
                            m_sTaskStatus = "error";
                            return;
                        }
                    }
                }
            }
            if(PreciseToPresetPoint(m_sPlatId, sPresetId))
            {
                m_sTaskStatus = "idle";
            }
            else
            {
                ROS_ERROR("[ReturnMotion] precise to preset point error");
                m_sTaskStatus = "error";
            }
        }
    }

    ROS_INFO("[ReturnMotion] end.");
}

/***********************************************************
Function: CControlManager::CreatPosData
Description: 创建并打开新的数据文件
Input: double dTimeLen 超时时间
Output: void
Others: void
************************************************************/
void CControlManager::CreatePosData()
{
    m_sDataFileName = m_sDataFilePath;
    m_sDataFileName += "Data";

    time_t t = time(nullptr);
    char tmp[64];
    strftime( tmp, sizeof(tmp), "%Y%m%d-%H%M%S",localtime(&t));
    m_sDataFileName += tmp;
    m_sDataFileName += ".txt";

    ROS_INFO("[CreatePosData] open file:%s",m_sDataFileName.c_str());
    m_cFileRw.OpenFile(m_sDataFileName, std::string(""));
}

/***********************************************************
Function: CControlManager::PreciseToOrigin
Description: 精确归零
Input: string sPlatId
Output: true or false
Others: void
************************************************************/
bool CControlManager::PreciseToPresetPoint(string sPlatId, string sPresetId, int nRetryTimes, int nType)
{
    ROS_INFO("[PreciseToPresetPoint] start.");
    double dOffset;
    for(int i=0; i<nRetryTimes; i++)
    {
        if(!GetOffset(sPlatId, sPresetId, &dOffset, nType))
        {
            ROS_ERROR("[PreciseToPresetPoint] get offset failed");
            m_sTaskStatus = "error";
            return false;
        }
        custom_msgs::TrainRobotControl cmd;

        cmd.sender = "control_manager";
        cmd.move_type = ONLY_MOVE;
        cmd.back = 0;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "cmd";

        cmd.cmd = "run";
        cmd.back = 0;
        cmd.speed = dOffset < 0 ? abs(m_dRobotSpeed) : -abs(m_dRobotSpeed);
        cmd.distance = abs(dOffset);

        //等待单步运动完成
        if(WaitMotionDone(cmd, true) == MOTION_ERROR)
            return false;
    }

    ROS_DEBUG("[PreciseToPresetPoint] update motor origin start.");
    custom_msgs::TrainRobotControl Cmd;

    Cmd.sender = "control_manager";
    Cmd.header.stamp = ros::Time::now();
    Cmd.header.frame_id = "cmd";
    Cmd.cmd = "update_origin";
    Cmd.distance = m_TaskRunCmd.distance;

    if(m_TaskRunCmd.speed >= 0)
    {
        Cmd.move_type = stod(sPresetId) < EPSINON ? UPDATE_ORIGIN : UPDATE_ORIGIN_SUB;
    }
    else
    {
        Cmd.move_type = stod(sPresetId) < EPSINON ? UPDATE_ORIGIN_PLUS : UPDATE_ORIGIN;
    }

    string sOutput;
    if(!PubMoveCmd(Cmd, sOutput))
    {
        ROS_ERROR("[PreciseToPresetPoint] %s", sOutput.c_str());
        m_sTaskStatus = "error";
        return false;
    }
    ROS_DEBUG("[PreciseToPresetPoint] update motor origin end.");

    ROS_INFO("[PreciseToPresetPoint] end.");
    return true;
}

/***********************************************************
Function: CControlManager::GetOffset
Description: 获取机器人里程偏差值
Input: string sPlatId, 站台ID
       string sPresetId 预置位ID
        double &dOffset, 偏移量
        int nOffsetType, 偏移量类型
Output: true or false
Others: 摄像头视角:
        偏移量获取时偏左为正,偏右为负,
        这里做了正负转换,及偏右为正 偏左为负,因为右代表前进方向,左代表后退方向
************************************************************/
bool CControlManager::GetOffset(string sPlatId, string sPresetId, double *dOffset, int nOffsetType)
{
    string sConfidenceLevel;
    custom_msgs::ControlService OffsetService;
    OffsetService.request.header.stamp = ros::Time::now();
    OffsetService.request.sender = "control_manager";
    OffsetService.request.cmd = "match";
    OffsetService.request.data = sPlatId + "," + sPresetId;
    ROS_INFO("[GetOffset] OffsetService.request.data=%s",OffsetService.request.data.c_str());

    bool bResult = false;
    int nReTryTimes = 3;
    string sOutput;
    for(int j=0; j<nReTryTimes; j++)
    {
        bResult = m_OffsetCaller.call(OffsetService);
        sOutput = OffsetService.response.data;
        if(bResult && OffsetService.response.ack == "ok")
        {
            break;
        }
    }

    if(!bResult || OffsetService.response.ack == "error")
    {
        ROS_WARN("[GetOffset] call offset service failed, bResult:%d, ack:%s, data:%s",bResult, sOutput.c_str(), OffsetService.response.ack.c_str());
        m_sTaskStatus = "error";
        return false;
    }

    vector<string> vOffsetList;
    boost::split(vOffsetList, OffsetService.response.data, boost::is_any_of(","), boost::token_compress_on);
    string sOffset;
    if(vOffsetList.size() != 2)
    {
        ROS_WARN("[GetOffset] call offset service failed, size:%d, data:%s", int(vOffsetList.size()),OffsetService.response.data.c_str());
        m_sTaskStatus = "error";
        return false;
    }

    sOffset = vOffsetList[0];
    sConfidenceLevel = vOffsetList[1];
    if(stoi(sConfidenceLevel) <= 0)
    {
        ROS_WARN("[GetOffset] get offset error, sConfidenceLevel:%s", sConfidenceLevel.c_str());
        *dOffset = 0.0;
        if(nOffsetType == PRECISE_MOVE)
        {
            ROS_ERROR("[GetOffset] get offset error, nOffsetType:%d", nOffsetType);
            return false;
        }
    }
    else
    {
        *dOffset = -stod(sOffset);
    }

    ROS_INFO("[GetOffset] resp.response.data=%s,dOffset=%f",OffsetService.response.data.c_str(), *dOffset);

    return true;
}

/***********************************************************
Function: CControlManager::DataManThreadFunc
Description: 数据二次处理线程函数
Input: void
Output: true or false
Others: void
************************************************************/
void CControlManager::DataManThreadFunc()
{
    ROS_INFO("[DataManThreadFunc] start");
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_DataMutex);
        if (!m_bManData)
        {
            m_DataCondition.wait(lock);
        }
        m_bManData = false;
        lock.unlock();

        vector<FileRectify> vFileRectify;
        if(m_bRectifyOdom)
        {
            vFileRectify.assign(m_vFileRectify.begin(), m_vFileRectify.end());
            m_vFileRectify.clear();
        }

        CFileRW cAllDatafileRw;
        CFileRW cLaserDatafileRw;

        string sAllDataFile = m_sDataFilePath;
        sAllDataFile += "All";

        time_t t = time(nullptr);
        char tmp[64];
        strftime( tmp, sizeof(tmp), "%Y%m%d-%H%M%S",localtime(&t));
        sAllDataFile += tmp;
        sAllDataFile += ".txt";

        ROS_INFO("[DataManThreadFunc] open all data file:%s",m_sDataFileName.c_str());
        cAllDatafileRw.OpenFile(sAllDataFile, std::string("wr"));
        if(1 == m_nRecordLaser)
        {
            string sLaserDataFile = m_sDataFilePath;
            sLaserDataFile.append("laser-").append(tmp).append(".txt");
            ROS_INFO("[DataManThreadFunc] open laser data file:%s",sLaserDataFile.c_str());

            cLaserDatafileRw.OpenFile(sLaserDataFile, std::string("wr"));
        }

        if(m_bRectifyOdom)
        {
            for(auto const dataRectify : vFileRectify)
            {
                ROS_INFO("[DataManThreadFunc] file:%s,start odom:%f, offset:%f",dataRectify.sFileName.c_str(), dataRectify.dStartOdom, dataRectify.dOffset);
                string sRetData;
                ifstream fileIn(dataRectify.sFileName);
                string sLine, sOdom, sSourceDataYZ;

                if(!fileIn)
                {
                    ROS_ERROR("[DataManThreadFunc] open file failed, %s", dataRectify.sFileName.c_str());
                    continue;
                }
                vector<string> vsPosData;
                while (getline (fileIn, sLine))
                {
                    vsPosData.push_back(sLine);
                }

                size_t nRowNum = vsPosData.size();
                double dOffsetStep = dataRectify.dOffset / (nRowNum-1);
                double dStartOdom = dataRectify.dStartOdom;

                for(size_t j=0; j<nRowNum; j++)
                {
                    string sLineData = vsPosData[j];
                    vector<string> vCmdList;
                    boost::split(vCmdList, sLineData, boost::is_any_of(":"), boost::token_compress_on);

                    if(vCmdList.size() < 2)
                    {
                        ROS_ERROR("[DataManThreadFunc] source data error: %s",sLineData.c_str());
                        break;
                    }
                    double dOdom = stod(vCmdList[0]);
                    if(0 == j)
                        sOdom = to_string(dOdom + dStartOdom);
                    else
                        sOdom = to_string(dOdom - dOffsetStep*j + dStartOdom);
                    sSourceDataYZ = vCmdList[1];

                    vector<string> vDataYZList;
                    boost::split(vDataYZList, sSourceDataYZ, boost::is_any_of(","), boost::token_compress_on);
                    vector<double> vdDataY;
                    vector<double> vdDataZ;
                    if(vDataYZList.size() < 2 || vDataYZList.size() % 2 > 0)
                    {
                        ROS_WARN("[DataManThreadFunc] source y/z data error");
                        continue;
                    }
                    else
                    {
                        for(int m=0; m<vDataYZList.size(); m+=2)
                        {
                            vdDataY.push_back(stod(vDataYZList[m]));
                            vdDataZ.push_back(stod(vDataYZList[m+1]));
                        }
                    }
                    string sDataYZ = FilterPoints(vdDataY, vdDataZ);

                    if(sDataYZ.empty())
                    {
                        continue;
                    }

                    if(1 == m_nRecordLaser && 3 == vCmdList.size())
                    {
                        string sLaserData = sOdom;
                        string sLaserDis = vCmdList[2];
                        sLaserData.append(":").append(sLaserDis).append("\n");
                        cLaserDatafileRw.Output(sLaserData);
                    }

                    sOdom.append(":").append(sDataYZ).append("\n");
                    cAllDatafileRw.Output(sOdom);
                }
            }
        }
        else
        {
            ifstream fileIn(m_sDataFileName);
            string sLine, sOdom, sSourceDataYZ;

            if(!fileIn)
            {
                ROS_ERROR("[DataManThreadFunc] open file failed, %s", m_sDataFileName.c_str());
                continue;
            }
            vector<string> vsPosData;
            while (getline (fileIn, sLine))
            {
                vsPosData.push_back(sLine);
            }

            size_t nRowNum = vsPosData.size();

            for(size_t j=0; j<nRowNum; j++)
            {
                string sLineData = vsPosData[j];
                vector<string> vCmdList;
                boost::split(vCmdList, sLineData, boost::is_any_of(":"), boost::token_compress_on);
                if(vCmdList.size() < 2)
                {
                    ROS_ERROR("[DataManThreadFunc] source data error: %s",sLineData.c_str());
                    break;
                }
                sOdom = vCmdList[0];
                sSourceDataYZ = vCmdList[1];

                vector<string> vDataYZList;
                boost::split(vDataYZList, sSourceDataYZ, boost::is_any_of(","), boost::token_compress_on);
                vector<double> vdDataY;
                vector<double> vdDataZ;
                if(vDataYZList.size() < 2 || vDataYZList.size() % 2 > 0)
                {
                    ROS_WARN("[DataManThreadFunc] source y/z data error");
                    continue;
                }
                else
                {
                    for(int m=0; m<vDataYZList.size(); m+=2)
                    {
                        vdDataY.push_back(stod(vDataYZList[m]));
                        vdDataZ.push_back(stod(vDataYZList[m+1]));
                    }
                }
                string sDataYZ = FilterPoints(vdDataY, vdDataZ);
                if(sDataYZ.empty())
                {
                    continue;
                }

                if(1 == m_nRecordLaser && 3 == vCmdList.size())
                {
                    string sLaserData = sOdom;
                    string sLaserDis = vCmdList[2];
                    sLaserData.append(":").append(sLaserDis).append("\n");
                    cLaserDatafileRw.Output(sLaserData);
                }

                sOdom.append(":").append(sDataYZ).append("\n");
                cAllDatafileRw.Output(sOdom);
            }
        }

        if(1 == m_nRecordLaser)
        {
            cLaserDatafileRw.CloseFile();
        }

        cAllDatafileRw.CloseFile();
        custom_msgs::GeneralTopic dataFile;

        dataFile.sender = "track_robot";
        dataFile.header.stamp = ros::Time::now();
        dataFile.header.frame_id = "data_file";
        dataFile.data = sAllDataFile;

        m_DataFileNamePub.publish(dataFile);

        ROS_INFO("[DataManThreadFunc] all data man over");

        this_thread::sleep_for(std::chrono::seconds(1));
    }
    ROS_INFO("[DataManThreadFunc] end");
}

/***********************************************************
Function: CControlManager::TaskRunManage
Description: task_run指令分割功能函数
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::TaskRunManage()
{
    ROS_INFO("[TaskRunManage] start");
    m_bTaskReturning = false;
    m_bRecordResume = false;
    m_sTaskStatus = m_TaskRunCmd.speed > 0 ? "forward" : "backward";
    //创建该次任务的新文件夹
    m_sDataFilePath = "/home/ros/data/";
    time_t t = time(nullptr);
    char tmp[64];
    strftime( tmp, sizeof(tmp), "%Y%m%d-%H%M%S",localtime(&t));
    m_sDataFilePath += tmp;
    m_sDataFilePath += "/";

    string sCmd = "mkdir -p ";
    sCmd.append(m_sDataFilePath);
    int nStatus = system(sCmd.c_str());
    if(-1 == nStatus)
    {
        ROS_ERROR("[TaskRunManage] create data path failed, system error");
        return;
    }
    else
    {
        if(0 == int(WIFEXITED(nStatus)))
        {
            ROS_ERROR("[TaskRunManage] create data path failed, run shell error, exit code:%d", WIFEXITED(nStatus));
            return;
        }
    }

    //根据INI文件对运动任务进行分割
    FileRectify dataRectify;
    double dOffset = 0.0, dStepDis;

    int nTotal = m_ConfigIni.nTotal;
    m_sPlatId = m_ConfigIni.sPlatId;

    if(0 == nTotal)
    {
        EmptyPresetMotion();
        return;
    }

    vector<string> vsMileage = m_ConfigIni.vsMileage;
    double dTargetDis[nTotal];
    int nCount = 0;
    for(auto dDis : m_ConfigIni.dTargetDis)
    {
        dTargetDis[nCount] = dDis;
        nCount++;
    }

    if((m_TaskRunCmd.speed > 0 && abs(dTargetDis[0]) > EPSINON) || \
        (m_TaskRunCmd.speed < 0 && abs(dTargetDis[nTotal-1] - m_TaskRunCmd.distance) > EPSINON))
    {
        ROS_ERROR("[TaskRunManage] start preset point error");
        m_sTaskStatus = "error";
        return;
    }

    m_sOriginPresetId = m_TaskRunCmd.speed > 0 ? vsMileage[0] : vsMileage[nTotal-1];

    //run时，每个预置点都停下
    if(!PreciseToPresetPoint(m_sPlatId, m_sOriginPresetId, 2, PRECISE_MOVE))
    {
        ROS_ERROR("[TaskRunManage] PreciseToPresetPoint error");
        return;
    }

    m_vFileRectify.clear();

    if(1 == nTotal)
    {
        CreatePosData();

        custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
        ControlCmd.move_type = UPDATE_ORIGIN_MOVE_PUB;
        ControlCmd.real_distance = m_TaskRunCmd.distance;

        if(WaitMotionDone(ControlCmd) != MOTION_DONE)
        {
            return;
        }

        //唤醒数据处理进程处理数据
        m_bRectifyOdom = false;
        std::unique_lock<std::mutex> locker(m_DataMutex);
        m_bManData = true;
        m_DataCondition.notify_one();
        locker.unlock();

        if(ControlCmd.back == 1)
        {
            ReturnMotion();
        }
        else
        {
            m_sTaskStatus = "idle";
        }
    }
    else if(nTotal > 1)
    {
        bool bLessPresetPoint = (abs(dTargetDis[nTotal-1] - m_TaskRunCmd.distance) > EPSINON) || \
                                    (abs(dTargetDis[0] - 0.0) > EPSINON);

        for(int i=0; i<nTotal-1 && !m_bReturn; i++)
        {
            CreatePosData();

            //下达单步运动后的指令
            int nPresetOrder = m_TaskRunCmd.speed > 0 ? i+1 : nTotal-2-i;
            ROS_INFO("[TaskRunManage] nPresetOrder=%d",nPresetOrder);

            if(m_TaskRunCmd.speed > 0)
                dataRectify.dStartOdom = dTargetDis[nPresetOrder-1] - dOffset;
            else
                dataRectify.dStartOdom = dTargetDis[nPresetOrder] - dOffset;

            string sPresetId = vsMileage[nPresetOrder];


            custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
            if(0 == i)
            {
                ControlCmd.move_type = UPDATE_ORIGIN_MOVE_PUB;
                ControlCmd.real_distance = m_TaskRunCmd.distance;
            }
            else
                ControlCmd.move_type = MOVE_PUB;
            ControlCmd.back = 0;

            if(ControlCmd.speed < 0)
                dStepDis = abs(dTargetDis[nPresetOrder+1] - dTargetDis[nPresetOrder]);
            else
                dStepDis = abs(dTargetDis[nPresetOrder-1] - dTargetDis[nPresetOrder]);

            ControlCmd.distance = dStepDis + dOffset;
            m_dRobotSpeed = ControlCmd.speed;

            //等待单步运动完成
            if(WaitMotionDone(ControlCmd) != MOTION_DONE)
            {
                return;
            }

            //请求位移偏差
            if(!GetOffset(m_sPlatId, sPresetId, &dOffset))
            {
                ROS_ERROR("[TaskRunManage] get offset error");
                return;
            }

            if(m_TaskRunCmd.speed > 0)
            {
                dOffset = -dOffset;
            }

            dataRectify.dOffset = dOffset;
            dataRectify.sFileName = m_sDataFileName;
            m_dPauseOdom = dOffset;
            m_vFileRectify.push_back(dataRectify);

            //实时矫正里程偏差
            if(1 == m_nAdjustMileage && abs(dOffset) > m_dAdjustMinRange && abs(dOffset) < m_dAdjustMaxRange)
            {
                custom_msgs::TrainRobotControl AdjustCmd = ControlCmd;
                AdjustCmd.cmd = "adjust_mileage";
                AdjustCmd.expected_mileage = int(AdjustCmd.distance * 1000);
                AdjustCmd.actual_mileage = int((AdjustCmd.distance - dOffset) * 1000);
                string sOutput;
                if(!PubMoveCmd(AdjustCmd, sOutput))
                {
                    ROS_ERROR("[TaskRunManage] %s", sOutput.c_str());
                    m_sTaskStatus = "error";
                    return;
                }
            }
        }

        if(bLessPresetPoint)
        {
            CreatePosData();

            if(m_TaskRunCmd.speed > 0)
                dataRectify.dStartOdom = dTargetDis[nTotal-1] - dOffset;
            else
                dataRectify.dStartOdom = -dOffset;

            custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
            ControlCmd.move_type = MOVE_PUB;
            ControlCmd.back = 0;

            if(ControlCmd.speed < 0)
                dStepDis = dTargetDis[0];
            else
                dStepDis = abs(ControlCmd.distance - dTargetDis[nTotal-1]);

            ControlCmd.distance = dStepDis + dOffset;
            ROS_INFO("[TaskRunManage] less preset point, move_type:%d, speed:%f, remain dis:%f",
                     ControlCmd.move_type, ControlCmd.speed, ControlCmd.distance);

            //等待单步运动完成
            if(WaitMotionDone(ControlCmd) != MOTION_DONE)
            {
                return;
            }

            dataRectify.dOffset = 0.0;
            dataRectify.sFileName = m_sDataFileName;
            m_vFileRectify.push_back(dataRectify);
        }

        //唤醒数据处理进程处理数据
        m_bRectifyOdom = true;
        std::unique_lock<std::mutex> locker(m_DataMutex);
        m_bManData = true;
        m_DataCondition.notify_one();
        locker.unlock();

        if(!bLessPresetPoint)
        {
            //单程结束后的最后一个点也进行下校准位移
            m_sOriginPresetId = m_TaskRunCmd.speed < 0 ? vsMileage[0] : vsMileage[nTotal-1];
            if(!PreciseToPresetPoint(m_sPlatId, m_sOriginPresetId))
            {
                return;
            }
        }

        //自动返回逻辑
        if(m_TaskRunCmd.back == 1)
        {
            ReturnMotion();
        }
        else
        {
            m_sTaskStatus = "idle";
        }
    }

    ROS_INFO("[TaskRunManage] end.");
}

/***********************************************************
Function: CControlManager::TaskResume
Description: task_run任务恢复功能函数
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::TaskResume()
{
    ROS_INFO("[TaskResume] start");
    //根据INI文件对运动任务进行分割
    m_sTaskStatus = m_TaskRunCmd.speed > 0 ? "forward" : "backward";
    FileRectify dataRectify;
    double dOffset = 0.0, dStepDis;

    int nTotal = m_ConfigIni.nTotal;
    m_sPlatId = m_ConfigIni.sPlatId;

    if(m_bTaskReturning)
    {
        ReturnMotion();
        return;
    }

    if(0 == nTotal)
    {
        EmptyPresetResume();
        return;
    }

    vector<string> vsMileage = m_ConfigIni.vsMileage;
    double dTargetDis[nTotal];
    int nCount = 0;
    for(auto dDis : m_ConfigIni.dTargetDis)
    {
        dTargetDis[nCount] = dDis;
        nCount++;
    }

    //读取
    custom_msgs::CurrentPosition CurrentPos;
    CurrentPos.request.data = "control_manager";
    bool bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_ERROR("[TaskResume] call current position service failed");
        m_sTaskStatus = "error";
        return;
    }
    double dCurrentOdom = CurrentPos.response.x;

    //判断恢复时是否在暂停点,否则运动到暂停点再进行数据记录
    if(abs(m_dPauseOdom - dCurrentOdom) > 0.001)
    {
        custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
        ControlCmd.distance = abs(m_dPauseOdom - dCurrentOdom);
        ROS_DEBUG("[TaskResume] m_dPauseOdom: %f, dCurrentOdom: %f", m_dPauseOdom, dCurrentOdom);
        ControlCmd.speed = dCurrentOdom > m_dPauseOdom ? -ControlCmd.speed : ControlCmd.speed;

        if(m_dPauseOdom > dCurrentOdom)
        {
            ControlCmd.distance = m_dPauseOdom - dCurrentOdom;
        }

        ControlCmd.move_type = ONLY_MOVE;
        if(WaitMotionDone(ControlCmd) != MOTION_DONE)
        {
            m_sTaskStatus = "error";
            return;
        }
    }

    dCurrentOdom = m_dPauseOdom;

    ifstream fileIn(m_sDataFileName);
    string sEndLine = "0:0", sLine;
    if(!fileIn)
    {
        ROS_ERROR("[TaskResume] open file failed, %s", m_sDataFileName.c_str());
        m_sTaskStatus = "error";
        return;
    }
    if(fileIn.eof())
    {
        m_dPauseRecordOdom = 0.0;
        ROS_ERROR("[TaskResume] data file is null, m_dPauseRecordOdom=%f", m_dPauseRecordOdom);
    }
    else
    {
        while(!fileIn.eof())
        {
            getline (fileIn, sLine);
            sEndLine = sLine.empty() ? sEndLine : sLine;
        }

        vector<string> vsDataList;
        boost::split(vsDataList, sEndLine, boost::is_any_of(":"), boost::token_compress_on);
        if(vsDataList.empty())
        {
            ROS_ERROR("[TaskResume] the end line data error, %s", sEndLine.c_str());
            m_sTaskStatus = "error";
            return;
        }
        m_dPauseRecordOdom = stod(vsDataList[0]);
    }

    if(!m_cFileRw.OpenFile(m_sDataFileName, std::string("O_RDWR|O_APPEND")))
    {
        ROS_ERROR("[TaskResume] open data file error");
        m_sTaskStatus = "error";
    }
    m_bRecordResume = m_TaskRunCmd.speed >= 0;


    if(nTotal == 1)
    {
        custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
        ControlCmd.move_type = MOVE_PUB;
        if(ControlCmd.speed >= 0 && ControlCmd.distance > dCurrentOdom)
        {
            ControlCmd.distance = ControlCmd.distance - dCurrentOdom;
        }
        else if(ControlCmd.speed < 0 && EPSINON < dCurrentOdom)
        {
            ControlCmd.distance = dCurrentOdom;
        }
        else
        {
            ReturnMotion();
            ROS_DEBUG("[TaskResume] end.");
            return;
        }

        if(ControlCmd.distance > 0)
        {
            if(WaitMotionDone(ControlCmd) != MOTION_DONE)
            {
                return;
            }
        }
        else
        {
            m_cFileRw.CloseFile();
        }

        //唤醒数据处理进程处理数据
        m_bRectifyOdom = false;
        std::unique_lock<std::mutex> locker(m_DataMutex);
        m_bManData = true;
        m_DataCondition.notify_one();
        locker.unlock();

        if(ControlCmd.back == 1)
        {
            ReturnMotion();
        }
        else
        {
            m_sTaskStatus = "idle";
        }

        m_bRecordResume = false;
    }
    else if(nTotal > 1)
    {
        //计算到达目标定经过的预置位点数
        int nGetOffsetTimes = 0;
        for(int i=0; i<nTotal; i++)
        {
            if(m_TaskRunCmd.speed > 0 && dTargetDis[i] > dCurrentOdom)
                nGetOffsetTimes++;
            else if(m_TaskRunCmd.speed < 0 && dTargetDis[i] < dCurrentOdom)
                nGetOffsetTimes++;
        }
        ROS_INFO("[TaskResume] nGetOffsetTimes=%d",nGetOffsetTimes);
        if(nGetOffsetTimes == 0)
        {
            if((m_TaskRunCmd.speed >= 0 && m_TaskRunCmd.distance < dCurrentOdom) || \
                (m_TaskRunCmd.speed < 0 && EPSINON > dCurrentOdom))
            {
                ROS_INFO("[TaskResume] pause odom is over the range of task dis");
                m_cFileRw.CloseFile();
            }
            else
            {
                custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
                ControlCmd.move_type = MOVE_PUB;
                ControlCmd.back = 0;

                if(ControlCmd.speed < 0)
                    dStepDis = dCurrentOdom;
                else
                    dStepDis = abs(ControlCmd.distance - dCurrentOdom);

                ControlCmd.distance = dStepDis;
                ROS_INFO("[TaskResume] less preset point, move_type:%d, speed:%f, remain dis:%f",
                         ControlCmd.move_type, ControlCmd.speed, ControlCmd.distance);

                //等待单步运动完成
                if(WaitMotionDone(ControlCmd) != MOTION_DONE)
                {
                    return;
                }
            }

            if(m_TaskRunCmd.speed > 0)
                dataRectify.dStartOdom = dTargetDis[nTotal-1] - m_dPauseOffset;
            else
                dataRectify.dStartOdom = -dOffset;

            m_bRecordResume = false;

            dataRectify.dOffset = 0.0;
            dataRectify.sFileName = m_sDataFileName;
            m_vFileRectify.push_back(dataRectify);

            //唤醒数据处理进程处理数据
            m_bRectifyOdom = true;
            std::unique_lock<std::mutex> locker(m_DataMutex);
            m_bManData = true;
            m_DataCondition.notify_one();
            locker.unlock();
        }
        if(nGetOffsetTimes > 0)
        {
            for(int i=nTotal-1-nGetOffsetTimes; i<nTotal-1 && !m_bReturn; i++)
            {
                //下达单步运动后的指令
                if(i != nTotal-1-nGetOffsetTimes)
                {
                    CreatePosData();
                }
                if(i == nTotal-1-nGetOffsetTimes)
                {
                    dOffset = m_dPauseOffset;
                }
                int nPresetOrder = m_TaskRunCmd.speed > 0 ? i+1 : nTotal-2-i;
                ROS_INFO("[TaskResume] nPresetOrder=%d",nPresetOrder);

                if(m_TaskRunCmd.speed > 0)
                    dataRectify.dStartOdom = dTargetDis[nPresetOrder-1] - dOffset;
                else
                    dataRectify.dStartOdom = dTargetDis[nPresetOrder] - dOffset;

                string sPresetId = vsMileage[nPresetOrder];

                custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
                ControlCmd.move_type = MOVE_PUB;
                ControlCmd.back = 0;

                if(i == nTotal-1-nGetOffsetTimes)
                {
                    ControlCmd.distance = abs(dTargetDis[nPresetOrder] - dCurrentOdom);
                    ROS_INFO("[TaskResume] target dis:%f, current dis:%f", dTargetDis[nPresetOrder], dCurrentOdom);
                }
                else
                {
                    if(ControlCmd.speed < 0)
                        dStepDis = abs(dTargetDis[nPresetOrder+1] - dTargetDis[nPresetOrder]);
                    else
                        dStepDis = abs(dTargetDis[nPresetOrder-1] - dTargetDis[nPresetOrder]);

                    ControlCmd.distance = dStepDis + dOffset;
                }

                m_dRobotSpeed = ControlCmd.speed;

                //等待单步运动完成
                if(WaitMotionDone(ControlCmd) != MOTION_DONE)
                {
                    return;
                }

                //请求位移偏差
                if(!GetOffset(m_sPlatId, sPresetId, &dOffset))
                {
                    ROS_ERROR("[TaskResume] get offset error");
                    return;
                }

                if(m_TaskRunCmd.speed > 0)
                {
                    dOffset = -dOffset;
                }

                dataRectify.dOffset = dOffset;
                dataRectify.sFileName = m_sDataFileName;

                m_vFileRectify.push_back(dataRectify);
                if(0 == i)
                {
                    m_bRecordResume = false;
                }

                //实时矫正里程偏差
                if(1 == m_nAdjustMileage && abs(dOffset) > m_dAdjustMinRange  && abs(dOffset) < m_dAdjustMaxRange && i != nTotal-1-nGetOffsetTimes)
                {
                    custom_msgs::TrainRobotControl AdjustCmd = ControlCmd;
                    AdjustCmd.cmd = "adjust_mileage";
                    AdjustCmd.expected_mileage = int(AdjustCmd.distance * 1000);
                    AdjustCmd.actual_mileage = int((AdjustCmd.distance - dOffset) * 1000);
                    string sOutput;
                    if(!PubMoveCmd(AdjustCmd, sOutput))
                    {
                        ROS_ERROR("[TaskResume] %s", sOutput.c_str());
                        m_sTaskStatus = "error";
                        return;
                    }
                }
            }

            bool bLessPresetPoint = (abs(dTargetDis[nTotal-1] - m_TaskRunCmd.distance) > EPSINON) || \
                                    (abs(dTargetDis[0] - 0.0) > EPSINON);

            if(bLessPresetPoint)
            {
                CreatePosData();

                if(m_TaskRunCmd.speed > 0)
                    dataRectify.dStartOdom = dTargetDis[nTotal-1] - dOffset;
                else
                    dataRectify.dStartOdom = -dOffset;

                custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
                ControlCmd.move_type = MOVE_PUB;
                ControlCmd.back = 0;

                if(ControlCmd.speed < 0)
                    dStepDis = dTargetDis[0];
                else
                    dStepDis = abs(ControlCmd.distance - dTargetDis[nTotal-1]);

                ControlCmd.distance = dStepDis + dOffset;
                ROS_INFO("[TaskResume] less preset point, move_type:%d, speed:%f, remain dis:%f",
                         ControlCmd.move_type, ControlCmd.speed, ControlCmd.distance);

                //等待单步运动完成
                if(WaitMotionDone(ControlCmd) != MOTION_DONE)
                {
                    return;
                }

                dataRectify.dOffset = 0.0;
                dataRectify.sFileName = m_sDataFileName;
                m_vFileRectify.push_back(dataRectify);
            }

            //唤醒数据处理进程处理数据
            m_bRectifyOdom = true;
            std::unique_lock<std::mutex> locker(m_DataMutex);
            m_bManData = true;
            m_DataCondition.notify_one();
            locker.unlock();

            if(!bLessPresetPoint)
            {
                //单程结束后的最后一个点也进行下校准位移
                m_sOriginPresetId = m_TaskRunCmd.speed < 0 ? vsMileage[0] : vsMileage[nTotal-1];
                if(!PreciseToPresetPoint(m_sPlatId, m_sOriginPresetId))
                {
                    return;
                }
            }
        }

        //自动返回逻辑
        if(m_TaskRunCmd.back == 1)
        {
            this_thread::sleep_for(std::chrono::milliseconds(500));
            ReturnMotion();
        }
        else
        {
            m_sTaskStatus = "idle";
        }
    }
    ROS_INFO("[TaskResume] end.");
}

/***********************************************************
Function: CControlManager::AlarmRunManage
Description: alarm_run指令分割功能函数
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::AlarmRunManage()
{
    ROS_INFO("[AlarmRunManage] start.");
    m_sTaskStatus = "alarm_run";
    bool bIgnorePause = true;

    //读取当前点绝对位置里程
    custom_msgs::CurrentPosition CurrentPos;
    CurrentPos.request.data = "control_manager";
    bool bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_ERROR("[AlarmRunManage] call current position service failed");
        m_sTaskStatus = "error";
        return;
    }
    ROS_DEBUG("[AlarmRunManage] current pos0:%f", CurrentPos.response.x);
    double dCurrentPos = CurrentPos.response.x;

    //快速移动至需要检测侵限的位置点
    m_dAlarmGoalPos =  m_AlarmRunCmd.speed > 0 ? dCurrentPos + m_AlarmRunCmd.distance : dCurrentPos - m_AlarmRunCmd.distance;

    ROS_DEBUG("[AlarmRunManage] dCurrentPos:%f, m_dAlarmGoalPos:%f", dCurrentPos, m_dAlarmGoalPos);

    custom_msgs::TrainRobotControl AlarmRunCmd = m_AlarmRunCmd;
    AlarmRunCmd.move_type = ONLY_MOVE;
    if(AlarmRunCmd.distance >= m_dDetectRange)
    {
        AlarmRunCmd.distance = AlarmRunCmd.distance - m_dDetectRange;
    }
    else
    {
        AlarmRunCmd.speed = -AlarmRunCmd.speed;
        AlarmRunCmd.distance = m_dDetectRange - AlarmRunCmd.distance;
    }
    if(!AlarmRunFast(AlarmRunCmd))
        return;


    bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_ERROR("[AlarmRunManage] call current position service failed");
        m_sTaskStatus = "error";
        return;
    }

    dCurrentPos = CurrentPos.response.x;
    ROS_DEBUG("[AlarmRunManage] current pos1:%f", CurrentPos.response.x);

    //检测是否侵限的缓慢移动
    m_vAlarmPos.clear();

    AlarmRunCmd.move_type = MOVE_PUB;
    AlarmRunCmd.speed = dCurrentPos < m_dAlarmGoalPos ? 0.05 : -0.05;
    AlarmRunCmd.distance = m_dDetectRange * 2;
    if(WaitMotionDone(AlarmRunCmd, bIgnorePause) != MOTION_DONE)
    {
        return;
    }

    //移动至离目标最近的侵限点处,没有侵限则原点不动
    if(!m_vAlarmPos.empty())
    {
        ROS_DEBUG("[AlarmRunManage] m_vAlarmPos size:%d", int(m_vAlarmPos.size()));

        double dAlarmDestination = 0.0;
        int nMid = m_vAlarmPos.size() / 2;
        if(nMid == 0)
            nMid = 1;
        double dMidPos = m_vAlarmPos[nMid];

        for(auto dAlarmPos : m_vAlarmPos)
        {
//            ROS_DEBUG("[AlarmRunManage] dAlarmPos:%f", dAlarmPos);
            double dNewDiff = abs(m_dAlarmGoalPos - dAlarmPos);
            double dOldDiff = abs(m_dAlarmGoalPos - dAlarmDestination);

            bool bMid;

            if(m_dAlarmGoalPos <= dMidPos)
            {
                bMid = (dAlarmPos >= m_dAlarmGoalPos && dAlarmPos <= dMidPos);
            }
            else
            {
                bMid = (dAlarmPos <= m_dAlarmGoalPos && dAlarmPos >= dMidPos);
            }

            if(dAlarmDestination < EPSINON || ((dOldDiff > dNewDiff) && bMid))
            {
                dAlarmDestination = dAlarmPos;
            }
        }

        bResult = m_CurrentPosCaller.call(CurrentPos);
        if(!bResult || CurrentPos.response.data != "succeed")
        {
            ROS_ERROR("[AlarmRunManage] call current position service failed");
            m_sTaskStatus = "error";
            return;
        }
        dCurrentPos = CurrentPos.response.x;
        ROS_DEBUG("[AlarmRunManage] dAlarmDestination:%f, dMidPos:%f, dCurrentPos:%f", dAlarmDestination, dMidPos, dCurrentPos);

        AlarmRunCmd.move_type = ONLY_MOVE;
        AlarmRunCmd.distance = abs(dAlarmDestination - dCurrentPos);
        AlarmRunCmd.speed = dAlarmDestination < dCurrentPos ? -0.05 : 0.05;

        if(WaitMotionDone(AlarmRunCmd, bIgnorePause) != MOTION_DONE)
        {
            return;
        }

        //二次震荡
        int nDetectTimes = 2;
        AlarmDetectAgain(nDetectTimes);

        //三次震荡
        nDetectTimes = 3;
        if(!AlarmDetectAgain(nDetectTimes))
        {
            m_sTaskStatus = "error";
            ROS_INFO("[AlarmRunManage] end, detected more.");
            return;
        }
    }

    m_sTaskStatus = "idle";
    ROS_INFO("[AlarmRunManage] end.");
}

/***********************************************************
Function: CControlManager::AlarmDetctSecondTime
Description: alarm_run指令分割功能函数
Input: void
Output: void
Others: void
************************************************************/
bool CControlManager::AlarmDetectAgain(int nDetectTimes)
{
    ROS_INFO("[AlarmDetectAgain] start, nDetectTimes:%d.", nDetectTimes);
    this_thread::sleep_for(std::chrono::milliseconds(200));
    bool bIgnorePause = true;

    double dDetectRange = (nDetectTimes == 2) ? m_dDetectRange / 2 : m_dDetectRange/4;
    double dSpeed = (nDetectTimes == 2) ? 0.035 : 0.02;

    //读取当前点绝对位置里程
    custom_msgs::CurrentPosition CurrentPos;
    CurrentPos.request.data = "control_manager";
    bool bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_ERROR("[AlarmDetectAgain] call current position service failed");
        m_sTaskStatus = "error";
        return false;
    }
    double dCurrentPos = CurrentPos.response.x;

    if(FilterAndDetect(CurrentPos))
    {
        ROS_INFO("[AlarmDetectAgain] ens, got the alarm position, nDetectTimes:%d.", nDetectTimes);
        return true;
    }

    //快速移动至需要检测侵限的位置点
    m_dAlarmGoalPos =  dCurrentPos;

    ROS_DEBUG("[AlarmDetectAgain] m_dAlarmGoalPos:%f", m_dAlarmGoalPos);

    custom_msgs::TrainRobotControl AlarmRunCmd = m_AlarmRunCmd;
    AlarmRunCmd.move_type = ONLY_MOVE;

    AlarmRunCmd.speed = -AlarmRunCmd.speed;
    AlarmRunCmd.distance = dDetectRange;

    if(!AlarmRunFast(AlarmRunCmd))
        return false;


    bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_ERROR("[AlarmDetectAgain] call current position service failed");
        m_sTaskStatus = "error";
        return false;
    }
    dCurrentPos = CurrentPos.response.x;
    ROS_DEBUG("[AlarmDetectAgain] current pos1:%f", CurrentPos.response.x);

    //检测是否侵限的缓慢移动
    m_vAlarmPos.clear();

    AlarmRunCmd.move_type = MOVE_PUB;
    AlarmRunCmd.speed = dCurrentPos < m_dAlarmGoalPos ? dSpeed : -dSpeed;
    AlarmRunCmd.distance = dDetectRange * 2;
    if(WaitMotionDone(AlarmRunCmd, bIgnorePause) != MOTION_DONE)
    {
        return false;
    }

    bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_ERROR("[AlarmDetectAgain] call current position service failed");
        m_sTaskStatus = "error";
        return false;
    }
    dCurrentPos = CurrentPos.response.x;
    ROS_DEBUG("[AlarmDetectAgain] current pos2:%f", CurrentPos.response.x);

    //移动至离目标最近的侵限点处,没有侵限则原点不动
    if(!m_vAlarmPos.empty())
    {
        ROS_DEBUG("[AlarmDetectAgain] m_vAlarmPos size:%d", int(m_vAlarmPos.size()));

        double dAlarmDestination = 0.0;
        int nMid = m_vAlarmPos.size() / 2;
        if(nMid == 0)
            nMid = 1;
        double dMidPos = m_vAlarmPos[nMid];

        if(3 == nDetectTimes)
        {
            dAlarmDestination = dMidPos;
            ROS_DEBUG("[AlarmDetectAgain] third times retry use the mid pos");
        }
        else
        {
            for(auto dAlarmPos : m_vAlarmPos)
            {
//            ROS_DEBUG("[AlarmDetectAgain] dAlarmPos:%f", dAlarmPos);
                double dNewDiff = abs(m_dAlarmGoalPos - dAlarmPos);
                double dOldDiff = abs(m_dAlarmGoalPos - dAlarmDestination);

                bool bMid;

                if(m_dAlarmGoalPos <= dMidPos)
                {
                    bMid = (dAlarmPos >= m_dAlarmGoalPos && dAlarmPos <= dMidPos);
                }
                else
                {
                    bMid = (dAlarmPos <= m_dAlarmGoalPos && dAlarmPos >= dMidPos);
                }

                if(dAlarmDestination < EPSINON || ((dOldDiff > dNewDiff) && bMid))
                {
                    dAlarmDestination = dAlarmPos;
                }
            }
        }

        ROS_DEBUG("[AlarmDetectAgain] dAlarmDestination:%f, dMidPos:%f", dAlarmDestination, dMidPos);

        AlarmRunCmd.move_type = ONLY_MOVE;
        AlarmRunCmd.distance = abs(dAlarmDestination - dCurrentPos);
        AlarmRunCmd.speed = dAlarmDestination < dCurrentPos ? -dSpeed : dSpeed;

        if(WaitMotionDone(AlarmRunCmd, bIgnorePause) != MOTION_DONE)
        {
            return false;
        }
    }

    ROS_INFO("[AlarmDetectAgain] end, nDetectTimes:%d.", nDetectTimes);
    return true;
}


/***********************************************************
Function: CControlManager::ReturnThreadFunc
Description: return 指令分割功能函数
Input: void
Output: void
Others: void
************************************************************/
bool CControlManager::AlarmRunFast(custom_msgs::TrainRobotControl &Command)
{
    ROS_INFO("[AlarmRunFast] start.");
    bool bIgnorePause = true;

    //根据INI文件对运动任务进行分割
    int nTotal = m_ConfigIni.nTotal;
    m_sPlatId = m_ConfigIni.sPlatId;
    if(nTotal == 0)
    {
        if(WaitMotionDone(Command, bIgnorePause) != MOTION_DONE)
        {
            return false;
        }
        return true;
    }
    vector<string> vsMileage = m_ConfigIni.vsMileage;
    double dTargetDis[nTotal];
    int nCount = 0;
    for(auto dDis : m_ConfigIni.dTargetDis)
    {
        dTargetDis[nCount] = dDis;
        nCount++;
    }

    //读取当前点绝对位置里程
    custom_msgs::CurrentPosition CurrentPos;
    CurrentPos.request.data = "control_manager";
    bool bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_ERROR("[AlarmRunFast] call current position service failed");
        m_sTaskStatus = "error";
        return false;
    }
    double dCurrentOdom = CurrentPos.response.x;

    double dAbsoluteOdom;
    if(Command.speed > 0)
    {
        dAbsoluteOdom = dCurrentOdom + Command.distance;
    }
    else
    {
        dAbsoluteOdom = dCurrentOdom - Command.distance;
    }

    //计算到达目标定经过的预置位点数
    int nGetOffsetTimes = 0;
    for(int i=0; i<nTotal; i++)
    {
        if(Command.speed > 0 && dTargetDis[i] > dCurrentOdom && dTargetDis[i] < dAbsoluteOdom)
            nGetOffsetTimes++;
        else if(Command.speed < 0 && dTargetDis[i] < dCurrentOdom && dTargetDis[i] > dAbsoluteOdom)
            nGetOffsetTimes++;
    }

    ROS_INFO("[AlarmRunFast] nGetOffsetTimes=%d",nGetOffsetTimes);
    if(0 == nGetOffsetTimes)
    {
        if(WaitMotionDone(Command, bIgnorePause) != MOTION_DONE)
        {
            return false;
        }
    }
    else
    {
        custom_msgs::TrainRobotControl ControlCmd = Command;
        double dOffset = 0.0;
        double dPointDis = 0.0;
        int nTimes = 0;
        double dLastOdom;
        if(Command.speed > 0)
        {
            for(int i=0; i<nTotal; i++)
            {
                if(dTargetDis[i] > dCurrentOdom && dTargetDis[i] < dAbsoluteOdom)
                {
                    if(0 == nTimes)
                    {
                        dLastOdom = dCurrentOdom;
                    }
                    else
                    {
                        dLastOdom = dTargetDis[i-1];
                    }
                    string sPresetId = vsMileage[i];
                    dPointDis = dTargetDis[i];
                    ControlCmd.distance = abs(dPointDis - dLastOdom) + dOffset;
                    if(WaitMotionDone(ControlCmd, bIgnorePause) != MOTION_DONE)
                    {
                        return false;
                    }

                    //请求位移偏差
                    if(!GetOffset(m_sPlatId, sPresetId, &dOffset))
                    {
                        ROS_ERROR("[AlarmRunFast] get offset error");
                        m_sTaskStatus = "error";
                        return false;
                    }

                    dOffset = -dOffset;
                    nTimes++;
                }
            }
            ControlCmd.distance = abs(dAbsoluteOdom - dPointDis) + dOffset;
            if(WaitMotionDone(ControlCmd, bIgnorePause) != MOTION_DONE)
            {
                return false;
            }
        }
        else
        {
            for(int i=nTotal-1; i>=0; i--)
            {
                if(dTargetDis[i] < dCurrentOdom && dTargetDis[i] > dAbsoluteOdom)
                {
                    if(0 == nTimes)
                    {
                        dLastOdom = dCurrentOdom;
                    }
                    else
                    {
                        dLastOdom = dTargetDis[i+1];
                    }

                    string sPresetId = vsMileage[i];
                    dPointDis = dTargetDis[i];
                    ControlCmd.distance = abs(dPointDis - dLastOdom) + dOffset;
                    if(WaitMotionDone(ControlCmd, bIgnorePause) != MOTION_DONE)
                    {
                        return false;
                    }
                    //请求位移偏差
                    if(!GetOffset(m_sPlatId, sPresetId, &dOffset))
                    {
                        ROS_ERROR("[AlarmRunFast] get offset error");
                        return false;
                    }
                    nTimes++;
                }
            }
            ControlCmd.distance = abs(dAbsoluteOdom - dPointDis) + dOffset;
            if(WaitMotionDone(ControlCmd, bIgnorePause) != MOTION_DONE)
            {
                return false;
            }
        }
    }
    ROS_INFO("[AlarmRunFast] end.");
    return true;
}

/***********************************************************
Function: CControlManager::ReturnThreadFunc
Description: return 指令分割功能函数
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::ReturnThreadFunc()
{
    ROS_INFO("[ReturnThreadFunc] start.");
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_ReturnMutex);
        if (!m_bManReturnCmd)
        {
            m_ReturnCondition.wait(lock);
        }
        lock.unlock();
        if(ReadIniConfig(m_ConfigIni))
        {
            m_bPause = false;
            ReturnMotion();
        }
        m_bManReturnCmd = false;
    }
    ROS_INFO("[ReturnThreadFunc] end.");
}

/***********************************************************
Function: CControlManager::MonitorThreadFunc
Description: return 指令分割功能函数
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::MonitorThreadFunc()
{
    ROS_INFO("[MonitorThreadFunc] start");
    while(ros::ok())
    {
        timespec CurrentTime;
        timespec_get(&CurrentTime, TIME_UTC);

        long pollInterval=(CurrentTime.tv_sec - m_tMotorHeartTime.tv_sec)\
                         + (CurrentTime.tv_nsec - m_tMotorHeartTime.tv_nsec)/ 1000000000 ;
        if(pollInterval > 10)
        {
            if(!m_bMotorError)
                ROS_ERROR("[MonitorThreadFunc] motor heart beat timeout:10s");
            m_bMotorError = true;
        }
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

/***********************************************************
Function: CControlManager::ExecuteThread
Description: 唤醒指令分割处理线程
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::ExecuteTask()
{
    std::unique_lock<std::mutex> lock(m_CmdMutex);
    m_bManControlCmd = true;
    m_CmdCondition.notify_one();
}

/***********************************************************
Function: CControlManager::ExecuteReturn
Description: 唤醒回原点线程函数
Input: void
Output: void
Others: void
************************************************************/
void CControlManager::ExecuteReturn()
{
    std::unique_lock<std::mutex> lock(m_ReturnMutex);
    m_bManReturnCmd = true;
    m_ReturnCondition.notify_one();
}

/*************************************************
Function: CTrackRobot::EmptyPresetMotion
Description: 没有预置位情况下的运动控制
Input: void
Output: void
Others: void
**************************************************/
void CControlManager::EmptyPresetMotion()
{
    ROS_DEBUG("[EmptyPresetMotion] start.");
    m_sPlatId = "";

    CreatePosData();

    custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
    ControlCmd.move_type = UPDATE_ORIGIN_MOVE_PUB;
    ControlCmd.real_distance = m_TaskRunCmd.distance;
    if(WaitMotionDone(ControlCmd) != MOTION_DONE)
    {
        return;
    }

    //唤醒数据处理进程处理数据
    m_bRectifyOdom = false;
    std::unique_lock<std::mutex> locker(m_DataMutex);
    m_bManData = true;
    m_DataCondition.notify_one();
    locker.unlock();

    if(ControlCmd.back == 1)
    {
        ReturnMotion();
    }
    else
    {
        m_sTaskStatus = "idle";
    }
    ROS_DEBUG("[EmptyPresetMotion] end.");
}

/*************************************************
Function: CTrackRobot::EmptyPresetResume
Description: 没有预置位情况下的任务恢复
Input: void
Output: void
Others: void
**************************************************/
void CControlManager::EmptyPresetResume()
{
    ROS_DEBUG("[EmptyPresetResume] start, m_bTaskReturning=%d",m_bTaskReturning);
    m_sPlatId = "";
    if(m_bTaskReturning)
    {
        ReturnMotion();
    }
    else
    {
        //读取
        custom_msgs::CurrentPosition CurrentPos;
        CurrentPos.request.data = "control_manager";
        double dCurrentOdom;
        bool bResult = m_CurrentPosCaller.call(CurrentPos);
        if(!bResult || CurrentPos.response.data != "succeed")
        {
            ROS_WARN("[EmptyPresetResume] call current position service failed");
            m_sTaskStatus = "error";
            return;
        }
        dCurrentOdom = CurrentPos.response.x;

        //判断恢复时是否在暂停点,否则运动到暂停点再进行数据记录
        if(abs(m_dPauseOdom - dCurrentOdom) > 0.001)
        {
            ROS_INFO("[EmptyPresetResume] new start point is not pause point, m_dPauseOdom:%f, dCurrentOdom:%f", m_dPauseOdom,dCurrentOdom);
            custom_msgs::TrainRobotControl ToPausePointCmd = m_TaskRunCmd;
            ToPausePointCmd.distance = abs(m_dPauseOdom - dCurrentOdom);
            ToPausePointCmd.speed = dCurrentOdom > m_dPauseOdom ? -ToPausePointCmd.speed : ToPausePointCmd.speed;

            if(m_dPauseOdom > dCurrentOdom)
            {
                ToPausePointCmd.distance = m_dPauseOdom - dCurrentOdom;
            }

            ToPausePointCmd.move_type = ONLY_MOVE;
            if(WaitMotionDone(ToPausePointCmd) != MOTION_DONE)
            {
                m_sTaskStatus = "error";
                return;
            }
        }

        custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;
        if(ControlCmd.speed >= 0 && ControlCmd.distance > dCurrentOdom)
        {
            ControlCmd.distance = ControlCmd.distance - dCurrentOdom;
        }
        else if(ControlCmd.speed < 0 && EPSINON < dCurrentOdom)
        {
            ControlCmd.distance = dCurrentOdom;
        }
        else
        {
            ReturnMotion();
            ROS_DEBUG("[EmptyPresetResume] end.");
            return;
        }

        ControlCmd.move_type = MOVE_PUB;

        ifstream fileIn(m_sDataFileName);
        string sEndLine = "0:0", sLine;
        if(!fileIn)
        {
            ROS_ERROR("[EmptyPresetResume] open file failed, %s", m_sDataFileName.c_str());
            m_sTaskStatus = "error";
            return;
        }
        if(fileIn.eof())
        {
            m_dPauseRecordOdom = 0.0;
            ROS_ERROR("[EmptyPresetResume] data file is null, m_dPauseRecordOdom=%f", m_dPauseRecordOdom);
        }
        else
        {
            while(!fileIn.eof())
            {
                getline (fileIn, sLine);
                sEndLine = sLine.empty() ? sEndLine : sLine;
            }


            vector<string> vsDataList;
            boost::split(vsDataList, sEndLine, boost::is_any_of(":"), boost::token_compress_on);
            if(vsDataList.empty())
            {
                ROS_ERROR("[EmptyPresetResume] the end line data error, %s", sEndLine.c_str());
                m_sTaskStatus = "error";
                return;
            }
            m_dPauseRecordOdom = stod(vsDataList[0]);
            ROS_INFO("[EmptyPresetResume] get pause odom from file, m_dPauseRecordOdom=%f", m_dPauseRecordOdom);
        }

        if(!m_cFileRw.OpenFile(m_sDataFileName, std::string("O_RDWR|O_APPEND")))
        {
            ROS_ERROR("[EmptyPresetResume] open data file error");
            m_sTaskStatus = "error";
            return;
        }

        m_bRecordResume = m_TaskRunCmd.speed >= 0;
        if(WaitMotionDone(ControlCmd) != MOTION_DONE)
        {
            return;
        }
        m_bRecordResume = false;

        //唤醒数据处理进程处理数据
        m_bRectifyOdom = false;
        std::unique_lock<std::mutex> locker(m_DataMutex);
        m_bManData = true;
        m_DataCondition.notify_one();
        locker.unlock();

        if(ControlCmd.back == 1)
        {
            ReturnMotion();
        }
        else
        {
            m_sTaskStatus = "idle";
        }
    }

    ROS_DEBUG("[EmptyPresetResume] end.");
}

/*************************************************
Function: CTrackRobot::EmptyPresetReturn
Description: 没有预置位情况下的任务恢复
Input: void
Output: void
Others: void
**************************************************/
void CControlManager::EmptyPresetReturn()
{
    ROS_DEBUG("[EmptyPresetReturn] start.");
    //读取
    custom_msgs::CurrentPosition CurrentPos;

    CurrentPos.request.data = "control_manager";

    double dCurrentOdom;
    bool bResult = m_CurrentPosCaller.call(CurrentPos);
    if(!bResult || CurrentPos.response.data != "succeed")
    {
        ROS_WARN("[EmptyPresetReturn] call current position service failed");
        return;
    }
    dCurrentOdom = CurrentPos.response.x;

    custom_msgs::TrainRobotControl ControlCmd = m_TaskRunCmd;

    if(ControlCmd.cmd != "run")
    {
        ControlCmd.cmd = "run";
        ControlCmd.speed = 0.5;
    }

    if(ControlCmd.speed > 0)
    {
        if(dCurrentOdom >= 0)
        {
            ControlCmd.speed = - ControlCmd.speed;
        }

        ControlCmd.distance = abs(dCurrentOdom);
    }
    else
    {
        if(dCurrentOdom <= ControlCmd.distance)
        {
            ControlCmd.speed = - ControlCmd.speed;
        }

        ControlCmd.distance = abs(ControlCmd.distance - dCurrentOdom);
    }

    if(ControlCmd.distance < 0.001)
    {
        m_bReturn = false;
        m_sTaskStatus = "idle";
        return;
    }
    ControlCmd.move_type = ONLY_MOVE;

    if(WaitMotionDone(ControlCmd, false) != MOTION_DONE)
    {
        return;
    }

    if(!m_sPlatId.empty())
    {
        if(!PreciseToPresetPoint(m_sPlatId, m_sOriginPresetId))
        {
            ROS_ERROR("[ReturnMotion] precise to preset point error");
            m_sTaskStatus = "error";
            return;
        }
    }
    m_sTaskStatus = "idle";
    ROS_DEBUG("[EmptyPresetReturn] end.");
}

/*************************************************
Function: CTrackRobot::PubMoveCmd
Description: custom_msgs::TrainRobotControl &ControlCmd, 移动控制消息
             string &sOutput, 输出信息
Input: void
Output: void
Others: void
**************************************************/
bool CControlManager::PubMoveCmd(custom_msgs::TrainRobotControl &ControlCmd, string &sOutput)
{
    std::unique_lock<std::mutex> Locker(m_MoveAckMutex);
    m_nTransId++;
    ControlCmd.trans_id = m_nTransId;
    int nTimeout = 2;
    if(ControlCmd.cmd == "stop")
        nTimeout = 15;
    else if(ControlCmd.cmd == "run" || ControlCmd.cmd == "alarm_run")
    {
        ROS_DEBUG("[PubMoveCmd] cmd:%s, distance:%f, speed:%f, id:%d", \
                  ControlCmd.cmd.c_str(), ControlCmd.distance, ControlCmd.speed, ControlCmd.trans_id);
    }
    m_CommandPub.publish(ControlCmd);
    if(m_MoveAckCondition.wait_for(Locker,std::chrono::seconds(nTimeout)) == std::cv_status::timeout)
    {
        ROS_ERROR("[PubMoveCmd] wait for msg motor ack timeout: %ds.",nTimeout);
        sOutput = "timeout error";
        return false;
    }

    if(m_MoveAckMsg.ack == "failed")
    {
        ROS_ERROR("[PubMoveCmd] pub move cmd failed: %s.", m_MoveAckMsg.data.c_str());
        sOutput = m_MoveAckMsg.data;
        return false;
    }
    Locker.unlock();
    return true;
}

/*************************************************
Function: CTrackRobot::PubMoveCmd
Description: custom_msgs::TrainRobotControl &ControlCmd, 移动控制消息
             string &sOutput, 输出信息
Input: void
Output: void
Others: void
**************************************************/
bool CControlManager::ReadIniConfig(ConfigIni &ConfigIniData)
{
    ROS_INFO("[ReadIniConfig] start");

    ConfigIniData.nTotal = 0;
    ConfigIniData.sPlatId = "";
    ConfigIniData.vsMileage.clear();
    ConfigIniData.dTargetDis.clear();

    boost::property_tree::ptree IniWR;

    try
    {
        ConfigIniData.sIniFile = m_sPresetFile;
        ROS_INFO("[ReadIniConfig] ini file:%s", m_sPresetFile.c_str());
        boost::property_tree::ini_parser::read_ini(m_sPresetFile, IniWR);
    }
    catch (std::exception& e)
    {
        ROS_ERROR("[ReadIniConfig] read ini file error: %s",e.what());
        ConfigIniData.nTotal = 0;
        return true;
    }

    try
    {
        boost::property_tree::basic_ptree<string, string> GeneralItems = IniWR.get_child("General");
        boost::property_tree::basic_ptree<string, string> InfoItems = IniWR.get_child("Info");
        try
        {
            //读取ini中的配置
            ConfigIniData.sPlatId = GeneralItems.get<string>("plat_id");
            ConfigIniData.nTotal = GeneralItems.get<int>("total");

            string sMileage = InfoItems.get<string>("mileage");
            boost::split(ConfigIniData.vsMileage, sMileage, boost::is_any_of(","), boost::token_compress_on);
            if(ConfigIniData.vsMileage.size() != ConfigIniData.nTotal)
            {
                ROS_ERROR("[ReadIniConfig] vsMileage size:%d, nTotal:%d", (int)ConfigIniData.vsMileage.size() ,ConfigIniData.nTotal);
                m_sTaskStatus = "error";
                return false;
            }

            for(int i=0; i<ConfigIniData.nTotal; i++)
            {
                ConfigIniData.dTargetDis.push_back(stod(ConfigIniData.vsMileage[i]));
            }

            ROS_DEBUG("[ReadIniConfig] plat_id: %s, total:%d, mileage:%s", \
                             ConfigIniData.sPlatId.c_str(), ConfigIniData.nTotal, sMileage.c_str());
        }
        catch (std::exception& e)
        {
            ROS_ERROR("[ReadIniConfig] read ini error: %s",e.what());
            m_sTaskStatus = "error";
            return false;
        }
    }
    catch (std::exception& e)
    {
        ROS_ERROR("[ReadIniConfig] read ini error: %s",e.what());
        m_sTaskStatus = "error";
        return false;
    }
    ROS_INFO("[ReadIniConfig] end");
}

CControlManager::~CControlManager()
{
    m_pOdomManThread->join();
    delete m_pOdomManThread;
}
