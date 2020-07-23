/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2019-3-4
Description: CControlManager
**************************************************/

#ifndef PROJECT_CCONTROLMANAGER_H
#define PROJECT_CCONTROLMANAGER_H

#include <iostream>
#include <ros/ros.h>
//added by btrmg for filter 20200411
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
//added end
#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <queue>
#include <time.h>
#include <condition_variable>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "custom_msgs/TrainRobotControl.h"
#include "custom_msgs/TrainRobotPosition.h"
#include "custom_msgs/TrainRobotControlAck.h"
#include "custom_msgs/TrainRobotHeartBeat.h"
#include "custom_msgs/CurrentPosition.h"
#include "custom_msgs/ControlService.h"
#include "custom_msgs/GeneralTopic.h"
#include "CFileRW.h"

//added by btrmg for filter 2020.04.11
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>  //直通滤波器头文件
//#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
//#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
//#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
//#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
//#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
//added end

using namespace std;

#define EPSINON 1e-6

typedef struct _tagFileRectify
{
    string sFileName;
    double dStartOdom;
    double dOffset;
} FileRectify, *PFileRectify;

typedef struct _tagConfigIni
{
    string sIniFile;
    string sPlatId;
    int nTotal;
    vector<string> vsMileage;
    vector<double> dTargetDis;
} ConfigIni, *PConfigIni;

typedef enum _tagTaskName
{
    TASK_RUN,
    ALARM_RUN,
    Task_RESUME
}TaskName;

typedef enum _tagMoveType
{
    ONLY_MOVE,
    UPDATE_ORIGIN_MOVE_PUB,
    MOVE_PUB,
    SINGLE_MOVE,
    UPDATE_ORIGIN,
    UPDATE_ORIGIN_PLUS,
    UPDATE_ORIGIN_SUB,
}MoveType;

typedef enum _tagOffsetType
{
    IDENTIFY,
    LASER_LINE,
    PRECISE_MOVE
}OffsetType;

typedef enum _tagWaitReturnStatus
{
    MOTION_ERROR,
    MOTION_DONE,
    MOTION_PAUSE,
}WaitReturnStatus;

class CControlManager
{
public:
    CControlManager();
    ~CControlManager();
    void CommandCallBack(const custom_msgs::TrainRobotControl::ConstPtr &Command);
    void StatusCallBack(const custom_msgs::GeneralTopic::ConstPtr &Status);
    void DataCallBack(const custom_msgs::TrainRobotPosition::ConstPtr &PosData);
    void MotorHeartBeatCallBack(const custom_msgs::TrainRobotHeartBeat::ConstPtr &HeartBeat);
    void MoveAckCallBack(const custom_msgs::TrainRobotControlAck::ConstPtr &MoveAck);
    void TaskManThreadFunc();
    void DataManThreadFunc();
    void ReturnThreadFunc();
    void MonitorThreadFunc();
    void CreatePosData();
    void TaskRunManage();
    void AlarmRunManage();
    bool AlarmDetectAgain(int nDetectTimes);
    bool AlarmRunFast(custom_msgs::TrainRobotControl &Command);
    void TaskResume();
    void ExecuteTask();
    void ExecuteReturn();
    void ReturnMotion();
    void EmptyPresetMotion();
    void EmptyPresetResume();
    void EmptyPresetReturn();

    bool PubMoveCmd(custom_msgs::TrainRobotControl &ControlCmd, string &sOutput);
    bool PreciseToPresetPoint(string sPlatId, string sPresetId, int nRetryTimes = 2, int nType = IDENTIFY);
    bool GetOffset(string sPlatId, string sPresetId, double *dOffset, int nOffsetType=IDENTIFY);
    bool ReadIniConfig(ConfigIni &ConfigIniData);

    bool FilterAndDetect(custom_msgs::TrainRobotPosition &yzData);
    bool FilterAndDetect(custom_msgs::CurrentPosition &yzData);

    //added by btrmg for filter 2020.04.11
    string FilterPoints(vector<double> y,vector<double> z);
    //added end

    int WaitMotionDone(custom_msgs::TrainRobotControl &ControlCmd, bool bIgnorePause=false);

private:
    bool m_bWaitForDone;
    bool m_bManControlCmd;
    bool m_bManReturnCmd;
    bool m_bManData;
    bool m_bPause;
    bool m_bPausedDone;
    bool m_bReturn;
    bool m_bMotorError;
    bool m_bMotorStatusError;
    bool m_bMotorBusy;
    bool m_bRectifyOdom;
    bool m_bTaskReturning;
    bool m_bRecordResume;

    double m_dRobotSpeed;
    double m_d1stFilterRadius;
    double m_d2ndFilterRadius;
    double m_dPauseOdom;
    double m_dPauseRecordOdom;
    double m_dPauseOffset;
    double m_dDetectRange;
    double m_dAlarmGoalPos;
    double m_dAdjustMinRange;
    double m_dAdjustMaxRange;

    int m_nTaskName;
    int m_nRecordLaser;
    int m_nPreciseReturn;
    int m_n1stFilterNum;
    int m_n2ndFilterNum;
    int m_nTransId;
    int m_nAdjustMileage;
    uint8_t unMotorStatus;

    string m_sPrintLevel;
    string m_sSubCmdTopic;
    string m_sSubDataTopic;
    string m_sSubManagerStatusTopic;
    string m_sSubHeartBeatTopic;
    string m_sSubMoveAckTopic;
    string m_sPubAckTopic;
    string m_sPubManagerCmdTopic;
    string m_sPubDataFileTopic;
    string m_sPubHearBeatTopic;
    string m_sDataFileName;
    string m_sPresetFile;
    string m_sDataFilePath;
    string m_sPlatId;
    string m_sOriginPresetId;
    string m_sTaskStatus;

    timespec m_tMotorHeartTime;

    ros::Subscriber m_CommandSub;
    ros::Subscriber m_StatusSub;
    ros::Subscriber m_DataSub;
    ros::Subscriber m_HeartBeatSub;
    ros::Subscriber m_MoveAckSub;

    ros::Publisher m_CommandPub;
    ros::Publisher m_CtrlAckPub;
    ros::Publisher m_DataFileNamePub;
    ros::Publisher m_HeartBeatPub;

    ros::ServiceClient m_OffsetCaller;
    ros::ServiceClient m_CurrentPosCaller;

    CFileRW m_cFileRw;

    std::thread *m_pOdomManThread;
    std::thread *m_pDataManThread;
    std::thread *m_pReturnThread;
    std::thread *m_pHeartBeatThread;
    std::thread *m_pMonitorThread;

    custom_msgs::TrainRobotControl m_TaskRunCmd;
    custom_msgs::TrainRobotControl m_AlarmRunCmd;
    custom_msgs::TrainRobotHeartBeat m_HeartBeat;
    custom_msgs::TrainRobotControlAck m_MoveAckMsg;

    vector<FileRectify> m_vFileRectify;
    vector<double> m_vAlarmPos;

    std::condition_variable m_CmdCondition;
    std::condition_variable m_DataCondition;
    std::condition_variable m_ReturnCondition;
    std::condition_variable m_MoveAckCondition;

    std::mutex m_CmdMutex;
    std::mutex m_DataMutex;
    std::mutex m_ReturnMutex;
    std::mutex m_MoveAckMutex;

    ConfigIni m_ConfigIni;
};

#endif //PROJECT_CCONTROLMANAGER_H
