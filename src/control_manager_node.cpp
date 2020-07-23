/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2020-3-4
Description: control_manager_node.cpp 节点函数
**************************************************/

#include "CControlManager.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_manager");

    CControlManager control_manager;

    ros::MultiThreadedSpinner spinner(6); // Use 6 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
}