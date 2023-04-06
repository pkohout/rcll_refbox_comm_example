#include "refbox_comm/peer.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <thread>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "refbox_comm_node");
    ros::NodeHandle n;

    Peer p("localhost", 4441, 4444);

    ros::Rate r(10);
    p.sendPrepareMachineCs();
    while (ros::ok()) 
    {
        ros::spinOnce();
    }
    return 0;
}