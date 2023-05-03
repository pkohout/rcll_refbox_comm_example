#include "refbox_comm/robot.h"

Robot::Robot() : m_mr(new MessageRegister())
{
    m_private_peer = std::make_shared<ProtobufStreamServer>(2016);
    m_private_peer->message_register().add_message_type<BeaconSignal>();
    m_private_peer->signal_received().connect(
        boost::bind(&Robot::handleMessage, this, _1, _2, _3, _4));
    m_running = true;
    // m_beacon_thread = std::thread(&Peer::sendBeaconSignal, this);
}

Robot::~Robot()
{
    m_running = false;
}

void Robot::handleMessage(unsigned int id, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
{
    ROS_INFO_STREAM("Recv message on private");
    std::shared_ptr<BeaconSignal> beaconMsg;
    if ((beaconMsg = std::dynamic_pointer_cast<BeaconSignal>(msg)))
    {
        ROS_INFO_STREAM("received BeaconMsg: " << beaconMsg->ShortDebugString());
    }
}