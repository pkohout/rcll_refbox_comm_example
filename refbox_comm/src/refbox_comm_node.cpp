#include <ros/ros.h>
#include <protobuf_comm/peer.h>
#include <refbox_protobuf_msgs/OrderInfo.pb.h>
#include <refbox_protobuf_msgs/GameState.pb.h>

using namespace protobuf_comm;

class Peer 
{
    using OrderInfo = llsf_msgs::OrderInfo;
    using GameState = llsf_msgs::GameState;
    using Order = llsf_msgs::Order;
    public:
        Peer(std::string host, int port)
            : m_host(host), m_port(port), m_mr(new MessageRegister())
        {
            m_mr->add_message_type<OrderInfo>();
            m_mr->add_message_type<GameState>();
            
            m_private_peer = new ProtobufBroadcastPeer(m_host, m_port, m_mr);
            m_private_peer->signal_received().connect(
                boost::bind(&Peer::handleRefboxMessagePrivate, this, _1, _2, _3, _4)
            );

            m_public_peer = new ProtobufBroadcastPeer(m_host, 4444, m_mr);
            m_public_peer->signal_received().connect(
                boost::bind(&Peer::handleRefboxMessagePublic, this, _1, _2, _3, _4)
            );

            m_team_name = "";
            m_is_cyan = false;            
        }

        ~Peer() {delete this->m_private_peer;}

        void handleRefboxMessagePrivate(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
        {
            ROS_INFO_STREAM("Recv message on private");
            std::shared_ptr<OrderInfo> order_info;
            if ((order_info = std::dynamic_pointer_cast<OrderInfo>(msg)))
            {
                ROS_INFO_STREAM(""<< order_info->ShortDebugString());
            }
        }

        void handleRefboxMessagePublic(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
        {
            std::shared_ptr<GameState> game_state;
            if ((game_state = std::dynamic_pointer_cast<GameState>(msg)))
            {
                if (game_state->team_cyan() == "GRIPS") 
                {
                    m_team_name = game_state->team_cyan();
                    m_is_cyan = true;
                }
            }
            std::shared_ptr<OrderInfo> order_info;
            if ((order_info = std::dynamic_pointer_cast<OrderInfo>(msg)))
            {
                ROS_INFO_STREAM(""<< order_info->ShortDebugString());
            }
        }

    private:
        std::string m_host;
        int m_port;
        MessageRegister *m_mr;
        ProtobufBroadcastPeer *m_private_peer;
        ProtobufBroadcastPeer *m_public_peer;
        std::string m_team_name;
        bool m_is_cyan;

};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "refbox_comm_node");
    ros::NodeHandle n;

    Peer p("localhost", 4441);

    ros::Rate r(10);
    while (ros::ok()) 
    {
        ros::spinOnce();
    }
    return 0;
}