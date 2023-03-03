#include <ros/ros.h>
#include <protobuf_comm/peer.h>
#include <refbox_protobuf_msgs/OrderInfo.pb.h>
#include <refbox_protobuf_msgs/GameState.pb.h>
#include <refbox_protobuf_msgs/MachineInfo.pb.h>
#include <refbox_protobuf_msgs/RingInfo.pb.h>
#include <refbox_protobuf_msgs/BeaconSignal.pb.h>
#include <refbox_protobuf_msgs/Time.pb.h>
#include <refbox_protobuf_msgs/Team.pb.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <thread>

#define TEAM_NAME "GRIPS"
#define CRYPTO_KEY "randomkey"

using namespace protobuf_comm;

class Peer 
{
    using OrderInfo = llsf_msgs::OrderInfo;
    using MachineInfo = llsf_msgs::MachineInfo;
    using GameState = llsf_msgs::GameState;
    using RingInfo = llsf_msgs::RingInfo;
    using Order = llsf_msgs::Order;
    using BeaconSignal = llsf_msgs::BeaconSignal;
    using Time = llsf_msgs::Time;
    using Team = llsf_msgs::Team;
    public:
        Peer(std::string host, int priv_recv_port, int pub_recv_port)
            : m_host(host), m_priv_recv_port(priv_recv_port), m_pub_recv_port(pub_recv_port),
              m_mr(new MessageRegister())
        {
            m_mr->add_message_type<OrderInfo>();
            m_mr->add_message_type<GameState>();
            m_mr->add_message_type<MachineInfo>();
            m_mr->add_message_type<RingInfo>();
            m_mr->add_message_type<BeaconSignal>();
            
            m_private_peer = std::make_shared<ProtobufBroadcastPeer>(m_host, m_priv_recv_port, m_mr, CRYPTO_KEY);
            m_private_peer->signal_received().connect(
                boost::bind(&Peer::handleRefboxMessagePrivate, this, _1, _2, _3, _4)
            );
            m_private_peer->signal_recv_error().connect(
                boost::bind(&Peer::handleRecvErrorPrivate, this, _1, _2)
            );
            m_private_peer->signal_send_error().connect(
                boost::bind(&Peer::handleSendErrorPrivate, this, _1)
            );

            m_public_peer =  std::make_shared<ProtobufBroadcastPeer>(m_host, m_pub_recv_port, m_mr);
            m_public_peer->signal_received().connect(
                boost::bind(&Peer::handleRefboxMessagePublic, this, _1, _2, _3, _4)
            );

            m_team_name = "";
            m_is_cyan = false;
            m_running = true;
            m_sequence_nr_ = 0;
            m_beacon_thread = std::thread(&Peer::sendBeaconSignal, this);
        }

        ~Peer() 
        {
            m_running = false;
        }

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
                ROS_INFO_STREAM("----------------" << comp_id << " : " << msg_type);
                ROS_INFO_STREAM(""<< game_state->ShortDebugString());

                auto cyan = game_state->team_cyan();
                m_team_name = TEAM_NAME;
                if (cyan == TEAM_NAME)
                {
                    m_is_cyan = true;
                }
            }
        }

        void handleRecvErrorPrivate(boost::asio::ip::udp::endpoint &endpoint, std::string msg)
        {
            ROS_ERROR_STREAM("Error receiving on private port : " << msg);
        }

        void handleSendErrorPrivate(std::string msg)
        {
            ROS_ERROR_STREAM("Error sending on private port : " << msg);
        }

        // this method should notify the refbox of a robot, somehow it does not work 
        // however the general way of sending like this should be correct
        void sendBeaconSignal()
        {
            while (m_running)
            {
                if (m_team_name != TEAM_NAME) 
                    continue;
                auto cur_time = ros::Time::now();
                std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> now =
                std::chrono::high_resolution_clock::now();
                std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
                std::chrono::nanoseconds nanoseconds = now.time_since_epoch() - seconds;
                
                std::shared_ptr<BeaconSignal> msg(new BeaconSignal());
                Time *time = msg->mutable_time();
                time->set_sec(static_cast<google::protobuf::int64>(seconds.count()));
                time->set_nsec(static_cast<google::protobuf::int64>(nanoseconds.count()));
                
                msg->set_seq(++m_sequence_nr_);
                msg->set_number(1);
                msg->set_team_name(m_team_name);
                msg->set_peer_name("robot_name");
                msg->set_team_color(m_is_cyan ? Team::CYAN : Team::MAGENTA);
                ROS_INFO_STREAM("Sending: " << msg->ShortDebugString());
                m_private_peer->send(2000, 1, msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }

    private:
        std::string m_host;
        int m_priv_recv_port;
        int m_pub_recv_port;
        unsigned long int m_sequence_nr_;
        MessageRegister *m_mr;
        std::shared_ptr<ProtobufBroadcastPeer> m_private_peer;
        std::shared_ptr<ProtobufBroadcastPeer> m_public_peer;
        std::string m_team_name;
        bool m_is_cyan;
        std::thread m_beacon_thread;
        bool m_running;
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "refbox_comm_node");
    ros::NodeHandle n;

    Peer p("localhost", 4441, 4444);

    ros::Rate r(10);
    while (ros::ok()) 
    {
        ros::spinOnce();
    }
    return 0;
}