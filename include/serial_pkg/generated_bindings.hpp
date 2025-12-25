#pragma once
#include <functional>
#include "serial_pkg/serial_controller.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "../../mcu_output/protocol.h"

namespace auto_serial_bridge {
namespace generated {

template <typename T> void register_subscriber(SerialController* node, const std::string& topic, PacketID id);

inline void register_all(SerialController* node) {
    // Handshake (ROS -> MCU)
    node->add_subscription(node->create_subscription<std_msgs::msg::UInt32>(
        "/serial/handshake_cmd", 10,
        [node](const std_msgs::msg::UInt32::SharedPtr msg) {
            Packet_Handshake pkt;
            pkt.protocol_hash = msg->data;
            node->send_packet(PACKET_ID_HANDSHAKE, pkt);
        }));

    // Heartbeat (ROS -> MCU)
    node->add_subscription(node->create_subscription<std_msgs::msg::UInt8>(
        "/serial/heartbeat_cmd", 10,
        [node](const std_msgs::msg::UInt8::SharedPtr msg) {
            Packet_Heartbeat pkt;
            pkt.count = msg->data;
            node->send_packet(PACKET_ID_HEARTBEAT, pkt);
        }));

    // CmdVel (ROS -> MCU)
    node->add_subscription(node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [node](const geometry_msgs::msg::Twist::SharedPtr msg) {
            Packet_CmdVel pkt;
            pkt.linear_x = msg->linear.x;
            pkt.angular_z = msg->angular.z;
            node->send_packet(PACKET_ID_CMDVEL, pkt);
        }));

}

inline void dispatch_packet(SerialController* node, uint8_t id, const std::vector<uint8_t>& data) {
    switch(id) {
        case PACKET_ID_HANDSHAKE: {
            if (data.size() != sizeof(Packet_Handshake)) break;
            const Packet_Handshake* pkt = reinterpret_cast<const Packet_Handshake*>(data.data());
            auto msg = std_msgs::msg::UInt32();
            msg.data = pkt->protocol_hash;
            node->publish_message<std_msgs::msg::UInt32>("/serial/handshake", msg);
            break;
        }
        case PACKET_ID_HEARTBEAT: {
            if (data.size() != sizeof(Packet_Heartbeat)) break;
            const Packet_Heartbeat* pkt = reinterpret_cast<const Packet_Heartbeat*>(data.data());
            auto msg = std_msgs::msg::UInt8();
            msg.data = pkt->count;
            node->publish_message<std_msgs::msg::UInt8>("/serial/heartbeat", msg);
            break;
        }
        case PACKET_ID_CHASSISSTATUS: {
            if (data.size() != sizeof(Packet_ChassisStatus)) break;
            const Packet_ChassisStatus* pkt = reinterpret_cast<const Packet_ChassisStatus*>(data.data());
            auto msg = nav_msgs::msg::Odometry();
            msg.pose.pose.position.x = pkt->pos_x;
            msg.pose.pose.position.y = pkt->pos_y;
            node->publish_message<nav_msgs::msg::Odometry>("/odom", msg);
            break;
        }
    }
}
}
}
