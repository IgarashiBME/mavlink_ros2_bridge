#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <common/mavlink.h>
#include "mavlink_ros2_bridge/udp_socket.hpp"

class MAVLinkBridgeNode : public rclcpp::Node {
public:
    MAVLinkBridgeNode();

private:
    // --- Constants ---
    static constexpr uint16_t LOCAL_PORT = 14551;
    static constexpr uint16_t REMOTE_PORT = 14550;
    static constexpr uint32_t ARDUPILOT_GUIDED_ARMED = 217;
    static constexpr uint32_t ARDUPILOT_GUIDED_DISARMED = 89;

    // --- QGC parameter definition ---
    struct ParamEntry {
        std::string name;
        float default_value;
    };
    static const std::vector<ParamEntry>& paramEntries();

    // --- UDP & MAVLink ---
    UDPSocket socket_;
    mavlink_system_t mavlink_system_;
    std::string gcs_ip_;

    // --- MAVLink state ---
    // Mode
    uint64_t base_mode_ = 0;
    uint64_t custom_mode_ = 0;
    bool mission_start_ = false;

    // Mission protocol
    bool is_mission_request_ = false;
    int mission_total_seq_ = 0;
    uint16_t mission_seq_ = 0;
    int prev_mission_seq_ = -1;

    // Parameter protocol
    bool param_set_pending_ = false;
    char pending_param_id_[17] = {};
    float pending_param_value_ = 0.0f;
    uint8_t pending_param_type_ = 0;

    // --- Sensor data (from ROS subscribers) ---
    int32_t gps_lat_ = static_cast<int32_t>(35.736805 * 1e7);
    int32_t gps_lon_ = static_cast<int32_t>(139.539676 * 1e7);
    int32_t gps_alt_ = 10000;
    uint8_t gps_fix_type_ = GPS_FIX_TYPE_RTK_FIXED;
    uint8_t gps_satellites_ = 0;

    double yaw_ = 0.0;
    uint16_t estimator_flags_ = 0;

    int current_seq_ = 0;
    float cross_track_error_ = 0.0f;
    float angular_z_ = 0.0f;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr receive_timer_;
    rclcpp::TimerBase::SharedPtr mission_request_timer_;

    // --- Subscribers ---
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gnss_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr auto_log_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr heading_sub_;

    // --- Publishers ---
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mission_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr modes_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr joystick_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr mission_set_current_pub_;

    // --- Timer callbacks ---
    void onHeartbeatTimer();
    void onReceiveTimer();
    void onMissionRequestTimer();

    // --- Subscriber callbacks ---
    void onGnssReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void onAutoLogReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void onHeadingReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // --- MAVLink message dispatch ---
    void handleMavlinkMessage(const mavlink_message_t& msg);

    // --- Individual MAVLink message handlers ---
    void handleHeartbeat();
    void handleSetMode(const mavlink_message_t& msg);
    void handleParamRequestList();
    void handleParamSet(const mavlink_message_t& msg);
    void handleMissionSetCurrent(const mavlink_message_t& msg);
    void handleMissionCount(const mavlink_message_t& msg);
    void handleManualControl(const mavlink_message_t& msg);
    void handleMissionItemInt(const mavlink_message_t& msg);
    void handleCommandLong(const mavlink_message_t& msg);

    // --- Helpers ---
    void sendMavlinkMessage(mavlink_message_t& msg);
    void sendParamValue(const std::string& param_id, float value, uint8_t type,
                        uint16_t count, uint16_t index);
    uint64_t microsSinceEpoch() const;
};
