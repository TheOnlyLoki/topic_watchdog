#ifndef TOPIC_WATCHDOG_HPP_
#define TOPIC_WATCHDOG_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class TopicWatchdog : public rclcpp::Node {
    private:
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
        rclcpp::TimerBase::SharedPtr watchdog_timer;
        rclcpp::Time last_rx_time;
        rclcpp::Duration threshold;
        std::chrono::milliseconds check_interval;
        std::string input_topic_name;
        bool enable_debug_msg;
        int error_code;
        
        void timer_callback();
        void received_callback(const std_msgs::msg::String::SharedPtr msg);
        void check_watchdog();
    public:
        static constexpr int CODE_OK = 0;
        TopicWatchdog();

    

};

#endif