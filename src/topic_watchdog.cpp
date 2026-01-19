#include "topic_watchdog/topic_watchdog.hpp"

using std::placeholders::_1;

TopicWatchdog::TopicWatchdog() : Node("watchdog"), threshold(0,0){
    this->declare_parameter("threshold_ms", 5000);
    this->declare_parameter("timer_interval_ms",100);
    this->declare_parameter("enable_debug_msg",false);
    this->declare_parameter("input_topic","chatter");
    this->declare_parameter("error_code",1);
    
    // Convertion in Duration (ms -> ns)
    threshold = rclcpp::Duration(std::chrono::milliseconds(this->get_parameter("threshold_ms").as_int()));
    check_interval = std::chrono::milliseconds(this->get_parameter("timer_interval_ms").as_int());    enable_debug_msg = this->get_parameter("enable_debug_msg").as_bool();
    input_topic_name = this->get_parameter("input_topic").as_string();
    error_code = this->get_parameter("error_code").as_int();

    publisher = this->create_publisher<std_msgs::msg::Int32>("heartbeat",10);
    debug_publisher = enable_debug_msg ? this->create_publisher<std_msgs::msg::String>("heartbeat_debug",10) : nullptr;
    

    subscriber = this->create_subscription<std_msgs::msg::String>(
        input_topic_name, 10, std::bind(&TopicWatchdog::received_callback, this, _1));
    
    // watchdog time
    last_rx_time = this->now();
    watchdog_timer = this->create_wall_timer(
        check_interval, std::bind(&TopicWatchdog::check_watchdog, this));
}

void TopicWatchdog::check_watchdog() {
    auto age = this->now() - last_rx_time;

    auto code_msg = std_msgs::msg::Int32();
    std::string status_text;
    
    if(age > threshold) {
        code_msg.data = error_code;
        status_text = "ERROR: topic " + input_topic_name + " is dead";
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "RIP - No data on %s!", input_topic_name.c_str());
    } else {
        code_msg.data = CODE_OK;
        status_text = "CODE_OK";
    }

    publisher->publish(code_msg);

    // only send debug messages when topic is subsribed and enabled
    if(debug_publisher && debug_publisher->get_subscription_count() > 0) {
        auto debug_msg = std_msgs::msg::String();
        debug_msg.data = status_text;
        debug_publisher->publish(debug_msg);
    }
}

void TopicWatchdog::received_callback(const std_msgs::msg::String::SharedPtr msg) {
    last_rx_time = this->now();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicWatchdog>());
    rclcpp::shutdown();
    return 0;
}