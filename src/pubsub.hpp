#ifndef PUBSUB_HPP_4C802155646547D98711A7D27337EB3E
#define PUBSUB_HPP_4C802155646547D98711A7D27337EB3E

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <thread>
#include <memory>
#include <cstdint>
#include <mutex>
#include <condition_variable>

#define TOPIC_NAME  "HelloWorldTopic"
#define TEST4REAL
#define ROS2FOXY

class PubSub {
public:
    explicit PubSub(std::shared_ptr<rclcpp::Node> node, const std::string &topicName, int slaveCount = 0);

private:
    void OnSubCallback(const std_msgs::msg::String &info);
    void OnSubCallback2(const std_msgs::msg::String::SharedPtr msg);
    void run();
    std::shared_ptr<rclcpp::Node>   node_;
    std::mutex mutex_;
    std::condition_variable condition_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  subscription_;
    int             slaveCount_;
    int             tmpCount_;
    std::uint64_t   index_;
    bool            isWaiting_;
    std::shared_ptr<std::thread>    thread_;

};


#endif