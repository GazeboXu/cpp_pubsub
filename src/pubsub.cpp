#include "pubsub.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

PubSub::PubSub(std::shared_ptr<rclcpp::Node> node, const std::string &topicName, int slaveCount)
    : node_(node), slaveCount_(slaveCount), index_(0), isWaiting_(false) {
    rclcpp::PublisherOptions pubOptions;

#ifndef ROS2FOXY
    pubOptions.use_default_callbacks = false;
    pubOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &lostInfo) {
        RCLCPP_WARN(this->node_->get_logger(), "publisher deadline number: %d", lostInfo.total_count);
    };
    pubOptions.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessLostInfo &lostInfo) {
        RCLCPP_WARN(this->node_->get_logger(), "publisher liveliness number: %d", lostInfo.total_count);
    };
#endif

    rclcpp::QoS qos(10);
    qos.reliable();
    qos.transient_local();
    qos.keep_all();
    publisher_ = node->create_publisher<std_msgs::msg::String>(topicName, qos, pubOptions);

    rclcpp::SubscriptionOptions subOptions;
#ifndef ROS2FOXY
    subOptions.use_default_callbacks = false;
    subOptions.event_callbacks.message_lost_callback = [this](rclcpp::QOSMessageLostInfo &lostInfo) {
        RCLCPP_WARN(this->node_->get_logger(), "subscibtion lost number: %ld", lostInfo.total_count);
    };
    subOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &lostInfo) {
        RCLCPP_WARN(this->node_->get_logger(), "subscibtion deadline number: %d", lostInfo.total_count);
    };
    subOptions.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &lostInfo) {
        RCLCPP_WARN(this->node_->get_logger(), "subscibtion liveliness number: %d", lostInfo.not_alive_count);
    };
    subscription_ = node_->create_subscription<std_msgs::msg::String>(topicName, qos, std::bind(&PubSub::OnSubCallback, this, _1), subOptions);
#else
    subscription_ = node_->create_subscription<std_msgs::msg::String>(topicName, qos, std::bind(&PubSub::OnSubCallback2, this, _1), subOptions);
#endif

    if(slaveCount_ > 0) {
        thread_ = std::make_shared<std::thread>(std::bind(&PubSub::run, this));
    } else {
        RCLCPP_INFO(node_->get_logger(), "Subscriber %s ready", node_->get_name());
    }
}

void PubSub::run() {
    RCLCPP_INFO(node_->get_logger(), "Publisher preparing...");
    std::this_thread::sleep_for(5s);
    RCLCPP_INFO(node_->get_logger(), "Publisher ready");
    while(true) {
        tmpCount_ = 0;
        isWaiting_ = false;

        auto message = std_msgs::msg::String();
        message.data = "send " + std::to_string(++index_);

        RCLCPP_INFO(node_->get_logger(), "begin Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        RCLCPP_INFO(node_->get_logger(), "end Publishing: '%s'", message.data.c_str());

        std::unique_lock<std::mutex> lock(mutex_);
        if(slaveCount_ != tmpCount_) {
            isWaiting_ = true;
            condition_.wait(lock);
        }
    }
}

void PubSub::OnSubCallback(const std_msgs::msg::String &info) {
    RCLCPP_INFO(node_->get_logger(), "enter OnSubCallback info: %s", info);

    auto it = info.data.find(" ");
    auto cmd = info.data.substr(0, it);
    if(slaveCount_ > 0 && cmd == "back") {
        int index = std::stoi(info.data.substr(++it));
        auto needNotify = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if(index != index_) {
                RCLCPP_ERROR(node_->get_logger(), "OnSubCallback %ld != %ld", index, index_);
                rclcpp::shutdown();
            }
            tmpCount_++;
            needNotify = tmpCount_ == slaveCount_;
        }
        if(needNotify && isWaiting_) {
            condition_.notify_one();
        }
    } else if(slaveCount_ == 0 && cmd == "send") {
        auto message = std_msgs::msg::String();
        message.data = "back " + info.data.substr(++it);
        publisher_->publish(message);
    }
    RCLCPP_INFO(node_->get_logger(), "leave OnSubCallback info: %s", info);
}

void PubSub::OnSubCallback2(const std_msgs::msg::String::SharedPtr msg) {
    OnSubCallback(*msg);
}