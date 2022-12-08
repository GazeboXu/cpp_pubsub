#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "pubsub.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define FORCE_QUIT() \
do { \
  std::this_thread::sleep_for(2s); \
  rclcpp::shutdown(); \
  /*int *p = nullptr; */\
  /**p = 4; */\
}while(0)

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber(const std::string &nodeName)
    : Node(nodeName), lastCount_(0), lastCheckCount_(0)
    {
      rclcpp::QoS qos(10);
      qos.reliable();
      qos.transient_local();
      qos.keep_all();
      // rclcpp::QoS qos = rclcpp::ServicesQoS();

      rclcpp::SubscriptionOptions subOptions;
#ifndef ROS2FOXY
      subOptions.use_default_callbacks = false;
      subOptions.event_callbacks.message_lost_callback = [this](rclcpp::QOSMessageLostInfo &lostInfo) {
          RCLCPP_WARN(this->get_logger(), "subscibtion lost number: %ld", lostInfo.total_count);
      };
      subOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &lostInfo) {
          RCLCPP_WARN(this->get_logger(), "subscibtion deadline number: %d", lostInfo.total_count);
      };
      subOptions.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &lostInfo) {
          RCLCPP_WARN(this->get_logger(), "subscibtion liveliness number: %d", lostInfo.not_alive_count);
      };
      subOptions.event_callbacks.incompatible_qos_callback = [this](rclcpp::QOSRequestedIncompatibleQoSInfo &lostInfo) {
          RCLCPP_WARN(this->get_logger(), "subscibtion incompatible number: %d", lostInfo.total_count);
      };
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        TOPIC_NAME, qos, std::bind(&MinimalSubscriber::topic_callback, this, _1), subOptions);
#else
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        TOPIC_NAME, qos, std::bind(&MinimalSubscriber::topic_callback2, this, _1), subOptions);
#endif
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalSubscriber::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "Subscriber ready");
    }

private:
    void topic_callback(const std_msgs::msg::String & msg)
    {
      size_t count = static_cast<size_t>(std::stoll(msg.data));
      if(lastCount_ != 0 && lastCount_ + 1 != count) {
          RCLCPP_ERROR(this->get_logger(), "lastCount_=%ld, count=%ld", lastCount_, count);
          FORCE_QUIT();
      }
      lastCount_ = count;	
      if(count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      }
      // std::this_thread::sleep_for(2s);
    }

    void topic_callback2(const std_msgs::msg::String::SharedPtr msg) {
        topic_callback(*msg);
    }
    void timer_callback()
    {
      if(lastCheckCount_ != 0 && lastCheckCount_ == lastCount_) {
        RCLCPP_ERROR(this->get_logger(), "lastCount_=%ld, lastCheckCount_=%ld", lastCount_, lastCheckCount_);
        FORCE_QUIT();
      }
      lastCheckCount_ = lastCount_;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t lastCount_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t lastCheckCount_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
#ifndef TEST4REAL
  auto node = std::make_shared<MinimalSubscriber>(argv[1]);
#else
  auto node = std::make_shared<rclcpp::Node>(argv[1]);
  PubSub slave(node, TOPIC_NAME);
#endif
  // rclcpp::spin(node);
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

