#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "pubsub.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

static const auto PUBLISH_GAP = 100us;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      rclcpp::QoS qos(10);
      qos.reliable();
      qos.transient_local();
      qos.keep_all();
      // rclcpp::QoS qos = rclcpp::ServicesQoS();
      rclcpp::PublisherOptions pubOptions;
#ifndef ROS2FOXY
      pubOptions.use_default_callbacks = false;
      pubOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &lostInfo) {
          RCLCPP_WARN(this->get_logger(), "publisher deadline number: %d", lostInfo.total_count);
      };
      pubOptions.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessLostInfo &lostInfo) {
          RCLCPP_WARN(this->get_logger(), "publisher liveliness number: %d", lostInfo.total_count);
      };
      pubOptions.event_callbacks.incompatible_qos_callback = [this](rclcpp::QOSOfferedIncompatibleQoSInfo &lostInfo) {
          RCLCPP_WARN(this->get_logger(), "publisher incompatible number: %d", lostInfo.total_count);
      };
#endif
      publisher_ = this->create_publisher<std_msgs::msg::String>(TOPIC_NAME, qos, pubOptions);
#if 0
      timer_ = this->create_wall_timer(PUBLISH_GAP, std::bind(&MinimalPublisher::timer_callback, this));
#else
	    thread_ = std::make_shared<std::thread>(std::bind(&MinimalPublisher::run, this));
#endif
      RCLCPP_INFO(this->get_logger(), "Publisher ready");
    }

private:
    void run() {
      std::this_thread::sleep_for(5s);
      while(true) {
        timer_callback();
        // std::this_thread::sleep_for(PUBLISH_GAP);
      }
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "begin Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "end Publishing: '%s'", message.data.c_str());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    std::shared_ptr<std::thread>	thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
#ifndef TEST4REAL
  auto node = std::make_shared<MinimalPublisher>();
#else
  auto node = std::make_shared<rclcpp::Node>("minimal_publisher");
  PubSub master(node, TOPIC_NAME, std::stoi(argv[1]));
#endif
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
