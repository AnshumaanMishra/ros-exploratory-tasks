#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>
#include <string>

class Talker : public rclcpp::Node{
  private: 
    int num = 0;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
  public:
    Talker(): Node("talker"){
      publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

      timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Talker::talker_callback, this)
      );
    }

    void talker_callback(){
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(num);
      RCLCPP_INFO(this->get_logger(), message.data.c_str());
      num++;
      publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}