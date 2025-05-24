#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class Listener : public rclcpp::Node{
  private: 
    int num = 0;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  public:
    Listener(): Node("Listener"){
      subscriber_ = this->create_subscription<std_msgs::msg::String>("/chatter", 10, std::bind(&Listener::listener_callback, this, std::placeholders::_1));
    }

    void listener_callback(std_msgs::msg::String::SharedPtr response){
      RCLCPP_INFO(this->get_logger(), response->data.c_str());
      num++;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}