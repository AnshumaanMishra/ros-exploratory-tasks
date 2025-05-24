#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloWorld : public rclcpp::Node
{
  public:
    HelloWorld()
    : Node("hello_world")
    {
      RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "hello_world");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloWorld>());
  rclcpp::shutdown();
  return 0;
}