#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <random>

#include <memory>
#include <string>
#include <chrono>

class TurtlebotNoiseNode : public rclcpp::Node
{
  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> noise_dist_;

    void sub_callback(const geometry_msgs::msg::Twist::SharedPtr response){
      auto noisy_msg = *response;
      noisy_msg.linear.x += noise_dist_(gen_);
      noisy_msg.angular.z += noise_dist_(gen_);

      publisher_->publish(noisy_msg);

      RCLCPP_INFO(this->get_logger(), "Published noisy velocity: linear.x=%.3f angular.z=%.3f",
                    noisy_msg.linear.x, noisy_msg.angular.z);
    }
  public:
    TurtlebotNoiseNode(): Node("turtlebot_noise_node"), gen_(rd_()), noise_dist_(0.0, 0.05){
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&TurtlebotNoiseNode::sub_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_noisy", 10);
      RCLCPP_INFO(this->get_logger(), "NoisyCmdVelNode has started.");
    }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotNoiseNode>());
  rclcpp::shutdown();
  return 0;
}