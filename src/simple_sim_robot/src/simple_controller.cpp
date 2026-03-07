#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class SimpleController : public rclcpp::Node
{
public:
  SimpleController() : Node("simple_controller")
  {
    // Publisher vers le robot
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    // Subscriber depuis une source externe
    input_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/input_cmd_vel",
      10,
      std::bind(&SimpleController::input_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Simple C++ controller started");
  }

private:
  void input_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Tu peux modifier, filtrer ou limiter ici si tu veux
    geometry_msgs::msg::Twist cmd = *msg;

    // Exemple : limiter la vitesse
    if (cmd.linear.x > 0.5)
      cmd.linear.x = 0.5;

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleController>());
  rclcpp::shutdown();
  return 0;
}
