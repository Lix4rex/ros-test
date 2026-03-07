#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class LidarRangesNode : public rclcpp::Node
{
public:
  LidarRangesNode() : Node("lidar_ranges_node")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&LidarRangesNode::scan_callback, this, std::placeholders::_1)
    );

    ranges_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/lidar/ranges",
      10
    );

    RCLCPP_INFO(this->get_logger(), "Lidar ranges node started");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std_msgs::msg::Float32MultiArray out;
    out.data.reserve(msg->ranges.size());

    for (float r : msg->ranges)
    {
      // garder uniquement les valeurs valides
      if (std::isfinite(r))
      {
        out.data.push_back(r);
      }
    }

    // Publier seulement s'il y a au moins une mesure valide
    if (!out.data.empty())
    {
      ranges_pub_->publish(out);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ranges_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRangesNode>());
  rclcpp::shutdown();
  return 0;
}
