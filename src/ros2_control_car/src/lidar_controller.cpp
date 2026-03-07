
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class LidarController : public rclcpp::Node {

        public:
                LidarController() : Node("lidar_controller"){
                        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                                "/scan",
                                10,
                                std::bind(&LidarController::input_callback, this, std::placeholders::_1)
                        );

                        RCLCPP_INFO(this->get_logger(), "Node launched !");
                       
                }
        
        private:
                void input_callback(sensor_msgs::msg::LaserScan::SharedPtr scan){
                        float angle_min = scan->angle_min;
                        float angle_max = scan->angle_max;
                        float angle_increment = scan->angle_increment;
                        float current_angle = angle_min;

                        float range_min = scan->range_min;
                        float range_max = scan->range_max;

                        float min_distance = std::numeric_limits<float>::infinity();

                        for (size_t i = 0; i < scan->ranges.size(); ++i)
                        {
                                float r = scan->ranges[i];

                                if (!std::isfinite(r)) continue;
                                if (r < scan->range_min || r > scan->range_max) continue;

                                
                                if (r <= SELF_FILTER_RADIUS) continue;

                                current_angle = angle_min + i*angle_increment;
                                if (MIN_FRONT_ANGLE < current_angle && current_angle < MAX_FRONT_ANGLE){
                                        if (r < min_distance){
                                                min_distance=r;
                                        }
                                }
                        }
                }


        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

        const float SELF_FILTER_RADIUS = 0.2;
        const float MINIMUM_RANGE_OBSTACLE = 1;
        bool obstacle = false;

        const float pi = M_PI;
        const float MIN_FRONT_ANGLE = -pi/4;
        const float MAX_FRONT_ANGLE = pi/4;

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<LidarController>());
        rclcpp::shutdown();
        return 0;
}