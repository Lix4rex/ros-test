#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node {

        public:
                ArmController() : Node("arm_controller"){


                        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                                "/arm_position_controller/commands", 10
                        );

                        timer_ = this->create_wall_timer(
                                PUBLISH_PERIOD,
                                std::bind(&ArmController::publish_command, this)
                        );

                        position_msg.data = {0.0, 0.0, 0.0, 0.0};
                }

        private:
                void publish_command(){
                        period += PUBLISH_PERIOD;
                        if (opening){
                                if (period >= OPENING_PERIOD){
                                        RCLCPP_INFO(this->get_logger(), "CLOSING");
                                        position_msg.data = {0.0, 0.0, 1.5, 1.5};
                                        opening = false;
                                        period = std::chrono::milliseconds{0};
                                }
                        }
                        else{
                                if (period >= CLOSING_PERIOD){
                                        RCLCPP_INFO(this->get_logger(), "OPENING");
                                        position_msg.data = {0.0, 0.0, 0.0, 0.0};
                                        opening=true;
                                        period = std::chrono::milliseconds{0};
                                }

                        }
                        position_pub_->publish(position_msg);
                }

                
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
        std_msgs::msg::Float64MultiArray position_msg;

        rclcpp::TimerBase::SharedPtr timer_;

        const std::chrono::milliseconds PUBLISH_PERIOD{100};
        
        const std::chrono::milliseconds OPENING_PERIOD{5000};
        const std::chrono::milliseconds CLOSING_PERIOD{5000};

        
        std::chrono::milliseconds period{0};
        bool opening = true;
                
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}