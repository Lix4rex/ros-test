#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class OdomCalculator : public rclcpp::Node {

        public:
                OdomCalculator() : Node("odom_calculator"){

                        joint_states = this->create_subscription<sensor_msgs::msg::JointState>(
                                "/joint_states", 10,
                                std::bind(&OdomCalculator::calculate_odom, this, std::placeholders::_1)
                        );

                }

        private: 

                void calculate_odom(const sensor_msgs::msg::JointState::SharedPtr joint_state){                        

                        if (joint_state->name.size() > 4){
                                return;
                        }
                        for (size_t i = 0; i < joint_state->name.size(); i++) {
                                RCLCPP_INFO(this->get_logger(), "Joint[%zu]: %s = %.4f",
                                        i,
                                        joint_state->name[i].c_str(),
                                        joint_state->position[i]
                                );
                        }
                }


                rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states;

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OdomCalculator>());
        rclcpp::shutdown();
        return 0;
}
