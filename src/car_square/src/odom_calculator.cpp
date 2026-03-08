#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"

using namespace std::chrono_literals;

class OdomCalculator : public rclcpp::Node {

        public:
                OdomCalculator() : Node("odom_calculator"){

                        dynamic_joint_states = this->create_subscription<control_msgs::msg::DynamicJointState>(
                                "/dynamic_joint_states", 10,
                                std::bind(&OdomCalculator::calculate_odom, this, std::placeholders::_1)
                        );

                }

        private: 

                void calculate_odom(const control_msgs::msg::DynamicJointState::SharedPtr dynamic_joint_state){                        

                        if (dynamic_joint_state->joint_names.size() > 4){
                                return;
                        }
                        for (size_t i = 0; i < dynamic_joint_state->joint_names.size(); i++) {
                                RCLCPP_INFO(this->get_logger(), "Joint[%zu]: %s = %.4f",
                                        i,
                                        dynamic_joint_state->joint_names[i].c_str(),
                                        dynamic_joint_state->interface_values[i].values[1]
                                );
                        }
                }


                rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_joint_states;

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OdomCalculator>());
        rclcpp::shutdown();
        return 0;
}
