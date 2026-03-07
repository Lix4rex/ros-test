#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <cmath>

#include <Eigen/Dense>


using namespace std::chrono_literals;

class ArmController : public rclcpp::Node {

        public:
                ArmController() : Node("arm_controller"){


                        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                                "/arm_position_controller/commands", 10
                        );

                        position_angular_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                                "/arm_angular_position_controller", 10,
                                std::bind(&ArmController::input_callback_angular, this, std::placeholders::_1)
                        );

                        position_3D_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
                                "/arm_3D_position_controller", 10,
                                std::bind(&ArmController::input_callback_point, this, std::placeholders::_1)
                        );

                        timer_ = this->create_wall_timer(
                                PUBLISH_PERIOD,
                                std::bind(&ArmController::publish_command, this)
                        );

                        position_msg.data = {0.0, 0.0, 0.0, 0.0};

                }

        private:

                /*void update_J(double theta_1, double theta_2, double theta_3, double theta_4){

                        double temp = std::sin(-theta_2) + std::sin(theta_3 - theta_2) + std::sin(theta_4 + theta_3 - theta_2);
                        J_(0,0) = -temp*std::cos(theta_1);
                        J_(0,1) = (std::cos(theta_2) + std::cos(theta_3 - theta_2) - std::cos(theta_4 + theta_3 - theta_2))*std::sin(theta_1);
                        J_(0,2) = (std::cos(theta_3 - theta_2) + std::cos(theta_4 + theta_3 - theta_2))*std::sin(-theta_1);
                        J_(0,3) = std::cos(theta_4 + theta_3 - theta_2)*std::sin(-theta_1);
                        J_(1,0) = -temp*std::sin(theta_1);
                        J_(1,1) = (-std::cos(theta_2) - std::cos(theta_3 - theta_2) - std::cos(theta_4 + theta_3 - theta_2))*std::cos(theta_1);
                        J_(1,2) = (std::cos(theta_3 - theta_2) + std::cos(theta_4 + theta_3 - theta_2))*std::cos(theta_1);
                        J_(1,3) = (std::cos(theta_4 + theta_3 - theta_2))*std::cos(theta_1);
                        J_(2,0) = 0;
                        J_(2,1) = -std::sin(theta_2) + std::sin(theta_3 - theta_2) + std::sin(theta_4 + theta_3 - theta_2);
                        J_(2,2) = std::sin(-theta_3 + theta_2) - std::sin(theta_4 + theta_3 - theta_2);
                        J_(2,3) = -std::sin(theta_4 + theta_3 - theta_2);
                }*/

                void update_J(double t1, double t2, double t3, double t4){
                        double r =
                                std::sin(-t2) +
                                std::sin(t3 - t2) +
                                std::sin(t4 + t3 - t2);

                        double dr_dt2 =
                                -std::cos(-t2)
                                -std::cos(t3 - t2)
                                -std::cos(t4 + t3 - t2);

                        double dr_dt3 =
                                std::cos(t3 - t2)
                                +std::cos(t4 + t3 - t2);

                        double dr_dt4 =
                                std::cos(t4 + t3 - t2);

                        // dx/dθ
                        J_(0,0) = -r * std::cos(t1);
                        J_(0,1) = std::sin(-t1) * dr_dt2;
                        J_(0,2) = std::sin(-t1) * dr_dt3;
                        J_(0,3) = std::sin(-t1) * dr_dt4;

                        // dy/dθ
                        J_(1,0) = -r * std::sin(t1);
                        J_(1,1) = std::cos(t1) * dr_dt2;
                        J_(1,2) = std::cos(t1) * dr_dt3;
                        J_(1,3) = std::cos(t1) * dr_dt4;

                        // dz/dθ
                        J_(2,0) = 0.0;
                        J_(2,1) =
                                -std::sin(t2)
                                +std::sin(t3 - t2)
                                +std::sin(t4 + t3 - t2);

                        J_(2,2) =
                                -std::sin(t3 - t2)
                                -std::sin(t4 + t3 - t2);

                        J_(2,3) =
                                -std::sin(t4 + t3 - t2);
                }


                Eigen::Vector3d forward_kinematics(const Eigen::Vector4d &t){

                        double theta_1 = t(0), theta_2 = t(1), theta_3 = t(2), theta_4 = t(3);

                        double valeur1 = std::sin(-theta_2);
                        double valeur2 = std::sin(theta_3 - theta_2);
                        double valeur3 = std::sin(theta_4 + theta_3 - theta_2);
                        double r = valeur1 + valeur2 + valeur3;

                        double valeur4 = std::cos(theta_2);
                        double valeur5 = std::cos(theta_3 - theta_2);
                        double valeur6 = std::cos(theta_4 + theta_3 - theta_2);
                        double first_arm_length = 1.0;
                        double z = valeur4 + valeur5 + valeur6 + first_arm_length;

                        double x = r*std::sin(-theta_1);
                        double y = r*std::cos(theta_1);

                        return Eigen::Vector3d(x, y, z);
                }


                void input_callback_point(const geometry_msgs::msg::Point::SharedPtr msg){
                        
                        Eigen::Vector3d target;
                        target << msg->x, msg->y, msg->z;

                        RCLCPP_INFO(this->get_logger(),
                                "Received point: %.2f %.2f %.2f",
                                msg->x, msg->y, msg->z
                        );

                        theta_ << 0.0, 0.0, 0.0, 0.0;

                        for (int i = 0; i < 100000; ++i)
                        {
                                Eigen::Vector3d current = forward_kinematics(theta_);

                                // Erreur position
                                Eigen::Vector3d error = target - current;

                                if (error.norm() < 1e-3)
                                break;

                                update_J(
                                        theta_(0),
                                        theta_(1),
                                        theta_(2),
                                        theta_(3)
                                );

                                Eigen::Vector4d dtheta =
                                alpha_ * J_.transpose() * error;

                                theta_ += dtheta;
                        }

                        position_msg.data.resize(4);
                        for (int i = 0; i < 4; ++i)
                                position_msg.data[i] = theta_(i);

                        position_pub_->publish(position_msg);

                        RCLCPP_INFO(
                                this->get_logger(),
                                "Calculated θ = [%.2f %.2f %.2f %.2f]",
                                theta_(0), theta_(1), theta_(2), theta_(3)
                        );
                }



                void input_callback_angular(std_msgs::msg::Float64MultiArray::SharedPtr points){
                        RCLCPP_INFO(this->get_logger(),
                                "Received point: %.2f %.2f %.2f %.2f",
                                points->data[0], points->data[1], points->data[2], points->data[3]
                        );

                        double theta_1 = points->data[0];
                        double theta_2 = points->data[1];
                        double theta_3 = points->data[2];
                        double theta_4 = points->data[3];

                        Eigen::Vector4d theta_input;
                        theta_input << theta_1, theta_2, theta_3, theta_4;
                        Eigen::Vector3d fk = forward_kinematics(theta_input);

                        RCLCPP_INFO(this->get_logger(),
                                "Calculated 3D point = [%.2f %.2f %.2f]",
                                fk(0), fk(1), fk(2)
                        );

                        for (size_t i = 0; i < points->data.size(); i++){
                                position_msg.data[i] = points->data[i];
                        }
                }

                void publish_command(){
                        position_pub_->publish(position_msg);
                }

                
                
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
        std_msgs::msg::Float64MultiArray position_msg;
                
        rclcpp::TimerBase::SharedPtr timer_;

        const std::chrono::milliseconds PUBLISH_PERIOD{100};

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_3D_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_angular_sub;

        Eigen::Matrix<double, 3, 4> J_;
        Eigen::Vector4d theta_{0.0, 0.0, 0.0, 0.0};
        const double alpha_ = 0.005; 
        

                        
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}