#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <epuck_driver_interfaces/srv/change_robot_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#define IMAGE_CENTER_X 155 //TODO correct this
#define TURN_SPEED 250

class ImageCentering : public rclcpp::Node {
    public:
        ImageCentering();
    private:
        int16_t current_x_;

        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr coords_sub_;
        rclcpp::Client<epuck_driver_interfaces::srv::ChangeRobotState>::SharedPtr robot_control_srv_;
        rclcpp::TimerBase::SharedPtr timer_;
        control_toolbox::PidROS pid_;
        rclcpp::Time pid_last_call_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_srv_;

        void coordsSub(const std_msgs::msg::Int16::ConstPtr & data);
        void controlLoop();
        void srvCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

};
