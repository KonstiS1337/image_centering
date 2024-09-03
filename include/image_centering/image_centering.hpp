#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <image_centering/pid.hpp>
#include <epuck_driver_interfaces/srv/change_robot_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#define IMAGE_CENTER_X 155 //TODO correct this
#define TURN_SPEED 250

class ImageCentering : public rclcpp::Node {
    public:
        ImageCentering();
    private:
        int16_t current_x_;
        float pid_output_;

        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr coords_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_output_pub_;
        rclcpp::Client<epuck_driver_interfaces::srv::ChangeRobotState>::SharedPtr robot_control_srv_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<PIDController<float>> pid_;
        rclcpp::Time pid_last_call_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_srv_,pid_p_srv_,pid_i_srv_,pid_d_srv_;

        unsigned long pidTimeFunction();
        void coordsSub(const std_msgs::msg::Int16::ConstPtr & data);
        void controlLoop();
        void srvCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
        //only needed for parameter tuning
        void srvCBp(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
        void srvCBi(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
        void srvCBd(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

};
