#include "image_centering/image_centering.hpp"

ImageCentering::ImageCentering() : Node("image_centering_node"),
                                    pid_last_call_(builtin_interfaces::msg::Time()),
                                    pid_(this->get_node_base_interface(),this->get_node_logging_interface(),this->get_node_parameters_interface(),this->get_node_topics_interface(),std::string("image_center_pid"))
                                    {
    this->declare_parameter<std::string>("epuck_name","epuck");
    std::string name = this->get_parameter("epuck_name").as_string();
    coords_sub_ = this->create_subscription<std_msgs::msg::Int16>(name + "/ball_coordinates/",1,std::bind(&ImageCentering::coordsSub,this,std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&ImageCentering::controlLoop,this));
    activate_srv_ = this->create_service<std_srvs::srv::SetBool>(name + "/image_centering/activation",std::bind(&ImageCentering::srvCB,this,std::placeholders::_1,std::placeholders::_2));
    robot_control_srv_ = this->create_client<epuck_driver_interfaces::srv::ChangeRobotState>(name + "/robot_control");

}

void ImageCentering::coordsSub(const std_msgs::msg::Int16::ConstPtr & data) {
    current_x_ = data->data;
    return;
}

void ImageCentering::controlLoop() {
    rclcpp::Time current_time = this->now();
    double controller_output = pid_.computeCommand(static_cast<double>(current_x_),current_time - pid_last_call_);
    pid_last_call_ = current_time;
    // Sending command
    auto request_left = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    auto request_right = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    request_left->module = request_left->MODULE_LEFT_MOTOR;
    request_right->module = request_right->MODULE_RIGHT_MOTOR;

    int turn_speed = controller_output > 100 ? TURN_SPEED: //TODO adjust values to reasonable values
                                                            controller_output > 50 ? TURN_SPEED * 0.5 : 
                                                                controller_output <= 20 ? 0 : TURN_SPEED * 0.25;

    if(controller_output >= 0) {
        request_left->value = turn_speed;
        request_right->value = -turn_speed;
    }
    else {
        request_left->value = -turn_speed;
        request_right->value = turn_speed;
    }

    robot_control_srv_->async_send_request(request_left);
    robot_control_srv_->async_send_request(request_right);
    
    return;
}

void ImageCentering::srvCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    if(req->data) {
        current_x_ = 0;
        timer_->reset();
    } 
    else {
        timer_->cancel();
        //stopping robot
        auto request_left = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
        auto request_right = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
        request_left->module = request_left->MODULE_LEFT_MOTOR;
        request_right->module = request_right->MODULE_RIGHT_MOTOR;
        robot_control_srv_->async_send_request(request_left);
        robot_control_srv_->async_send_request(request_right);
        //reseting pid
        pid_.reset();
    }
    resp->success = true;
    return;
}

int main(int argc, char *argv[])
{    
    rclcpp::init(argc, argv);
    std::shared_ptr<ImageCentering> node = std::make_shared<ImageCentering>();
  
    rclcpp::spin(node);
    
    return 0;
}