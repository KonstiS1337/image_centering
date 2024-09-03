#include "image_centering/image_centering.hpp"

ImageCentering::ImageCentering() : Node("image_centering_node"),
                                    pid_last_call_(builtin_interfaces::msg::Time())
                                    {
    this->declare_parameter<std::string>("epuck_name","epuck");
    std::string name = this->get_parameter("epuck_name").as_string();
    coords_sub_ = this->create_subscription<std_msgs::msg::Int16>(name + "/ball_coordinates/",1,std::bind(&ImageCentering::coordsSub,this,std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&ImageCentering::controlLoop,this));
    activate_srv_ = this->create_service<std_srvs::srv::SetBool>(name + "/image_centering/activation",std::bind(&ImageCentering::srvCB,this,std::placeholders::_1,std::placeholders::_2));
    robot_control_srv_ = this->create_client<epuck_driver_interfaces::srv::ChangeRobotState>(name + "/robot_control");
    pid_output_pub_ = this->create_publisher<std_msgs::msg::Float32>(name + "/pid_output",1);
    // setting up pidsetTarget
    pid_ = std::make_shared<PIDController<float>>(1,0,0,
    [this]() {
        return current_x_;
    },
    [this] (float output) {
        pid_output_ = output;
        return;
    });

    pid_->registerTimeFunction([this]() -> unsigned long {
    return this->pidTimeFunction();
    });

    pid_->setTarget(0);
    pid_->setEnabled(true);
}

void ImageCentering::srvCBp(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    pid_->setP(req->data);
    resp->success = true;
    return;
}
void ImageCentering::srvCBi(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp){
    pid_->setI(req->data);
    resp->success = true;
    return;
}
void ImageCentering::srvCBd(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp){
    pid_->setD(req->data);
    resp->success = true;
    return;
}

unsigned long ImageCentering::pidTimeFunction() {
    return (unsigned long) this->now().nanoseconds() / 1e+6;
}

void ImageCentering::coordsSub(const std_msgs::msg::Int16::ConstPtr & data) {
    current_x_ = data->data;
    return;
}

void ImageCentering::controlLoop() {
    rclcpp::Time current_time = this->now();
    pid_->tick();
    std_msgs::msg::Float32 msg;
    msg.data = pid_output_;
    pid_output_pub_->publish(msg);
    // Sending command
    auto request_left = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    auto request_right = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    request_left->module = request_left->MODULE_LEFT_MOTOR;
    request_right->module = request_right->MODULE_RIGHT_MOTOR;

    int turn_speed = pid_output_ > 100 ? TURN_SPEED: //TODO adjust values to reasonable values
                                                            pid_output_ > 50 ? TURN_SPEED * 0.5 : 
                                                                pid_output_ <= 20 ? 0 : TURN_SPEED * 0.25;

    if(pid_output_ >= 0) {
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
        pid_->setEnabled(true);
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
        pid_->setEnabled(false);
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