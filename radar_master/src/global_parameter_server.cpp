#include "rclcpp/rclcpp.hpp"
#include "radar_interfaces/msg/global_param.hpp"

namespace global_param_server
{
    class GlobalParamServer : public rclcpp::Node
    {
    public:
        GlobalParamServer() : Node("global_parameter_server",
                                   rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true))
        {
            param_pub_ = this->create_publisher<radar_interfaces::msg::GlobalParam>("global_param", 10);
            param_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GlobalParamServer::param_pub_callback, this));
        }

    private:
        void param_pub_callback()
        {
            auto message = radar_interfaces::msg::GlobalParam();
            this->get_parameter("status", message.status);
            this->get_parameter("color", message.color);

            param_pub_->publish(message);
        }

        rclcpp::Publisher<radar_interfaces::msg::GlobalParam>::SharedPtr param_pub_;
        rclcpp::TimerBase::SharedPtr param_pub_timer_;
    };
} // namespace global_parameter_server

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<global_param_server::GlobalParamServer>());
    rclcpp::shutdown();
    return 0;
}