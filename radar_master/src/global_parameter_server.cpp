#include "rclcpp/rclcpp.hpp"
class GlobalParameterServer : public rclcpp::Node
{
public:
    GlobalParameterServer() : Node("global_parameter_server",
                                   rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)) {}
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalParameterServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}