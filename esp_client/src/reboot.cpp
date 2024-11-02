#include <rclcpp/rclcpp.hpp>
#include "esp_srvs/srv/simple_command.hpp"

class RebootClient : public rclcpp::Node
{
public:
    RebootClient() : Node("reboot_client")
    {
        client_ = this->create_client<esp_srvs::srv::SimpleCommand>("reboot");
    }

    void send_reboot_request()
    {
        auto request = std::make_shared<esp_srvs::srv::SimpleCommand::Request>();

        auto future = client_->async_send_request(request);

        // 結果を待機
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Reboot response: %s", future.get()->message.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service reboot");
        }
    }

private:
    rclcpp::Client<esp_srvs::srv::SimpleCommand>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<RebootClient>();
    client_node->send_reboot_request();
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}
