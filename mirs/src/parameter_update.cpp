#include "rclcpp/rclcpp.hpp"
#include "mirs_msgs/srv/parameter_update.hpp"

class ParameterUpdateClient : public rclcpp::Node
{
public:
    ParameterUpdateClient() : Node("parameter_update_client")
    {
        // パラメータを宣言
        this->declare_parameter<double>("wheel_radius", 0.1);
        this->declare_parameter<double>("wheel_base", 0.15);
        this->declare_parameter<double>("rkp", 10);
        this->declare_parameter<double>("rki", 10);
        this->declare_parameter<double>("rkd", 10);
        this->declare_parameter<double>("lkp", 10);
        this->declare_parameter<double>("lki", 10);
        this->declare_parameter<double>("lkd", 10);

        // YAMLファイルからパラメータを取得
        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double wheel_base = this->get_parameter("wheel_base").as_double();
        double rkp = this->get_parameter("rkp").as_double();
        double rki = this->get_parameter("rki").as_double();
        double rkd = this->get_parameter("rkd").as_double();
        double lkp = this->get_parameter("lkp").as_double();
        double lki = this->get_parameter("lki").as_double();
        double lkd = this->get_parameter("lkd").as_double();

        // サービスクライアントの作成
        client_ = this->create_client<mirs_msgs::srv::ParameterUpdate>("parameter_update");
        
        // サービスが利用可能になるまで待機
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        // サービスリクエストの作成
        auto request = std::make_shared<mirs_msgs::srv::ParameterUpdate::Request>();
        request->wheel_radius = wheel_radius;
        request->wheel_base = wheel_base;
        request->rkp = rkp;
        request->rkd = rki;
        request->rkd = rkd;
        request->lkp = lkp;
        request->lkd = lki;
        request->lkd = lkd;

        // サービス呼び出し
        auto result_future = client_->async_send_request(request);
        
        // 結果を待機
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Service response: %s", result_future.get()->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

private:
    rclcpp::Client<mirs_msgs::srv::ParameterUpdate>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterUpdateClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
