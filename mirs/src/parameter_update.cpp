#include "rclcpp/rclcpp.hpp"
<<<<<<< HEAD
#include "rclcpp/executor.hpp"
#include "mirs_msgs/srv/parameter_update.hpp"

class ParameterUpdateClient : public rclcpp::Node
{
public:
    ParameterUpdateClient() : Node("parameter_update_client")
    {
        // パラメータを宣言
        this->declare_parameter<double>("wheel_radius", 0.04);
        this->declare_parameter<double>("wheel_base", 0.4);
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
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Service response: %s", result_future.get()->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

private:
    rclcpp::Client<mirs_msgs::srv::ParameterUpdate>::SharedPtr client_;
};
=======
#include "mirs_msgs/srv/parameter_update.hpp"  // 新しいサービスファイルをインクルード

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
>>>>>>> 381be135721e80767c7192cc7c16001d53d3b3c2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ノードの作成
    auto node = rclcpp::Node::make_shared("parameter_update_client");

    // サービスクライアントの作成
    auto client = node->create_client<mirs_msgs::srv::ParameterUpdate>("update_parameters");

    // YAMLファイルからパラメータを読み込むための宣言
    node->declare_parameter("wheel_radius", 1.0);
    node->declare_parameter("wheel_base", 0.0);
    node->declare_parameter("rkp", 0.0);
    node->declare_parameter("rki", 0.0);
    node->declare_parameter("rkd", 0.0);
    node->declare_parameter("lkp", 0.0);
    node->declare_parameter("lki", 0.0);
    node->declare_parameter("lkd", 0.0);

    // サービスが利用可能になるまで待機
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "サービス待機中に中断されました。終了します。");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "サービスが利用できません。再試行します...");
    }

    // リクエストの作成
    auto request = std::make_shared<mirs_msgs::srv::ParameterUpdate::Request>();

    // YAMLからパラメータを取得してリクエストに設定
    request->wheel_radius = node->get_parameter("wheel_radius").as_double();
    request->wheel_base = node->get_parameter("wheel_base").as_double();
    request->rkp = node->get_parameter("rkp").as_double();
    request->rki = node->get_parameter("rki").as_double();
    request->rkd = node->get_parameter("rkd").as_double();
    request->lkp = node->get_parameter("lkp").as_double();
    request->lki = node->get_parameter("lki").as_double();
    request->lkd = node->get_parameter("lkd").as_double();

    // サービスを呼び出す
    auto result = client->async_send_request(request);

    // 結果を待機
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "リクエストの成功状態: %s", result.get()->success ? "true" : "false");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "サービス呼び出しに失敗しました。");
    }

    rclcpp::shutdown();
    return 0;
}
