#include "rclcpp/rclcpp.hpp"
#include "mirs_msgs/msg/basic_param.hpp"
#include <chrono>

class ParameterPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
        : Node("parameter_publisher")
    {
        // パラメータを宣言
        this->declare_parameter<double>("wheel_radius", 0.04);
        this->declare_parameter<double>("wheel_base", 0.38);
        //this->declare_parameter<double>("count_per_rev", 4096);
        this->declare_parameter<double>("rkp", 10.0);
        this->declare_parameter<double>("rki", 0.0);
        this->declare_parameter<double>("rkd", 0.0);
        this->declare_parameter<double>("lkp", 10.0);
        this->declare_parameter<double>("lki", 0.0);
        this->declare_parameter<double>("lkd", 0.0);

        // YAMLファイルからパラメータを取得
        wheel_radius = this->get_parameter("wheel_radius").as_double();
        wheel_base = this->get_parameter("wheel_base").as_double();
        //count_per_rev = this->get_parameter("count_per_rev").as_double();
        rkp = this->get_parameter("rkp").as_double();
        rki = this->get_parameter("rki").as_double();
        rkd = this->get_parameter("rkd").as_double();
        lkp = this->get_parameter("lkp").as_double();
        lki = this->get_parameter("lki").as_double();
        lkd = this->get_parameter("lkd").as_double();

        // パブリッシャーの作成
        param_pub_ = this->create_publisher<mirs_msgs::msg::BasicCommand>("/params", 10);

        // タイマーで定期的にオドメトリをパブリッシュ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&ParameterPublisher::publish_parameter, this));
    }

private:
    void publish_parameter()
    {
        auto param_msg = mirs_msgs::msg::BasicParam();
        param_pub_.wheel_base = wheel_base;
        param_pub_.wheel_radius = wheel_radius;
        //param_pub_.count_per_rev = count_per_rev;
        param_pub_.rkp = rkp;
        param_pub_.rki = rki;
        param_pub_.rkd = rkd;
        param_pub_.lkp = lkp;
        param_pub_.lki = lki;
        param_pub_.lkd = lkd;
        param_pub_->publish(odom_msg);
    }

    // パブリッシャー
    rclcpp::Publisher<mirs_msgs::msg::BasicParam>::SharedPtr param_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;

    // エンコーダの変数
    double wheel_radius,wheel_base,count_per_rev,rkp,rki,rkd,lkp,lki,lkd;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterPublisher>());
    rclcpp::shutdown();
    return 0;
}
