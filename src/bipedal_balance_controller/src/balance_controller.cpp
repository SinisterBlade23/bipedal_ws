#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>


using namespace std::chrono_literals;
using namespace std::placeholders;

class balance_controller : public rclcpp::Node
{
    public:
    
    balance_controller(): Node("balance_controller")
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",10,
            std::bind(&balance_controller::sensor_callback,this,_1));

    }

    private:

    double x;
    double y;
    double z;
    double w;
    double pitch;

    void sensor_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
         x = msg->orientation.x;
         y = msg->orientation.y;
         z = msg->orientation.z;
         w = msg->orientation.w;

         pitch = std::atan2(2.0 * (w * y - z * x),1.0 - 2.0 * (y * y + x * x));

        RCLCPP_INFO_STREAM(this->get_logger(), x <<" "<< y <<" "<< z <<" "<< w << "Pitch is "<< pitch );
        

    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;

};

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<balance_controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}