#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <cmath>


using namespace std::chrono_literals;
using namespace std::placeholders;

class balance_controller : public rclcpp::Node
{
    public:

    balance_controller(): Node("balance_controller"), kp(3.15), kd(1.101), ki(6.8/2)
    {
        prev_time = this->get_clock()->now();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",10,
            std::bind(&balance_controller::sensor_callback,this,_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);
    }


    private:

    double x;
    double y;
    double z;
    double w;
    double pitch;
    const double max_speed = 3.0;
    const double deadband = 0.01;

    double kp,kd,ki;
    double prev_error;
    double integral;
    double derivative;
    double dt;
    rclcpp::Time prev_time;

    void sensor_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
         x = msg->orientation.x;
         y = msg->orientation.y;
         z = msg->orientation.z;
         w = msg->orientation.w;

         pitch = std::atan2(2.0 * (w * y - z * x),1.0 - 2.0 * (y * y + x * x));
        PIDloop();        
        

    }


    void PIDloop()
    {
        double error = pitch;

        if (std::abs(error) < deadband)
        {
            prev_error = error;
            prev_time  = this->get_clock()->now();
            return;
        }
        auto now = this->get_clock()->now();

        dt = (now-prev_time).seconds();
        integral += error*dt;
        derivative = (error-prev_error)/dt;

        double output = kp*error + ki*integral + kd*derivative;

        output = std::clamp(output, -max_speed, max_speed);

        RCLCPP_INFO_STREAM(this->get_logger(), "output: " << output <<" pitch:"<< pitch <<" error:"<< error );
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->get_clock()->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = output;
        publisher_->publish(cmd); 

        prev_error = error;
        prev_time = now;        
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<balance_controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}