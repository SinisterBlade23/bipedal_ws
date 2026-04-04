#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <cmath>
#include <algorithm>
using namespace std::placeholders;

class balance_controller : public rclcpp::Node
{
public:
    balance_controller() : Node("balance_controller"), kp(30), ki(0.01), kd(0.5)
    {
        prev_time_  = this->get_clock()->now();
        start_time_ = this->get_clock()->now();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&balance_controller::sensor_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);
    }

private:
    double kp, ki, kd;
    const double setpoint  = 0.0;
    const double max_speed = 30.0;
    const double deadband  = 0.001;
    const double startup_wait = 3.0;

    double pitch_    = 0.0;
    double theta_dot_= 0.0;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool   ready_    = false;

    rclcpp::Time prev_time_;
    rclcpp::Time start_time_;

    void sensor_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double x = msg->orientation.x;
        double y = msg->orientation.y;
        double z = msg->orientation.z;
        double w = msg->orientation.w;

        pitch_     = std::atan2(2.0 * (w*y - z*x), 1.0 - 2.0*(y*y + x*x));
        theta_dot_ = msg->angular_velocity.y;

        computeAndPublish();
    }

    void computeAndPublish()
    {
        auto now = this->get_clock()->now();
        double dt = (now - prev_time_).seconds();
        if (dt <= 1e-6) return;
        prev_time_ = now;

        // Startup delay
        if (!ready_)
        {
            if ((now - start_time_).seconds() < startup_wait)
            {
                publishVelocity(0.0);
                return;
            }
            ready_ = true;
            RCLCPP_INFO(this->get_logger(), "PID controller active");
        }

        double error = setpoint - pitch_;

        if (std::abs(error) < deadband)
        {
            publishVelocity(0.0);
            return;
        }
        if ((error > 0 && prev_error_ < 0) || (error < 0 && prev_error_ > 0))
        {
            integral_ = 0.0;  // reset on sign change
        }

        integral_ += error * dt;
        integral_  = std::clamp(integral_, -0.3, 0.30);

        // Use IMU angular velocity directly for derivative — no noise from differentiation
        double derivative = -theta_dot_;

        double u = -(kp * error + ki * integral_ + kd * derivative);
        u = std::clamp(u, -max_speed, max_speed);

        RCLCPP_INFO_STREAM(this->get_logger(),
            "pitch: " << pitch_ << "  error: " << error << "  u: " << u);

        publishVelocity(u);
        prev_error_ = error;
    }

    void publishVelocity(double v)
    {
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp    = this->get_clock()->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x  = v;
        publisher_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<balance_controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}