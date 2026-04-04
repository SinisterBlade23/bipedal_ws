#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>    
#include <utility>

using namespace std::chrono_literals;
using namespace std::placeholders;

class ik_height_control: public rclcpp::Node
{
    public:
        ik_height_control() : Node("ik_height_control"), Y(250.0), Y_step(10.0), X(-50)
        {
            

            subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "input_key", 10,
                std::bind(&ik_height_control::callbackKeyInput, this, _1));
            
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/joint_position_controller/commands", 10);

            timer_ = this->create_wall_timer(
                0.05s,std::bind(&ik_height_control::publishJoints,this)); //50hz 
        }



    private:

    void callbackKeyInput(const std_msgs::msg::String::SharedPtr msg)
    {
        if(msg->data == "UP")
        {
            Y += Y_step;
        }

        else if(msg->data == "DOWN")
        {
            Y -= Y_step;
        }

    }



    std::pair<double,double> solveIK()
    {
        const double L1 = 155.44;
        const double L2 = 171;

        double cos2 = (X*X + Y*Y - L1*L1 - L2*L2)/(2*L1*L2);
        
        if(std::abs(cos2)> 1.0)
        {
            RCLCPP_WARN(this->get_logger(), "Out of reach\n");
            return{NAN, NAN};
        }

        double theta2 = std::acos(cos2);

        double A = L1 + L2*std::cos(theta2);
        double B = L2*std::sin(theta2);

        double theta1 = std::atan2(A*Y-B*X, A*X+B*Y);

        return {theta1,theta2};

    }

    void publishJoints()
    {

    auto [th1, th2] = solveIK();

    if (std::isnan(th1) || std::isnan(th2))
    {
        return;
    }

    
    double th1_mirror = -th1;
    double th2_mirror = M_PI - th2;

    th2 -= 1.57;
    th2_mirror -= 1.57;

    RCLCPP_INFO_STREAM(this->get_logger(),"Hip angle: " << th1 << " Knee angle:" << th2);

    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {th1, th2, th1_mirror, th2_mirror};

    this->publisher_->publish(message);
    }

    double Y;
    double Y_step;
    double X;
    double frequency;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;



};


int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ik_height_control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
