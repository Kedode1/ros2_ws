#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

class IkPublisher : public rclcpp::Node
{
public:
  IkPublisher() : Node("ik_publisher"), counter_(0)
  {
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("position_command", 10, std::bind(&IkPublisher::msgCallback, this, _1));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("scara_position_controller/commands", 10);

    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
  }


private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  unsigned int counter_;
  // %------------- ROBOT DESIGN PARAMETERS (FIXED)
  const double a1 = 0.425;
  const double a2 = 0.345;
  const double d4 = 0.02;

  // -------------- TARGET COMMANDS
  double Ox;
  double Oy;
  double Oz;
  double al;

  // -------------- CALCULATED JOINTS
  double c2;
  double th1;
  double th2;
  double d3;
  double th4;

  
void msgCallback(const std_msgs::msg::Float64MultiArray &msg)
  {
    Ox = msg.data[0];
    Oy = msg.data[1];
    Oz = msg.data[2];
    al = msg.data[3];
    
    RCLCPP_INFO(this->get_logger(),"Received values: Ox = %.3f, Oy = %.3f, Oz = %.3f, al = %.3f",Ox, Oy, Oz, al);

    c2 = (Ox*Ox + Oy*Oy - a1*a1 - a2*a2) / (2.0 * a1 * a2);
    th1 = std::atan2(Oy, Ox) - std::atan2(a2 * std::sqrt(1.0 - c2*c2),a1 + a2 * c2);
    th2 = std::atan2(std::sqrt(1.0 - c2*c2),c2);
    d3 = Oz - d4;
    th4 = th1 + th2 - al;

    RCLCPP_INFO(this->get_logger(),"Sending out: J1 = %.4f, J2 = %.4f, J3 = %.4f, J4 = %.4f",th1, th2, d3, th4);

     auto joints = std_msgs::msg::Float64MultiArray();
    joints.data = {th1, th2, d3, th4};
    pub_->publish(joints);
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}