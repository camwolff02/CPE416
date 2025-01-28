#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PidController : public rclcpp::Node
{
  public:
    PidController()
    : Node("pid_controller"), P_(1), I_(0), D_(0), i_error_(0), previous_error_(0), previous_time_(0)
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "error", 10, std::bind(&PidController::next, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Float64>("control", 10);

      // declare PID parameters
      this->declare_parameter("P", 1);
      this->declare_parameter("I", 0);
      this->declare_parameter("D", 0);

      // monitor changes in PID constants
      param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
      auto cb = [this](const rclcpp::Parameter& param) -> void {
          if (param.get_name() == "P") {
              P_ = param.as_double();
          } else if (param.get_name() == "I") {
              I_ = param.as_double();
          } else if (param.get_name() == "D") {
              D_ = param.as_double();
          }
      };

      p_handle_ = param_subscriber_->add_parameter_callback("P", cb);
      i_handle_ = param_subscriber_->add_parameter_callback("I", cb);
      d_handle_ = param_subscriber_->add_parameter_callback("D", cb);
    }

  private:
    // Calculates and publishes control with new error
    void next(const std_msgs::msg::Float64::SharedPtr msg)
    {
      double error = msg->data;
      rclcpp::Time now_ = this->get_clock()->now();
      double dt_ = (now_ - previous_time_).nanoseconds();
      previous_time_ = now_;

      // calculate derivative and integral
      double d_error_ = (error - previous_error_) / dt_;
      previous_error_ = error;
      i_error_ += error;

      auto control = std_msgs::msg::Float64();
      control.data = P_*error + I_*i_error_ + D_*d_error_;

      RCLCPP_INFO(this->get_logger(), "Control: '%f'", control.data);
      publisher_->publish(control);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;

    double P_, I_, D_;
    double i_error_, previous_error_;
    rclcpp::Time previous_time_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> p_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> i_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> d_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidController>());
  rclcpp::shutdown();
  return 0;
}