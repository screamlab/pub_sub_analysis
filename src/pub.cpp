#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node {
   public:
    JointTrajectoryPublisher() : Node("joint_trajectory_publisher"), counter_(0) {
        // Declare parameters:
        // - publish_rate: double (Hz) with a default value of 1.0.
        // - publish_topic: string parameter with the topic name. Default:
        // "/joint_trajectory_point".
        // - num_positions: int parameter for how many numbers (positions) to include. Default: 6.
        this->declare_parameter<double>("publish_rate", 100.0);
        this->declare_parameter<std::string>("publish_topic", "/right_arm");
        this->declare_parameter<int>("num_positions", 6);

        // Get parameter values
        double publish_rate = this->get_parameter("publish_rate").as_double();
        std::string topic_name = this->get_parameter("publish_topic").as_string();
        num_positions_ = this->get_parameter("num_positions").as_int();

        // Create the publisher on the specified topic.
        publisher_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(topic_name, 10);

        // Compute the timer period from the desired publish rate.
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            period, std::bind(&JointTrajectoryPublisher::timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "Publishing on topic '%s' at %.2f Hz with %d positions per message",
                    topic_name.c_str(), publish_rate, num_positions_);
    }

   private:
    void timer_callback() {
        auto msg = trajectory_msgs::msg::JointTrajectoryPoint();
        // Resize the positions vector to the number given by the parameter and fill with the same
        // counter value.
        msg.positions.resize(num_positions_, static_cast<double>(counter_));

        // Log the published message.
        RCLCPP_INFO(get_logger(), "Publishing: [%s]", vectorToString(msg.positions).c_str());

        publisher_->publish(msg);
        ++counter_;
    }

    // Helper function to convert the positions vector to a string for logging.
    std::string vectorToString(const std::vector<double> &vec) {
        std::string result;
        for (const auto &v : vec) {
            result += std::to_string(v) + " ";
        }
        return result;
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    int num_positions_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
