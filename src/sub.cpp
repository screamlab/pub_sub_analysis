#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

/**
 * @brief A subscriber node that calculates package loss percentage and loss statistics.
 *
 * This node subscribes to a topic that publishes trajectory_msgs::msg::JointTrajectoryPoint
 * messages. It expects that the first element in the positions vector represents a counter
 * (increasing by 1 each time). If the difference between two subsequent counter values is greater
 * than 1, the extra count is considered lost. Additionally, the node computes the number of lost
 * messages per one-second interval and calculates the standard deviation (stdev) of these loss
 * samples. The log message every second is formatted as:
 *
 * [data count: <total_received>, loss count: <total_lost>, loss percentage: <percent>%, current
 * loss/sec: <current>, stdev loss/sec: <stdev>]
 */
class LossCalculatorSubscriber : public rclcpp::Node {
   public:
    LossCalculatorSubscriber()
        : Node("loss_calculator_subscriber"),
          first_msg_received_(false),
          data_count_(0),
          loss_count_(0),
          prev_value_(0),
          last_loss_count_(0) {
        // Declare a parameter for topic name to subscribe (default: "/right_arm_republish")
        this->declare_parameter<std::string>("subscribe_topic", "/right_arm_republish");
        std::string topic = this->get_parameter("subscribe_topic").as_string();

        // Create the subscription.
        subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
            topic, 10,
            std::bind(&LossCalculatorSubscriber::topic_callback, this, std::placeholders::_1));

        // Create a timer to log loss statistics every 1 second.
        timer_ = this->create_wall_timer(1s, std::bind(&LossCalculatorSubscriber::log_stats, this));

        RCLCPP_INFO(this->get_logger(), "LossCalculatorSubscriber subscribed to '%s'",
                    topic.c_str());
    }

    /**
     * @brief Returns a formatted string with overall statistics.
     *
     * The string contains total data count, loss count, and loss percentage.
     */
    std::string get_overall_stats() const {
        double percentage = 0.0;
        // Calculate total expected messages = received + lost.
        if ((data_count_ + loss_count_) > 0) {
            percentage = (static_cast<double>(loss_count_) / (data_count_ + loss_count_)) * 100.0;
        }
        std::stringstream ss;
        ss << "[data count: " << data_count_ << ", loss count: " << loss_count_
           << ", loss percentage: " << percentage << "%]";
        return ss.str();
    }

   private:
    /**
     * @brief Callback for processing received messages.
     *
     * Extracts the counter from the first element of the positions vector.
     * For the first message, it initializes the previous counter.
     * For subsequent messages, if the counter difference exceeds 1,
     * it increases the overall loss count by (difference - 1).
     *
     * @param msg Shared pointer to the received message.
     */
    void topic_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
        if (msg->positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received message with an empty positions vector");
            return;
        }

        int current_value = static_cast<int>(msg->positions[0]);

        if (!first_msg_received_) {
            first_msg_received_ = true;
            prev_value_ = current_value;
            data_count_ = 1;
        } else {
            int diff = current_value - prev_value_;
            if (diff > 1) {
                loss_count_ += (diff - 1);
            }
            prev_value_ = current_value;
            data_count_++;
        }
    }

    /**
     * @brief Logs loss statistics every 1 second.
     *
     * It calculates the loss in the most recent one-second interval and stores the sample,
     * then computes the standard deviation (stdev) of loss per second based on all samples taken
     * so far. Finally, it prints a log message with overall stats, current loss per sec, and stdev.
     */
    void log_stats() {
        // Calculate current loss in this interval.
        int current_interval_loss = loss_count_ - last_loss_count_;
        last_loss_count_ = loss_count_;
        loss_samples_.push_back(current_interval_loss);

        // Compute standard deviation for loss per second.
        double mean = 0.0;
        for (const auto &sample : loss_samples_) {
            mean += sample;
        }
        mean /= loss_samples_.size();

        double variance = 0.0;
        for (const auto &sample : loss_samples_) {
            variance += (sample - mean) * (sample - mean);
        }
        variance /= loss_samples_.size();
        double stdev = std::sqrt(variance);

        // Log the statistics.
        std::stringstream ss;
        ss << get_overall_stats() << ", current loss/sec: " << current_interval_loss
           << ", stdev loss/sec: " << stdev;
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Overall counters.
    bool first_msg_received_;
    int data_count_;
    int loss_count_;
    int prev_value_;

    // Variables to record one-second interval loss samples.
    int last_loss_count_;
    std::vector<int> loss_samples_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LossCalculatorSubscriber>();

    // Spin until SIGINT is received.
    rclcpp::spin(node);

    // Log final summary.
    RCLCPP_INFO(node->get_logger(), "Final loss statistics: %s", node->get_overall_stats().c_str());
    rclcpp::shutdown();
    return 0;
}
