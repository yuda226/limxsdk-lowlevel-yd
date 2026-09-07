#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "limxsdk/tron2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

class Tron2CmdVelTranslator : public rclcpp::Node
{
public:
    Tron2CmdVelTranslator()
    : Node("tron2_cmd_vel_translator"),
        enable_sdk_output_(declare_parameter<bool>("enable_sdk_output", true)),
        enable_relay_output_(declare_parameter<bool>("enable_relay_output", false)),
        cmd_vel_topic_in_(declare_parameter<std::string>("cmd_vel_topic_in", "/cmd_vel")),
        cmd_vel_topic_out_(declare_parameter<std::string>("cmd_vel_topic_out", "/tron2/cmd_vel")),
        robot_ip_(declare_parameter<std::string>("robot_ip", "10.192.1.2")),
        max_linear_x_(declare_parameter<double>("max_linear_x", 0.6)),
        max_linear_y_(declare_parameter<double>("max_linear_y", 0.4)),
        max_angular_z_(declare_parameter<double>("max_angular_z", 1.2)),
        deadman_timeout_s_(declare_parameter<double>("deadman_timeout_s", 0.5)),
        sdk_json_template_(declare_parameter<std::string>(
            "sdk_json_template",
            "{\"accid\":\"nav_bridge\",\"title\":\"request_cmd_vel\",\"timestamp\":{timestamp_ms},\"guid\":\"{guid}\",\"data\":{\"linear_x\":{linear_x},\"linear_y\":{linear_y},\"angular_z\":{angular_z}}}")),
        last_cmd_time_(this->now()),
        sent_timeout_stop_(true),
        sequence_id_(0)
    {
        if (!enable_sdk_output_ && !enable_relay_output_) {
            RCLCPP_WARN(
                get_logger(),
                "Both enable_sdk_output and enable_relay_output are false. Node will only subscribe and log.");
        }

        if (enable_sdk_output_) {
            robot_ = limxsdk::Tron2::getInstance();
            if (!robot_->init(robot_ip_)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to initialize TRON2 SDK with robot_ip=%s. SDK output will be disabled.",
                    robot_ip_.c_str());
                enable_sdk_output_ = false;
            } else {
                RCLCPP_INFO(get_logger(), "TRON2 SDK connected to robot_ip=%s", robot_ip_.c_str());
            }
        }

        if (enable_relay_output_) {
            relay_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_out_, 10);
            RCLCPP_INFO(get_logger(), "Relay output enabled: %s", cmd_vel_topic_out_.c_str());
        }

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_in_,
            10,
            std::bind(&Tron2CmdVelTranslator::cmdVelCallback, this, std::placeholders::_1));

        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Tron2CmdVelTranslator::watchdogTimerCallback, this));

        RCLCPP_INFO(
            get_logger(),
            "Initialized. cmd_vel_topic_in=%s, sdk_output=%s, relay_output=%s",
            cmd_vel_topic_in_.c_str(),
            enable_sdk_output_ ? "true" : "false",
            enable_relay_output_ ? "true" : "false");
    }

private:
    static double clampAbs(double value, double limit)
    {
        if (limit <= 0.0) {
            return 0.0;
        }
        return std::max(-limit, std::min(value, limit));
    }

    static void replaceAll(std::string &target, const std::string &from, const std::string &to)
    {
        if (from.empty()) {
            return;
        }

        std::size_t start_pos = 0;
        while ((start_pos = target.find(from, start_pos)) != std::string::npos) {
            target.replace(start_pos, from.length(), to);
            start_pos += to.length();
        }
    }

    static std::string toFixed6(double value)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << value;
        return oss.str();
    }

    static uint64_t nowMsSinceEpoch()
    {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    }

    geometry_msgs::msg::Twist sanitizeCmd(const geometry_msgs::msg::Twist &in) const
    {
        geometry_msgs::msg::Twist out;
        out.linear.x = clampAbs(in.linear.x, max_linear_x_);
        out.linear.y = clampAbs(in.linear.y, max_linear_y_);
        out.angular.z = clampAbs(in.angular.z, max_angular_z_);
        return out;
    }

    bool isZero(const geometry_msgs::msg::Twist &cmd) const
    {
        return std::abs(cmd.linear.x) <= std::numeric_limits<double>::epsilon() &&
                     std::abs(cmd.linear.y) <= std::numeric_limits<double>::epsilon() &&
                     std::abs(cmd.angular.z) <= std::numeric_limits<double>::epsilon();
    }

    std::string buildJson(const geometry_msgs::msg::Twist &cmd)
    {
        std::string payload = sdk_json_template_;
        replaceAll(payload, "{linear_x}", toFixed6(cmd.linear.x));
        replaceAll(payload, "{linear_y}", toFixed6(cmd.linear.y));
        replaceAll(payload, "{angular_z}", toFixed6(cmd.angular.z));
        replaceAll(payload, "{timestamp_ms}", std::to_string(nowMsSinceEpoch()));
        replaceAll(payload, "{guid}", std::to_string(sequence_id_++));
        return payload;
    }

    void dispatchCommand(const geometry_msgs::msg::Twist &cmd)
    {
        if (enable_relay_output_ && relay_pub_) {
            relay_pub_->publish(cmd);
        }

        if (enable_sdk_output_ && robot_) {
            robot_->publishJsonMessage(buildJson(cmd));
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const auto cmd = sanitizeCmd(*msg);
        latest_cmd_ = cmd;
        last_cmd_time_ = this->now();
        sent_timeout_stop_ = false;

        dispatchCommand(cmd);

        RCLCPP_DEBUG(
            get_logger(),
            "cmd_vel in=(%.3f, %.3f, %.3f) out=(%.3f, %.3f, %.3f)",
            msg->linear.x,
            msg->linear.y,
            msg->angular.z,
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z);
    }

    void watchdogTimerCallback()
    {
        if (deadman_timeout_s_ <= 0.0) {
            return;
        }

        const double dt = (this->now() - last_cmd_time_).seconds();
        if (dt < deadman_timeout_s_) {
            return;
        }

        if (sent_timeout_stop_ && isZero(latest_cmd_)) {
            return;
        }

        geometry_msgs::msg::Twist stop_cmd;
        latest_cmd_ = stop_cmd;
        dispatchCommand(stop_cmd);
        sent_timeout_stop_ = true;

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "No cmd_vel received for %.2f s, sending zero command.",
            dt);
    }

private:
    bool enable_sdk_output_;
    bool enable_relay_output_;
    std::string cmd_vel_topic_in_;
    std::string cmd_vel_topic_out_;
    std::string robot_ip_;
    double max_linear_x_;
    double max_linear_y_;
    double max_angular_z_;
    double deadman_timeout_s_;
    std::string sdk_json_template_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr relay_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    limxsdk::Tron2 *robot_{nullptr};
    geometry_msgs::msg::Twist latest_cmd_;
    rclcpp::Time last_cmd_time_;
    bool sent_timeout_stop_;
    uint64_t sequence_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tron2CmdVelTranslator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}