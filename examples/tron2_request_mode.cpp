#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "limxsdk/tron2.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

enum class RobotMode
{
    UNKNOWN,
    IDLE,      // Robot not initialized or sitting
    STANDING,  // Standing mode
    WALKING,   // Walking mode
    ERROR      // Error state
};

class Tron2RequestMode : public rclcpp::Node
{
public:
    Tron2RequestMode()
    : Node("tron2_request_mode"),
        robot_ip_(declare_parameter<std::string>("robot_ip", "10.192.1.2")),
        robot_type_(declare_parameter<std::string>("robot_type", "wheel_leg")),  // wheel_leg or sole_foot
        enable_stair_(declare_parameter<bool>("enable_stair", false)),
        offline_mode_(declare_parameter<bool>("offline_mode", false)),
        current_mode_(RobotMode::UNKNOWN),
        accid_(declare_parameter<std::string>("accid", "tron2a-405"))
    {
        if (!offline_mode_) {
            // Initialize the robot
            robot_ = limxsdk::Tron2::getInstance();
            if (!robot_->init(robot_ip_)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to initialize TRON2 SDK with robot_ip=%s",
                    robot_ip_.c_str());
                current_mode_ = RobotMode::ERROR;
                return;
            }

            RCLCPP_INFO(get_logger(), "TRON2 SDK connected to robot_ip=%s", robot_ip_.c_str());
        } else {
            RCLCPP_WARN(
                get_logger(),
                "Offline mode enabled. Starting without robot connection. accid=%s",
                accid_.c_str());
            current_mode_ = RobotMode::IDLE;
        }

        current_mode_ = RobotMode::IDLE;

        // Subscribe to robot notifications to get the actual serial number (accid).
        // Keep a manual fallback value so the node can be tested offline with a known ID.
        if (!offline_mode_ && robot_) {
            robot_->subscribeJsonMessage([this](const std::string& json_payload) {
                try {
                    const auto message = json::parse(json_payload);
                    if (!message.is_object()) {
                        return;
                    }

                    auto extractString = [](const json& obj, const std::string& key) -> std::string {
                        if (obj.is_object() && obj.contains(key) && obj[key].is_string()) {
                            return obj[key].get<std::string>();
                        }
                        return {};
                    };

                    std::string new_accid;
                    new_accid = extractString(message, "accid");
                    if (new_accid.empty() && message.contains("data")) {
                        new_accid = extractString(message["data"], "accid");
                    }
                    if (new_accid.empty() && message.contains("data") && message["data"].contains("robot_info")) {
                        new_accid = extractString(message["data"]["robot_info"], "accid");
                    }
                    if (new_accid.empty() && message.contains("data") && message["data"].contains("robot_id")) {
                        new_accid = extractString(message["data"], "robot_id");
                    }
                    if (new_accid.empty() && message.contains("data") && message["data"].contains("robot")) {
                        new_accid = extractString(message["data"]["robot"], "accid");
                    }

                    if (!new_accid.empty() && new_accid != "nav_bridge" && new_accid != "UNKNOWN" &&
                        new_accid != accid_) {
                        accid_ = new_accid;
                        RCLCPP_INFO(get_logger(), "Robot accid updated from notification: %s", accid_.c_str());
                    }
                } catch (const json::parse_error& e) {
                    RCLCPP_WARN(get_logger(), "Ignoring malformed JSON notification: %s", e.what());
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "Failed to parse robot notification for accid: %s", e.what());
                }
            });
        }

        // Create service servers for all modes (3.8.1 - 3.8.9)
        stand_service_ = create_service<std_srvs::srv::Trigger>(
            "tron2_stand_mode",
            std::bind(&Tron2RequestMode::handleStandMode, this, std::placeholders::_1, std::placeholders::_2));

        walk_service_ = create_service<std_srvs::srv::Trigger>(
            "tron2_walk_mode",
            std::bind(&Tron2RequestMode::handleWalkMode, this, std::placeholders::_1, std::placeholders::_2));

        sitdown_service_ = create_service<std_srvs::srv::Trigger>(
            "tron2_sitdown",
            std::bind(&Tron2RequestMode::handleSitdown, this, std::placeholders::_1, std::placeholders::_2));

        emgy_stop_service_ = create_service<std_srvs::srv::Trigger>(
            "tron2_emgy_stop",
            std::bind(&Tron2RequestMode::handleEmgyStop, this, std::placeholders::_1, std::placeholders::_2));

        recover_service_ = create_service<std_srvs::srv::Trigger>(
            "tron2_recover",
            std::bind(&Tron2RequestMode::handleRecover, this, std::placeholders::_1, std::placeholders::_2));

        stair_service_ = create_service<std_srvs::srv::Trigger>(
            "tron2_stair_mode",
            std::bind(&Tron2RequestMode::handleStairMode, this, std::placeholders::_1, std::placeholders::_2));

        imu_service_ = create_service<std_srvs::srv::Trigger>(
            "tron2_enable_imu",
            std::bind(&Tron2RequestMode::handleEnableIMU, this, std::placeholders::_1, std::placeholders::_2));

        // Subscribers for dynamic commands
        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "tron2_base_height_cmd",
            10,
            std::bind(&Tron2RequestMode::handleBaseHeightCmd, this, std::placeholders::_1));

        // Publisher for mode status
        mode_status_pub_ = create_publisher<std_msgs::msg::String>("tron2_mode_status", 10);

        // Timer to periodically query accid and publish status
        status_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Tron2RequestMode::publishStatus, this));

        RCLCPP_INFO(
            get_logger(),
            "Tron2RequestMode initialized. robot_type=%s, accid=%s%s",
            robot_type_.c_str(),
            accid_.c_str(),
            offline_mode_ ? " (offline mode)" : " (live mode)");
        RCLCPP_INFO(
            get_logger(),
            "Available services: tron2_stand_mode, tron2_walk_mode, tron2_sitdown, "
            "tron2_emgy_stop, tron2_recover, tron2_stair_mode, tron2_enable_imu");
    }

private:
    // Helper: Convert RobotMode to string
    static std::string modeToString(RobotMode mode)
    {
        switch (mode) {
            case RobotMode::UNKNOWN:
                return "UNKNOWN";
            case RobotMode::IDLE:
                return "IDLE";
            case RobotMode::STANDING:
                return "STANDING";
            case RobotMode::WALKING:
                return "WALKING";
            case RobotMode::ERROR:
                return "ERROR";
            default:
                return "UNKNOWN";
        }
    }

    // Helper: Get current timestamp in milliseconds
    uint64_t getCurrentTimestampMs()
    {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    }

    // Helper: Build JSON request with proper accid
    std::string buildModeRequestJson(const std::string& title, const json& data = json::object())
    {
        if (accid_.empty() || accid_ == "UNKNOWN" || accid_ == "nav_bridge") {
            RCLCPP_WARN(
                get_logger(),
                "accid is not yet known; sending with current value '%s'. The robot may reject the command until it sends robot_info.",
                accid_.c_str());
        }

        json message;
        message["accid"] = accid_;  // Use actual robot serial number
        message["title"] = title;
        message["timestamp"] = getCurrentTimestampMs();
        message["guid"] = std::to_string(sequence_id_++);
        message["data"] = data;
        return message.dump();
    }

    // 3.8.1: Stand Mode
    void handleStandMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (current_mode_ == RobotMode::ERROR) {
            response->success = false;
            response->message = "Robot is in ERROR state";
            RCLCPP_ERROR(get_logger(), "Cannot stand: Robot in ERROR state");
            return;
        }

        try {
            RCLCPP_INFO(get_logger(), "Sending request_stand_mode to accid=%s...", accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_stand_mode"));
            }
            
            current_mode_ = RobotMode::STANDING;
            response->success = true;
            response->message = "Stand mode requested";
            RCLCPP_INFO(get_logger(), "Stand mode: SUCCESS");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Stand mode failed: ") + e.what();
            current_mode_ = RobotMode::ERROR;
            RCLCPP_ERROR(get_logger(), "Stand mode exception: %s", e.what());
        }
    }

    // 3.8.2: Walk Mode
    void handleWalkMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (current_mode_ == RobotMode::ERROR) {
            response->success = false;
            response->message = "Robot is in ERROR state";
            RCLCPP_ERROR(get_logger(), "Cannot walk: Robot in ERROR state");
            return;
        }

        if (current_mode_ != RobotMode::STANDING) {
            response->success = false;
            response->message = "Robot must be in STANDING mode first. Current: " + modeToString(current_mode_);
            RCLCPP_WARN(
                get_logger(),
                "Cannot walk: Robot not in STANDING mode. Current: %s",
                modeToString(current_mode_).c_str());
            return;
        }

        try {
            RCLCPP_INFO(get_logger(), "Sending request_walk_mode to accid=%s...", accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_walk_mode"));
            }
            
            current_mode_ = RobotMode::WALKING;
            response->success = true;
            response->message = "Walk mode requested. Ready for cmd_vel commands.";
            RCLCPP_INFO(get_logger(), "Walk mode: SUCCESS");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Walk mode failed: ") + e.what();
            current_mode_ = RobotMode::ERROR;
            RCLCPP_ERROR(get_logger(), "Walk mode exception: %s", e.what());
        }
    }

    // 3.8.5: Sit Down
    void handleSitdown(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (current_mode_ == RobotMode::ERROR) {
            response->success = false;
            response->message = "Robot is in ERROR state";
            RCLCPP_ERROR(get_logger(), "Cannot sit: Robot in ERROR state");
            return;
        }

        try {
            RCLCPP_INFO(get_logger(), "Sending request_sitdown to accid=%s...", accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_sitdown"));
            }
            
            current_mode_ = RobotMode::IDLE;
            response->success = true;
            response->message = "Sitdown requested";
            RCLCPP_INFO(get_logger(), "Sitdown: SUCCESS");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Sitdown failed: ") + e.what();
            current_mode_ = RobotMode::ERROR;
            RCLCPP_ERROR(get_logger(), "Sitdown exception: %s", e.what());
        }
    }

    // 3.8.7: Emergency Stop
    void handleEmgyStop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        try {
            RCLCPP_WARN(get_logger(), "EMERGENCY STOP requested! Sending to accid=%s...", accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_emgy_stop"));
            }
            
            current_mode_ = RobotMode::IDLE;
            response->success = true;
            response->message = "Emergency stop sent";
            RCLCPP_WARN(get_logger(), "Emergency stop: SUCCESS");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Emergency stop failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "Emergency stop exception: %s", e.what());
        }
    }

    // 3.8.9: Fall Recovery
    void handleRecover(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        try {
            RCLCPP_INFO(get_logger(), "Sending request_recover to accid=%s...", accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_recover"));
            }
            
            response->success = true;
            response->message = "Recovery initiated. Robot attempting to stand and recover.";
            RCLCPP_INFO(get_logger(), "Recovery: SUCCESS");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Recovery failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "Recovery exception: %s", e.what());
        }
    }

    // 3.8.6: Stair Mode (Dual-Wheel-Leg Only)
    void handleStairMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (robot_type_ != "wheel_leg") {
            response->success = false;
            response->message = "Stair mode only supported on wheel-leg robots";
            RCLCPP_WARN(get_logger(), "Stair mode not supported on robot_type=%s", robot_type_.c_str());
            return;
        }

        try {
            bool enable = !enable_stair_;  // Toggle stair mode
            json data;
            data["enable"] = enable;
            
            RCLCPP_INFO(
                get_logger(),
                "Sending request_stair_mode (enable=%s) to accid=%s...",
                enable ? "true" : "false",
                accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_stair_mode", data));
            }
            
            enable_stair_ = enable;
            response->success = true;
            response->message = std::string("Stair mode ") + (enable ? "enabled" : "disabled");
            RCLCPP_INFO(get_logger(), "Stair mode: SUCCESS (%s)", enable ? "enabled" : "disabled");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Stair mode failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "Stair mode exception: %s", e.what());
        }
    }

    // 3.8.8: Enable IMU Data
    void handleEnableIMU(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        try {
            bool enable = !imu_enabled_;  // Toggle IMU
            json data;
            data["enable"] = enable;
            
            RCLCPP_INFO(
                get_logger(),
                "Sending request_enable_imu (enable=%s) to accid=%s...",
                enable ? "true" : "false",
                accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_enable_imu", data));
            }
            
            imu_enabled_ = enable;
            response->success = true;
            response->message = std::string("IMU ") + (enable ? "enabled" : "disabled");
            RCLCPP_INFO(get_logger(), "IMU: SUCCESS (%s)", enable ? "enabled" : "disabled");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("IMU control failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "IMU exception: %s", e.what());
        }
    }

    // 3.8.4: Adjust Robot Body Height (via cmd_vel subscriber)
    void handleBaseHeightCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Use linear.z to encode height direction: positive=raise, negative=lower
        int direction = (msg->linear.z > 0.0) ? 1 : -1;

        try {
            json data;
            data["direction"] = direction;
            
            RCLCPP_INFO(
                get_logger(),
                "Sending request_base_height (direction=%d) to accid=%s...",
                direction,
                accid_.c_str());
            if (robot_) {
                robot_->publishJsonMessage(buildModeRequestJson("request_base_height", data));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Base height adjustment failed: %s", e.what());
        }
    }

    void publishStatus()
    {
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "Mode=" + modeToString(current_mode_) + " | accid=" + accid_;
        mode_status_pub_->publish(status_msg);
    }

private:
    std::string robot_ip_;
    std::string robot_type_;  // "wheel_leg" or "sole_foot"
    RobotMode current_mode_;
    std::string accid_;  // Robot's actual serial number
    uint64_t sequence_id_ = 0;
    bool enable_stair_ = false;
    bool imu_enabled_ = false;
    bool offline_mode_ = false;

    limxsdk::Tron2* robot_{nullptr};

    // Services for all modes (3.8.1-3.8.9)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr walk_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sitdown_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emgy_stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stair_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_service_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tron2RequestMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
