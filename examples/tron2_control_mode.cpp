#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "limxsdk/tron2.h"
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using json = nlohmann::json;

class Tron2ControlMode : public rclcpp::Node
{
public:
    explicit Tron2ControlMode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("tron2_control_mode", options)
    {
        const auto robot_ip = declare_parameter<std::string>("robot_ip", "10.192.1.2");
        robot_type_ = declare_parameter<std::string>("robot_type", "wheel_leg");
        accid_ = declare_parameter<std::string>("accid", "");
        offline_ = declare_parameter<bool>("offline_mode", false);
        ack_timeout_ = declare_parameter<double>("ack_timeout_s", 3.0);
        stale_timeout_ = declare_parameter<double>("stale_timeout_s", 5.0);
        result_pointer_ = declare_parameter<std::string>("ack_result_pointer", "/data/result");
        success_value_ = declare_parameter<std::string>("ack_success_value", "success");
        if (!std::isfinite(ack_timeout_) || ack_timeout_ <= 0.0 ||
            !std::isfinite(stale_timeout_) || stale_timeout_ <= 0.0 ||
            (robot_type_ != "wheel_leg" && robot_type_ != "sole_foot")) {
            throw std::invalid_argument("Invalid timeout or robot_type parameter");
        }
        json::json_pointer validated_pointer(result_pointer_);
        (void)validated_pointer;

        rx_pub_ = create_publisher<std_msgs::msg::String>("~/json_rx", 100);
        tx_pub_ = create_publisher<std_msgs::msg::String>("~/json_tx", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
        addTrigger("~/stand", "request_stand_mode", "STANDING");
        addTrigger("~/walk", "request_walk_mode", "WALKING");
        addTrigger("~/sitdown", "request_sitdown", "IDLE");
        addTrigger("~/emgy_stop", "request_emgy_stop", "EMERGENCY_STOP");
        addTrigger("~/recover", "request_recover", "RECOVERING");
        addBoolean("~/enable_imu", "request_enable_imu", "enable");
        addBoolean("~/stair_mode", "request_stair_mode", "enable");
        addBoolean("~/base_height", "request_base_height", "direction");
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { poll(); });

        if (!offline_) {
            robot_ = limxsdk::Tron2::getInstance();
            initialized_ = robot_->init(robot_ip);
            if (!initialized_) {
                RCLCPP_ERROR(get_logger(), "SDK initialization failed; services will reject commands");
            } else {
                std::weak_ptr<Inbox> weak_inbox = inbox_;
                robot_->subscribeJsonMessage([weak_inbox](const std::string& payload) {
                    if (auto inbox = weak_inbox.lock()) {
                        std::lock_guard<std::mutex> lock(inbox->mutex);
                        if (inbox->messages.size() >= 1000) {
                            ++inbox->dropped;
                            return;
                        }
                        inbox->messages.push_back({payload, Clock::now()});
                    }
                });
            }
        }
        RCLCPP_INFO(get_logger(), "SDK JSON control over MROS (not a direct WebSocket client). offline=%s",
                    offline_ ? "true" : "false");
    }

private:
    friend class Tron2ControlModeTest;
    using Clock = std::chrono::steady_clock;
    using Completion = std::function<void(bool, const std::string&)>;
    struct Received {
        std::string payload;
        Clock::time_point received;
    };
    struct Inbox {
        std::mutex mutex;
        std::deque<Received> messages;
        uint64_t dropped = 0;
    };
    struct Pending {
        std::string title;
        std::string accid;
        bool mode_change;
        Clock::time_point deadline;
        Completion complete;
    };

    template<typename Service>
    Completion completionFor(std::shared_ptr<rclcpp::Service<Service>> service,
                             std::shared_ptr<rmw_request_id_t> header)
    {
        return [this, service, header](bool success, const std::string& message) {
            typename Service::Response response;
            response.success = success;
            response.message = message;
            try {
                service->send_response(*header, response);
            } catch (const std::exception& error) {
                RCLCPP_WARN(get_logger(), "Unable to return service response: %s", error.what());
            }
        };
    }

    void addTrigger(const std::string& name, const std::string& title, const std::string& mode)
    {
        using Service = std_srvs::srv::Trigger;
        trigger_services_.push_back(create_service<Service>(name,
            [this, title, mode](std::shared_ptr<rclcpp::Service<Service>> service,
                               std::shared_ptr<rmw_request_id_t> header,
                               std::shared_ptr<Service::Request>) {
                send(title, json::object(), mode, completionFor<Service>(service, header));
            }));
    }

    void addBoolean(const std::string& name, const std::string& title, const std::string& field)
    {
        using Service = std_srvs::srv::SetBool;
        boolean_services_.push_back(create_service<Service>(name,
            [this, title, field](std::shared_ptr<rclcpp::Service<Service>> service,
                                std::shared_ptr<rmw_request_id_t> header,
                                std::shared_ptr<Service::Request> request) {
                auto complete = completionFor<Service>(service, header);
                if (title == "request_stair_mode" && robot_type_ != "wheel_leg") {
                    complete(false, "Stair mode requires wheel_leg");
                    return;
                }
                json data;
                if (field == "direction") {
                    data[field] = request->data ? 1 : -1;
                } else {
                    data[field] = request->data;
                }
                send(title, data, "", std::move(complete));
            }));
    }

    void send(const std::string& title, const json& data, const std::string& mode, Completion complete)
    {
        if (offline_) {
            complete(false, "Dry run: command not sent; no robot acknowledgment");
            return;
        }
        if (!initialized_) {
            complete(false, "SDK not initialized");
            return;
        }
        if (accid_.empty() || accid_ == "UNKNOWN" || accid_ == "nav_bridge") {
            complete(false, "Robot accid unknown; wait for notification or set accid parameter");
            return;
        }
        const bool emergency = title == "request_emgy_stop";
        for (const auto& entry : pending_) {
            if (!mode.empty() && entry.second.mode_change && !emergency) {
                complete(false, "Another mode request is pending");
                return;
            }
        }
        if (emergency) {
            for (auto iterator = pending_.begin(); iterator != pending_.end();) {
                if (iterator->second.mode_change) {
                    auto interrupted = std::move(iterator->second.complete);
                    iterator = pending_.erase(iterator);
                    interrupted(false, "Superseded by emergency stop; robot outcome unknown");
                } else {
                    ++iterator;
                }
            }
        } else if (pending_.size() >= 8) {
            complete(false, "Too many pending requests");
            return;
        }
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const std::string guid = session_id_ + "-" + std::to_string(sequence_++);
        const json request = {{"accid", accid_}, {"title", title}, {"guid", guid},
                              {"timestamp", timestamp}, {"data", data}};
        pending_.emplace(guid, Pending{title, accid_, !mode.empty(),
            Clock::now() + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(ack_timeout_)), std::move(complete)});
        last_command_ = title;
        last_result_ = "PENDING";
        if (!mode.empty()) {
            requested_mode_ = mode;
        }
        try {
            const auto payload = request.dump();
            robot_->publishJsonMessage(payload);
            publish(tx_pub_, payload);
            RCLCPP_INFO(get_logger(), "Sent %s guid=%s", title.c_str(), guid.c_str());
        } catch (const std::exception& error) {
            finish(guid, false, "SEND_FAILED", std::string("Send failed; outcome unknown: ") + error.what());
        }
    }

    void finish(const std::string& guid, bool success, const std::string& result,
                const std::string& detail)
    {
        auto found = pending_.find(guid);
        if (found == pending_.end()) {
            return;
        }
        last_command_ = found->second.title;
        last_result_ = result;
        auto complete = std::move(found->second.complete);
        pending_.erase(found);
        RCLCPP_INFO(get_logger(), "%s guid=%s: %s", last_command_.c_str(), guid.c_str(), detail.c_str());
        complete(success, detail);
    }

    static std::string stringField(const json& message, const std::string& key)
    {
        if (message.is_object() && message.contains(key) && message[key].is_string()) {
            return message[key].get<std::string>();
        }
        return {};
    }

    void receive(const Received& received)
    {
        publish(rx_pub_, received.payload);
        const auto message = json::parse(received.payload, nullptr, false);
        if (!message.is_object()) {
            RCLCPP_WARN(get_logger(), "Ignoring malformed/non-object JSON");
            return;
        }
        const auto title = stringField(message, "title");
        const auto identity = stringField(message, "accid");
        if (identity.empty() || identity == "nav_bridge" || identity == "UNKNOWN" ||
            (!accid_.empty() && identity != accid_)) {
            return;
        }
        accid_ = identity;
        last_rx_ = received.received;
        const auto guid = stringField(message, "guid");
        auto found = pending_.find(guid);
        if (found == pending_.end() || identity != found->second.accid ||
            title != "response_" + found->second.title.substr(8)) {
            return;
        }
        if (received.received > found->second.deadline) {
            finish(guid, false, "TIMED_OUT", "No timely robot acknowledgment; outcome unknown");
            return;
        }
        const json::json_pointer pointer(result_pointer_);
        if (!message.contains(pointer) || !message.at(pointer).is_string()) {
            finish(guid, false, "UNKNOWN_RESULT", "Matched response but unrecognized result schema: " + received.payload);
            return;
        }
        const auto result = message.at(pointer).get<std::string>();
        if (result == success_value_) {
            finish(guid, true, "ACCEPTED", "Robot acknowledged request; physical completion not confirmed: " + received.payload);
        } else if (result.rfind("fail_", 0) == 0 || result == "fail" || result == "failed" || result == "error") {
            finish(guid, false, "REJECTED", "Robot rejected request: " + received.payload);
        } else {
            finish(guid, false, "UNKNOWN_RESULT", "Unrecognized robot result; outcome unknown: " + received.payload);
        }
    }

    static void publish(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher,
                        const std::string& payload)
    {
        std_msgs::msg::String message;
        message.data = payload;
        publisher->publish(message);
    }

    void poll()
    {
        std::deque<Received> received;
        {
            std::lock_guard<std::mutex> lock(inbox_->mutex);
            received.swap(inbox_->messages);
            if (inbox_->dropped) {
                RCLCPP_WARN(get_logger(), "Dropped %llu JSON messages due to full receive queue",
                            static_cast<unsigned long long>(inbox_->dropped));
                inbox_->dropped = 0;
            }
        }
        for (const auto& message : received) {
            receive(message);
        }
        const auto now = Clock::now();
        std::vector<std::string> expired;
        for (const auto& entry : pending_) {
            if (now >= entry.second.deadline) {
                expired.push_back(entry.first);
            }
        }
        for (const auto& guid : expired) {
            finish(guid, false, "TIMED_OUT", "No robot acknowledgment; outcome unknown (do not blindly retry)");
        }
        if (now - last_status_ < std::chrono::seconds(1)) {
            return;
        }
        last_status_ = now;
        const std::string connection = offline_ ? "OFFLINE" : !initialized_ ? "SDK_INIT_FAILED" :
            last_rx_ == Clock::time_point{} ? "UNKNOWN" :
            std::chrono::duration<double>(now - last_rx_).count() > stale_timeout_ ? "STALE" : "RECEIVING";
        publish(status_pub_, json{{"transport", "sdk_mros_json"}, {"connection", connection},
            {"accid", accid_}, {"requested_mode", requested_mode_}, {"confirmed_mode", "UNKNOWN"},
            {"last_command", last_command_}, {"last_result", last_result_},
            {"pending_requests", pending_.size()}}.dump());
    }

    std::string robot_type_, accid_, result_pointer_, success_value_;
    bool offline_ = false;
    bool initialized_ = false;
    double ack_timeout_ = 3.0;
    double stale_timeout_ = 5.0;
    limxsdk::Tron2* robot_ = nullptr;
    std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
    std::map<std::string, Pending> pending_;
    std::string requested_mode_ = "UNKNOWN";
    std::string last_command_, last_result_ = "NONE";
    const std::string session_id_ = std::to_string(
        std::chrono::system_clock::now().time_since_epoch().count());
    uint64_t sequence_ = 0;
    Clock::time_point last_rx_{}, last_status_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rx_pub_, tx_pub_, status_pub_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> trigger_services_;
    std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> boolean_services_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef TRON2_CONTROL_MODE_NO_MAIN
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Tron2ControlMode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("tron2_control_mode"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
#endif