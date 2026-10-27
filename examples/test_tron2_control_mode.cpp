#define TRON2_CONTROL_MODE_NO_MAIN
#include "tron2_control_mode.cpp"

#include <iostream>
#include <stdexcept>

class Tron2ControlModeTest
{
public:
    static void run()
    {
        testResponses();
        testTimeouts();
        testOfflineAndUninitialized();
        testDeferredService();
    }

private:
    using Node = Tron2ControlMode;
    using Clock = Node::Clock;

    struct Outcome {
        int calls = 0;
        bool success = false;
        std::string message;
    };

    static void require(bool condition, const std::string& message)
    {
        if (!condition) {
            throw std::runtime_error(message);
        }
    }

    static std::shared_ptr<Node> makeNode()
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("offline_mode", true),
            rclcpp::Parameter("accid", "test-robot")});
        return std::make_shared<Node>(options);
    }

    static Node::Completion completion(Outcome& outcome)
    {
        return [&outcome](bool success, const std::string& message) {
            ++outcome.calls;
            outcome.success = success;
            outcome.message = message;
        };
    }

    static void addPending(Node& node, Outcome& outcome,
                           Clock::time_point deadline = Clock::now() + std::chrono::seconds(2))
    {
        node.pending_.emplace("test-guid", Node::Pending{
            "request_stand_mode", "test-robot", true, deadline, completion(outcome)});
        node.requested_mode_ = "STANDING";
        node.last_result_ = "PENDING";
    }

    static json response(const std::string& result = "success")
    {
        return {{"accid", "test-robot"}, {"guid", "test-guid"},
                {"title", "response_stand_mode"}, {"data", {{"result", result}}}};
    }

    static void receive(Node& node, const json& message)
    {
        node.receive(Node::Received{message.dump(), Clock::now()});
    }

    static void testResponses()
    {
        auto node = makeNode();
        Outcome outcome;
        addPending(*node, outcome);

        node->receive(Node::Received{"{invalid", Clock::now()});
        receive(*node, json::array());
        auto unrelated = response();
        unrelated["guid"] = "different-guid";
        receive(*node, unrelated);
        unrelated = response();
        unrelated["title"] = "response_walk_mode";
        receive(*node, unrelated);
        unrelated = response();
        unrelated["accid"] = "different-robot";
        receive(*node, unrelated);
        require(outcome.calls == 0 && node->pending_.size() == 1,
                "Malformed or unrelated replies must not complete a request");

        receive(*node, response());
        require(outcome.calls == 1 && outcome.success && node->pending_.empty(),
                "A matching success reply must complete the request");
        require(node->last_result_ == "ACCEPTED" &&
                outcome.message.find("physical completion not confirmed") != std::string::npos,
                "Acceptance must not claim physical completion");
        receive(*node, response());
        require(outcome.calls == 1, "Duplicate replies must not complete twice");

        for (const auto& result : {"fail_imu", "fail_motor", "fail", "failed", "error"}) {
            outcome = Outcome{};
            addPending(*node, outcome);
            receive(*node, response(result));
            require(outcome.calls == 1 && !outcome.success && node->last_result_ == "REJECTED",
                    std::string("Expected rejection for ") + result);
        }

        outcome = Outcome{};
        addPending(*node, outcome);
        receive(*node, response("unexpected"));
        require(outcome.calls == 1 && !outcome.success && node->last_result_ == "UNKNOWN_RESULT",
                "Unknown result values must not imply success");

        for (const auto& data : {json::object(), json{{"result", true}}}) {
            outcome = Outcome{};
            addPending(*node, outcome);
            auto malformed_result = response();
            malformed_result["data"] = data;
            receive(*node, malformed_result);
            require(outcome.calls == 1 && !outcome.success && node->last_result_ == "UNKNOWN_RESULT",
                    "Missing or non-string result must have an unknown outcome");
        }

        outcome = Outcome{};
        node->result_pointer_ = "/data/status";
        node->success_value_ = "ok";
        addPending(*node, outcome);
        auto custom = response();
        custom["data"] = {{"status", "ok"}};
        receive(*node, custom);
        require(outcome.calls == 1 && outcome.success,
                "Configured acknowledgment schema must be honored");
    }

    static void testTimeouts()
    {
        auto node = makeNode();
        Outcome outcome;
        addPending(*node, outcome, Clock::now() - std::chrono::seconds(1));
        node->poll();
        require(outcome.calls == 1 && !outcome.success && node->pending_.empty() &&
                node->last_result_ == "TIMED_OUT",
                "Expired requests must complete with a timeout");
        receive(*node, response());
        require(outcome.calls == 1, "Replies after timeout must be ignored");

        outcome = Outcome{};
        addPending(*node, outcome, Clock::now() - std::chrono::seconds(1));
        receive(*node, response());
        require(outcome.calls == 1 && !outcome.success && node->last_result_ == "TIMED_OUT",
                "Late success replies must not imply timely acceptance");

        outcome = Outcome{};
        const auto received_at = Clock::now() - std::chrono::seconds(2);
        addPending(*node, outcome, received_at + std::chrono::seconds(1));
        {
            std::lock_guard<std::mutex> lock(node->inbox_->mutex);
            node->inbox_->messages.push_back(Node::Received{response().dump(), received_at});
        }
        node->poll();
        require(outcome.calls == 1 && outcome.success,
                "Timely replies queued before the deadline must still be accepted");
    }

    static void testOfflineAndUninitialized()
    {
        auto node = makeNode();
        Outcome outcome;
        node->send("request_stand_mode", json::object(), "STANDING", completion(outcome));
        require(outcome.calls == 1 && !outcome.success && node->pending_.empty() &&
                node->requested_mode_ == "UNKNOWN" &&
                outcome.message.find("Dry run") != std::string::npos,
                "Offline commands must not report robot success or change requested mode");

        outcome = Outcome{};
        node->offline_ = false;
        node->send("request_stand_mode", json::object(), "STANDING", completion(outcome));
        require(outcome.calls == 1 && !outcome.success && node->pending_.empty() &&
                outcome.message == "SDK not initialized",
                "Uninitialized SDK must reject requests without sending");
    }

    static void testDeferredService()
    {
        using Service = std_srvs::srv::Trigger;
        auto node = makeNode();
        auto client_node = std::make_shared<rclcpp::Node>("tron2_control_mode_test_client");
        auto client = client_node->create_client<Service>("/tron2_control_mode/stand");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(client_node);
        require(client->wait_for_service(std::chrono::seconds(2)), "Stand service was not discovered");
        auto future = client->async_send_request(std::make_shared<Service::Request>());
        require(executor.spin_until_future_complete(future, std::chrono::seconds(2)) ==
                    rclcpp::FutureReturnCode::SUCCESS,
                "Deferred service response was not delivered");
        const auto reply = future.get();
        require(!reply->success && reply->message.find("Dry run") != std::string::npos,
                "ROS service must return the offline failure response");
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        Tron2ControlModeTest::run();
        std::cout << "All TRON2 control tests passed (no robot connection)." << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "TRON2 control test failed: " << error.what() << std::endl;
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}