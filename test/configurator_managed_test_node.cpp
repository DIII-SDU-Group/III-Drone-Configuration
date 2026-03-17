#include <iii_drone_configuration/configurator.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <csignal>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using iii_drone::configuration::Configurator;

namespace {

volatile std::sig_atomic_t g_shutdown_requested = 0;

void SignalHandler(int)
{
    g_shutdown_requested = 1;
}

template <typename NodeT>
int RunNode(const std::string & node_name)
{
    auto node = std::make_shared<NodeT>(node_name);
    auto configurator = std::make_shared<Configurator<NodeT>>(node.get(), node_name);

    configurator->DeclareParameters(
        {
            "/control/gains/p",
            "/control/gains/i",
            "/control/mode",
            "/control/immutable_name",
        },
        {
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            rclcpp::ParameterType::PARAMETER_STRING,
            rclcpp::ParameterType::PARAMETER_STRING,
        }
    );
    configurator->validate();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    while (rclcpp::ok() && g_shutdown_requested == 0) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    executor.remove_node(node->get_node_base_interface());
    return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
    setenv("III_DRONE_SCHEMA_FILE", TEST_SCHEMA_FILE, 1);
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    rclcpp::init(argc, argv);

    std::string node_name = "cpp_managed_test_node";
    std::string node_type = "node";
    if (argc > 1) {
        node_type = argv[1];
    }
    if (argc > 2) {
        node_name = argv[2];
    }

    const int result = node_type == "lifecycle"
        ? RunNode<rclcpp_lifecycle::LifecycleNode>(node_name)
        : RunNode<rclcpp::Node>(node_name);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return result;
}
