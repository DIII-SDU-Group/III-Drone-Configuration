#pragma once

#include <iii_drone_configuration/configuration.hpp>
#include <iii_drone_configuration/schema_validator.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace iii_drone {
namespace configuration {

template <typename nodeT>
class Configurator {
public:
    Configurator(
        nodeT *node,
        const std::string & node_name,
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
    );

    ~Configurator();

    rclcpp::Parameter GetParameter(const std::string & parameter_full_name) const;

    std::vector<rclcpp::Parameter> GetParameters(const std::vector<std::string> & parameter_full_names) const;

    Configuration::SharedPtr GetConfiguration(const std::string & name) const;

    void DeclareParameter(
        const std::string & parameter_full_name,
        rclcpp::ParameterType parameter_type
    );

    void DeclareParameters(
        const std::vector<std::string> & parameter_full_names,
        const std::vector<rclcpp::ParameterType> & parameter_types
    );

    Configuration::SharedPtr CreateConfiguration(
        const std::string & name,
        const std::vector<configuration_entry_t> & entries
    );

    void SyncParameters(const std::vector<std::string> & parameter_full_names = {});

    static std::string GetParameterTypeString(rclcpp::ParameterType parameter_type);

    static rclcpp::ParameterType GetParameterTypeFromString(const std::string & parameter_type);

    void PrintParameters() const;

    void PrintConfigurations() const;

    void validate() const;

    using SharedPtr = std::shared_ptr<Configurator>;

private:
    nodeT *node_;

    std::vector<Configuration::SharedPtr> configurations_;

    std::vector<std::string> managed_parameter_names_;

    mutable std::shared_mutex parameters_mutex_;

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_events_subscriber_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr managed_node_announcement_publisher_;

    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

    SchemaValidator schema_validator_;

    std::string schema_file_path_;

    bool managed_node_announced_ = false;

    void declareParameters(
        const std::vector<std::string> & parameter_full_names,
        const std::vector<rclcpp::ParameterType> & parameter_types
    );

    bool undeclareParameters();

    void parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event);

    rcl_interfaces::msg::SetParametersResult onSetParametersCallback(
        const std::vector<rclcpp::Parameter> & parameters
    );

    std::unordered_map<std::string, rclcpp::ParameterValue> getCurrentManagedParameterValues() const;

    std::unordered_map<std::string, rclcpp::ParameterValue> getCurrentValuesWithSchemaDefaults() const;

    std::string resolveSchemaFilePath();

    void announceManagedNode() const;
};

}  // namespace configuration
}  // namespace iii_drone
