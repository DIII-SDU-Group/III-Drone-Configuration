#include "iii_drone_configuration/configurator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

using namespace iii_drone::configuration;

namespace {

std::string ExpandHome(const std::string & path)
{
    if (!path.empty() && path[0] == '~') {
        const char * home = std::getenv("HOME");
        if (home != nullptr) {
            return std::string(home) + path.substr(1);
        }
    }
    return path;
}

}  // namespace

template <typename nodeT>
Configurator<nodeT>::Configurator(
    nodeT *node,
    const std::string & node_name,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : node_(node), after_parameter_change_callback_(after_parameter_change_callback)
{
    (void)node_name;

    schema_file_path_ = resolveSchemaFilePath();
    try {
        schema_validator_ = SchemaValidator::FromFile(schema_file_path_);
    } catch (const std::exception & ex) {
        throw std::runtime_error(
            "Configurator failed to load parameter schema '" + schema_file_path_ + "': " + ex.what()
        );
    }

    parameter_events_subscriber_ = node_->template create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        10,
        std::bind(&Configurator<nodeT>::parameterEventCallback, this, std::placeholders::_1)
    );

    managed_node_announcement_publisher_ = node_->template create_publisher<std_msgs::msg::String>(
        "/configuration/configuration_server/managed_node_available",
        10
    );

    on_set_parameters_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&Configurator<nodeT>::onSetParametersCallback, this, std::placeholders::_1)
    );

    if (!managed_parameter_names_.empty() && !managed_node_announced_) {
        announceManagedNode();
    }
}

template <typename nodeT>
Configurator<nodeT>::~Configurator()
{
    if (parameter_events_subscriber_) {
        parameter_events_subscriber_->clear_on_new_message_callback();
        parameter_events_subscriber_.reset();
    }

    if (on_set_parameters_callback_handle_) {
        node_->remove_on_set_parameters_callback(on_set_parameters_callback_handle_.get());
        on_set_parameters_callback_handle_.reset();
    }

    managed_parameter_names_.clear();
    configurations_.clear();
}

template <typename nodeT>
rclcpp::Parameter Configurator<nodeT>::GetParameter(const std::string & parameter_full_name) const
{
    return GetParameters({parameter_full_name}).front();
}

template <typename nodeT>
std::vector<rclcpp::Parameter> Configurator<nodeT>::GetParameters(const std::vector<std::string> & parameter_full_names) const
{
    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);

    std::vector<rclcpp::Parameter> parameters;
    parameters.reserve(parameter_full_names.size());

    for (const auto & full_name : parameter_full_names) {
        rclcpp::Parameter parameter;
        if (!node_->get_parameter(full_name, parameter)) {
            throw std::runtime_error("Configurator::GetParameters(): Missing declared parameter " + full_name);
        }
        parameters.push_back(parameter);
    }

    return parameters;
}

template <typename nodeT>
Configuration::SharedPtr Configurator<nodeT>::GetConfiguration(const std::string & name) const
{
    for (const auto & configuration : configurations_) {
        if (configuration->name() == name) {
            return configuration;
        }
    }

    throw std::runtime_error("Configurator::GetConfiguration(): Configuration " + name + " does not exist.");
}

template <typename nodeT>
void Configurator<nodeT>::DeclareParameter(
    const std::string & parameter_full_name,
    rclcpp::ParameterType parameter_type
)
{
    DeclareParameters({parameter_full_name}, {parameter_type});
}

template <typename nodeT>
void Configurator<nodeT>::DeclareParameters(
    const std::vector<std::string> & parameter_full_names,
    const std::vector<rclcpp::ParameterType> & parameter_types
)
{
    declareParameters(parameter_full_names, parameter_types);

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    for (const auto & full_name : parameter_full_names) {
        if (std::find(managed_parameter_names_.begin(), managed_parameter_names_.end(), full_name) == managed_parameter_names_.end()) {
            managed_parameter_names_.push_back(full_name);
        }
    }
}

template <typename nodeT>
Configuration::SharedPtr Configurator<nodeT>::CreateConfiguration(
    const std::string & name,
    const std::vector<configuration_entry_t> & entries
)
{
    auto configuration = std::make_shared<Configuration>(
        name,
        entries,
        [this](const std::string & full_name) {
            rclcpp::Parameter parameter;
            if (!node_->get_parameter(full_name, parameter)) {
                throw std::runtime_error("Configuration getter could not read parameter " + full_name);
            }
            return parameter;
        }
    );

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    configurations_.push_back(configuration);
    return configuration;
}

template <typename nodeT>
void Configurator<nodeT>::SyncParameters(const std::vector<std::string> & parameter_full_names)
{
    if (parameter_full_names.empty()) {
        validate();
        return;
    }

    auto candidate_values = getCurrentValuesWithSchemaDefaults();
    for (const auto & full_name : parameter_full_names) {
        schema_validator_.ValidateParameterValue(full_name, candidate_values.at(full_name), candidate_values, true);
    }
}

template <typename nodeT>
std::string Configurator<nodeT>::GetParameterTypeString(rclcpp::ParameterType parameter_type)
{
    return SchemaValidator::ParameterTypeToString(parameter_type);
}

template <typename nodeT>
rclcpp::ParameterType Configurator<nodeT>::GetParameterTypeFromString(const std::string & parameter_type)
{
    return SchemaValidator::ParameterTypeFromString(parameter_type);
}

template <typename nodeT>
void Configurator<nodeT>::PrintParameters() const
{
    for (const auto & parameter_name : managed_parameter_names_) {
        rclcpp::Parameter parameter;
        if (node_->get_parameter(parameter_name, parameter)) {
            RCLCPP_INFO(node_->get_logger(), "%s: %s", parameter_name.c_str(), parameter.value_to_string().c_str());
        }
    }
}

template <typename nodeT>
void Configurator<nodeT>::PrintConfigurations() const
{
    for (const auto & configuration : configurations_) {
        RCLCPP_INFO(node_->get_logger(), "%s", configuration->name().c_str());
    }
}

template <typename nodeT>
void Configurator<nodeT>::validate() const
{
    if (managed_parameter_names_.empty()) {
        return;
    }
    schema_validator_.ValidateParameterMap(getCurrentValuesWithSchemaDefaults(), true);

    if (!on_set_parameters_callback_handle_) {
        auto * self = const_cast<Configurator<nodeT> *>(this);
        self->on_set_parameters_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&Configurator<nodeT>::onSetParametersCallback, self, std::placeholders::_1)
        );
    }

    if (!managed_node_announced_) {
        const_cast<Configurator<nodeT> *>(this)->announceManagedNode();
    }
}

template <typename nodeT>
void Configurator<nodeT>::declareParameters(
    const std::vector<std::string> & parameter_full_names,
    const std::vector<rclcpp::ParameterType> & parameter_types
)
{
    if (parameter_full_names.size() != parameter_types.size()) {
        throw std::runtime_error("Configurator::declareParameters(): Names/types size mismatch");
    }

    for (std::size_t i = 0; i < parameter_full_names.size(); ++i) {
        const auto & full_name = parameter_full_names[i];
        const auto & schema_entry = schema_validator_.GetParameter(full_name);

        if (schema_entry.parameter_type != parameter_types[i]) {
            throw std::runtime_error("Configurator::declareParameters(): Type mismatch for " + full_name);
        }

        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.name = full_name;
        descriptor.description = "Managed by iii_drone_configuration schema";
        descriptor.read_only = false;
        descriptor.dynamic_typing = false;

        if (!node_->has_parameter(full_name)) {
            node_->declare_parameter(full_name, schema_entry.default_value, descriptor, false);
        }
    }
}

template <typename nodeT>
bool Configurator<nodeT>::undeclareParameters()
{
    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    managed_parameter_names_.clear();
    return true;
}

template <typename nodeT>
void Configurator<nodeT>::parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event)
{
    if (parameter_event.node != node_->get_fully_qualified_name()) {
        return;
    }

    if (after_parameter_change_callback_ == nullptr) {
        return;
    }

    for (const auto & parameter : parameter_event.changed_parameters) {
        if (std::find(managed_parameter_names_.begin(), managed_parameter_names_.end(), parameter.name) != managed_parameter_names_.end()) {
            after_parameter_change_callback_(rclcpp::Parameter::from_parameter_msg(parameter));
        }
    }
}

template <typename nodeT>
rcl_interfaces::msg::SetParametersResult Configurator<nodeT>::onSetParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    auto candidate_values = getCurrentValuesWithSchemaDefaults();

    for (const auto & parameter : parameters) {
        if (std::find(managed_parameter_names_.begin(), managed_parameter_names_.end(), parameter.get_name()) == managed_parameter_names_.end()) {
            continue;
        }

        candidate_values[parameter.get_name()] = parameter.get_parameter_value();

        try {
            schema_validator_.ValidateParameterValue(
                parameter.get_name(),
                parameter.get_parameter_value(),
                candidate_values,
                false
            );
        } catch (const std::exception & ex) {
            result.successful = false;
            result.reason = ex.what();
            return result;
        }
    }

    try {
        schema_validator_.ValidateParameterMap(candidate_values, true);
    } catch (const std::exception & ex) {
        result.successful = false;
        result.reason = ex.what();
        return result;
    }

    return result;
}

template <typename nodeT>
std::unordered_map<std::string, rclcpp::ParameterValue> Configurator<nodeT>::getCurrentManagedParameterValues() const
{
    std::unordered_map<std::string, rclcpp::ParameterValue> values;
    for (const auto & parameter_name : managed_parameter_names_) {
        rclcpp::Parameter parameter;
        if (!node_->get_parameter(parameter_name, parameter)) {
            throw std::runtime_error("Configurator::getCurrentManagedParameterValues(): Missing parameter " + parameter_name);
        }
        values.emplace(parameter_name, parameter.get_parameter_value());
    }
    return values;
}

template <typename nodeT>
std::unordered_map<std::string, rclcpp::ParameterValue> Configurator<nodeT>::getCurrentValuesWithSchemaDefaults() const
{
    std::unordered_map<std::string, rclcpp::ParameterValue> values;
    for (const auto & [parameter_name, schema_entry] : schema_validator_.parameters()) {
        values.emplace(parameter_name, schema_entry.default_value);
    }

    for (const auto & [parameter_name, value] : getCurrentManagedParameterValues()) {
        values[parameter_name] = value;
    }

    return values;
}

template <typename nodeT>
std::string Configurator<nodeT>::resolveSchemaFilePath()
{
    if (const char * explicit_file = std::getenv("III_DRONE_SCHEMA_FILE"); explicit_file != nullptr && explicit_file[0] != '\0') {
        return ExpandHome(explicit_file);
    }

    const auto package_share = ament_index_cpp::get_package_share_directory(
        "iii_drone_configuration"
    );
    const auto installed_schema = std::filesystem::path(package_share) /
        "configuration_contract" / "schema" / "parameter_manifest.yaml";
    if (!std::filesystem::is_regular_file(installed_schema)) {
        throw std::runtime_error(
            "Installed immutable configuration schema is unavailable: " +
            installed_schema.string()
        );
    }
    return installed_schema.string();
}

template <typename nodeT>
void Configurator<nodeT>::announceManagedNode() const
{
    if (!managed_node_announcement_publisher_) {
        return;
    }

    std_msgs::msg::String message;
    message.data = node_->get_fully_qualified_name();
    managed_node_announcement_publisher_->publish(message);
    const_cast<Configurator<nodeT> *>(this)->managed_node_announced_ = true;
}

template class iii_drone::configuration::Configurator<rclcpp::Node>;
template class iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>;
