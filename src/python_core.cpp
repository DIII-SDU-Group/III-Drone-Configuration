#include <iii_drone_configuration/python_core.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace iii_drone {
namespace configuration {

PythonConfigurationCore::PythonConfigurationCore(
    std::string name,
    std::vector<configuration_entry_t> configuration_entries
) : name_(std::move(name)),
    configuration_entries_(std::move(configuration_entries))
{
}

const std::string & PythonConfigurationCore::name() const
{
    return name_;
}

bool PythonConfigurationCore::HasParameter(const std::string & parameter_full_name) const
{
    return std::any_of(
        configuration_entries_.begin(),
        configuration_entries_.end(),
        [&parameter_full_name](const configuration_entry_t & entry) {
            return entry.full_name == parameter_full_name;
        }
    );
}

PythonConfiguratorCore::PythonConfiguratorCore(std::string schema_file_path) : schema_file_path_(std::move(schema_file_path))
{
    schema_validator_ = SchemaValidator::FromFile(schema_file_path_);
}

PythonConfiguratorCore::PythonConfiguratorCore(std::string schema_source, bool from_raw_yaml_string)
    : schema_file_path_(std::move(schema_source))
{
    schema_validator_ = from_raw_yaml_string
        ? SchemaValidator::FromYamlString(schema_file_path_)
        : SchemaValidator::FromFile(schema_file_path_);
}

rclcpp::ParameterValue PythonConfiguratorCore::DeclareParameter(
    const std::string & parameter_full_name,
    rclcpp::ParameterType parameter_type
)
{
    return DeclareParameters({parameter_full_name}, {parameter_type}).front();
}

std::vector<rclcpp::ParameterValue> PythonConfiguratorCore::DeclareParameters(
    const std::vector<std::string> & parameter_full_names,
    const std::vector<rclcpp::ParameterType> & parameter_types
)
{
    if (parameter_full_names.size() != parameter_types.size()) {
        throw std::runtime_error("PythonConfiguratorCore::DeclareParameters(): Names/types size mismatch");
    }

    std::vector<rclcpp::ParameterValue> default_values;
    default_values.reserve(parameter_full_names.size());

    for (std::size_t i = 0; i < parameter_full_names.size(); ++i) {
        const auto & full_name = parameter_full_names[i];
        const auto & schema_entry = schema_validator_.GetParameter(full_name);

        if (schema_entry.parameter_type != parameter_types[i]) {
            throw std::runtime_error("PythonConfiguratorCore::DeclareParameters(): Type mismatch for " + full_name);
        }

        if (std::find(managed_parameter_names_.begin(), managed_parameter_names_.end(), full_name) == managed_parameter_names_.end()) {
            managed_parameter_names_.push_back(full_name);
        }

        default_values.push_back(schema_entry.default_value);
    }

    return default_values;
}

std::shared_ptr<PythonConfigurationCore> PythonConfiguratorCore::CreateConfiguration(
    const std::string & name,
    const std::vector<configuration_entry_t> & entries
)
{
    auto configuration = std::make_shared<PythonConfigurationCore>(name, entries);
    configurations_.push_back(configuration);
    return configuration;
}

std::shared_ptr<PythonConfigurationCore> PythonConfiguratorCore::GetConfiguration(const std::string & name) const
{
    for (const auto & configuration : configurations_) {
        if (configuration->name() == name) {
            return configuration;
        }
    }

    throw std::runtime_error("PythonConfiguratorCore::GetConfiguration(): Configuration " + name + " does not exist.");
}

void PythonConfiguratorCore::ValidateParameterValue(
    const std::string & name,
    const rclcpp::ParameterValue & value,
    const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
    bool allow_constant_override
) const
{
    schema_validator_.ValidateParameterValue(name, value, candidate_values, allow_constant_override);
}

void PythonConfiguratorCore::ValidateParameterMap(
    const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
    bool allow_constant_override
) const
{
    schema_validator_.ValidateParameterMap(candidate_values, allow_constant_override);
}

const std::vector<std::string> & PythonConfiguratorCore::managed_parameter_names() const
{
    return managed_parameter_names_;
}

std::vector<std::string> PythonConfiguratorCore::schema_parameter_names() const
{
    return schema_validator_.parameter_names();
}

std::string PythonConfiguratorCore::GetParameterTypeString(rclcpp::ParameterType parameter_type)
{
    return SchemaValidator::ParameterTypeToString(parameter_type);
}

rclcpp::ParameterType PythonConfiguratorCore::GetParameterTypeFromString(const std::string & parameter_type)
{
    return SchemaValidator::ParameterTypeFromString(parameter_type);
}

const schema_parameter_entry_t & PythonConfiguratorCore::GetSchemaEntry(const std::string & name) const
{
    return schema_validator_.GetParameter(name);
}

}  // namespace configuration
}  // namespace iii_drone
