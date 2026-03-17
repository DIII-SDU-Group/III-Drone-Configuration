#pragma once

#include <iii_drone_configuration/configuration.hpp>
#include <iii_drone_configuration/schema_validator.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace iii_drone {
namespace configuration {

class PythonConfigurationCore {
public:
    PythonConfigurationCore(
        std::string name,
        std::vector<configuration_entry_t> configuration_entries
    );

    const std::string & name() const;

    bool HasParameter(const std::string & parameter_full_name) const;

private:
    std::string name_;
    std::vector<configuration_entry_t> configuration_entries_;
};

class PythonConfiguratorCore {
public:
    explicit PythonConfiguratorCore(std::string schema_file_path);
    PythonConfiguratorCore(std::string schema_source, bool from_raw_yaml_string);

    rclcpp::ParameterValue DeclareParameter(
        const std::string & parameter_full_name,
        rclcpp::ParameterType parameter_type
    );

    std::vector<rclcpp::ParameterValue> DeclareParameters(
        const std::vector<std::string> & parameter_full_names,
        const std::vector<rclcpp::ParameterType> & parameter_types
    );

    std::shared_ptr<PythonConfigurationCore> CreateConfiguration(
        const std::string & name,
        const std::vector<configuration_entry_t> & entries
    );

    std::shared_ptr<PythonConfigurationCore> GetConfiguration(const std::string & name) const;

    void ValidateParameterValue(
        const std::string & name,
        const rclcpp::ParameterValue & value,
        const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
        bool allow_constant_override
    ) const;

    void ValidateParameterMap(
        const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
        bool allow_constant_override
    ) const;

    const std::vector<std::string> & managed_parameter_names() const;

    std::vector<std::string> schema_parameter_names() const;

    static std::string GetParameterTypeString(rclcpp::ParameterType parameter_type);

    static rclcpp::ParameterType GetParameterTypeFromString(const std::string & parameter_type);

    const schema_parameter_entry_t & GetSchemaEntry(const std::string & name) const;

private:
    SchemaValidator schema_validator_;
    std::string schema_file_path_;
    std::vector<std::string> managed_parameter_names_;
    std::vector<std::shared_ptr<PythonConfigurationCore>> configurations_;
};

}  // namespace configuration
}  // namespace iii_drone
