#pragma once

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace iii_drone {
namespace configuration {

struct schema_parameter_entry_t {
    std::string name;
    std::string type;
    rclcpp::ParameterType parameter_type;
    rclcpp::ParameterValue default_value;
    bool constant = false;
    std::optional<YAML::Node> min_value;
    std::optional<YAML::Node> max_value;
    std::vector<std::string> options;
};

class SchemaValidator {
public:
    SchemaValidator() = default;

    static SchemaValidator FromFile(const std::string & file_path);
    static SchemaValidator FromYamlString(const std::string & raw_yaml_string);

    bool HasParameter(const std::string & name) const;

    const schema_parameter_entry_t & GetParameter(const std::string & name) const;

    const std::unordered_map<std::string, schema_parameter_entry_t> & parameters() const;

    std::vector<std::string> parameter_names() const;

    void ValidateParameterValue(
        const std::string & name,
        const rclcpp::ParameterValue & value,
        const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
        bool allow_constant_override = false
    ) const;

    void ValidateParameterMap(
        const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
        bool allow_constant_override = false
    ) const;

    static rclcpp::ParameterType ParameterTypeFromString(const std::string & parameter_type_string);

    static std::string ParameterTypeToString(rclcpp::ParameterType parameter_type);

private:
    std::unordered_map<std::string, schema_parameter_entry_t> parameters_;

    static void LoadNode(
        const YAML::Node & node,
        const std::string & current_namespace,
        std::unordered_map<std::string, schema_parameter_entry_t> & parameters_out
    );

    static schema_parameter_entry_t LoadParameter(
        const std::string & full_name,
        const YAML::Node & parameter_node
    );

    static void ValidateKey(const std::string & key);

    static rclcpp::ParameterValue ParameterValueFromYaml(
        const YAML::Node & node,
        const std::string & parameter_type
    );

    static double EvaluateExpression(
        const std::string & expression,
        const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values
    );

    static double ParameterValueToDouble(const rclcpp::ParameterValue & value);
};

}  // namespace configuration
}  // namespace iii_drone
