#include <iii_drone_configuration/schema_validator.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stack>
#include <stdexcept>

namespace iii_drone {
namespace configuration {

namespace {

bool IsLeafParameterNode(const YAML::Node & node)
{
    return node.IsMap() && node["type"] && node["value"];
}

bool IsParameterPathChar(char ch)
{
    return std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '/';
}

bool IsExpressionString(const YAML::Node & node)
{
    if (!node.IsScalar()) {
        return false;
    }

    const std::string value = node.as<std::string>();
    return value.find('/') != std::string::npos ||
           value.find('+') != std::string::npos ||
           value.find('-') != std::string::npos ||
           value.find('*') != std::string::npos;
}

int OperatorPrecedence(char op)
{
    switch (op) {
        case '+':
        case '-':
            return 1;
        case '*':
        case '/':
            return 2;
        default:
            return -1;
    }
}

double ApplyOperator(double lhs, double rhs, char op)
{
    switch (op) {
        case '+':
            return lhs + rhs;
        case '-':
            return lhs - rhs;
        case '*':
            return lhs * rhs;
        case '/':
            if (rhs == 0.0) {
                throw std::runtime_error("Division by zero in schema expression");
            }
            return lhs / rhs;
        default:
            throw std::runtime_error("Unsupported operator in schema expression");
    }
}

void ReduceTopOperation(std::stack<double> & values, std::stack<char> & ops)
{
    if (values.size() < 2 || ops.empty()) {
        throw std::runtime_error("Invalid schema expression");
    }

    const double rhs = values.top();
    values.pop();
    const double lhs = values.top();
    values.pop();
    const char op = ops.top();
    ops.pop();
    values.push(ApplyOperator(lhs, rhs, op));
}

}  // namespace

SchemaValidator SchemaValidator::FromFile(const std::string & file_path)
{
    SchemaValidator validator;
    const YAML::Node root = YAML::LoadFile(file_path);
    LoadNode(root, "", validator.parameters_);
    return validator;
}

SchemaValidator SchemaValidator::FromYamlString(const std::string & raw_yaml_string)
{
    SchemaValidator validator;
    const YAML::Node root = YAML::Load(raw_yaml_string);
    LoadNode(root, "", validator.parameters_);
    return validator;
}

bool SchemaValidator::HasParameter(const std::string & name) const
{
    return parameters_.find(name) != parameters_.end();
}

const schema_parameter_entry_t & SchemaValidator::GetParameter(const std::string & name) const
{
    const auto it = parameters_.find(name);
    if (it == parameters_.end()) {
        throw std::runtime_error("Schema parameter not found: " + name);
    }
    return it->second;
}

const std::unordered_map<std::string, schema_parameter_entry_t> & SchemaValidator::parameters() const
{
    return parameters_;
}

std::vector<std::string> SchemaValidator::parameter_names() const
{
    std::vector<std::string> names;
    names.reserve(parameters_.size());
    for (const auto & entry : parameters_) {
        names.push_back(entry.first);
    }
    std::sort(names.begin(), names.end());
    return names;
}

void SchemaValidator::ValidateParameterValue(
    const std::string & name,
    const rclcpp::ParameterValue & value,
    const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
    bool allow_constant_override
) const
{
    const auto & entry = GetParameter(name);

    if (value.get_type() != entry.parameter_type) {
        throw std::runtime_error(
            "Parameter " + name + " has type " + ParameterTypeToString(value.get_type()) +
            " but schema expects " + entry.type
        );
    }

    if (entry.constant && !allow_constant_override) {
        throw std::runtime_error("Parameter " + name + " is constant and cannot be changed");
    }

    if (entry.min_value.has_value()) {
        const double current_value = ParameterValueToDouble(value);
        const YAML::Node & min_node = entry.min_value.value();
        const double min_value = IsExpressionString(min_node)
            ? EvaluateExpression(min_node.as<std::string>(), candidate_values)
            : ParameterValueToDouble(ParameterValueFromYaml(min_node, entry.type));

        if (current_value < min_value) {
            throw std::runtime_error("Parameter " + name + " is below schema minimum");
        }
    }

    if (entry.max_value.has_value()) {
        const double current_value = ParameterValueToDouble(value);
        const YAML::Node & max_node = entry.max_value.value();
        const double max_value = IsExpressionString(max_node)
            ? EvaluateExpression(max_node.as<std::string>(), candidate_values)
            : ParameterValueToDouble(ParameterValueFromYaml(max_node, entry.type));

        if (current_value > max_value) {
            throw std::runtime_error("Parameter " + name + " is above schema maximum");
        }
    }

    if (!entry.options.empty()) {
        const std::string string_value = value.get<std::string>();
        if (std::find(entry.options.begin(), entry.options.end(), string_value) == entry.options.end()) {
            throw std::runtime_error("Parameter " + name + " is not one of the schema options");
        }
    }
}

void SchemaValidator::ValidateParameterMap(
    const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values,
    bool allow_constant_override
) const
{
    for (const auto & [name, value] : candidate_values) {
        if (!HasParameter(name)) {
            throw std::runtime_error("Managed parameter missing from schema: " + name);
        }
        ValidateParameterValue(name, value, candidate_values, allow_constant_override);
    }
}

rclcpp::ParameterType SchemaValidator::ParameterTypeFromString(const std::string & parameter_type_string)
{
    if (parameter_type_string == "bool") {
        return rclcpp::ParameterType::PARAMETER_BOOL;
    }
    if (parameter_type_string == "int") {
        return rclcpp::ParameterType::PARAMETER_INTEGER;
    }
    if (parameter_type_string == "float") {
        return rclcpp::ParameterType::PARAMETER_DOUBLE;
    }
    if (parameter_type_string == "string") {
        return rclcpp::ParameterType::PARAMETER_STRING;
    }
    if (parameter_type_string == "bool_array") {
        return rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
    }
    if (parameter_type_string == "int_array") {
        return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    }
    if (parameter_type_string == "float_array") {
        return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
    }
    if (parameter_type_string == "string_array") {
        return rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
    }
    throw std::runtime_error("Unsupported schema parameter type: " + parameter_type_string);
}

std::string SchemaValidator::ParameterTypeToString(rclcpp::ParameterType parameter_type)
{
    switch (parameter_type) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return "bool";
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return "int";
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return "float";
        case rclcpp::ParameterType::PARAMETER_STRING:
            return "string";
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            return "bool_array";
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            return "int_array";
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return "float_array";
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return "string_array";
        default:
            return "unknown";
    }
}

void SchemaValidator::LoadNode(
    const YAML::Node & node,
    const std::string & current_namespace,
    std::unordered_map<std::string, schema_parameter_entry_t> & parameters_out
)
{
    if (!node.IsMap()) {
        throw std::runtime_error("Schema root must be a map");
    }

    for (const auto & item : node) {
        const std::string key = item.first.as<std::string>();
        ValidateKey(key);

        const YAML::Node child = item.second;
        const std::string next_namespace = current_namespace + "/" + key;

        if (IsLeafParameterNode(child)) {
            if (parameters_out.find(next_namespace) != parameters_out.end()) {
                throw std::runtime_error("Duplicate schema parameter: " + next_namespace);
            }
            parameters_out.emplace(next_namespace, LoadParameter(next_namespace, child));
            continue;
        }

        LoadNode(child, next_namespace, parameters_out);
    }
}

schema_parameter_entry_t SchemaValidator::LoadParameter(
    const std::string & full_name,
    const YAML::Node & parameter_node
)
{
    if (!parameter_node["type"] || !parameter_node["value"]) {
        throw std::runtime_error("Schema parameter " + full_name + " is missing type or value");
    }

    schema_parameter_entry_t entry;
    entry.name = full_name;
    entry.type = parameter_node["type"].as<std::string>();
    entry.parameter_type = ParameterTypeFromString(entry.type);
    entry.default_value = ParameterValueFromYaml(parameter_node["value"], entry.type);

    if (parameter_node["constant"]) {
        entry.constant = parameter_node["constant"].as<bool>();
    }

    if (parameter_node["min"]) {
        if (entry.type != "int" && entry.type != "float") {
            throw std::runtime_error("Schema parameter " + full_name + " uses min on a non-numeric type");
        }
        entry.min_value = parameter_node["min"];
    }

    if (parameter_node["max"]) {
        if (entry.type != "int" && entry.type != "float") {
            throw std::runtime_error("Schema parameter " + full_name + " uses max on a non-numeric type");
        }
        entry.max_value = parameter_node["max"];
    }

    if (parameter_node["options"]) {
        if (entry.type != "string") {
            throw std::runtime_error("Schema parameter " + full_name + " uses options on a non-string type");
        }
        for (const auto & option : parameter_node["options"]) {
            entry.options.push_back(option.as<std::string>());
        }
    }

    return entry;
}

void SchemaValidator::ValidateKey(const std::string & key)
{
    if (key.empty()) {
        throw std::runtime_error("Schema key must not be empty");
    }
    for (const char ch : key) {
        if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_') {
            throw std::runtime_error("Schema key must contain only alnum or underscore: " + key);
        }
    }
}

rclcpp::ParameterValue SchemaValidator::ParameterValueFromYaml(
    const YAML::Node & node,
    const std::string & parameter_type
)
{
    if (parameter_type == "bool") {
        return rclcpp::ParameterValue(node.as<bool>());
    }
    if (parameter_type == "int") {
        return rclcpp::ParameterValue(static_cast<int64_t>(node.as<int64_t>()));
    }
    if (parameter_type == "float") {
        return rclcpp::ParameterValue(node.as<double>());
    }
    if (parameter_type == "string") {
        return rclcpp::ParameterValue(node.as<std::string>());
    }
    if (parameter_type == "bool_array") {
        return rclcpp::ParameterValue(node.as<std::vector<bool>>());
    }
    if (parameter_type == "int_array") {
        return rclcpp::ParameterValue(node.as<std::vector<int64_t>>());
    }
    if (parameter_type == "float_array") {
        return rclcpp::ParameterValue(node.as<std::vector<double>>());
    }
    if (parameter_type == "string_array") {
        return rclcpp::ParameterValue(node.as<std::vector<std::string>>());
    }
    throw std::runtime_error("Unsupported schema parameter type: " + parameter_type);
}

double SchemaValidator::EvaluateExpression(
    const std::string & expression,
    const std::unordered_map<std::string, rclcpp::ParameterValue> & candidate_values
)
{
    std::stack<double> values;
    std::stack<char> operators;

    std::size_t pos = 0;
    while (pos < expression.size()) {
        if (std::isspace(static_cast<unsigned char>(expression[pos]))) {
            ++pos;
            continue;
        }

        const char ch = expression[pos];

        if (std::isdigit(static_cast<unsigned char>(ch)) || ch == '.') {
            std::size_t end = pos + 1;
            while (end < expression.size() &&
                   (std::isdigit(static_cast<unsigned char>(expression[end])) || expression[end] == '.')) {
                ++end;
            }
            values.push(std::stod(expression.substr(pos, end - pos)));
            pos = end;
            continue;
        }

        if (ch == '/' && pos + 1 < expression.size() && IsParameterPathChar(expression[pos + 1])) {
            std::size_t end = pos + 1;
            while (end < expression.size() && IsParameterPathChar(expression[end])) {
                ++end;
            }
            const std::string parameter_name = expression.substr(pos, end - pos);
            const auto it = candidate_values.find(parameter_name);
            if (it == candidate_values.end()) {
                throw std::runtime_error("Parameter reference not found in expression: " + parameter_name);
            }
            values.push(ParameterValueToDouble(it->second));
            pos = end;
            continue;
        }

        if (ch == '+' || ch == '-' || ch == '*' || ch == '/') {
            while (!operators.empty() && OperatorPrecedence(operators.top()) >= OperatorPrecedence(ch)) {
                ReduceTopOperation(values, operators);
            }
            operators.push(ch);
            ++pos;
            continue;
        }

        throw std::runtime_error("Unsupported token in schema expression: " + expression.substr(pos, 1));
    }

    while (!operators.empty()) {
        ReduceTopOperation(values, operators);
    }

    if (values.size() != 1) {
        throw std::runtime_error("Invalid schema expression: " + expression);
    }

    return values.top();
}

double SchemaValidator::ParameterValueToDouble(const rclcpp::ParameterValue & value)
{
    switch (value.get_type()) {
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return static_cast<double>(value.get<int64_t>());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return value.get<double>();
        default:
            throw std::runtime_error("Schema numeric comparison requested for non-numeric parameter");
    }
}

}  // namespace configuration
}  // namespace iii_drone
