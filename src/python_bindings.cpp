#include <iii_drone_configuration/python_core.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace py = pybind11;

namespace {

py::object ParameterValueToPythonObject(const rclcpp::ParameterValue & value)
{
    switch (value.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return py::bool_(value.get<bool>());
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return py::int_(value.get<int64_t>());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return py::float_(value.get<double>());
        case rclcpp::ParameterType::PARAMETER_STRING:
            return py::str(value.get<std::string>());
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            return py::cast(value.get<std::vector<bool>>());
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            return py::cast(value.get<std::vector<int64_t>>());
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return py::cast(value.get<std::vector<double>>());
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return py::cast(value.get<std::vector<std::string>>());
        default:
            throw std::runtime_error("Unsupported parameter type in ParameterValueToPythonObject");
    }
}

rclcpp::ParameterValue PythonObjectToParameterValue(py::handle value, rclcpp::ParameterType parameter_type)
{
    switch (parameter_type) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return rclcpp::ParameterValue(value.cast<bool>());
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return rclcpp::ParameterValue(value.cast<int64_t>());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return rclcpp::ParameterValue(value.cast<double>());
        case rclcpp::ParameterType::PARAMETER_STRING:
            return rclcpp::ParameterValue(value.cast<std::string>());
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            return rclcpp::ParameterValue(value.cast<std::vector<bool>>());
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            return rclcpp::ParameterValue(value.cast<std::vector<int64_t>>());
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return rclcpp::ParameterValue(value.cast<std::vector<double>>());
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return rclcpp::ParameterValue(value.cast<std::vector<std::string>>());
        default:
            throw std::runtime_error("Unsupported parameter type in PythonObjectToParameterValue");
    }
}

std::unordered_map<std::string, rclcpp::ParameterValue> PythonDictToParameterMap(py::dict values)
{
    std::unordered_map<std::string, rclcpp::ParameterValue> parameter_map;
    for (const auto & item : values) {
        const std::string name = py::cast<std::string>(item.first);
        const py::dict entry = py::cast<py::dict>(item.second);
        const auto type = static_cast<rclcpp::ParameterType>(py::cast<int>(entry["type"]));
        parameter_map.emplace(name, PythonObjectToParameterValue(entry["value"], type));
    }
    return parameter_map;
}

py::dict ConfigurationEntryToPythonDict(const iii_drone::configuration::schema_parameter_entry_t & entry)
{
    py::dict result;
    result["name"] = entry.name;
    result["type"] = entry.type;
    result["parameter_type"] = static_cast<int>(entry.parameter_type);
    result["default_value"] = ParameterValueToPythonObject(entry.default_value);
    result["constant"] = entry.constant;
    result["options"] = entry.options;
    return result;
}

}  // namespace

PYBIND11_MODULE(_native, m)
{
    using iii_drone::configuration::PythonConfigurationCore;
    using iii_drone::configuration::PythonConfiguratorCore;

    py::class_<PythonConfigurationCore, std::shared_ptr<PythonConfigurationCore>>(m, "NativeConfiguration")
        .def_property_readonly("name", &PythonConfigurationCore::name)
        .def("has_parameter", &PythonConfigurationCore::HasParameter);

    py::class_<PythonConfiguratorCore>(m, "NativeConfiguratorCore")
        .def(py::init<std::string>())
        .def(py::init<std::string, bool>(), py::arg("schema_source"), py::arg("from_raw_yaml_string"))
        .def("declare_parameter", [](PythonConfiguratorCore & self, const std::string & name, int parameter_type) {
            return ParameterValueToPythonObject(
                self.DeclareParameter(name, static_cast<rclcpp::ParameterType>(parameter_type))
            );
        })
        .def("declare_parameters", [](PythonConfiguratorCore & self, const std::vector<std::string> & names, const std::vector<int> & parameter_types) {
            std::vector<rclcpp::ParameterType> resolved_types;
            resolved_types.reserve(parameter_types.size());
            for (const auto type : parameter_types) {
                resolved_types.push_back(static_cast<rclcpp::ParameterType>(type));
            }

            py::list defaults;
            for (const auto & value : self.DeclareParameters(names, resolved_types)) {
                defaults.append(ParameterValueToPythonObject(value));
            }
            return defaults;
        })
        .def("create_configuration", [](PythonConfiguratorCore & self, const std::string & name, const std::vector<std::pair<std::string, int>> & entries) {
            std::vector<iii_drone::configuration::configuration_entry_t> resolved_entries;
            resolved_entries.reserve(entries.size());
            for (const auto & [full_name, parameter_type] : entries) {
                resolved_entries.emplace_back(full_name, static_cast<rclcpp::ParameterType>(parameter_type));
            }
            return self.CreateConfiguration(name, resolved_entries);
        })
        .def("get_configuration", &PythonConfiguratorCore::GetConfiguration)
        .def("validate_parameter_value", [](const PythonConfiguratorCore & self, const std::string & name, py::object value, int parameter_type, py::dict candidate_values, bool allow_constant_override) {
            self.ValidateParameterValue(
                name,
                PythonObjectToParameterValue(value, static_cast<rclcpp::ParameterType>(parameter_type)),
                PythonDictToParameterMap(candidate_values),
                allow_constant_override
            );
        })
        .def("validate_parameter_map", [](const PythonConfiguratorCore & self, py::dict candidate_values, bool allow_constant_override) {
            self.ValidateParameterMap(PythonDictToParameterMap(candidate_values), allow_constant_override);
        })
        .def("managed_parameter_names", [](const PythonConfiguratorCore & self) {
            return self.managed_parameter_names();
        })
        .def("schema_parameter_names", &PythonConfiguratorCore::schema_parameter_names)
        .def("get_schema_entry", [](const PythonConfiguratorCore & self, const std::string & name) {
            return ConfigurationEntryToPythonDict(self.GetSchemaEntry(name));
        })
        .def_static("get_parameter_type_string", [](int parameter_type) {
            return PythonConfiguratorCore::GetParameterTypeString(static_cast<rclcpp::ParameterType>(parameter_type));
        })
        .def_static("get_parameter_type_from_string", [](const std::string & parameter_type) {
            return static_cast<int>(PythonConfiguratorCore::GetParameterTypeFromString(parameter_type));
        });
}
