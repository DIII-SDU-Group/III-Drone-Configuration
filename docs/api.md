# API Reference

## C++ API

### `Configurator<nodeT>`

Header:
- `include/iii_drone_configuration/configurator.hpp`

Key methods:

```cpp
Configurator(nodeT * node, const std::string & node_name, callback = nullptr);
```

Creates a schema-backed local configuration helper for a ROS 2 node.

```cpp
void DeclareParameter(const std::string & parameter_full_name, rclcpp::ParameterType parameter_type);
void DeclareParameters(const std::vector<std::string> & names, const std::vector<rclcpp::ParameterType> & types);
```

Declares managed parameters on the node. The schema:
- must contain the parameter
- must match the declared type
- supplies the declaration default value

```cpp
void validate() const;
```

Validates all managed parameters currently declared on the node against the schema.

```cpp
rclcpp::Parameter GetParameter(const std::string & parameter_full_name) const;
std::vector<rclcpp::Parameter> GetParameters(const std::vector<std::string> & parameter_full_names) const;
```

Reads managed parameters directly from the node.

```cpp
Configuration::SharedPtr CreateConfiguration(
    const std::string & name,
    const std::vector<configuration_entry_t> & entries
);

Configuration::SharedPtr GetConfiguration(const std::string & name) const;
```

Creates and retrieves named `Configuration` views.

```cpp
void SyncParameters(const std::vector<std::string> & parameter_full_names = {});
```

Revalidates a subset or all managed parameters.

### `Configuration`

Header:
- `include/iii_drone_configuration/configuration.hpp`

Key methods:

```cpp
rclcpp::Parameter GetParameter(const std::string & parameter_full_name) const;
bool HasParameter(const std::string & parameter_full_name) const;
std::string name() const;
```

`Configuration` is read-only and live. It does not copy parameter values.

### `SchemaValidator`

Header:
- `include/iii_drone_configuration/schema_validator.hpp`

Primary responsibilities:
- load schema files
- flatten hierarchical YAML into canonical full names
- validate types, constants, min/max, and option constraints
- evaluate expression-based numeric limits

Important entry points:

```cpp
static SchemaValidator FromFile(const std::string & file_path);
void ValidateParameterValue(...);
void ValidateParameterMap(...);
```

## Python API

### `Configurator`

File:
- `iii_drone_configuration/configurator.py`

Key methods:

```python
Configurator(node, after_parameter_change_callback=None, qos=QoSProfile(depth=10))
declare_parameter(parameter_full_name, parameter_type)
declare_parameters(parameter_full_names, parameter_types)
validate()
get_parameter(parameter_full_name)
get_parameters(parameter_full_names)
create_configuration(configuration_name, entries)
get_configuration(configuration_name)
cleanup()
```

The Python class mirrors the C++ behavior closely.

### `Configuration`

File:
- `iii_drone_configuration/configuration.py`

Key methods:

```python
get_parameter(parameter_full_name)
has_parameter(parameter_full_name)
name
```

### `ParameterHandler`

File:
- `iii_drone_configuration/parameter_handler.py`

This is the Python schema and parameter file utility used by the Python `Configurator` and server-side flows.

Important capabilities:
- load schema-style YAML
- flatten nested keys to canonical full names
- validate values
- update/add/remove parameters
- save parameter YAML back to disk

## Server API

File:
- `iii_drone_configuration/configuration_server_node.py`

The server uses standard ROS parameter services for managed node interaction:
- `list_parameters`
- `get_parameters`
- `set_parameters`

It also exposes custom III-Drone services for operator tooling:
- `get_parameter_yaml`
- `get_declared_parameters`
- `save_parameters`
- `load_parameters`
- `get_parameter_files`
- `get_current_parameter_file`
- `set_current_parameter_file_as_default`
- `set_parameter_from_gc`

The companion terminal client lives in:
- `iii_drone_configuration/configuration_client_node.py`
