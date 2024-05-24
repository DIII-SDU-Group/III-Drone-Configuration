/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_configuration/configurator.hpp"

using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

#include <iostream>

Configurator::Configurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : after_parameter_change_callback_(after_parameter_change_callback) {

    RCLCPP_DEBUG(node->get_logger(), "Configurator::Configurator(): Initializing configurator");

    node_ = node;

    declare_parameter_client_ = node_->create_client<iii_drone_interfaces::srv::DeclareParameter>("/configuration/configuration_server/declare_parameter");
    get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/configuration/configuration_server/configuration_server/get_parameters");

    parameter_events_subscriber_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        10,
        std::bind(
            &Configurator::parameterEventCallback, 
            this, 
            std::placeholders::_1
        )
    );

    node_->declare_parameter<std::string>("node_parameters_dir", "~/.config/iii_drone/node_parameters/");

    std::string parameter_yaml_path = node_->get_parameter("node_parameters_dir").as_string();

    if (parameter_yaml_path[0] == '~') {

        parameter_yaml_path = std::string(getenv("HOME")) + parameter_yaml_path.substr(1);

    }
    
    if (parameter_yaml_path.back() != '/') {

        parameter_yaml_path += "/";

    }

    parameter_yaml_path += node_->get_name() + std::string(".yaml");

    initialize(parameter_yaml_path);

}

Configurator::Configurator(
    rclcpp::Node *node,
    const rclcpp::QoS &qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : after_parameter_change_callback_(after_parameter_change_callback) {

    RCLCPP_DEBUG(node->get_logger(), "Configurator::Configurator(): Initializing configurator");

    node_ = node;

    qos_ = std::make_unique<rclcpp::QoS>(qos);

    declare_parameter_client_ = node_->create_client<iii_drone_interfaces::srv::DeclareParameter>("/configuration/configuration_server/declare_parameter");
    get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/configuration/configuration_server/configuration_server/get_parameters");

    parameter_events_subscriber_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        *qos_,
        std::bind(
            &Configurator::parameterEventCallback, 
            this, 
            std::placeholders::_1
        )
    );

    node_->declare_parameter<std::string>("node_parameters_dir", "~/.config/iii_drone/node_parameters/");

    std::string parameter_yaml_path = node_->get_parameter("node_parameters_dir").as_string();

    if (parameter_yaml_path[0] == '~') {

        parameter_yaml_path = std::string(getenv("HOME")) + parameter_yaml_path.substr(1);

    }
    
    if (parameter_yaml_path.back() != '/') {

        parameter_yaml_path += "/";

    }

    parameter_yaml_path += node_->get_name() + std::string(".yaml");

    initialize(parameter_yaml_path);

}

Configurator::~Configurator() { }

rclcpp::Parameter Configurator::GetParameter(const std::string & simple_name) const {

    std::vector<rclcpp::Parameter> parameters = GetParameters({simple_name});

    return parameters[0];

}

std::vector<rclcpp::Parameter> Configurator::GetParameters(const std::vector<std::string> & simple_names) const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);

    std::vector<rclcpp::Parameter> parameters;  

    // Search for the parameter:
    for (auto & name : simple_names) {

        std::string full_name = getParameterFullName(name);

        // Check if the parameter is in the list of parameters:
        for (auto & p : parameters_) {

            if (p.get_name() == full_name) {

                parameters.push_back(p);

                break;

            }
        }
    }

    if (parameters.size() != simple_names.size()) {

        std::string fatal_message = "Configurator::GetParameters(): Some parameters could not be found.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    return parameters;

}

ParameterBundle::SharedPtr Configurator::GetParameterBundle(const std::string & name) const {

    for (auto & parameter_bundle : parameter_bundles_) {

        if (parameter_bundle->name() == name) {

            return parameter_bundle;

        }

    }

    std::string fatal_message = "Configurator::GetParameterBundle(): Parameter bundle " + name + " does not exist.";

    RCLCPP_FATAL(
        node_->get_logger(),
        fatal_message.c_str()
    );

    throw std::runtime_error(fatal_message);

}

void Configurator::SyncParameters(const std::vector<std::string> & simple_names) {

    std::vector<std::string> names_to_sync;

    if (simple_names.size() == 0) {

        // Get all parameters:
        for (auto & p : parameters_) {

            names_to_sync.push_back(p.get_name());

        }

    } else {

        std::vector<std::string> full_names;

        for (auto & name : simple_names) {

            full_names.push_back(getParameterFullName(name));

        }

        names_to_sync = full_names;

    }

    std::vector<rclcpp::Parameter> parameters;

    if (!sendGetParametersRequest(
            names_to_sync,
            parameters
        )
    ) {

        std::string fatal_message = "Configurator::SyncParameters(): Failed to get parameters.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Check if the parameters are correct:
    for (unsigned int i = 0; i < parameters.size(); i++) {

        if (parameters[i].get_type() != parameters_[i].get_type() || parameters[i].get_name() != parameters_[i].get_name()) {

            std::string fatal_message = "Configurator::SyncParameters(): Parameter " + parameters[i].get_name() + " has type " + std::to_string(parameters[i].get_type()) + " but expected type " + std::to_string(parameters_[i].get_type()) + ".";

            RCLCPP_FATAL(
                node_->get_logger(),
                fatal_message.c_str()
            );

            throw std::runtime_error(fatal_message);

        }

    }

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);

    // Update parameter bundles:
    for (unsigned int i = 0; i < parameters.size(); i++) {

        std::string simple_name = getParameterSimpleName(parameters[i].get_name());

        for (auto & parameter_bundle : parameter_bundles_) {

            if (parameter_bundle->HasUpdatableParameter(simple_name)) {

                parameter_bundle->SetParameter(
                    simple_name,
                    parameters[i]
                );

            }

        }

    }

    // Update parameters:
    parameters_ = parameters;

}

template <typename T>
std::string Configurator::GetParameterTypeString() {

    if (std::is_same<T, bool>::value) {

        return "bool";

    } else if (std::is_same<T, int>::value) {

        return "int";

    } else if (std::is_same<T, float>::value) {

        return "float";

    } else if (std::is_same<T, double>::value) {

        return "float";

    } else if (std::is_same<T, std::string>::value) {

        return "string";

    } else if (std::is_same<T, std::vector<bool>>::value) {

        return "bool_array";

    } else if (std::is_same<T, std::vector<int>>::value) {

        return "int_array";

    } else if (std::is_same<T, std::vector<float>>::value) {

        return "float_array";

    } else if (std::is_same<T, std::vector<double>>::value) {

        return "float_array";

    } else if (std::is_same<T, std::vector<std::string>>::value) {

        return "string_array";

    } else {

        std::string fatal_message = "Configurator::GetParameterTypeString(): Parameter type not supported.";

        throw std::runtime_error(fatal_message);

    }

}

void Configurator::PrintParameters() const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);

    RCLCPP_INFO(
        node_->get_logger(),
        "Configurator::PrintParameters(): Printing parameters:"
    );

    for (auto & p : parameters_) {

        RCLCPP_INFO(
            node_->get_logger(),
            "Configurator::PrintParameters(): %s: %s",
            p.get_name().c_str(),
            p.value_to_string().c_str()
        );

    }

}

void Configurator::PrintParameterBundles() const {

    RCLCPP_INFO(
        node_->get_logger(),
        "Configurator::PrintParameterBundles(): Printing parameter bundles:"
    );

    for (auto & parameter_bundle : parameter_bundles_) {

        RCLCPP_INFO(
            node_->get_logger(),
            "Configurator::PrintParameterBundles(): %s",
            parameter_bundle->name().c_str()
        );

    }

}

void Configurator::initialize(const std::string & parameter_yaml_path) {

    RCLCPP_DEBUG(node_->get_logger(), "Configurator::initialize(): Declaring parameters from %s", parameter_yaml_path.c_str());

    // Load parameters from YAML file:
    YAML::Node config = YAML::LoadFile(parameter_yaml_path);

    // Initialize parameters:
    YAML::Node parameters = config["parameters"];

    initializeParameters(parameters);

    // Initialize parameter bundles:
    YAML::Node parameter_bundles = config["parameter_bundles"];

    if (parameter_bundles.Type() != YAML::NodeType::Null) {

        initializeParameterBundles(parameter_bundles);

    }

}

void Configurator::initializeParameters(const YAML::Node & parameters) {

    for (YAML::const_iterator it = parameters.begin(); it != parameters.end(); ++it) {

        std::string simple_name = it->first.as<std::string>();

        // Check if simple name already in name map:
        if (parameter_name_map_.find(simple_name) != parameter_name_map_.end()) {

            RCLCPP_WARN(
                node_->get_logger(),
                "Configurator::initializeParameters(): Parameter %s already declared, skipping.",
                simple_name.c_str()
            );

            continue;

        }

        YAML::Node parameter = it->second;

        std::string name = parameter["name"].as<std::string>();
        std::string type = parameter["type"].as<std::string>();

        if (type == "bool") {

            declareParameter<bool>(name);

        } else if (type == "int") {

            declareParameter<int>(name);

        } else if (type == "float") {

            declareParameter<float>(name);

        } else if (type == "string") {

            declareParameter<std::string>(name);

        } else if (type == "bool_array") {

            declareParameter<std::vector<bool>>(name);

        } else if (type == "int_array") {

            declareParameter<std::vector<int>>(name);

        } else if (type == "float_array") {

            declareParameter<std::vector<float>>(name);

        } else if (type == "string_array") {

            declareParameter<std::vector<std::string>>(name);

        } else {

            std::string fatal_message = "Configurator::declareParameters(): Parameter type " + type + " not supported.";

            RCLCPP_FATAL(
                node_->get_logger(),
                fatal_message.c_str()
            );

            throw std::runtime_error(fatal_message);

        }

        std::pair<std::string, std::string> parameter_name_map_entry(simple_name, name);

        parameter_name_map_.insert(parameter_name_map_entry);

    }
}

void Configurator::initializeParameterBundles(const YAML::Node & parameter_bundles) {

    for (YAML::const_iterator it = parameter_bundles.begin(); it != parameter_bundles.end(); ++it) {

        std::string name = it->first.as<std::string>();

        YAML::Node parameter_bundle = it->second;

        std::vector<parameter_bundle_entry_t> parameter_bundle_entries;

        for (YAML::const_iterator it = parameter_bundle.begin(); it != parameter_bundle.end(); ++it) {

            std::string parameter_name = it->first.as<std::string>();

            YAML::Node parameter = it->second;

            std::string remap_name = parameter["remap_name"].as<std::string>();
            bool updatable = parameter["update"].as<bool>();

            parameter_bundle_entries.push_back(
                parameter_bundle_entry_t(
                    GetParameter(parameter_name),
                    parameter_name,
                    remap_name,
                    updatable
                )
            );

        }

        parameter_bundles_.push_back(
            std::make_shared<ParameterBundle>(
                name,
                parameter_bundle_entries
            )
        );

    }

}

template <typename T>
void Configurator::declareParameter(const std::string & parameter_full_name) {

    RCLCPP_DEBUG(node_->get_logger(), "Configurator::DeclareParameter(): Declaring parameter %s", parameter_full_name.c_str());

    {

        std::shared_lock<std::shared_mutex> shared_lock(parameters_mutex_);

        // Check if parameter is already declared:
        for (auto & p : parameters_) {

            if (p.get_name() == parameter_full_name) {

                RCLCPP_WARN(
                    node_->get_logger(),
                    "Configurator::DeclareParameter(): Parameter %s already declared.",
                    parameter_full_name.c_str()
                );

                return;

            }
        }
    }

    // Send DeclareParameter request:
    std::string message;

    if (!sendDeclareParameterRequest(
            parameter_full_name,
            GetParameterTypeString<T>(),
            message
        )
    ) {

        std::string fatal_message = "Configurator::DeclareParameter(): Failed to declare parameter " + parameter_full_name + " with error message " + message + ".";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Get parameter value:
    rclcpp::Parameter parameter;

    if (!sendGetParameterRequest(
            parameter_full_name,
            parameter
        )
    ) {

        std::string fatal_message = "Configurator::DeclareParameter(): Failed to get parameter " + parameter_full_name + ".";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }



    // Add parameter to the list of parameters:
    std::unique_lock<std::shared_mutex> unique_lock(parameters_mutex_);
    parameters_.push_back(parameter);


    // lock.unlock();

}

std::string Configurator::getParameterFullName(const std::string & simple_name) const {

    auto parameter_name_map_it = parameter_name_map_.find(simple_name);

    if (parameter_name_map_it == parameter_name_map_.end()) {

        std::string fatal_message = "Configurator::getParameterFullName(): Parameter " + simple_name + " not found.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    std::string parameter_full_name = parameter_name_map_it->second;

    return parameter_full_name;

}

std::string Configurator::getParameterSimpleName(const std::string & full_name) const {

    for (auto & parameter_name_map_entry : parameter_name_map_) {

        if (parameter_name_map_entry.second == full_name) {

            return parameter_name_map_entry.first;

        }

    }

    std::string fatal_message = "Configurator::getParameterSimpleName(): Parameter with full name " + full_name + " not found.";

    RCLCPP_FATAL(
        node_->get_logger(),
        fatal_message.c_str()
    );

    throw std::runtime_error(fatal_message);

}


void Configurator::parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event) {

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);

    // Search for the parameter:
    for (auto & p : parameter_event.changed_parameters) {

        // Check if the parameter is in the list of parameters:
        for (unsigned int i = 0; i < parameters_.size(); i++) {

            rclcpp::Parameter & parameter = parameters_[i];

            if (parameter.get_name() == p.name) {

                if (p.value.type != parameter.get_type()) {

                    std::string fatal_message = "Configurator::parameterEventCallback(): Parameter " + p.name + " has type " + std::to_string(p.value.type) + " but expected type " + std::to_string(parameter.get_type()) + ".";

                    RCLCPP_FATAL(
                        node_->get_logger(),
                        fatal_message.c_str()
                    );

                    throw std::runtime_error(fatal_message);

                }

                parameters_[i] = rclcpp::Parameter::from_parameter_msg(p);

                std::string simple_name = getParameterSimpleName(p.name);

                // Search for the parameter bundle:
                for (auto & parameter_bundle : parameter_bundles_) {

                    if (parameter_bundle->HasUpdatableParameter(simple_name)) {

                        parameter_bundle->SetParameter(
                            simple_name,
                            parameters_[i]
                        );

                    }

                }

                if (after_parameter_change_callback_ != nullptr) {

                    after_parameter_change_callback_(parameters_[i]);

                }

                break;

            }
        }
    }
}

bool Configurator::sendDeclareParameterRequest(
    const std::string & parameter_full_name,
    const std::string & type,
    std::string & message
) {

    // Call DeclareParameter service:
    auto request = std::make_shared<iii_drone_interfaces::srv::DeclareParameter::Request>();

    request->name = parameter_full_name;
    request->type = type;

    // Wait for service:
    if (!declare_parameter_client_->wait_for_service(std::chrono::seconds(5))) {

        std::string fatal_message = "Configurator::DeclareParameter(): Service DeclareParameter not available after 5 seconds.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    if (!declare_parameter_client_->service_is_ready()) {

        std::string fatal_message = "Configurator::DeclareParameter(): Service DeclareParameter not available.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Call service:
    auto future = declare_parameter_client_->async_send_request(request);

    if(rclcpp::spin_until_future_complete(
        node_->get_node_base_interface(), 
        future
    ) != rclcpp::FutureReturnCode::SUCCESS) {

        std::cout << "6.7" << std::endl;

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendDeclareParameterRequest(): Service DeclareParameter timed out."
        );

        throw std::runtime_error("Failed to call service DeclareParameter.");

    }

    if (!future.valid()) {

        std::cout << "6.8" << std::endl;

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendDeclareParameterRequest(): Service DeclareParameter failed."
        );

        throw std::runtime_error("Failed to call service DeclareParameter.");

    }

    auto result = future.get();

    message = result->message;

    return result->succeeded;

}

bool Configurator::sendGetParameterRequest(
    const std::string & parameter_full_name,
    rclcpp::Parameter & parameter
) {

    std::vector<std::string> names;
    names.push_back(parameter_full_name);

    std::vector<rclcpp::Parameter> parameters;

    if (!sendGetParametersRequest(
            names,
            parameters
        )
    ) {

        return false;

    }

    parameter = parameters[0];

    return true;

}

bool Configurator::sendGetParametersRequest(
    const std::vector<std::string> & parameter_full_names,
    std::vector<rclcpp::Parameter> & parameters
) {

    // Call GetParameters service:
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    request->names = parameter_full_names;

    // Wait for service:
    if (!get_parameters_client_->wait_for_service(std::chrono::seconds(5))) {

        std::string fatal_message = "Configurator::GetParameter(): Service GetParameter not available after 5 seconds.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Call service:
    auto future = get_parameters_client_->async_send_request(request);

    // Wait for result:
    if (rclcpp::spin_until_future_complete(
            node_->get_node_base_interface(), 
            future
        ) != rclcpp::FutureReturnCode::SUCCESS
    ) {

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendGetParametersRequest(): Service GetParameters timed out."
        );

        throw std::runtime_error("Failed to call service GetParameters.");

    }

    if (!future.valid()) {

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendGetParametersRequest(): Service GetParameters failed."
        );

        throw std::runtime_error("Failed to call service GetParameters.");

    }

    auto result = future.get();

    // Check if the service call was successful:
    if (result->values.size() != parameter_full_names.size()) {

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendGetParametersRequest(): Service GetParameters failed, some parameters could not be found."
        );

        throw std::runtime_error("Failed to call service GetParameters failed, some parameters could not be found.");

    }

    parameters.resize(parameter_full_names.size());

    for (unsigned int i = 0; i < parameter_full_names.size(); i++) {

        parameters[i] = rclcpp::Parameter(
            parameter_full_names[i], 
            result->values[i]
        );

    }

    return true;

}

// Declare template specializations:
template void Configurator::declareParameter<bool>(const std::string & name);
template void Configurator::declareParameter<int>(const std::string & name);
template void Configurator::declareParameter<double>(const std::string & name);
template void Configurator::declareParameter<float>(const std::string & name);
template void Configurator::declareParameter<std::string>(const std::string & name);
template void Configurator::declareParameter<std::vector<bool>>(const std::string & name);
template void Configurator::declareParameter<std::vector<int>>(const std::string & name);
template void Configurator::declareParameter<std::vector<double>>(const std::string & name);
template void Configurator::declareParameter<std::vector<float>>(const std::string & name);
template void Configurator::declareParameter<std::vector<std::string>>(const std::string & name);

// Same for GetParameterTypeString:
template std::string Configurator::GetParameterTypeString<bool>();
template std::string Configurator::GetParameterTypeString<int>();
template std::string Configurator::GetParameterTypeString<double>();
template std::string Configurator::GetParameterTypeString<float>();
template std::string Configurator::GetParameterTypeString<std::string>();
template std::string Configurator::GetParameterTypeString<std::vector<bool>>();
template std::string Configurator::GetParameterTypeString<std::vector<int>>();
template std::string Configurator::GetParameterTypeString<std::vector<double>>();
template std::string Configurator::GetParameterTypeString<std::vector<float>>();
template std::string Configurator::GetParameterTypeString<std::vector<std::string>>();