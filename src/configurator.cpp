/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_configuration/configurator.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>

using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

#include <iostream>

template <typename nodeT>
Configurator<nodeT>::Configurator(
    nodeT *node,
    const std::string & node_name,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : after_parameter_change_callback_(after_parameter_change_callback) {

    RCLCPP_DEBUG(node->get_logger(), "Configurator::Configurator(): Initializing configurator");

    node_ = node;

    std::string namespace_ = node_->get_namespace();

    std::string _node_name = node_->get_name();

    std::string config_node_name = _node_name + "_configurator";

    configurator_node_ = std::make_shared<rclcpp::Node>(
        config_node_name,
        namespace_
    );

    declare_parameters_client_ = configurator_node_->create_client<iii_drone_interfaces::srv::DeclareParameters>(
        "/configuration/configuration_server/declare_parameters",
        rmw_qos_profile_services_default
    );
    undeclare_parameters_client_ = configurator_node_->create_client<iii_drone_interfaces::srv::UndeclareParameters>(
        "/configuration/configuration_server/undeclare_parameters",
        rmw_qos_profile_services_default
    );
    get_parameters_client_ = configurator_node_->create_client<rcl_interfaces::srv::GetParameters>(
        "/configuration/configuration_server/configuration_server/get_parameters",
        rmw_qos_profile_services_default
    );

    parameter_events_subscriber_ = node_->template create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        10,
        std::bind(
            &Configurator<nodeT>::parameterEventCallback, 
            this, 
            std::placeholders::_1
        )
    );

    if (!node_->has_parameter("node_parameters_path_postfix"))
        node_->template declare_parameter<std::string>("node_parameters_path_postfix", "node_parameters/");

    std::string config_base_dir = std::string(getenv("CONFIG_BASE_DIR"));

    if (config_base_dir.empty()) {
        config_base_dir = std::string(getenv("HOME")) + "/.config";
    }

    if (config_base_dir.back() != '/') {

        config_base_dir += "/";

    }

    std::string parameter_yaml_path = config_base_dir + "iii_drone/" + node_->get_parameter("node_parameters_path_postfix").as_string();

    if (parameter_yaml_path[0] == '~') {

        parameter_yaml_path = std::string(getenv("HOME")) + parameter_yaml_path.substr(1);

    }
    
    if (parameter_yaml_path.back() != '/') {

        parameter_yaml_path += "/";

    }

    parameter_yaml_path += node_->get_name() + std::string(".yaml");

    initialize(parameter_yaml_path);

}

template <typename nodeT>
Configurator<nodeT>::~Configurator() { 

    if (rclcpp::ok()) RCLCPP_DEBUG(node_->get_logger(), "Configurator::~Configurator(): Destructing configurator");

    parameter_events_subscriber_->clear_on_new_message_callback();
    parameter_events_subscriber_.reset();

    std::vector<std::string> bundles_still_in_use;

    for (auto & parameter_bundle : parameter_bundles_) {

        bool bundle_is_still_in_use = false;

        if (parameter_bundle.use_count() > 1) {

            bundles_still_in_use.push_back(parameter_bundle->name());

            bundle_is_still_in_use = true;

        }

        if (!bundle_is_still_in_use)
            parameter_bundle.reset();

    }
    
    if (bundles_still_in_use.size() > 0) {

        std::string fatal_message = "Configurator::~Configurator(): Parameter bundles still in use: ";

        for (auto & bundle_name : bundles_still_in_use) {

            fatal_message += bundle_name + ", ";

        }

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    parameter_bundles_.clear();

    if (rclcpp::ok()) {

        // std::vector<std::string> parameter_names;

        // {

        //     std::shared_lock<std::shared_mutex> lock(parameters_mutex_);

        //     for (auto & parameter : parameters_) {

        //         std::string parameter_name = parameter.get_name();

        //         parameter_names.push_back(parameter_name);

        //     }

        // }

        // bool skip_undeclare = false;

        // for (auto & parameter_name : parameter_names) {

        //     skip_undeclare = !undeclareParameter(parameter_name, skip_undeclare);

        // }

        undeclareParameters();

    }

    get_parameters_client_.reset();

    undeclare_parameters_client_.reset();

    declare_parameters_client_.reset();

    configurator_node_.reset();

    if (rclcpp::ok()) RCLCPP_DEBUG(node_->get_logger(), "Configurator::~Configurator(): Configurator destructed");

}

template <typename nodeT>
rclcpp::Parameter Configurator<nodeT>::GetParameter(const std::string & simple_name) const {

    std::vector<rclcpp::Parameter> parameters = GetParameters({simple_name});

    return parameters[0];

}

template <typename nodeT>
std::vector<rclcpp::Parameter> Configurator<nodeT>::GetParameters(const std::vector<std::string> & simple_names) const {

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

template <typename nodeT>
ParameterBundle::SharedPtr Configurator<nodeT>::GetParameterBundle(const std::string & name) const {

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

template <typename nodeT>
void Configurator<nodeT>::SyncParameters(const std::vector<std::string> & simple_names) {

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

template <typename nodeT>
std::string Configurator<nodeT>::GetParameterTypeString(rclcpp::ParameterType parameter_type) {

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
            std::string fatal_message = "Configurator::GetParameterTypeString(): Parameter type not supported.";
            throw std::runtime_error(fatal_message);
    }
}

template <typename nodeT>
rclcpp::ParameterType Configurator<nodeT>::GetParameterTypeFromString(const std::string & parameter_type_string) {

    if (parameter_type_string == "bool") {
        return rclcpp::ParameterType::PARAMETER_BOOL;
    } else if (parameter_type_string == "int") {
        return rclcpp::ParameterType::PARAMETER_INTEGER;
    } else if (parameter_type_string == "float") {
        return rclcpp::ParameterType::PARAMETER_DOUBLE;
    } else if (parameter_type_string == "string") {
        return rclcpp::ParameterType::PARAMETER_STRING;
    } else if (parameter_type_string == "bool_array") {
        return rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
    } else if (parameter_type_string == "int_array") {
        return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    } else if (parameter_type_string == "float_array") {
        return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
    } else if (parameter_type_string == "string_array") {
        return rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
    } else {
        std::string fatal_message = "Configurator::GetParameterTypeFromString(): Parameter type " + parameter_type_string + " not supported.";
        throw std::runtime_error(fatal_message);
    }

}

template <typename nodeT>
void Configurator<nodeT>::PrintParameters() const {

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

template <typename nodeT>
void Configurator<nodeT>::PrintParameterBundles() const {

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

template <typename nodeT>
void Configurator<nodeT>::initialize(const std::string & parameter_yaml_path) {

    if (initialized_) {

        std::string fatal_message = "Configurator::initialize(): Already initialized.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

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

    initialized_ = true;

}

template <typename nodeT>
void Configurator<nodeT>::initializeParameters(const YAML::Node & parameters) {

    auto parameter_name_map_temp = std::map<std::string, std::string>();

    std::vector<std::string> parameter_full_names;
    std::vector<rclcpp::ParameterType> parameter_types;

    for (YAML::const_iterator it = parameters.begin(); it != parameters.end(); ++it) {

        std::string simple_name = it->first.as<std::string>();

        YAML::Node parameter = it->second;

        std::string name = parameter["name"].as<std::string>();
        std::string type = parameter["type"].as<std::string>();

        rclcpp::ParameterType parameter_type = GetParameterTypeFromString(type);

        parameter_full_names.push_back(name);
        parameter_types.push_back(parameter_type);

        std::pair<std::string, std::string> parameter_name_map_entry(simple_name, name);

        parameter_name_map_temp.insert(parameter_name_map_entry);

    }

    declareParameters(
        parameter_full_names,
        parameter_types
    );

    parameter_name_map_ = parameter_name_map_temp;

}

template <typename nodeT>
void Configurator<nodeT>::initializeParameterBundles(const YAML::Node & parameter_bundles) {

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

template <typename nodeT>
void Configurator<nodeT>::declareParameters(
    const std::vector<std::string> & parameter_full_names,
    const std::vector<rclcpp::ParameterType> & parameter_types
) {

    RCLCPP_DEBUG(node_->get_logger(), "Configurator::DeclareParameter(): Declaring parameters");

    // Send DeclareParameter request:
    std::string message;

    std::vector<std::string> types;

    for (auto & parameter_type : parameter_types) {

        types.push_back(GetParameterTypeString(parameter_type));

    }

    std::vector<rclcpp::Parameter> parameters;

    if (!sendDeclareParametersRequest(
            parameter_full_names,
            types,
            parameters,
            message
        )
    ) {

        std::string fatal_message = "Configurator::DeclareParameter(): Failed to declare parameters with error message: " + message;

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Add parameters to the list of parameters:
    std::unique_lock<std::shared_mutex> unique_lock(parameters_mutex_);
    
    for (auto & parameter : parameters) {

        parameters_.push_back(parameter);

    }

}

template <typename nodeT>
bool Configurator<nodeT>::undeclareParameters(bool skip_server_undeclare) {

    if (parameter_bundles_.size() > 0) {

        std::string fatal_message = "Configurator::UndeclareParameters(): Cannot undeclare parameters while parameter bundles are still active.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    RCLCPP_DEBUG(node_->get_logger(), "Configurator::UndeclareParameters(): Undeclaring parameters");

    // Send UndeclareParameters request:
    std::string message;

    bool undeclare_success = true;

    if (!skip_server_undeclare) {

        if (!sendUndeclareParametersRequest(message)) {

            std::string warn_message = "Configurator::UndeclareParameters(): Failed to undeclare parameters with error message " + message;

            RCLCPP_WARN(
                node_->get_logger(),
                warn_message.c_str()
            );

            undeclare_success = false;

        }

    } else {

        undeclare_success = false;

    }

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);

    // Clear list of parameters:
    // for (unsigned int i = 0; i < parameters_.size(); i++) {

    //     if (parameters_[i].get_name() == parameter_full_name) {

    //         RCLCPP_DEBUG(node_->get_logger(), "Configurator::UndeclareParameter(): Removing parameter %s", parameter_full_name.c_str());

    //         parameters_.erase(parameters_.begin() + i);

    //         break;

    //     }

    // }

    parameters_.clear();

    RCLCPP_DEBUG(node_->get_logger(), "Configurator::UndeclareParameter(): Parameters undeclared");

    return undeclare_success;

}

template <typename nodeT>
std::string Configurator<nodeT>::getParameterFullName(const std::string & simple_name) const {

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

template <typename nodeT>
std::string Configurator<nodeT>::getParameterSimpleName(const std::string & full_name) const {

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

template <typename nodeT>
void Configurator<nodeT>::parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event) {

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

template <typename nodeT>
bool Configurator<nodeT>::sendDeclareParametersRequest(
        const std::vector<std::string> & names,
        const std::vector<std::string> & types,
        std::vector<rclcpp::Parameter> & parameters,
        std::string & message
) {

    // Call DeclareParameter service:
    auto request = std::make_shared<iii_drone_interfaces::srv::DeclareParameters::Request>();

    request->names = names;
    request->types = types;
    request->node_name = node_->get_name();

    // Wait for service:
    if (!declare_parameters_client_->wait_for_service(std::chrono::seconds(5))) {

        std::string fatal_message = "Configurator::DeclareParameter(): Service DeclareParameter not available after 5 seconds.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    if (!declare_parameters_client_->service_is_ready()) {

        std::string fatal_message = "Configurator::DeclareParameter(): Service DeclareParameter not available.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Call service:
    auto future = declare_parameters_client_->async_send_request(request);

    if(rclcpp::spin_until_future_complete(
        configurator_node_->get_node_base_interface(), 
        future
    ) != rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendDeclareParameterRequest(): Service DeclareParameter timed out."
        );

        throw std::runtime_error("Failed to call service DeclareParameter.");

    }

    auto result = future.get();

    message = result->message;

    if (result->values.size() != names.size() && result->succeeded) {

        std::string fatal_msg = "Configurator::sendDeclareParameterRequest(): Service DeclareParameter failed, received parameter values not same amount as requested declared.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_msg.c_str()
        );

        throw std::runtime_error(fatal_msg);

    }

    parameters.clear();

    if (!result->succeeded) {

        return false;

    }

    for (int i = 0; i < result->values.size(); i++) {

        parameters.push_back(rclcpp::Parameter(
            names[i],
            rclcpp::ParameterValue(result->values[i])
        ));

    }

    return true;

}

template <typename nodeT>
bool Configurator<nodeT>::sendUndeclareParametersRequest(std::string & message) {

    // Call UndeclareParameter service:
    auto request = std::make_shared<iii_drone_interfaces::srv::UndeclareParameters::Request>();

    request->node_name = node_->get_name();

    // Wait for service:
    if (!undeclare_parameters_client_->wait_for_service(std::chrono::seconds(5))) {

        std::string warn_message = "Configurator::sendUndeclareParametersRequest(): Service UndeclareParameters not available after 5 seconds.";

        RCLCPP_WARN(
            node_->get_logger(),
            warn_message.c_str()
        );

        message = warn_message;

        return false;

    }


    if (!undeclare_parameters_client_->service_is_ready()) {

        std::string fatal_message = "Configurator::sendUndeclareParametersRequest(): Service UndeclareParameters not available.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Call service:
    auto future = undeclare_parameters_client_->async_send_request(request);

    if(rclcpp::spin_until_future_complete(
        configurator_node_->get_node_base_interface(), 
        future
    ) != rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendUndeclareParametersRequest(): Service UndeclareParameters timed out."
        );

        throw std::runtime_error("Failed to call service UndeclareParameters.");

    }

    if (!future.valid()) {

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendUndeclareParametersRequest(): Service UndeclareParameters failed."
        );

        throw std::runtime_error("Failed to call service UndeclareParameters.");

    }

    auto result = future.get();

    message = result->message;

    return result->succeeded;

}

template <typename nodeT>
bool Configurator<nodeT>::sendGetParameterRequest(
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

template <typename nodeT>
bool Configurator<nodeT>::sendGetParametersRequest(
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

    auto future = get_parameters_client_->async_send_request(request);

    if(rclcpp::spin_until_future_complete(
        configurator_node_->get_node_base_interface(), 
        future
    ) != rclcpp::FutureReturnCode::SUCCESS) {

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

// /*****************************************************************************/
// // Explicit template instantiations:
// /*****************************************************************************/

// Template class:
template class Configurator<rclcpp::Node>;
template class Configurator<rclcpp_lifecycle::LifecycleNode>;