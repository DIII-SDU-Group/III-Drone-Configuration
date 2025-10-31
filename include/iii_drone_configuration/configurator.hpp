#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/declare_parameters.hpp>
#include <iii_drone_interfaces/srv/undeclare_parameters.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

/*****************************************************************************/
// Std:

#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <atomic>

/*****************************************************************************/
// Yaml-CPP:

#include <yaml-cpp/yaml.h>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace configuration {

/**
 * @brief Class for handling parameters of a node. The class works in conjunction with the ConfigurationServer node.
 * The class will declare parameters with the ConfigurationServer node and will subscribe to the parameter event topic
 * in order to update the local copy of the parameters.
 * The class declares a ROS2 parameter, node_parameters_dir. Here, it looks for the file with the same name
 * as the node. If the file doesn't exist, it will fail.
*/
template <typename nodeT>
class Configurator {
public:
    /**
     * @brief Constructor
     *
     * @param node Reference to the handling node
     * @param node_name Name of the node
     * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
     */
    Configurator(
        nodeT *node,
        const std::string & node_name,
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
    );

 
    /*
    * @brief Destructor
    */
    ~Configurator();

    /**
     * @brief Get a parameter.
     *
     * @param simple_name Name of the parameter
     * 
     * @return The parameter
     */
    rclcpp::Parameter GetParameter(const std::string & simple_name) const;

    /**
     * @brief Gets multiple parameters.
     * 
     * @param simple_names Names of the parameters
     * 
     * @return std::vector<rclcpp::Parameter> Vector of parameters
     */
    std::vector<rclcpp::Parameter> GetParameters(const std::vector<std::string> & simple_names) const;

    /**
     * @brief Gets a parameter bundle.
     * 
     * @param name Name of the bundle
     * 
     * @return The bundle
     * 
     * @throws std::runtime_error if the bundle does not exist.
     */
    ParameterBundle::SharedPtr GetParameterBundle(const std::string & name) const;

    /**
     * @brief Synchronizes parameters with the parameter server. Should never be necessary.
     * 
     * @param simple_names Names of the parameters, default is empty (all parameters)
     * 
     * @return void
     */
    void SyncParameters(const std::vector<std::string> & simple_names = {});

    /**
     * @brief Static function for getting string representation of a parameter type from template type.
     * 
     * @param parameter_type Type of the parameter
     * 
     * @return String representation of the parameter type
     */
    static std::string GetParameterTypeString(rclcpp::ParameterType parameter_type);

    static rclcpp::ParameterType GetParameterTypeFromString(const std::string & parameter_type);

    /**
     * @brief Prints all parameters to the console.
     * 
     * @return void
     */
    void PrintParameters() const;

    /**
     * @brief Prints all parameter bundles to the console.
     * 
     * @return void
     */
    void PrintParameterBundles() const;

    /**
     * @brief Shared pointer type.
     */
    typedef std::shared_ptr<Configurator> SharedPtr;

private:
    /*
    * @brief Reference to the handling node.
    */
    nodeT *node_;

    /**
     * @brief Configurator node.
     */
    rclcpp::Node::SharedPtr configurator_node_;

    /**
     * @brief Parameter bundles.
     */
    std::vector<ParameterBundle::SharedPtr> parameter_bundles_;

    /**
     * @brief Declares all parameters from the YAML file and initializes parameter bundles
     * 
     * @param parameter_yaml_path File path to the YAML file containing the parameters and bundles for the node
     * 
     * @return void
     */
    void initialize(const std::string & parameter_yaml_path);

    bool initialized_ = false;

    /**
     * @brief Initializes the parameters.
     * 
     * @param parameters Parameters yaml node
     * 
     * @return void
     */
    void initializeParameters(const YAML::Node & parameters);

    /**
     * @brief Initializes the parameter bundles.
     * 
     * @param parameter_bundles Parameter bundles yaml node
     * 
     * @return void
     */
    void initializeParameterBundles(const YAML::Node & parameter_bundles);

    /*
    * @brief Adds parameters to the node.
    *
    * @param parameter_full_names Names of the parameter
    * 
    * @param parameter_types Types of the parameter
    * 
    * @return void
    */
    void declareParameters(
        const std::vector<std::string> & parameter_full_names,
        const std::vector<rclcpp::ParameterType> & parameter_types
    );

    /**
     * @brief Undeclares parameters from the node.
     * 
     * @param skip_server_undeclare If true, will only remove parameters internally and not undeclare them from the server
     * 
     * @return bool
     */
    bool undeclareParameters(bool skip_server_undeclare = false);

    /**
     * @brief Gets the full name of a parameter.
     * 
     * @param simple_name Simple name of the parameter
     * 
     * @return Full name of the parameter
     */
    std::string getParameterFullName(const std::string & simple_name) const;

    /**
     * @brief Gets the simple name of a parameter.
     * 
     * @param full_name Full name of the parameter
     * 
     * @return Simple name of the parameter
     */
    std::string getParameterSimpleName(const std::string & full_name) const;

    /*
    * @brief The parameters of the node.
    */
    std::vector<rclcpp::Parameter> parameters_;

    /**
     * @brief Parameter simple name to full name map.
     */
    std::map<std::string, std::string> parameter_name_map_;

    /**
     * @brief QoS settings for the parameter event subscription.
     */
    std::unique_ptr<rclcpp::QoS> qos_ = nullptr;

    /*
    * @brief Mutex for access to the parameters vector.
    */
    mutable std::shared_mutex parameters_mutex_;

    /**
     * @brief DeclareParameter service client.
     */
    rclcpp::Client<iii_drone_interfaces::srv::DeclareParameters>::SharedPtr declare_parameters_client_;

    /**
     * @brief UndeclareParameter service client.
     */
    rclcpp::Client<iii_drone_interfaces::srv::UndeclareParameters>::SharedPtr undeclare_parameters_client_;

    /**
     * @brief GetParameters service client.
     */
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;

    /**
     * @brief /parameter_events subscriber.
     */
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_events_subscriber_;

    /*
    * @brief Parameter event callback function.
    *   
    * @param parameter_evemt Parameter event
    * 
    * @return void
    */
    void parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event);

    /*
    * @brief Callback function reference called after successful parameter change.
    */
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

    /**
     * @brief Sends a DeclareParameters request to the ConfigurationServer node,
     *       which will declare the parameters in the parameter server.
     * 
     * @param names Names of the parameter
     * @param type Type string representations of the parameters
     * @param parameters Parameter object out reference
     * @param message Message out reference
     * 
     * @return bool True if the parameters were declared successfully, false otherwise
     */
    bool sendDeclareParametersRequest(
        const std::vector<std::string> & names,
        const std::vector<std::string> & types,
        std::vector<rclcpp::Parameter> & parameters,
        std::string & message
    );

    /**
     * @brief Sends an UndeclareParameters request to the ConfigurationServer node,
     * 
     * @param message Message out reference
     * 
     * @return bool True if the parameters were undeclared successfully, false otherwise
     */
    bool sendUndeclareParametersRequest(std::string & message);

    /**
     * @brief Sends a GetParameters request to the ConfigurationServer node,
     *       which will return the parameter.
     * 
     * @param name Name of the parameter
     * @param parameter Parameter out reference
     * 
     * @return bool True if the parameter could be fetched.
     */
    bool sendGetParameterRequest(
        const std::string & name,
        rclcpp::Parameter & parameter
    );

    /**
     * @brief Sends a GetParameters request to the ConfigurationServer node,
     *       which will return the parameters.
     * 
     * @param names Names of the parameters
     * @param parameters Parameters out reference
     * 
     * @return bool True if the parameters could be fetched.
     */
    bool sendGetParametersRequest(
        const std::vector<std::string> & names,
        std::vector<rclcpp::Parameter> & parameters
    );


};

} // namespace configuration
} // namespace iii_ros2