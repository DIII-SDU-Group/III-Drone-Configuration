#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/declare_parameter.hpp>
#include <iii_drone_interfaces/srv/undeclare_parameter.hpp>

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
     * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
     */
    Configurator(
        nodeT *node,
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
    );

    /**
     * @brief Constructor
     *
     * @param node Reference to the handling node
     * @param qos Quality of service for the parameter event subscription
     * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
     */
    Configurator(
        nodeT *node,
        const rclcpp::QoS &qos,
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
     * @tparam T Type of the parameter
     * 
     * @return String representation of the parameter type
     */
    template<typename T>
    static std::string GetParameterTypeString();

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
    * @brief Adds a parameter to the node.
    *
    * @param parameter_full_name Name of the parameter
    * 
    * @tparam T Type of the parameter
    * 
    * @return void
    */
    template<typename T>
    void declareParameter(const std::string & parameter_full_name);

    /**
     * @brief Undeclares a parameter from the node.
     * 
     * @param parameter_full_name Name of the parameter
     * @param skip_undeclare Skip undeclare if true
     * 
     * @return bool
     */
    bool undeclareParameter(
        const std::string & parameter_full_name,
        bool skip_undeclare = false
    );

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

    /**
     * @brief Service client callback group
     */
    rclcpp::CallbackGroup::SharedPtr service_client_callback_group_;

    /*
    * @brief Mutex for access to the parameters vector.
    */
    mutable std::shared_mutex parameters_mutex_;

    /**
     * @brief DeclareParameter service client.
     */
    rclcpp::Client<iii_drone_interfaces::srv::DeclareParameter>::SharedPtr declare_parameter_client_;

    /**
     * @brief UndeclareParameter service client.
     */
    rclcpp::Client<iii_drone_interfaces::srv::UndeclareParameter>::SharedPtr undeclare_parameter_client_;

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
     * @brief Sends a DeclareParameter request to the ConfigurationServer node,
     *       which will declare the parameter in the parameter server.
     * 
     * @param name Name of the parameter
     * @param type Type string representation of the parameter
     * 
     * @return bool True if the parameter was declared successfully, false otherwise
     */
    bool sendDeclareParameterRequest(
        const std::string & name,
        const std::string & type,
        std::string & message
    );

    /**
     * @brief Sends an UndeclareParameter request to the ConfigurationServer node,
     * 
     * @param name Name of the parameter
     * @param message Message out reference
     * 
     * @return bool True if the parameter was undeclared successfully, false otherwise
     */
    bool sendUndeclareParameterRequest(
        const std::string & name,
        std::string & message
    );

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