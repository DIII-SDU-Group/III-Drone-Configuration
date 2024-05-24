#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <vector>
#include <memory>
#include <shared_mutex>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {

namespace configuration {

    struct parameter_bundle_entry_t {
        rclcpp::Parameter parameter;
        std::string simple_name;
        std::string remap_name;
        bool update;

        parameter_bundle_entry_t(
            rclcpp::Parameter parameter,
            std::string simple_name,
            std::string remap_name,
            bool update
        ) : parameter(parameter), simple_name(simple_name), remap_name(remap_name), update(update) {}
    };

} // namespace configuration
} // namespace iii_drone

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace configuration {

/**
 * @brief Class for bundling parameters together.
 */
class ParameterBundle {
    public:
        /**
         * @brief Constructor. Takes a vector of parameters bundle entries.
         * 
         * @param name Name of the parameter bundle
         * @param parameter_bundle_entries Vector of parameters bundle entries
         */
        ParameterBundle(
            std::string name,
            std::vector<parameter_bundle_entry_t> parameter_bundle_entries
        );

        /**
         * @brief Get a parameter.
         * 
         * @param parameter_remap_name Name of the parameter
         * 
         * @return The parameter
         * 
         * @throws std::runtime_error if the parameter does not exist.
         */
        rclcpp::Parameter GetParameter(const std::string & parameter_remap_name) const;

        /**
         * @brief Sets a parameter.
         * 
         * @param parameter_simple_name Name of the parameter
         * @param parameter The parameter
         * 
         * @throws std::runtime_error if the parameter does not exist or if it is not marked for updates.
         */
        void SetParameter(
            const std::string & parameter_simple_name, 
            const rclcpp::Parameter & parameter
        );

        /**
         * @brief Whether a parameter is contained in the bundle.
         * 
         * @param parameter_simple_name Name of the parameter
         * 
         * @return True if the parameter is contained in the bundle, false otherwise
         */
        bool HasParameter(const std::string & parameter_simple_name) const;

        /**
         * @brief Whether a parameter is marked for updates.
         * 
         * @param parameter_simple_name Name of the parameter
         * 
         * @return True if the parameter is contained and is marked for updates, false otherwise
         */
        bool HasUpdatableParameter(const std::string & parameter_simple_name) const;

        /**
         * @brief Name getter.
         */
        std::string name() const;

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<ParameterBundle> SharedPtr;

    private:
        /**
         * @brief Shared mutex for pointer access.
         */
        mutable std::shared_mutex mutex_;

        /**
         * @brief Name of the parameter bundle.
         */
        std::string name_;

        /**
         * @brief Vector of parameters bundle entries.
         */
        std::vector<parameter_bundle_entry_t> parameter_bundle_entries_;

};

} // namespace configuration
} // namespace iii_drone