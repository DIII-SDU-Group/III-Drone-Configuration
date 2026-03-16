#pragma once

#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace iii_drone {
namespace configuration {

struct configuration_entry_t {
    std::string full_name;
    rclcpp::ParameterType parameter_type;

    configuration_entry_t(
        std::string full_name,
        rclcpp::ParameterType parameter_type
    ) : full_name(std::move(full_name)), parameter_type(parameter_type) {}
};

class Configuration {
public:
    Configuration(
        std::string name,
        std::vector<configuration_entry_t> configuration_entries,
        std::function<rclcpp::Parameter(const std::string &)> parameter_getter
    );

    rclcpp::Parameter GetParameter(const std::string & parameter_full_name) const;

    bool HasParameter(const std::string & parameter_full_name) const;

    std::string name() const;

    using SharedPtr = std::shared_ptr<Configuration>;

private:
    mutable std::shared_mutex mutex_;
    std::string name_;
    std::function<rclcpp::Parameter(const std::string &)> parameter_getter_;
    std::vector<configuration_entry_t> configuration_entries_;
};

}  // namespace configuration
}  // namespace iii_drone
