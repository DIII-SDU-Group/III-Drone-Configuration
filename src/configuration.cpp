#include <iii_drone_configuration/configuration.hpp>

using namespace iii_drone::configuration;

Configuration::Configuration(
    std::string name,
    std::vector<configuration_entry_t> configuration_entries,
    std::function<rclcpp::Parameter(const std::string &)> parameter_getter
) : name_(std::move(name)),
    parameter_getter_(std::move(parameter_getter)),
    configuration_entries_(std::move(configuration_entries))
{
}

rclcpp::Parameter Configuration::GetParameter(const std::string & parameter_full_name) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);

    for (const auto & configuration_entry : configuration_entries_) {
        if (configuration_entry.full_name == parameter_full_name) {
            return parameter_getter_(configuration_entry.full_name);
        }
    }

    throw std::runtime_error(
        "Configuration::GetParameter(): Parameter " + parameter_full_name + " does not exist."
    );
}

bool Configuration::HasParameter(const std::string & parameter_full_name) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);

    for (const auto & configuration_entry : configuration_entries_) {
        if (configuration_entry.full_name == parameter_full_name) {
            return true;
        }
    }

    return false;
}

std::string Configuration::name() const
{
    return name_;
}
