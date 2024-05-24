/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_configuration/parameter_bundle.hpp>

using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ParameterBundle::ParameterBundle(
    std::string name,
    std::vector<parameter_bundle_entry_t> parameter_bundle_entries
) {
    name_ = name;
    parameter_bundle_entries_ = parameter_bundle_entries;
}

rclcpp::Parameter ParameterBundle::GetParameter(const std::string & parameter_remap_name) const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    for (auto & parameter_bundle_entry : parameter_bundle_entries_) {
        if (parameter_bundle_entry.remap_name == parameter_remap_name) {
            return parameter_bundle_entry.parameter;
        }
    }
    
    throw std::runtime_error("ParameterBundle::GetParameter(): Parameter " + parameter_remap_name + " does not exist.");

}

void ParameterBundle::SetParameter(
    const std::string & parameter_simple_name, 
    const rclcpp::Parameter & parameter
) {

    std::unique_lock<std::shared_mutex> lock(mutex_);

    for (auto & parameter_bundle_entry : parameter_bundle_entries_) {

        if (parameter_bundle_entry.simple_name == parameter_simple_name) {

            if (!parameter_bundle_entry.update) {
                throw std::runtime_error("ParameterBundle::SetParameter(): Parameter " + parameter_simple_name + " is not updatable.");
            }

            if (parameter_bundle_entry.parameter.get_type() != parameter.get_type()) {
                throw std::runtime_error("ParameterBundle::SetParameter(): Parameter " + parameter_simple_name + " type mismatch.");
            }

            parameter_bundle_entry.parameter = parameter;
            
            return;
        }
    }

    throw std::runtime_error("ParameterBundle::SetParameter(): Parameter " + parameter_simple_name + " does not exist.");

}

bool ParameterBundle::HasParameter(const std::string & parameter_simple_name) const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    for (auto & parameter_bundle_entry : parameter_bundle_entries_) {
        if (parameter_bundle_entry.simple_name == parameter_simple_name) {
            return true;
        }
    }

    return false;

}

bool ParameterBundle::HasUpdatableParameter(const std::string & parameter_simple_name) const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    for (auto & parameter_bundle_entry : parameter_bundle_entries_) {
        if (parameter_bundle_entry.simple_name == parameter_simple_name) {
            return parameter_bundle_entry.update;
        }
    }

    return false;

}

std::string ParameterBundle::name() const {
    return name_;
}