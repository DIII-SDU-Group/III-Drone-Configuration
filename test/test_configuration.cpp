#include <gtest/gtest.h>

#include <iii_drone_configuration/configuration.hpp>

#include <string>

using iii_drone::configuration::Configuration;
using iii_drone::configuration::configuration_entry_t;

TEST(ConfigurationTest, ReadsLiveValuesThroughGetter)
{
    double value = 1.5;
    Configuration configuration(
        "controller",
        {configuration_entry_t("/control/gains/p", rclcpp::ParameterType::PARAMETER_DOUBLE)},
        [&value](const std::string & name) {
            return rclcpp::Parameter(name, value);
        }
    );

    EXPECT_DOUBLE_EQ(configuration.GetParameter("/control/gains/p").as_double(), 1.5);
    value = 4.0;
    EXPECT_DOUBLE_EQ(configuration.GetParameter("/control/gains/p").as_double(), 4.0);
    EXPECT_TRUE(configuration.HasParameter("/control/gains/p"));
    EXPECT_EQ(configuration.name(), "controller");
}

TEST(ConfigurationTest, RejectsUnknownParameterLookup)
{
    Configuration configuration(
        "controller",
        {configuration_entry_t("/control/gains/p", rclcpp::ParameterType::PARAMETER_DOUBLE)},
        [](const std::string & name) {
            return rclcpp::Parameter(name, 1.0);
        }
    );

    EXPECT_FALSE(configuration.HasParameter("/control/gains/i"));
    EXPECT_THROW(configuration.GetParameter("/control/gains/i"), std::runtime_error);
}
