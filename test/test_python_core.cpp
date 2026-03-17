#include <gtest/gtest.h>

#include <iii_drone_configuration/python_core.hpp>

#include <vector>

using iii_drone::configuration::PythonConfiguratorCore;
using iii_drone::configuration::configuration_entry_t;

TEST(PythonConfiguratorCoreTest, DeclaresManagedParametersFromSchemaDefaults)
{
    PythonConfiguratorCore core(TEST_SCHEMA_FILE);

    const auto value = core.DeclareParameter("/control/gains/p", rclcpp::ParameterType::PARAMETER_DOUBLE);
    EXPECT_DOUBLE_EQ(value.get<double>(), 1.5);

    const auto names = core.managed_parameter_names();
    ASSERT_EQ(names.size(), 1u);
    EXPECT_EQ(names.front(), "/control/gains/p");
}

TEST(PythonConfiguratorCoreTest, RejectsDeclarationTypeMismatch)
{
    PythonConfiguratorCore core(TEST_SCHEMA_FILE);

    EXPECT_THROW(
        core.DeclareParameter("/control/gains/p", rclcpp::ParameterType::PARAMETER_STRING),
        std::runtime_error
    );
}

TEST(PythonConfiguratorCoreTest, CreatesConfigurationsAndValidatesMaps)
{
    PythonConfiguratorCore core(TEST_SCHEMA_FILE);
    auto configuration = core.CreateConfiguration(
        "mapper",
        {
            configuration_entry_t("/control/gains/p", rclcpp::ParameterType::PARAMETER_DOUBLE),
            configuration_entry_t("/control/mode", rclcpp::ParameterType::PARAMETER_STRING),
        }
    );

    EXPECT_TRUE(configuration->HasParameter("/control/gains/p"));
    EXPECT_FALSE(configuration->HasParameter("/control/gains/i"));
    EXPECT_EQ(core.GetConfiguration("mapper")->name(), "mapper");

    std::unordered_map<std::string, rclcpp::ParameterValue> values{
        {"/control/gains/p", rclcpp::ParameterValue(2.0)},
        {"/control/gains/i", rclcpp::ParameterValue(1.0)},
        {"/control/mode", rclcpp::ParameterValue(std::string("auto"))},
        {"/control/enabled", rclcpp::ParameterValue(true)},
        {"/control/immutable_name", rclcpp::ParameterValue(std::string("alpha"))},
        {"/perception/pl_mapper/weights", rclcpp::ParameterValue(std::vector<double>{1.0, 2.0})},
        {"/perception/pl_mapper/ids", rclcpp::ParameterValue(std::vector<int64_t>{1, 2})},
    };

    EXPECT_NO_THROW(core.ValidateParameterMap(values, true));
}
