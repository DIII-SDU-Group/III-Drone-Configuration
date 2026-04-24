#include <gtest/gtest.h>

#include <iii_drone_configuration/schema_validator.hpp>

#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

using iii_drone::configuration::SchemaValidator;

namespace {

std::unordered_map<std::string, rclcpp::ParameterValue> ValidParameterMap()
{
    return {
        {"/control/gains/p", rclcpp::ParameterValue(2.0)},
        {"/control/gains/i", rclcpp::ParameterValue(1.0)},
        {"/control/mode", rclcpp::ParameterValue(std::string("manual"))},
        {"/control/enabled", rclcpp::ParameterValue(true)},
        {"/control/immutable_name", rclcpp::ParameterValue(std::string("alpha"))},
        {"/perception/pl_mapper/weights", rclcpp::ParameterValue(std::vector<double>{1.0, 2.0})},
        {"/perception/pl_mapper/ids", rclcpp::ParameterValue(std::vector<int64_t>{1, 2, 3})},
    };
}

}  // namespace

TEST(SchemaValidatorTest, LoadsSchemaFromFileAndExposesEntries)
{
    const auto validator = SchemaValidator::FromFile(TEST_SCHEMA_FILE);

    EXPECT_TRUE(validator.HasParameter("/control/gains/p"));
    EXPECT_TRUE(validator.HasParameter("/perception/pl_mapper/weights"));
    EXPECT_EQ(validator.GetParameter("/control/gains/p").type, "float");
    EXPECT_EQ(validator.GetParameter("/control/immutable_name").default_value.get<std::string>(), "alpha");
}

TEST(SchemaValidatorTest, ValidatesWholeParameterMap)
{
    const auto validator = SchemaValidator::FromFile(TEST_SCHEMA_FILE);
    EXPECT_NO_THROW(validator.ValidateParameterMap(ValidParameterMap(), true));
}

TEST(SchemaValidatorTest, RejectsConstantOverrideWhenNotAllowed)
{
    const auto validator = SchemaValidator::FromFile(TEST_SCHEMA_FILE);
    auto values = ValidParameterMap();
    values["/control/immutable_name"] = rclcpp::ParameterValue(std::string("beta"));

    EXPECT_THROW(
        validator.ValidateParameterValue("/control/immutable_name", values.at("/control/immutable_name"), values, false),
        std::runtime_error
    );
}

TEST(SchemaValidatorTest, RejectsExpressionConstraintViolations)
{
    const auto validator = SchemaValidator::FromFile(TEST_SCHEMA_FILE);
    auto values = ValidParameterMap();
    values["/control/gains/p"] = rclcpp::ParameterValue(1.0);
    values["/control/gains/i"] = rclcpp::ParameterValue(2.0);

    EXPECT_THROW(validator.ValidateParameterMap(values, true), std::runtime_error);
}

TEST(SchemaValidatorTest, SupportsLoadingFromYamlString)
{
    std::ifstream file(TEST_SCHEMA_FILE);
    ASSERT_TRUE(file.good());
    const std::string raw_yaml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    const auto validator = SchemaValidator::FromYamlString(raw_yaml);
    EXPECT_TRUE(validator.HasParameter("/control/mode"));
    EXPECT_EQ(validator.GetParameter("/control/mode").default_value.get<std::string>(), "auto");
}

TEST(SchemaValidatorTest, LoadsProductionSchemaFile)
{
    const auto validator = SchemaValidator::FromFile(PRODUCTION_SCHEMA_FILE);
    const auto parameter_names = validator.parameter_names();

    EXPECT_FALSE(parameter_names.empty());
    EXPECT_TRUE(validator.HasParameter("/control/maneuver_controller/landed_altitude_threshold"));
    EXPECT_TRUE(validator.HasParameter("/payload/charger_gripper/gripper_command_interface"));
}

TEST(SchemaValidatorTest, ProductionSchemaDefaultsValidate)
{
    const auto validator = SchemaValidator::FromFile(PRODUCTION_SCHEMA_FILE);
    std::unordered_map<std::string, rclcpp::ParameterValue> values;
    for (const auto & [name, entry] : validator.parameters()) {
        values.emplace(name, entry.default_value);
    }

    EXPECT_NO_THROW(validator.ValidateParameterMap(values, true));
}

TEST(SchemaValidatorTest, ProductionRosParamFilesOnlyReferenceManagedSchemaKeys)
{
    const auto validator = SchemaValidator::FromFile(PRODUCTION_SCHEMA_FILE);

    for (const auto * ros_params_path : {PRODUCTION_ROS_PARAMS_REAL_FILE, PRODUCTION_ROS_PARAMS_SIM_FILE}) {
        const auto ros_params_root = YAML::LoadFile(ros_params_path);
        const auto ros_parameters = ros_params_root["/**"]["ros__parameters"];
        ASSERT_TRUE(ros_parameters.IsMap());

        for (const auto & item : ros_parameters) {
            const auto name = item.first.as<std::string>();
            EXPECT_TRUE(validator.HasParameter(name)) << "Unexpected production ros param key: " << name;
        }
    }
}
