#include <gtest/gtest.h>

#include <iii_drone_configuration/configurator.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <unistd.h>

using iii_drone::configuration::Configurator;
using iii_drone::configuration::configuration_entry_t;

template <typename NodeT>
class ConfiguratorTypedTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        setenv("III_DRONE_SCHEMA_FILE", TEST_SCHEMA_FILE, 1);
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override
    {
        static int node_counter = 0;
        const auto node_name = "configurator_test_node_" + std::to_string(node_counter++);
        if constexpr (std::is_same_v<NodeT, rclcpp::Node>) {
            node_ = new rclcpp::Node(node_name);
        } else {
            node_ = new rclcpp_lifecycle::LifecycleNode(node_name);
        }
    }

    NodeT * node_;
};

using ConfiguratorNodeTypes = ::testing::Types<rclcpp::Node, rclcpp_lifecycle::LifecycleNode>;
TYPED_TEST_SUITE(ConfiguratorTypedTest, ConfiguratorNodeTypes);

TEST(ConfiguratorPathResolutionTest, IgnoresWorkspaceSourceAndWritableSchemaShadow)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    const char * previous_schema_file = std::getenv("III_DRONE_SCHEMA_FILE");
    const std::string previous_schema_file_value = previous_schema_file != nullptr ? previous_schema_file : "";
    const char * previous_config_base = std::getenv("CONFIG_BASE_DIR");
    const std::string previous_config_base_value = previous_config_base != nullptr ? previous_config_base : "";
    const char * previous_workspace_dir = std::getenv("WORKSPACE_DIR");
    const std::string previous_workspace_dir_value = previous_workspace_dir != nullptr ? previous_workspace_dir : "";
    const char * previous_simulation = std::getenv("SIMULATION");
    const std::string previous_simulation_value = previous_simulation != nullptr ? previous_simulation : "";

    const auto temp_config_base = std::filesystem::temp_directory_path() /
        ("iii_configurator_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(temp_config_base);
    std::filesystem::create_directories(temp_config_base);
    const auto source_shadow = temp_config_base / "workspace" / "src" /
        "III-Drone-Configuration" / "config" / "parameters";
    std::filesystem::create_directories(source_shadow);
    std::ofstream(source_shadow / "parameter_manifest.yaml") << "malformed: source shadow\n";
    const auto writable_shadow = temp_config_base / "iii_drone" / "parameters";
    std::filesystem::create_directories(writable_shadow);
    std::ofstream(writable_shadow / "parameter_manifest.yaml") << "malformed: writable shadow\n";

    unsetenv("III_DRONE_SCHEMA_FILE");
    setenv("CONFIG_BASE_DIR", temp_config_base.string().c_str(), 1);
    setenv("WORKSPACE_DIR", (temp_config_base / "workspace").string().c_str(), 1);
    setenv("SIMULATION", "true", 1);

    auto node = std::make_shared<rclcpp::Node>("configurator_installed_schema_test");
    EXPECT_NO_THROW({
        Configurator<rclcpp::Node> configurator(node.get(), node->get_name());
        configurator.DeclareParameter(
            "/perception/hough_transformer/canny_low_threshold",
            rclcpp::ParameterType::PARAMETER_INTEGER
        );
    });

    std::filesystem::remove_all(temp_config_base);
    if (previous_schema_file != nullptr) {
        setenv("III_DRONE_SCHEMA_FILE", previous_schema_file_value.c_str(), 1);
    } else {
        unsetenv("III_DRONE_SCHEMA_FILE");
    }
    if (previous_config_base != nullptr) {
        setenv("CONFIG_BASE_DIR", previous_config_base_value.c_str(), 1);
    } else {
        unsetenv("CONFIG_BASE_DIR");
    }
    if (previous_workspace_dir != nullptr) {
        setenv("WORKSPACE_DIR", previous_workspace_dir_value.c_str(), 1);
    } else {
        unsetenv("WORKSPACE_DIR");
    }
    if (previous_simulation != nullptr) {
        setenv("SIMULATION", previous_simulation_value.c_str(), 1);
    } else {
        unsetenv("SIMULATION");
    }
}

TYPED_TEST(ConfiguratorTypedTest, DeclaresManagedParametersWithSchemaDefaults)
{
    auto * configurator = new Configurator<TypeParam>(this->node_, this->node_->get_name());
    (void)configurator;

    configurator->DeclareParameter("/control/gains/p", rclcpp::ParameterType::PARAMETER_DOUBLE);

    EXPECT_TRUE(this->node_->has_parameter("/control/gains/p"));
    EXPECT_DOUBLE_EQ(this->node_->get_parameter("/control/gains/p").as_double(), 1.5);
    EXPECT_NO_THROW(configurator->validate());
}

TYPED_TEST(ConfiguratorTypedTest, UsesSchemaDefaultsForUndeclaredExpressionReferences)
{
    auto * configurator = new Configurator<TypeParam>(this->node_, this->node_->get_name());
    (void)configurator;

    configurator->DeclareParameter("/control/gains/i", rclcpp::ParameterType::PARAMETER_DOUBLE);

    EXPECT_NO_THROW(configurator->validate());
    const auto result = this->node_->set_parameter(rclcpp::Parameter("/control/gains/i", 0.4));
    EXPECT_TRUE(result.successful);
}

TYPED_TEST(ConfiguratorTypedTest, RejectsInvalidRuntimeUpdates)
{
    auto * configurator = new Configurator<TypeParam>(this->node_, this->node_->get_name());
    (void)configurator;

    configurator->DeclareParameters(
        {"/control/gains/p", "/control/gains/i", "/control/immutable_name"},
        {
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            rclcpp::ParameterType::PARAMETER_STRING,
        }
    );

    ASSERT_NO_THROW(configurator->validate());

    auto result = this->node_->set_parameter(rclcpp::Parameter("/control/gains/i", 2.0));
    EXPECT_FALSE(result.successful);

    result = this->node_->set_parameter(rclcpp::Parameter("/control/immutable_name", std::string("beta")));
    EXPECT_FALSE(result.successful);

    result = this->node_->set_parameter(rclcpp::Parameter("/control/gains/p", 4.0));
    EXPECT_TRUE(result.successful);
}

TYPED_TEST(ConfiguratorTypedTest, CreatesLiveConfigurations)
{
    auto * configurator = new Configurator<TypeParam>(this->node_, this->node_->get_name());
    (void)configurator;

    configurator->DeclareParameters(
        {"/control/gains/p", "/control/gains/i"},
        {rclcpp::ParameterType::PARAMETER_DOUBLE, rclcpp::ParameterType::PARAMETER_DOUBLE}
    );

    auto configuration = configurator->CreateConfiguration(
        "controller",
        {
            configuration_entry_t("/control/gains/p", rclcpp::ParameterType::PARAMETER_DOUBLE),
            configuration_entry_t("/control/gains/i", rclcpp::ParameterType::PARAMETER_DOUBLE),
        }
    );

    EXPECT_DOUBLE_EQ(configuration->GetParameter("/control/gains/p").as_double(), 1.5);
    const auto result = this->node_->set_parameter(rclcpp::Parameter("/control/gains/p", 3.0));
    ASSERT_TRUE(result.successful);
    EXPECT_DOUBLE_EQ(configuration->GetParameter("/control/gains/p").as_double(), 3.0);
    EXPECT_EQ(configurator->GetConfiguration("controller")->name(), "controller");
}
