#include <gtest/gtest.h>

#include <iii_drone_configuration/configurator.hpp>

#include <cstdlib>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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

TYPED_TEST(ConfiguratorTypedTest, DeclaresManagedParametersWithSchemaDefaults)
{
    auto * configurator = new Configurator<TypeParam>(this->node_, this->node_->get_name());
    (void)configurator;

    configurator->DeclareParameter("/control/gains/p", rclcpp::ParameterType::PARAMETER_DOUBLE);

    EXPECT_TRUE(this->node_->has_parameter("/control/gains/p"));
    EXPECT_DOUBLE_EQ(this->node_->get_parameter("/control/gains/p").as_double(), 1.5);
    EXPECT_NO_THROW(configurator->validate());
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
