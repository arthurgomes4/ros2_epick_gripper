// Copyright (c) 2023 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <epick_driver/epick_gripper_hardware_interface.hpp>
#include <epick_driver/hardware_interface_utils.hpp>
#include <epick_driver/default_driver_factory.hpp>
#include <epick_driver/fake/fake_driver.hpp>

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

namespace epick_driver::test
{
using ::testing::Contains;
using ::testing::Eq;

using hardware_interface_utils::is_false;
using hardware_interface_utils::is_true;

// This factory will populate the injected driver with data read form the HardwareInfo.
class TestDriverFactory : public DefaultDriverFactory
{
public:
  explicit TestDriverFactory(std::unique_ptr<Driver> driver) : driver_{ std::move(driver) }
  {
  }

protected:
  std::unique_ptr<Driver> create_driver([[maybe_unused]] const hardware_interface::HardwareInfo& info) const override
  {
    return std::move(driver_);
  }

private:
  mutable std::unique_ptr<Driver> driver_;
};

// Jazzy's ResourceManager and its components require a clock and a logger. A
// standalone ROS-time clock and a named logger need no initialized ROS context,
// so they are safe to construct in a plain gtest binary.
rclcpp::Clock::SharedPtr make_test_clock()
{
  return std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

rclcpp::Logger make_test_logger()
{
  return rclcpp::get_logger("test_epick_gripper_hardware_interface");
}

/**
 * This method allow a busy wait on a given condition until a timeout is exceeded.
 * @param condition The condition to be met.
 * @param timeout The maximum waiting time.
 * @return True if the condition is met, false otherwise.
 */
bool wait_for_condition(std::function<bool()> condition, std::chrono::milliseconds timeout)
{
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    if (condition())
    {
      return true;  // Condition met
    }
    std::this_thread::yield();  // Yield the current thread to reduce busy-waiting overhead.
  }
  return false;  // Timeout reached
}

/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestEpickGripperHardwareInterface, load_urdf)
{
  std::string urdf_control_ =
      R"(
         <ros2_control name="EpickGripperHardwareInterface" type="system">
           <hardware>
             <plugin>epick_driver/EpickGripperHardwareInterface</plugin>
             <param name="usb_port">/dev/whatever</param>
             <param name="baudrate">9600</param>
             <param name="timeout">0.5</param>
           </hardware>
           <gpio name="gripper">
               <command_interface name="grip_cmd"/>
               <state_interface name="grip_cmd"/>
               <state_interface name="object_detection_status"/>
           </gpio>
         </ros2_control>
       )";

  auto urdf = ros2_control_test_assets::urdf_head + urdf_control_ + ros2_control_test_assets::urdf_tail;

  // Jazzy removed the URDF-only ResourceManager constructor; it now requires a
  // clock and logger and loads + initializes the components in the constructor.
  hardware_interface::ResourceManager rm(urdf, make_test_clock(), make_test_logger());

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
}

/**
 * In this test we startup the hardware interface with a fake driver so we can
 * test read and write operations.
 */
TEST(TestEpickGripperHardwareInterface, grip)
{
  auto driver = std::make_unique<FakeDriver>();

  auto hardware = std::make_unique<epick_driver::EpickGripperHardwareInterface>(
      std::make_unique<TestDriverFactory>(std::move(driver)));

  // Parse the HardwareInfo for the epick GPIO interfaces from a URDF fragment
  // instead of hand-building the struct. The ComponentInfo/InterfaceInfo
  // aggregate layout changes across ros2_control releases, so parsing keeps
  // this test aligned with the current schema.
  std::string urdf_control_ =
      R"(
         <ros2_control name="EpickGripperHardwareInterface" type="system">
           <hardware>
             <plugin>epick_driver/EpickGripperHardwareInterface</plugin>
             <param name="usb_port">/dev/whatever</param>
             <param name="baudrate">9600</param>
             <param name="timeout">0.5</param>
           </hardware>
           <gpio name="gripper">
               <command_interface name="grip_cmd"/>
               <state_interface name="grip_cmd"/>
               <state_interface name="object_detection_status"/>
           </gpio>
         </ros2_control>
       )";

  auto urdf = ros2_control_test_assets::urdf_head + urdf_control_ + ros2_control_test_assets::urdf_tail;
  const std::vector<hardware_interface::HardwareInfo> control_resources =
      hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(1u, control_resources.size());

  hardware_interface::HardwareInfo hardware_info = control_resources.front();

  // Append the optional gripper/position joint state interface. It is added
  // here rather than in the URDF fragment because
  // parse_control_resources_from_urdf cross-validates <joint> names against the
  // robot description, and the ros2_control test-asset skeleton defines no
  // "gripper" joint. The hardware interface exports gripper/position (mirrored
  // from grip status) only when this joint state interface is present in the
  // HardwareInfo. It is state-only: the interface exports no joint command
  // interface, so declaring one would make the ResourceManager expect an
  // interface the hardware never provides.
  hardware_interface::InterfaceInfo position_state;
  position_state.name = hardware_interface::HW_IF_POSITION;
  position_state.size = 1;
  position_state.enable_limits = false;
  hardware_interface::ComponentInfo gripper_joint;
  gripper_joint.name = "gripper";
  gripper_joint.type = "joint";
  gripper_joint.state_interfaces = { position_state };
  hardware_info.joints.push_back(gripper_joint);

  // Load the component. Jazzy's import_component takes a HardwareComponentParams
  // carrying the HardwareInfo plus the clock and logger.
  const auto clock = make_test_clock();
  const auto logger = make_test_logger();
  hardware_interface::ResourceManager rm(clock, logger);

  hardware_interface::HardwareComponentParams params;
  params.hardware_info = hardware_info;
  params.clock = clock;
  params.logger = logger;
  rm.import_component(std::move(hardware), params);

  // Connect the hardware.
  rclcpp_lifecycle::State active_state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                        hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("EpickGripperHardwareInterface", active_state);

  EXPECT_THAT(rm.command_interface_keys(), Contains(Eq("gripper/grip_cmd")));
  EXPECT_THAT(rm.state_interface_keys(), Contains(Eq("gripper/grip_cmd")));
  EXPECT_THAT(rm.state_interface_keys(), Contains(Eq("gripper/position")));

  // The gripper/grip_cmd GPIO interface is intended to be used as a boolean.
  // Unfortunately, because of limitations of ros2_controls, it is implemented
  // as a double. We use the convention that 0.0 means false and 1.0 means true.
  // To avoid possible comparison issues with double, to check if a value is
  // false or true we use the following formula: bool is_true = value >= 0.5;

  // The gripper/grip_cmd GPIO state interface follows the value of the
  // corresponding gripper/grip_cmd GPIO command interface with a small delay.

  // Claim the gripper/grip_cmd gpio command interface.
  hardware_interface::LoanedCommandInterface gripper_gpio_command_interface =
      rm.claim_command_interface("gripper/grip_cmd");
  ASSERT_TRUE(is_false(gripper_gpio_command_interface.get_optional().value()));

  // Claim the gripper/grip_cmd gpio state interface.
  hardware_interface::LoanedStateInterface gripper_gpio_state_interface = rm.claim_state_interface("gripper/grip_cmd");
  ASSERT_TRUE(is_false(gripper_gpio_state_interface.get_optional().value()));

  // Claim the gripper/position joint state interface.
  hardware_interface::LoanedStateInterface gripper_joint_state_interface = rm.claim_state_interface("gripper/position");
  ASSERT_TRUE(is_false(gripper_joint_state_interface.get_optional().value()));

  // Ask the gripper to grip.
  ASSERT_TRUE(gripper_gpio_command_interface.set_value(1.0));
  rm.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

  auto gripper_gripping = [&]() {
    rm.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    return is_true(gripper_gpio_state_interface.get_optional().value());
  };
  ASSERT_TRUE(wait_for_condition(gripper_gripping, std::chrono::milliseconds(500)))
      << "Timeout exceeded waiting for the gripper to grip.";

  // Test the content of the optional joint.
  ASSERT_TRUE(is_true(gripper_joint_state_interface.get_optional().value()));

  // Ask the gripper to release.
  ASSERT_TRUE(gripper_gpio_command_interface.set_value(0.0));
  rm.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

  auto gripper_released = [&]() {
    rm.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    return is_false(gripper_gpio_state_interface.get_optional().value());
  };
  ASSERT_TRUE(wait_for_condition(gripper_released, std::chrono::milliseconds(500)))
      << "Timeout exceeded waiting for the gripper to release.";

  // Test the content of the optional joint.
  ASSERT_TRUE(is_false(gripper_joint_state_interface.get_optional().value()));

  // Deactivate the hardware.
  rclcpp_lifecycle::State inactive_state{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                          hardware_interface::lifecycle_state_names::INACTIVE };
  rm.set_component_state("EpickGripperHardwareInterface", inactive_state);
}

}  // namespace epick_driver::test
