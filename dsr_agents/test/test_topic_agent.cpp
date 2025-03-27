// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "dsr_agents/topic_agent.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class TopicAgentFixture : public dsr_agents::TopicAgent
{
public:
  TopicAgentFixture()
  : dsr_agents::TopicAgent()
  {
  }

  ~TopicAgentFixture() = default;

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }

  template<typename node_type>
  std::optional<DSR::Node> add_node(const std::string & name)
  {
    return dsr_util::NodeAgent::add_node<node_type>(name);
  }

  void set_source(const std::string & source)
  {
    source_ = source;
  }

  void handle_topic_type(
    const std::shared_ptr<rclcpp::SerializedMessage> & msg, const std::string & topic_type)
  {
    return dsr_agents::TopicAgent::handle_topic_type(msg, topic_type);
  }

  template<typename ROS_TYPE> void modify_attributes(
    std::optional<DSR::Node> & node, const ROS_TYPE & msg)
  {
    dsr_agents::TopicAgent::modify_attributes(node, msg);
  }
};

TEST_F(DsrUtilTest, topicAgentConfigure) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  const auto state_after_configure = node_agent->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_agent->activate();
  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentConfigureEmptyTopic) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));

  const auto state_after_configure = node_agent->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentHandleTopic) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create a serialized twiststamped message
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.frame_id = "test_frame";
  auto serializer_twist = rclcpp::Serialization<geometry_msgs::msg::TwistStamped>();
  auto msg = std::make_shared<rclcpp::SerializedMessage>();
  serializer_twist.serialize_message(&twist_msg, msg.get());
  // And handle the message
  std::string topic_type = "geometry_msgs/msg/TwistStamped";
  node_agent->handle_topic_type(msg, topic_type);

  // Create a serialized battery message
  sensor_msgs::msg::BatteryState ros_msg;
  ros_msg.header.frame_id = "test_frame";
  auto serializer_battery = rclcpp::Serialization<sensor_msgs::msg::BatteryState>();
  msg = std::make_shared<rclcpp::SerializedMessage>();
  serializer_battery.serialize_message(&ros_msg, msg.get());
  // And handle the message
  topic_type = "sensor_msgs/msg/BatteryState";
  node_agent->handle_topic_type(msg, topic_type);

  // Create a serialized image message
  sensor_msgs::msg::Image image_msg;
  image_msg.header.frame_id = "test_frame";
  auto serializer_img = rclcpp::Serialization<sensor_msgs::msg::Image>();
  msg = std::make_shared<rclcpp::SerializedMessage>();
  serializer_img.serialize_message(&image_msg, msg.get());
  // And handle the message
  topic_type = "sensor_msgs/msg/Image";
  node_agent->handle_topic_type(msg, topic_type);

  // Create a serialized IMU message
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "test_frame";
  auto serializer_imu = rclcpp::Serialization<sensor_msgs::msg::Imu>();
  msg = std::make_shared<rclcpp::SerializedMessage>();
  serializer_imu.serialize_message(&imu_msg, msg.get());
  // And handle the message
  topic_type = "sensor_msgs/msg/Imu";
  node_agent->handle_topic_type(msg, topic_type);

  // Create a serialized laser scan message
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.frame_id = "test_frame";
  auto serializer_laser = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
  msg = std::make_shared<rclcpp::SerializedMessage>();
  serializer_laser.serialize_message(&scan_msg, msg.get());
  // And handle the message
  topic_type = "sensor_msgs/msg/LaserScan";
  node_agent->handle_topic_type(msg, topic_type);

  // Handle an unknown message
  std::string unknown_type = "unknown_type";
  node_agent->handle_topic_type(msg, unknown_type);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentModifyAttributeVelocity) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create TwistedStamped message
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.frame_id = "test_frame";
  twist_msg.header.stamp.sec = 1;
  twist_msg.header.stamp.nanosec = 500000000;
  twist_msg.twist.linear.x = 1.0;
  twist_msg.twist.linear.y = 2.0;
  twist_msg.twist.linear.z = 3.0;
  twist_msg.twist.angular.x = 4.0;
  twist_msg.twist.angular.y = 5.0;
  twist_msg.twist.angular.z = 6.0;

  // Modify the attributes
  auto node = node_agent->add_node<robot_node_type>("vel_node");
  node_agent->modify_attributes<geometry_msgs::msg::TwistStamped>(node, twist_msg);
  node_agent->get_graph()->update_node(node.value());

  // Check the attributes
  EXPECT_TRUE(node_agent->get_graph()->get_node("vel_node").has_value());
  auto attrs = node_agent->get_graph()->get_node("vel_node").value().attrs();
  auto linear_velocity = std::get<std::vector<float>>(attrs["robot_local_linear_velocity"].value());
  EXPECT_FLOAT_EQ(linear_velocity[0], twist_msg.twist.linear.x);
  EXPECT_FLOAT_EQ(linear_velocity[1], twist_msg.twist.linear.y);
  EXPECT_FLOAT_EQ(linear_velocity[2], twist_msg.twist.linear.z);
  auto angular_velocity =
    std::get<std::vector<float>>(attrs["robot_local_angular_velocity"].value());
  EXPECT_FLOAT_EQ(angular_velocity[0], twist_msg.twist.angular.x);
  EXPECT_FLOAT_EQ(angular_velocity[1], twist_msg.twist.angular.y);
  EXPECT_FLOAT_EQ(angular_velocity[2], twist_msg.twist.angular.z);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentModifyAttributeBatteryState) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create a battery state message
  sensor_msgs::msg::BatteryState battery_state;
  battery_state.voltage = 12.5;
  battery_state.temperature = 25.0;
  battery_state.current = 1.2;
  battery_state.charge = 50.0;
  battery_state.capacity = 100.0;
  battery_state.design_capacity = 110.0;
  battery_state.percentage = 0.5;
  battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  battery_state.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state.present = true;
  battery_state.cell_voltage = {3.7, 3.7, 3.7};
  battery_state.cell_temperature = {25.0, 25.0, 25.0};
  battery_state.location = "base_link";
  battery_state.serial_number = "123456789";

  // Modify the attributes
  auto node = node_agent->add_node<robot_node_type>("battery_node");
  node_agent->modify_attributes<sensor_msgs::msg::BatteryState>(node, battery_state);
  node_agent->get_graph()->update_node(node.value());

  // Check the attributes
  EXPECT_TRUE(node_agent->get_graph()->get_node("battery_node").has_value());
  auto attrs = node_agent->get_graph()->get_node("battery_node").value().attrs();
  EXPECT_FLOAT_EQ(std::get<float>(attrs["battery_voltage"].value()), battery_state.voltage);
  EXPECT_FLOAT_EQ(std::get<float>(attrs["battery_temperature"].value()), battery_state.temperature);
  EXPECT_FLOAT_EQ(std::get<float>(attrs["battery_current"].value()), battery_state.current);
  EXPECT_FLOAT_EQ(std::get<float>(attrs["battery_charge"].value()), battery_state.charge);
  EXPECT_FLOAT_EQ(std::get<float>(attrs["battery_capacity"].value()), battery_state.capacity);
  EXPECT_FLOAT_EQ(
    std::get<float>(attrs["battery_design_capacity"].value()), battery_state.design_capacity);
  EXPECT_FLOAT_EQ(
    std::get<float>(attrs["battery_percentage"].value()), battery_state.percentage * 100.0);
  EXPECT_EQ(std::get<std::string>(attrs["battery_power_supply_status"].value()), "charging");
  EXPECT_EQ(
    std::get<int>(attrs["battery_power_supply_health"].value()), battery_state.power_supply_health);
  EXPECT_EQ(
    std::get<int>(
      attrs["battery_power_supply_technology"].value()), battery_state.power_supply_technology);
  EXPECT_EQ(std::get<bool>(attrs["battery_present"].value()), battery_state.present);
  EXPECT_EQ(
    std::get<std::vector<float>>(attrs["battery_cell_voltage"].value()),
    battery_state.cell_voltage);
  EXPECT_EQ(
    std::get<std::vector<float>>(
      attrs["battery_cell_temperature"].value()), battery_state.cell_temperature);
  EXPECT_EQ(std::get<std::string>(attrs["battery_location"].value()), battery_state.location);
  EXPECT_EQ(
    std::get<std::string>(
      attrs["battery_serial_number"].value()), battery_state.serial_number);

  // Modify the different power supply status
  battery_state.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  node_agent->modify_attributes<sensor_msgs::msg::BatteryState>(node, battery_state);
  node_agent->get_graph()->update_node(node.value());
  attrs = node_agent->get_graph()->get_node("battery_node").value().attrs();
  EXPECT_EQ(std::get<std::string>(attrs["battery_power_supply_status"].value()), "discharging");

  battery_state.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  node_agent->modify_attributes<sensor_msgs::msg::BatteryState>(node, battery_state);
  node_agent->get_graph()->update_node(node.value());
  attrs = node_agent->get_graph()->get_node("battery_node").value().attrs();
  EXPECT_EQ(std::get<std::string>(attrs["battery_power_supply_status"].value()), "not_charging");

  battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  node_agent->modify_attributes<sensor_msgs::msg::BatteryState>(node, battery_state);
  node_agent->get_graph()->update_node(node.value());
  attrs = node_agent->get_graph()->get_node("battery_node").value().attrs();
  EXPECT_EQ(std::get<std::string>(attrs["battery_power_supply_status"].value()), "full");

  battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  node_agent->modify_attributes<sensor_msgs::msg::BatteryState>(node, battery_state);
  node_agent->get_graph()->update_node(node.value());
  attrs = node_agent->get_graph()->get_node("battery_node").value().attrs();
  EXPECT_EQ(std::get<std::string>(attrs["battery_power_supply_status"].value()), "unknown");

  battery_state.power_supply_status = 5;  // Unknown value
  node_agent->modify_attributes<sensor_msgs::msg::BatteryState>(node, battery_state);
  node_agent->get_graph()->update_node(node.value());
  attrs = node_agent->get_graph()->get_node("battery_node").value().attrs();
  EXPECT_EQ(std::get<std::string>(attrs["battery_power_supply_status"].value()), "unknown");

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentModifyAttributeImage) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create an image message
  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera_frame";
  image.height = 480;
  image.width = 640;
  image.encoding = sensor_msgs::image_encodings::RGB8;
  image.data = std::vector<uint8_t>(image.height * image.width * 3, 255);

  // Modify the attributes
  auto node = node_agent->add_node<robot_node_type>("image_node");
  node_agent->modify_attributes<sensor_msgs::msg::Image>(node, image);
  node_agent->get_graph()->update_node(node.value());

  // Check the attributes
  EXPECT_TRUE(node_agent->get_graph()->get_node("image_node").has_value());
  auto attrs = node_agent->get_graph()->get_node("image_node").value().attrs();
  EXPECT_EQ(std::get<std::vector<uint8_t>>(attrs["cam_rgb"].value()), image.data);
  EXPECT_EQ(std::get<int>(attrs["cam_rgb_height"].value()), static_cast<int>(image.height));
  EXPECT_EQ(std::get<int>(attrs["cam_rgb_width"].value()), static_cast<int>(image.width));

  // Modify the attributes with a different encoding
  image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  node_agent->modify_attributes<sensor_msgs::msg::Image>(node, image);
  node_agent->get_graph()->update_node(node.value());
  attrs = node_agent->get_graph()->get_node("image_node").value().attrs();
  EXPECT_EQ(std::get<std::vector<uint8_t>>(attrs["cam_depth"].value()), image.data);
  EXPECT_EQ(std::get<int>(attrs["cam_depth_height"].value()), static_cast<int>(image.height));
  EXPECT_EQ(std::get<int>(attrs["cam_depth_width"].value()), static_cast<int>(image.width));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentModifyAttributeImu) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create an IMU message
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "imu_frame";
  imu_msg.header.stamp.sec = 123456789;
  imu_msg.header.stamp.nanosec = 987654321;
  imu_msg.angular_velocity.x = 0.1;
  imu_msg.angular_velocity.y = 0.2;
  imu_msg.angular_velocity.z = 0.3;
  imu_msg.linear_acceleration.x = 1.0;
  imu_msg.linear_acceleration.y = 1.1;
  imu_msg.linear_acceleration.z = 1.2;

  // Modify the attributes
  auto node = node_agent->add_node<robot_node_type>("imu_node");
  node_agent->modify_attributes<sensor_msgs::msg::Imu>(node, imu_msg);
  node_agent->get_graph()->update_node(node.value());

  // Check the attributes
  EXPECT_TRUE(node_agent->get_graph()->get_node("imu_node").has_value());
  auto attrs = node_agent->get_graph()->get_node("imu_node").value().attrs();
  float timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1e9;
  EXPECT_FLOAT_EQ(std::get<float>(attrs["imu_time_stamp"].value()), timestamp);
  auto angular_velocity = std::get<std::vector<float>>(attrs["imu_gyroscope"].value());
  EXPECT_FLOAT_EQ(angular_velocity[0], imu_msg.angular_velocity.x);
  EXPECT_FLOAT_EQ(angular_velocity[1], imu_msg.angular_velocity.y);
  EXPECT_FLOAT_EQ(angular_velocity[2], imu_msg.angular_velocity.z);
  auto linear_acceleration = std::get<std::vector<float>>(attrs["imu_accelerometer"].value());
  EXPECT_FLOAT_EQ(linear_acceleration[0], imu_msg.linear_acceleration.x);
  EXPECT_FLOAT_EQ(linear_acceleration[1], imu_msg.linear_acceleration.y);
  EXPECT_FLOAT_EQ(linear_acceleration[2], imu_msg.linear_acceleration.z);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentModifyAttributeLaserScan) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create a scan message with 7 points
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 180;
  scan.time_increment = 0.1;
  scan.scan_time = 0.1;
  scan.range_min = 0.0;
  scan.range_max = 10.0;
  scan.ranges.push_back(1.0);
  scan.ranges.push_back(1.1);
  scan.ranges.push_back(2.0);
  scan.ranges.push_back(2.1);
  scan.ranges.push_back(3.0);
  scan.ranges.push_back(3.2);
  scan.ranges.push_back(12.0);

  // Modify the attributes
  auto node = node_agent->add_node<robot_node_type>("test_node");
  node_agent->modify_attributes<sensor_msgs::msg::LaserScan>(node, scan);
  node_agent->get_graph()->update_node(node.value());

  // Check the attributes
  EXPECT_TRUE(node_agent->get_graph()->get_node("test_node").has_value());
  auto attrs = node_agent->get_graph()->get_node("test_node").value().attrs();
  auto angles = std::get<std::vector<float>>(attrs["laser_angles"].value());
  EXPECT_FLOAT_EQ(angles[0], scan.angle_min);
  EXPECT_FLOAT_EQ(angles[1], scan.angle_min + scan.angle_increment);
  EXPECT_FLOAT_EQ(angles[6], scan.angle_max);
  auto ranges = std::get<std::vector<float>>(attrs["laser_dists"].value());
  EXPECT_FLOAT_EQ(ranges[0], scan.ranges[0]);
  EXPECT_FLOAT_EQ(ranges[1], scan.ranges[1]);
  EXPECT_FLOAT_EQ(ranges[2], scan.ranges[2]);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, topicAgentModifyAttributeString) {
  // Create the node
  auto node_agent = std::make_shared<TopicAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("ros_topic", rclcpp::ParameterValue("test_topic"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create a string message
  std_msgs::msg::String str_msg;
  str_msg.data = "test_string";

  // Modify the attributes
  auto node = node_agent->add_node<robot_node_type>("test_node");
  node_agent->modify_attributes<std_msgs::msg::String>(node, str_msg);
  node_agent->get_graph()->update_node(node.value());

  // Check the attributes
  EXPECT_TRUE(node_agent->get_graph()->get_node("test_node").has_value());
  auto attrs = node_agent->get_graph()->get_node("test_node").value().attrs();
  auto text = std::get<std::string>(attrs["text"].value());
  EXPECT_EQ(text, str_msg.data);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
