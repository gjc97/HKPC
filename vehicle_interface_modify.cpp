// Copyright 2021 Tier IV, Inc.
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

#include "hkpc_interface/hkpc_interface.hpp"

typedef unsigned char byte; //int、double转换为unsigned char

namespace hkpc_interface
{

HKPCInterface::HKPCInterface(const rclcpp::NodeOptions & node_options)
: Node("hkpc_interface", node_options)
{
  using std::placeholders::_1;

  // Subscriber
  control_cmd_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&HKPCInterface::onControlCmd, this, _1));
  turn_indicator_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", rclcpp::QoS(1), std::bind(&HKPCInterface::onTurnIndicatorCmd, this, _1));
  gear_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
      "/control/command/gear_cmd", rclcpp::QoS(1), std::bind(&HKPCInterface::onGearCmd, this, _1));
  hazard_light_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", rclcpp::QoS(1), std::bind(&HKPCInterface::onHazardLightCmd, this, _1));
  can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "/hkpc/from_can_bus", rclcpp::QoS(500), std::bind(&HKPCInterface::onCanFrame, this, _1));

  // Publisher
  can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/hkpc/to_can_bus", rclcpp::QoS(500));
  steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS(10));
  velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS(10));
  control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS(10));
  gear_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS(10));
  turn_indicators_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", rclcpp::QoS(10));
  hazard_light_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS(10));
}

void HKPCInterface::onControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg_steer;
  can_msg_steer.header.stamp = msg->stamp;
  can_msg_steer.is_rtr = false;
  can_msg_steer.is_extended = false;
  can_msg_steer.is_error = false;
  can_msg_steer.dlc = 8;

//  can_msg_steer.id = 0x469;

//  can_msg_steer.data.at(0) = 0x20;
//  auto tire_angle_deg_1024offset = rad2deg(msg->lateral.steering_tire_angle) + 1024;  // constant 1024
//  can_msg_steer.data.at(3) = (static_cast<int16_t>(tire_angle_deg_1024offset)) >> 8;//转角高位
//  can_msg_steer.data.at(4) = (static_cast<int16_t>(tire_angle_deg_1024offset)) & 0xFF;低位转角
//  auto steering_tire_rotation_rate_rpm = msg->lateral.steering_tire_rotation_rate / (2 * M_PI) * 60;  // rad/s to rpm
//  can_msg_steer.data.at(6) =
//    static_cast<int16_t>(std::max(std::min(int(steering_tire_rotation_rate_rpm / 6), 250), 20));
//  can_msg_steer.data.at(7) =
//    static_cast<int16_t>(can_msg_steer.data.at(0) ^ can_msg_steer.data.at(3) ^ can_msg_steer.data.at(4) ^
//    can_msg_steer.data.at(6));

  double msg_steer = msg->lateral.steering_tire_angle*12*10;//左转为负数，右转为正数？
  //autoware发送的为前轮转角，需要通过转向系统传动系数转换成方向盘转角下发，此处假设传动系数为12（*12）
  can_msg_steer.id = 0xe2;
  can_msg_steer.data.at(0) = 0x48;//只控制转向
  can_msg_steer.data.at(1) = 0x00;
  can_msg_steer.data.at(2) = 0x00;

  can_msg_steer.data.at(3) = (steer>>8) & 0xff;
  can_msg_steer.data.at(4) = steer & 0xff;

  can_msg_steer.data.at(5) = 0x00;
  can_msg_steer.data.at(6) = 0x00;
  can_msg_steer.data.at(7) = 0x00;


  can_pub_->publish(can_msg_steer);


  can_msgs::msg::Frame can_msg_velocity;
  //autoware发送车速信息，底层接收油门踏板开度信息，需要使用控制算法（例如pid)将车速换算为踏板开度
  can_msg_velocity.header.stamp = msg->stamp;
  can_msg_velocity.is_rtr = false;
  can_msg_velocity.is_extended = false;
  can_msg_velocity.is_error = false;
  can_msg_velocity.dlc = 8;

//  can_msg_velocity.id = 0x470;
//  double speed_kmph_cmd = mps2kmph(msg->longitudinal.speed);
//  if ((speed_kmph_cmd < 5.0) && (speed_kmph_cmd > 0.0))
//  {
//    speed_kmph_cmd = 5.0;
//  }
//  can_msg_velocity.data.at(1) = (static_cast<int16_t>(speed_kmph_cmd)) & 0xFF;
//  can_msg_velocity.data.at(7) =
//    static_cast<int16_t>(can_msg_velocity.data.at(0) ^ can_msg_velocity.data.at(1));


  double tho = msg->longitudinal.speed;
  can_msg_velocity.id = 0xe2;
  can_msg_velocity.data.at(0) = 0x28;//只控制油门踏板
  can_msg_velocity.data.at(1) = (byte)(tho);
  can_msg_velocity.data.at(2) = 0x00;

  can_msg_velocity.data.at(3) = 0x00;
  can_msg_velocity.data.at(4) = 0x00;

  can_msg_velocity.data.at(5) = 0x00;
  can_msg_velocity.data.at(6) = 0x00;
  can_msg_velocity.data.at(7) = 0x00;

  can_pub_->publish(can_msg_velocity);
}

void HKPCInterface::onTurnIndicatorCmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header.stamp = msg->stamp;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

  //  can_msg.id = 0x31;
  //  can_msg.data.at(1) = msg->command;
  can_msg.id = 0xe2;
  //此处需要创建一个变量tureLeft作为判断左转右转的标志
  if(tureLeft)//autoware下发的msg->command是什么类型
  {
      can_msg.data.at(6) = (msg->command<<1) && 0x02;
  }
  if(turnRight)
  {
      can_msg.data.at(6) = msg->command && 0x01;
  }


  can_pub_->publish(can_msg);
}

void HKPCInterface::onGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header.stamp = msg->stamp;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

//  can_msg.id = 0x32;
  can_msg.id = 0xe2;
  can_msg.data.at(0) = 0x8c;
  can_msg.data.at(1) = 0x00;
  can_msg.data.at(2) = 0x14;//换挡需要下发一个制动信号
  can_msg.data.at(3) = 0x00;
  can_msg.data.at(4) = 0x00;
  can_msg.data.at(5) = 0x20;//P:0x08;D:0x20;N:0x18;R:0x10
  can_msg.data.at(6) = 0x00;
  can_msg.data.at(7) = 0x00;



  can_pub_->publish(can_msg);
}

void HKPCInterface::onHazardLightCmd(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header.stamp = msg->stamp;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

  can_msg.id = 0x33;
  can_msg.data.at(1) = 1;

  can_pub_->publish(can_msg);
}

void HKPCInterface::onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  static double current_steer_rad = 0.0;
  
  //current_steer_rad
  const double wheel_base_ = 2.875;  // [m]

  switch (msg->id) {
    case 0x401: {
        autoware_auto_vehicle_msgs::msg::SteeringReport steering_msg;
        steering_msg.stamp = msg->header.stamp;
        auto current_steer_deg = ((msg->data.at(3) << 8) + (msg->data.at(4))) - 1024;
        current_steer_rad = deg2rad(static_cast<int16_t>(current_steer_deg)); // [rad]
        steering_msg.steering_tire_angle = current_steer_rad;
        //steering_msg.steering_tire_angle = 0.0;

        steering_status_pub_->publish(steering_msg);

        break;
      }
    case 0x402: {
        autoware_auto_vehicle_msgs::msg::VelocityReport velocity_msg;
        velocity_msg.header = msg->header;
        velocity_msg.header.frame_id = "base_link";
        velocity_msg.longitudinal_velocity = 
          kmph2mps((static_cast<int16_t>(msg->data.at(1))));
        velocity_msg.heading_rate = velocity_msg.longitudinal_velocity * std::tan(current_steer_rad) / wheel_base_;  // [rad/s]

        velocity_status_pub_->publish(velocity_msg);

        break;
      }
    case 0x40: {
        autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
        control_mode_msg.stamp = msg->header.stamp;
        control_mode_msg.mode = (static_cast<int16_t>(msg->data.at(1)));

        control_mode_pub_->publish(control_mode_msg);

        break;
      }
    case 0x41: {        
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport indicator_status_msg;
        indicator_status_msg.stamp = msg->header.stamp;
        indicator_status_msg.report = (static_cast<int16_t>(msg->data.at(1)));

        turn_indicators_status_pub_->publish(indicator_status_msg);

        break;
      }
    case 0x42: {
        autoware_auto_vehicle_msgs::msg::GearReport gear_status_msg;
        gear_status_msg.stamp = msg->header.stamp;
        gear_status_msg.report = (static_cast<int16_t>(msg->data.at(1)));

        gear_status_pub_->publish(gear_status_msg);

        break;
      }
    case 0x43: {
        autoware_auto_vehicle_msgs::msg::HazardLightsReport hazardlight_status_msg;
        hazardlight_status_msg.stamp = msg->header.stamp;
        hazardlight_status_msg.report = (static_cast<int16_t>(msg->data.at(1)));

        hazard_light_status_pub_->publish(hazardlight_status_msg);

        break;
      }
    default: {
        break;
      }
    }
  }
}  // namespace hkpc_interface

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hkpc_interface::HKPCInterface)
