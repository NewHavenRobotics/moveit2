/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Keyboard_Autonomy");

bool transformPose(const geometry_msgs::msg::PoseStamped& input_pose_stamped, 
                   const std::string& target_frame, 
                   tf2_ros::Buffer& tf_buffer, 
                   geometry_msgs::msg::PoseStamped& output_pose_stamped)
{
  try
  {
    output_pose_stamped = tf_buffer.transform(input_pose_stamped, target_frame, tf2::durationFromSec(1.0));
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Transform failed: %s", ex.what());
    return false;
  }
}

bool planAndExecute(const geometry_msgs::msg::PoseStamped& target_pose_stamped, 
                    moveit::planning_interface::MoveGroupInterface& move_group)
{
  move_group.setPoseTarget(target_pose_stamped);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    return false;
  }

  success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    RCLCPP_ERROR(LOGGER, "Execution failed");
    return false;
  }

  return true;
}

void populatePose(geometry_msgs::msg::PoseStamped& pose_stamped, 
                  const std::string& frame_id, 
                  double x, double y, double z)
{
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp = rclcpp::Time(0);
  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;
  pose_stamped.pose.position.z = z;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, -M_PI_2, 0); // -90 degrees in pitch
  pose_stamped.pose.orientation.x = quaternion.x();
  pose_stamped.pose.orientation.y = quaternion.y();
  pose_stamped.pose.orientation.z = quaternion.z();
  pose_stamped.pose.orientation.w = quaternion.w();
}

void publishButton(const rclcpp::Node::SharedPtr& node, const std::string& topic, int duration_seconds)
{
  auto publisher = node->create_publisher<std_msgs::msg::Int32>(topic, 10);

  std_msgs::msg::Int32 msg;
  msg.data = 1;
  publisher->publish(msg); // Publish a single 1
  std::this_thread::sleep_for(std::chrono::seconds(duration_seconds)); // Wait for the specified duration

  msg.data = 0;
  publisher->publish(msg); // Publish a single 0
  RCLCPP_INFO(LOGGER, "Finished publishing to %s.", topic.c_str());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto keyboard_autonomy_node = rclcpp::Node::make_shared("keyboard_autonomy_node", node_options);

  const std::string PLANNING_GROUP = "arm";
  robot_model_loader::RobotModelLoader robot_model_loader(keyboard_autonomy_node, "robot_description");
  const auto& robot_model = robot_model_loader.getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const auto* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface move_group(keyboard_autonomy_node, PLANNING_GROUP);

  // Initialize TF2 buffer and listener
  tf2_ros::Buffer tf_buffer(keyboard_autonomy_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  RCLCPP_INFO(LOGGER, "Defining key poses...");

  geometry_msgs::msg::PoseStamped key1_pose_stamped;
  populatePose(key1_pose_stamped, "gripper_camera_link", -0.1934, 0.5668, -0.0622);

  geometry_msgs::msg::PoseStamped key2_pose_stamped;
  populatePose(key2_pose_stamped, "gripper_camera_link", 0.1934, 0.5668, -0.0622);

  RCLCPP_INFO(LOGGER, "Transforming key poses to 'arm_base_link' frame...");

  // Transform key poses to arm_base_link frame
  geometry_msgs::msg::PoseStamped key1_transformed_pose;
  geometry_msgs::msg::PoseStamped key2_transformed_pose;
  if (!transformPose(key1_pose_stamped, "arm_base_link", tf_buffer, key1_transformed_pose) || 
      !transformPose(key2_pose_stamped, "arm_base_link", tf_buffer, key2_transformed_pose))
  {
    RCLCPP_ERROR(LOGGER, "Failed to transform poses.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Planning and executing motion to key1 pose...");

  // Plan and execute the first pose
  if (!planAndExecute(key1_transformed_pose, move_group))
  {
    RCLCPP_ERROR(LOGGER, "Failed to plan and execute motion to key1 pose.");
    rclcpp::shutdown();
    return 1;
  }

  // Publish to /transmitter/button0
  RCLCPP_INFO(LOGGER, "Publishing to /transmitter/button0...");
  publishButton(keyboard_autonomy_node, "/transmitter/button0", 3);

  // Publish to /transmitter/button1
  RCLCPP_INFO(LOGGER, "Publishing to /transmitter/button1...");
  publishButton(keyboard_autonomy_node, "/transmitter/button1", 3);

  RCLCPP_INFO(LOGGER, "Planning and executing motion to key2 pose...");

  // Plan and execute the second pose
  if (!planAndExecute(key2_transformed_pose, move_group))
  {
    RCLCPP_ERROR(LOGGER, "Failed to plan and execute motion to key2 pose.");
    rclcpp::shutdown();
    return 1;
  }

  // Publish to /transmitter/button0
  RCLCPP_INFO(LOGGER, "Publishing to /transmitter/button0...");
  publishButton(keyboard_autonomy_node, "/transmitter/button0", 3);

  // Publish to /transmitter/button1
  RCLCPP_INFO(LOGGER, "Publishing to /transmitter/button1...");
  publishButton(keyboard_autonomy_node, "/transmitter/button1", 3);

  RCLCPP_INFO(LOGGER, "Motion execution completed successfully.");
  rclcpp::shutdown();
  return 0;
}
