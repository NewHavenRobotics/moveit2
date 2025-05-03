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
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Keyboard_Autonomy");

std::vector<float> position_array;

void keyboardPositionsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "Received new positions from /keyboard_positions.");
  position_array = msg->data;
}

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

std::vector<geometry_msgs::msg::PoseStamped> createTargetPoses(const std::string& frame_id, 
                                                               const std::vector<std::tuple<double, double, double>>& positions)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const auto& [x, y, z] : positions)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    populatePose(pose_stamped, frame_id, x, y, z);
    poses.push_back(pose_stamped);
  }
  return poses;
}

std::vector<std::tuple<double, double, double>> parsePositionsFromArray(const std::vector<float>& position_array)
{
  std::vector<std::tuple<double, double, double>> positions;

  if (position_array.empty())
  {
    RCLCPP_ERROR(LOGGER, "Position array is empty.");
    return positions;
  }

  size_t num_positions = static_cast<size_t>(position_array[0]);
  if (position_array.size() != 1 + num_positions * 3)
  {
    RCLCPP_ERROR(LOGGER, "Position array size does not match the expected format.");
    return positions;
  }

  for (size_t i = 0; i < num_positions; ++i)
  {
    double x = position_array[1 + i * 3];
    double y = position_array[2 + i * 3];
    double z = position_array[3 + i * 3];
    positions.emplace_back(x, y, z);
  }

  return positions;
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

  // Subscribe to /keyboard_positions
  auto subscription = keyboard_autonomy_node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/keyboard_positions", 10, keyboardPositionsCallback);

  RCLCPP_INFO(LOGGER, "Waiting for positions on /keyboard_positions...");
  while (rclcpp::ok() && position_array.empty())
  {
    rclcpp::spin_some(keyboard_autonomy_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(LOGGER, "Defining target poses...");

  // Parse positions from the array
  auto positions = parsePositionsFromArray(position_array);

  // Create target poses
  auto target_poses = createTargetPoses("gripper_camera_link", positions);

  RCLCPP_INFO(LOGGER, "Transforming target poses to 'arm_base_link' frame...");

  // Transform target poses to arm_base_link frame
  std::vector<geometry_msgs::msg::PoseStamped> transformed_poses;
  for (const auto& pose : target_poses)
  {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (!transformPose(pose, "arm_base_link", tf_buffer, transformed_pose))
    {
      RCLCPP_ERROR(LOGGER, "Failed to transform a pose.");
      rclcpp::shutdown();
      return 1;
    }
    transformed_poses.push_back(transformed_pose);
  }

  RCLCPP_INFO(LOGGER, "Planning and executing motions to target poses...");

  // Plan and execute motions to each target pose
  for (size_t i = 0; i < transformed_poses.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Planning and executing motion to target pose %zu...", i + 1);
    if (!planAndExecute(transformed_poses[i], move_group))
    {
      RCLCPP_ERROR(LOGGER, "Failed to plan and execute motion to target pose %zu.", i + 1);
      rclcpp::shutdown();
      return 1;
    }

    // Publish to /transmitter/button0 and /transmitter/button1 after each motion
    RCLCPP_INFO(LOGGER, "Publishing to /transmitter/button0...");
    publishButton(keyboard_autonomy_node, "/transmitter/button0", 3);

    RCLCPP_INFO(LOGGER, "Publishing to /transmitter/button1...");
    publishButton(keyboard_autonomy_node, "/transmitter/button1", 3);
  }

  RCLCPP_INFO(LOGGER, "Motion execution completed successfully.");
  rclcpp::shutdown();
  return 0;
}
