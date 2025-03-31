#include <cstdio>

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("keyboard_autonomy");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> keyboard_autonomy_node =
      rclcpp::Node::make_shared("keyboard_autonomy", node_options);

  // instantiate the robot model
  const std::string PLANNING_GROUP = "arm";
  robot_model_loader::RobotModelLoader robot_model_loader(keyboard_autonomy_node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  if (!joint_model_group)
  {
      RCLCPP_ERROR(LOGGER, "JointModelGroup for planning group '%s' is null. Check your MoveIt configuration.", PLANNING_GROUP.c_str());
      return 1;
  }

  // use robot model to instantiate the planning scene
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "folded");

  if (!planning_scene)
  {
      RCLCPP_ERROR(LOGGER, "Planning scene is null");
      return 1;
  }

  // Update the robot state with the latest joint states
  planning_scene->getCurrentStateNonConst().update();

  // Debugging: Check if the JointState is populated
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);

  if (joint_values.empty())
  {
      RCLCPP_ERROR(LOGGER, "JointState is empty. Ensure the robot state is properly initialized.");
  }
  else
  {
      RCLCPP_INFO(LOGGER, "JointState received: %zu joints", joint_values.size());
  }

  // load a planning plugin
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name = "ompl_interface/OMPLPlanner"; // Set the desired planning plugin

  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, keyboard_autonomy_node, keyboard_autonomy_node->get_namespace()))
      RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
    RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls : classes)
      ss << cls << " ";
    RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                ex.what(), ss.str().c_str());
  }

  RCLCPP_INFO(LOGGER, "Using planner plugin: %s", planner_plugin_name.c_str());

  moveit::planning_interface::MoveGroupInterface move_group(keyboard_autonomy_node, PLANNING_GROUP);

  // Ensure the start state is set to the current state
  move_group.setStartStateToCurrentState();

  // Define a goal pose
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "arm_base_link";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  RCLCPP_INFO(LOGGER, "Pose set to: x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z); // debug

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  RCLCPP_INFO(LOGGER, "Tolerance set to: %f", tolerance_pose[0]); // debug

  // Construct goal constraints
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("wrist3_link", pose, tolerance_pose, tolerance_angle);

  RCLCPP_INFO(LOGGER, "Setting up motion plan request"); // debug
  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  RCLCPP_INFO(LOGGER, "Planning group: %s", req.group_name.c_str());
  RCLCPP_INFO(LOGGER, "Number of goal constraints: %zu", req.goal_constraints.size());

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  RCLCPP_INFO(LOGGER, "Constructing planning context"); // debug
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  if (!context)
  {
      RCLCPP_ERROR(LOGGER, "Failed to create planning context");
      return 1;
  }
  if (res.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
      RCLCPP_ERROR(LOGGER, "Error code: %d", res.error_code_.val);
      return 1;
  }

  context->solve(res);
  
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    return 0;
  }

  printf("ooooooh donna parker pt 4\n");

  rclcpp::spin(keyboard_autonomy_node);
  rclcpp::shutdown();
  
  return 0;
}
