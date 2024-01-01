#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER);
  const moveit::core::JointModelGroup *joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper = move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper = move_group_gripper.getCurrentState(10);
  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);
  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");
  // joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -2.50; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = -1.50; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2
  // joint_group_positions_arm[5] = 0.00;  // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  // Pregrasp
  /*
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  joint_group_positions_arm[0] = -0.37;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.35;  // Shoulder Lift
  joint_group_positions_arm[2] = 1.65;   // Elbow
  joint_group_positions_arm[3] = -1.8;   // Wrist 1
  joint_group_positions_arm[4] = -1.7;   // Wrist 2
  joint_group_positions_arm[5] = 1.1226; // Wrist 3
  */
  
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  joint_group_positions_arm[0] = 3.5;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.57;  // Shoulder Lift
  joint_group_positions_arm[2] = -1.57;   // Elbow
  joint_group_positions_arm[3] = -1.57;   // Wrist 1
  joint_group_positions_arm[4] = 1.57;   // Wrist 2
  joint_group_positions_arm[5] = 1.937991; // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    move_group_arm.execute(my_plan);
  } else {
    RCLCPP_ERROR(LOGGER, "Joint movement failed!");
  }


  RCLCPP_INFO(LOGGER, "Open Gripper!");
  joint_group_positions_gripper[2] = 0.0;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);
  RCLCPP_INFO(LOGGER, "Approach to object!");
    geometry_msgs::msg::Pose target_pose1 = move_group_arm.getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    target_pose1.position.z -= 0.13;
    target_pose1.position.x += 0.014;
    approach_waypoints.push_back(target_pose1);
    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_arm.computeCartesianPath(
        approach_waypoints, eef_step, jump_threshold, trajectory_approach);
    move_group_arm.execute(trajectory_approach);

  
    RCLCPP_INFO(LOGGER, "Close Gripper!");
    joint_group_positions_gripper[2] = 0.657; //0.655
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper1;
    bool success_gripper1 = (move_group_gripper.plan(my_plan_gripper1) ==
                            moveit::core::MoveItErrorCode::SUCCESS);
    move_group_gripper.execute(my_plan_gripper1);
    RCLCPP_INFO(LOGGER, "Retreat from object!");
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    target_pose1.position.z += 0.14;
    retreat_waypoints.push_back(target_pose1);
    moveit_msgs::msg::RobotTrajectory trajectory_retreat;
    fraction = move_group_arm.computeCartesianPath(
        retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);
    move_group_arm.execute(trajectory_retreat);


      RCLCPP_INFO(LOGGER, "Rotating Arm");
      current_state_arm = move_group_arm.getCurrentState(10);
      current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                 joint_group_positions_arm);
      joint_group_positions_arm[0] = 1.0;
      move_group_arm.setJointValueTarget(joint_group_positions_arm);
      success_arm = (move_group_arm.plan(my_plan_arm) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
      move_group_arm.execute(my_plan_arm);
      RCLCPP_INFO(LOGGER, "Release Object!");
      move_group_gripper.setNamedTarget("open");
      success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                         moveit::core::MoveItErrorCode::SUCCESS);
      move_group_gripper.execute(my_plan_gripper);
      

  rclcpp::shutdown();
  return 0;
}