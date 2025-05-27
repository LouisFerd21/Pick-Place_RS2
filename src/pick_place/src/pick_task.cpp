#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <map>
#include <string>
 
using moveit::planning_interface::MoveGroupInterface;
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto const node = rclcpp::Node::make_shared("ur3_pick_tile_node");
 
    MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
 
    // Define joint positions for intermediate waypoints
    std::map<std::string, double> original_pose = {
        {"shoulder_pan_joint", -1.6306},
        {"shoulder_lift_joint", -1.8501},
        {"elbow_joint", -1.2195},
        {"wrist_1_joint", -1.5818},
        {"wrist_2_joint", -4.6681},
        {"wrist_3_joint", 1.4268}
    };
 
    std::map<std::string, double> waypoint_1 = {
        {"shoulder_pan_joint", -0.9825},
        {"shoulder_lift_joint", -2.1929},
        {"elbow_joint", -0.8825},
        {"wrist_1_joint", -1.5872},
        {"wrist_2_joint", -4.6686},
        {"wrist_3_joint", 1.4268}
    };
 
    std::map<std::string, double> waypoint_2 = {
        {"shoulder_pan_joint", -1.3401},
        {"shoulder_lift_joint", -2.2696},
        {"elbow_joint", -1.0022},
        {"wrist_1_joint", -1.3498},
        {"wrist_2_joint", -4.6592},
        {"wrist_3_joint", 1.4268}
    };
 
    std::map<std::string, double> waypoint_3 = {
        {"shoulder_pan_joint", -1.6718},
        {"shoulder_lift_joint", -2.2698},
        {"elbow_joint", -1.0021},
        {"wrist_1_joint", -1.3498},
        {"wrist_2_joint", -4.6591},
        {"wrist_3_joint", 1.4268}
    };
 
    // Final tile positions by color
    std::map<std::string, std::map<std::string, double>> color_positions = {
        {"red", {
            {"shoulder_pan_joint", -1.1087},
            {"shoulder_lift_joint", -2.9230},
            {"elbow_joint", -0.4769},
            {"wrist_1_joint", -1.2894},
            {"wrist_2_joint", -4.6641},
            {"wrist_3_joint", 1.4278}
        }},
        {"black", {
            {"shoulder_pan_joint", -1.3868},
            {"shoulder_lift_joint", -2.9126},
            {"elbow_joint", -0.4580},
            {"wrist_1_joint", -1.2753},
            {"wrist_2_joint", -4.6591},
            {"wrist_3_joint", 1.4267}
        }},
        {"white", {
            {"shoulder_pan_joint", -1.6705},
            {"shoulder_lift_joint", -2.9127},
            {"elbow_joint", -0.4579},
            {"wrist_1_joint", -1.2754},
            {"wrist_2_joint", -4.6591},
            {"wrist_3_joint", 1.4268}
        }}
    };
 
    // Helper to move to a joint pose
    auto move_to = [&](const std::map<std::string, double>& joints, const std::string& label) {
        move_group.setJointValueTarget(joints);
        auto success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
            RCLCPP_INFO(node->get_logger(), "Moved to %s", label.c_str());
        else
            RCLCPP_WARN(node->get_logger(), "Failed to move to %s", label.c_str());
        return success;
    };
 
    // Example: pick a red tile
    std::string selected_color = "red"; // can be "red", "black", or "white"
 
    // Execute movements
    move_to(original_pose, "original_pose");
    move_to(waypoint_1, "waypoint_1");
    move_to(waypoint_2, "waypoint_2");
    move_to(waypoint_3, "waypoint_3");
    move_to(color_positions[selected_color], selected_color + " tile pick");
 
    // Trigger magnet/gripper ON here (if applicable)
 
    rclcpp::shutdown();
    return 0;
}

 
 

