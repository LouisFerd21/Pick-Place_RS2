#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>
#include <queue>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <cctype>

class PositionSubscriber : public rclcpp::Node
{
public:
    PositionSubscriber()
    : Node("position_subscriber"), executing_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Setting up node...");

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "target_waypoints", 10,
            std::bind(&PositionSubscriber::position_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PositionSubscriber::process_next_waypoint, this));

        post_init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                post_init_timer_->cancel();  // run only once
                this->initialize_move_group();
            }
        );
    }

private:
    void position_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received combined input: %s", msg->data.c_str());

        std::stringstream ss(msg->data);
        std::string item;
        std::vector<std::string> tokens;

        while (std::getline(ss, item, ',')) {
            item.erase(0, item.find_first_not_of(" \t\n\r"));
            item.erase(item.find_last_not_of(" \t\n\r") + 1);
            if (!item.empty()) tokens.push_back(item);
        }

        if (tokens.size() % 2 != 0) {
            RCLCPP_WARN(this->get_logger(), "Input does not contain an even number of tokens.");
            return;
        }

        for (size_t i = 0; i < tokens.size(); i += 2) {
            waypoint_queue_.emplace(tokens[i], tokens[i + 1]);
        }
    }

    void process_next_waypoint()
    {
        if (executing_ || waypoint_queue_.empty()) return;
        executing_ = true;

        try {
            auto [canvas_target, color_source] = waypoint_queue_.front();
            waypoint_queue_.pop();

            char column = std::toupper(canvas_target[0]);
            std::map<char, std::string> column_to_waypoint = {
                {'A', "waypoint_1"}, {'B', "waypoint_1"},
                {'C', "waypoint_2"}, {'D', "waypoint_2"},
                {'E', "waypoint_3"}, {'F', "waypoint_3"},
                {'G', "waypoint_4"}, {'H', "waypoint_4"}
            };

            // Prepare color-based transit points
            std::string waypoint_color = "waypoint_" + color_source;
            std::string exit_color = "exit_" + color_source;

            if (column_to_waypoint.count(column)) {
                std::string transit_canvas = column_to_waypoint[column];

                RCLCPP_INFO(this->get_logger(),
                    "Routine: original → %s → %s → %s → original → %s → %s → %s → original",
                    waypoint_color.c_str(), color_source.c_str(), exit_color.c_str(),
                    transit_canvas.c_str(), canvas_target.c_str(), transit_canvas.c_str());

                send_joint_goal("original_pose");
                send_joint_goal(waypoint_color);       // approach color
                send_joint_goal(color_source);         // pick
                send_joint_goal(exit_color);           // retreat from color
                send_joint_goal(waypoint_color);
                send_joint_goal("original_pose");
                send_joint_goal(transit_canvas);       // move near canvas
                send_joint_goal(canvas_target);        // place
                send_joint_goal(transit_canvas);       // retreat
                send_joint_goal("original_pose");

            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown canvas column in '%s'. Skipping.", canvas_target.c_str());
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in process_next_waypoint: %s", e.what());
        }

        executing_ = false;
    }



    void initialize_move_group()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            rclcpp::Node::shared_from_this(), "ur_manipulator");

        int retries = 0;
        while (move_group_interface_->getPlanningFrame().empty() && rclcpp::ok() && retries++ < 50) {
            RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface to be ready...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        if (!move_group_interface_->getPlanningFrame().empty()) {
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface ready! Frame: %s",
                        move_group_interface_->getPlanningFrame().c_str());

            add_basic_collision_environment();
            attach_tool_to_end_effector();
            send_joint_goal("original_pose");
        } else {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface failed to initialize.");
        }
    }

    void add_basic_collision_environment()
    {
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = move_group_interface_->getPlanningFrame();
        table.id = "table";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {1.2, 1.2, 0.1};

        geometry_msgs::msg::Pose table_pose;
        table_pose.position.x = 0.0;
        table_pose.position.y = 0.0;
        table_pose.position.z = -0.055;
        table_pose.orientation.w = 1.0;

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;

        planning_scene_interface_.applyCollisionObjects({table});
    }

    void attach_tool_to_end_effector()
    {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = "tool0";
        attached_object.object.header.frame_id = "tool0";
        attached_object.object.id = "extension_block";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.04, 0.04, 0.02};

        geometry_msgs::msg::Pose pose;
        pose.position.z = 0.01;
        pose.orientation.w = 1.0;

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose);
        attached_object.object.operation = attached_object.object.ADD;

        planning_scene_interface_.applyAttachedCollisionObject(attached_object);
    }

    void send_joint_goal(const std::string& location_name)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to: %s", location_name.c_str());

        std::string filename = "/home/louis/Pick-Place_RS2/joint_goals.yaml";
        YAML::Node config = YAML::LoadFile(filename);

        if (!config[location_name]) {
            RCLCPP_ERROR(this->get_logger(), "Location '%s' not found in YAML.", location_name.c_str());
            return;
        }

        std::map<std::string, double> joint_values;
        for (const auto& joint : config[location_name]) {
            joint_values[joint.first.as<std::string>()] = joint.second.as<double>();
        }

        move_group_interface_->setJointValueTarget(joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_interface_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed for %s", location_name.c_str());
        }
    }

    rclcpp::TimerBase::SharedPtr post_init_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::queue<std::pair<std::string, std::string>> waypoint_queue_;
    bool executing_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
