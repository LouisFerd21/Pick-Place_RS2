# import launch
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
# from launch_ros.substitutions import FindPackageShare

# def get_robot_description():
#     return {
#         "robot_description": Command([
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
#             " ",
#             "ur_type:=ur3",
#             " ",
#             "name:=ur3",
#             " ",
#             "prefix:=''",
#             " ",
#             "robot_ip:=fake_ip_for_sim",
#             " ",
#             "safety_limits:=true",
#             " ",
#             "safety_pos_margin:=0.15",
#             " ",
#             "safety_k_position:=20",
#             " ",
#             "joint_limit_params:=",
#             PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3", "joint_limits.yaml"]),
#             " ",
#             "kinematics_params:=",
#             PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3", "default_kinematics.yaml"]),
#             " ",
#             "physical_params:=",
#             PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3", "physical_parameters.yaml"]),
#             " ",
#             "visual_params:=",
#             PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3", "visual_parameters.yaml"]),
#         ])
#     }

# def get_robot_description_semantic():
#     return {
#         "robot_description_semantic": Command([
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
#             " ",
#             "name:=ur",
#             " ",
#             "prefix:=''",
#         ])
#     }

# def get_kinematics_config():
#     return {
#         "robot_description_kinematics": PathJoinSubstitution([
#             FindPackageShare("ur_moveit_config"),
#             "config", "kinematics.yaml"  # ✅ Ensure this matches the actual file name
#         ])
#     }

# def generate_launch_description():
#     return launch.LaunchDescription([
#         Node(
#             package="positionconv_package",
#             executable="position_subscriber",
#             name="position_subscriber",
#             output="screen",
#             parameters=[
#                 get_robot_description(),
#                 get_robot_description_semantic(),
#                 get_kinematics_config()  # ✅ Kinematics now included!
#             ]
#         )
#     ])

import yaml
from ament_index_python.packages import get_package_share_directory
import os
import launch
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    joint_limit_params = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", "ur3", "joint_limits.yaml"
    ])
    kinematics_params = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", "ur3", "default_kinematics.yaml"
    ])
    physical_params = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", "ur3", "physical_parameters.yaml"
    ])
    visual_params = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", "ur3", "visual_parameters.yaml"
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ",
        "robot_ip:=192.168.56.101",
        " ",
        "joint_limit_params:=", joint_limit_params,
        " ",
        "kinematics_params:=", kinematics_params,
        " ",
        "physical_params:=", physical_params,
        " ",
        "visual_params:=", visual_params,
        " ",
        "safety_limits:=true",
        " ",
        "safety_pos_margin:=0.15",
        " ",
        "safety_k_position:=20",
        " ",
        "name:=ur",
        " ",
        "ur_type:=ur3",
        " ",
        "prefix:=''"
    ])

    return {"robot_description": robot_description_content}

def get_robot_description_semantic():
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " ",
        "name:=ur",
        " ",
        "prefix:=''"
    ])

    return {"robot_description_semantic": robot_description_semantic_content}

def get_kinematics_config():
    kinematics_path = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config", "kinematics.yaml"
    )

    with open(kinematics_path, "r") as f:
        kinematics_data = yaml.safe_load(f)

    return {"robot_description_kinematics": kinematics_data}

def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    robot_description_kinematics = get_kinematics_config()

    return launch.LaunchDescription([
        Node(
            package="positionconv_package",
            executable="position_subscriber",
            name="position_subscriber",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
            ],
        )
    ])
