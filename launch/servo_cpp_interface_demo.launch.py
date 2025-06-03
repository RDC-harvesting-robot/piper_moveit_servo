import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder

def generate_launch_description():

    pkg_piper = get_package_share_directory("piper_with_gripper_moveit")
    pkg_piper_moveit_servo = get_package_share_directory("piper_moveit_servo")

    urdf_path = os.path.join(pkg_piper, "config", "piper.urdf.xacro")
    srdf_path = os.path.join(pkg_piper, "config", "piper.srdf")
    kinematics_yaml = os.path.join(pkg_piper, "config", "kinematics.yaml")

    from xacro import process_file
    robot_description = {"robot_description": process_file(urdf_path).toxml()}


    with open(srdf_path, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # kinematics.yamlも読ませたい場合
    # with open(kinematics_yaml, 'r') as f:
    #     kinematics = yaml.safe_load(f)
    #     robot_description_kinematics = {"robot_description_kinematics": kinematics}

   
    servo_params = (
        ParameterBuilder("piper_moveit_servo")
        .yaml(parameter_namespace="moveit_servo", file_path="config/piper_simulated_config.yaml")
        .to_dict()
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    servo_node = Node(
        package="piper_moveit_servo",
        executable="piper_servo_cpp_interface_demo",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
    )



    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz_config_file = os.path.join( pkg_piper_moveit_servo, "config", "piper_demo_rviz_config.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    ros2_control_config = os.path.join(pkg_piper, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_control_config],
        output="both",
    )

    load_controllers = []
    for controller in ["piper_servo_arm_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [rviz_node, static_tf, servo_node, ros2_control_node, robot_state_publisher]
        + load_controllers
    )
