from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mlp_weights_path",
                description=(
                    "Absolute path to the mlp/weights/ directory "
                    "containing mlp_correction.onnx and scaler.json"
                ),
            ),
            Node(
                package="cine_mlp_correction",
                executable="mlp_node",
                name="mlp_correction_node",
                parameters=[
                    {
                        "mlp_weights_path": LaunchConfiguration("mlp_weights_path"),
                    }
                ],
            ),
        ]
    )
