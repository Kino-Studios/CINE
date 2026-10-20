from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mlp_weights_path',
                default_value='',
                description=(
                    'Directory containing mlp_correction.onnx and scaler.json; '
                    'empty means share/cine_mlp_correction/weights'
                ),
            ),
            Node(
                package='cine_mlp_correction',
                executable='mlp_node',
                name='mlp_correction_node',
                parameters=[
                    {
                        'mlp_weights_path': LaunchConfiguration('mlp_weights_path'),
                    }
                ],
            ),
        ]
    )
