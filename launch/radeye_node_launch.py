from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the name of your node
    node_name = 'radeye'

    return LaunchDescription([
        Node(
            package='radeye',
            executable='radeye_node',
            name=node_name,
            output='screen',
            emulate_tty=True,
            parameters=[{"serial_port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10LX3I0-if00-port0"
                         }]
        )
    ])