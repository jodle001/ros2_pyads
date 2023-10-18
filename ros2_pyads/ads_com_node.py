import yaml
import rclpy
from rclpy.node import Node
from ros2_pyads.ads_com import AdsCom
import pyads


class ADSComNode(Node):

    def __init__(self):
        super().__init__('ads_com_node',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Get the parameters
        com_config = self.get_parameter('com_config').value
        plc_admin = self.get_parameter('plc_admin').value

        if not com_config:
            self.get_logger().fatal('Failed to get "com_config" parameter')
            exit(2)

        if not plc_admin:
            self.get_logger().fatal('Failed to get "plc_admin" parameter')

        # Load the config data from the YAML file
        with open(com_config, 'r') as file:
            com_config_data = yaml.safe_load(file)

        with open(plc_admin, 'r') as file:
            plc_admin_data = yaml.safe_load(file)

        # Initialize the ADS communication object
        self.ads_com = AdsCom(com_config_data, plc_admin_data)

        self.ads_com.write_by_name('MAIN.testVar', 5, pyads.PLCTYPE_DWORD)


def main():
    rclpy.init()
    node = ADSComNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
