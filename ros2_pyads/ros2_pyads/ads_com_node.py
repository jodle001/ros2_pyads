import pyads
import yaml

import rclpy
from rclpy.node import Node
from ros2_pyads.ads_com import ADSCom
from ros2_pyads_interfaces.srv import ReadBool, WriteBool


class ADSComNode(Node):
    """
    A ROS2 node that tests the ADS communication object by writing to a boolean variable in the PLC.
    """

    def __init__(self):
        super().__init__('ads_com_node',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Get the parameters from the launch file
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

        # Load the PLC admin data from the YAML file
        with open(plc_admin, 'r') as file:
            plc_admin_data = yaml.safe_load(file)

        # Initialize the ADS communication object
        self.ads_com = ADSCom(com_config_data, plc_admin_data)

        # Create Service Servers
        self.srv_read_bool = self.create_service(ReadBool, self.get_name() + '/read_bool', self.read_bool_callback)
        self.srv_write_bool = self.create_service(WriteBool, self.get_name() + '/write_bool', self.write_bool_callback)

    def read_bool_callback(self, request, response):
        """
        Reads a boolean variable from the PLC.

        :param request: The request object containing the tag name to read.
        :param response: The response object containing the read value.
        """
        try:
            response.tag_value = self.ads_com.read_by_name(request.tag_name, pyads.PLCTYPE_BOOL)
            response.success = True
            response.msg = "Successfully read bool."
        except Exception as e:
            self.get_logger().error(f"Failed to read bool: {e}")
            response.success = False
            response.msg = str(e)
        return response

    def write_bool_callback(self, request, response):
        """
        Writes a boolean variable to the PLC.

        :param request: The request object containing the tag name and value to write.
        :param response: The response object contains the success status and message.
        """
        try:
            self.ads_com.write_by_name(request.tag_name, request.tag_value, pyads.PLCTYPE_BOOL)
            response.tag_list = request.tag_name  # assuming this is what you intended with 'tag_list'
            response.success = True
            response.msg = f"Successfully wrote {request.tag_value} to {request.tag_name}."
        except Exception as e:
            self.get_logger().error(f"Failed to write bool: {e}")
            response.success = False
            response.msg = str(e)
        return response


def main():
    rclpy.init()
    node = ADSComNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
