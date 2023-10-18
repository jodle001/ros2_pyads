import pyads


class AdsCom:

    def __init__(self, com_config, plc_admin):
        self.com_config = com_config
        self.plc_admin = plc_admin

        # Get the parameters from the config file
        self.sender_ams = self.com_config['sender_ams']
        self.plc_ip = self.com_config['plc_ip']
        self.route_name = self.com_config['route_name']
        self.host_name = self.com_config['host_name']
        self.remote_ads = self.com_config['remote_ads']

        # Get Admin credentials
        self.plc_admin_user = self.plc_admin['plc_admin_user']
        self.plc_admin_pass = self.plc_admin['plc_admin_pass']

        # Initialize the route
        self.initialize_route()

    def initialize_route(self):
        pyads.open_port()
        pyads.set_local_address(self.sender_ams)
        pyads.add_route_to_plc(
            sending_net_id=self.sender_ams,
            adding_host_name=self.host_name,
            ip_address=self.plc_ip,
            username=self.plc_admin_user,
            password=self.plc_admin_pass,
            route_name=self.route_name)
        pyads.close_port()

    def read_by_name(self, var_name, var_type):
        with pyads.Connection(self.remote_ads, pyads.PORT_TC3PLC1, self.plc_ip) as plc:
            var = plc.read_by_name(var_name, var_type)
        return var

    def write_by_name(self, var_name, var_value, var_type):
        with pyads.Connection(self.remote_ads, pyads.PORT_TC3PLC1, self.plc_ip) as plc:
            plc.write_by_name(var_name, var_value, var_type)
