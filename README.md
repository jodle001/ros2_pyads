# ros2_pyads
This is a ros2 package that will be a wrapper around the pyads library to have communication between ros2 and beckhoff PLCs.

# Dependencies
Please view the [requirements.txt](ros2_pyads/requirements.txt) file for the python dependencies.

pyads is the main dependency and can be installed with pip:

* [pyads](https://pyads.readthedocs.io/en/latest/)

This is a ros2 python package, so ROS2 Humble will also need to be installed:
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)


# Installation

Open a terminal and navigate to the src directory of your ros2 workspace.

```bash
cd /opt/ros2_ws/
```

Clone the repository into the src directory.

```bash
git clone https://github.com/jodle001/ros2_pyads.git src/ros2_pyads
```

Install the dependencies with pip.

```bash
pip3 install -r src/ros2_pyads/requirements.txt
```

Build the package.

```bash
colcon build --packages-select ros2_pyads
```

# Configuration

## Communications:

Read the [com_config.yaml](ros2_pyads/config/com_config.yaml) file in the config directory carefully and set up the inputs, here is an example:

```yaml
# This configuration file is used to set up the pyads library objects to establish communication between
# a machine with ROS2 and a machine running the TwinCat PLC Software.

# The IP address should match the machine with ROS2 with the
# added '.1.1' on the end of the IP address.
sender_ams: '192.168.33.10.1.1'
# This is the IP address of the host machine with the TwinCat Software
plc_ip: '192.168.33.4'
# This is arbitrarily named
route_name: 'ROS2PLC'
# This should match the plc_ip address
host_name: '192.168.33.4'
# This is gotten from the TwinCat System About menu "AMS Net id"
remote_ads: '10.100.101.14.1.1'
```

## Administrator Access:

To have the pyads library to successfully communicate with the Beckhoff Software, it needs to have administrator access. 
This can be done by creating a local user with admin access.

**Create and fill out a yaml for PLC Admin Access**
* Create a yaml called `plc_admin.yaml` in the config directory
* This file is not tracked because it contains sensitive information, so it will not be pushed to the repository.
* Fill out the yaml with the following, replacing the values with your own:
```yaml
plc_admin_user: 'youreadminuser'
plc_admin_pass: 'adminpassword'
```

# Usage

## Running the Bool Test

This is a simple test to see if the communication is working. 
It will read a bool from the PLC and then write the opposite value back to the PLC.

1. For this to work, add a boolean variable in the MAIN program of the PLC with the name `bTest`:

    ```c
    PROGRAM MAIN
    VAR
        bTest : BOOL;
    END_VAR
    ```

2. Then build the solution and run the PLC with the TwinCat software, and login to the PLC to view the variable 
which should start as false.

   You should see the following:

   ![bool_false](images/bool_false.png)

3. After the PLC is running, open a terminal and source the ros2 workspace, and launch the
[ads_com_bool_test.launch.py](ros2_pyads/launch/ads_com_bool_test.launch.py) launch file.

    ```bash
    source /opt/ros2_ws/install/setup.bash
    ros2 launch ros2_pyads ads_com_bool_test.launch.py
    ```

4. The terminal should output the following:

    ```bash
   username:~$ ros2 launch ros2_pyads ads_com_bool_test.launch.py 
   [INFO] [launch]: All log files can be found below /home/username/.ros/log/2023-10-18-16-14-47-618657-OROC-LINUX01-3370411
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [ads_com_bool_test_node-1]: process started with pid [3370412]
   [ads_com_bool_test_node-1] 2023-10-18T16:14:47-0400 Info: Connected to 192.168.33.4
   [ads_com_bool_test_node-1] 2023-10-18T16:14:47-0400 Info: connection closed by remote
   [ads_com_bool_test_node-1] 2023-10-18T16:14:47-0400 Info: Connected to 192.168.33.4
   [ads_com_bool_test_node-1] 2023-10-18T16:14:47-0400 Info: connection closed by remote
   [INFO] [ads_com_bool_test_node-1]: process has finished cleanly [pid 3370412]
    ```
   
   And the boolean in the Beckhoff PLC should toggle each time you run this:
   
   ![bool_true](images/bool_true.png)


## Using Services

In order to call the services the ads_com_node needs to be running:

```bash
ros2 launch ros2_pyads ads_com.launch.py
```

Once the node is running, you can call the services with the following commands:

* Reading a bool:
   ```bash
    ros2 service call /ads_com_node/read_bool ros2_pyads_interfaces/srv/ReadBool "{tag_name: 'MAIN.bTest'}"
   ```
* Writing a bool:
   ```bash
   ros2 service call /ads_com_node/write_bool ros2_pyads_interfaces/srv/WriteBool "{tag_name: 'MAIN.bTest', tag_value: true}"
   ```
  
Other service will work the same way to view them just run the following command:

```bash 
ros2 service list | grep -i "ads_com_node"
```
