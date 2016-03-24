3-21 playing two wheel robot

## Demo tests
In different terminals:

launch the gazebo environment setting (parameter values are default):

		roslaunch swarm_robot_description initialize.launch robot_quantity:=10 half_range:=1.0

launch the controllers for two wheel robot:

		roslaunch swarm_robot_controller two_wheel_robot_controller.launch

test 1: dispersion (parameter values are default):

		rosrun swarm_robot_simulation two_wheel_dispersion _spring_length:=0.7 _wheel_speed:=2.0

test 2: line formation (parameter values are default)

		rosrun swarm_robot_simulation two_wheel_line_formation _sensing_range:=3.0 _spring_length:=0.7 _wheel_speed:=2.0

test 3: driving_single_robot (parameter values are default)

		rosrun swarm_robot_simulation driving_single_robot _sensing_range:=3.0 _spring_length:=0.7 _wheel_speed:=2.0

******************************************
Passing command line parameters to rosrun
******************************************
It doesn't seem to be very clearly explained in the documentation, or rather, finding out how to do this is a matter of piecing together information from several sources. Suppose that you want to pass some parameters when using rosrun. In my case I want to pass Phidget device serial numbers so that multiple devices of the same type can be connected and used.

The usual way to run a node is something like this:

    rosrun [package] [node] 

To pass a parameter, use the following syntax:

    rosrun [package] [node] _param:=value

Where param is the name of the parameter or option.

So in my case to pass the device serial number I do the following.

    rosrun phidgets advanced_servo _serial:=88209

Then within the code for the advanced_servo node the value can be accessed like this.

    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    ROS_INFO("Device serial number %d", serial_number);

And that's all there is to it. Passing a value of -1 for the serial number just connects to the first device which can be found. 
