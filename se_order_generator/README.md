# Order generator

The order generator has the task of the generation, acquisition and distribution of the order to ros. 
It is able to create custom and random orders, that get stored in a json file and also to read from an already created json file.

## Usage

The order generator is implemented as an ros-action. The action-client node is to be used during the whole workshop, while a corresponding action-server should be created by each group.
Like described in the ros tutorials (look at 
	[Actionserver tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29) and [Actionclient tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29)), you should start the server before the client. 

For the purpose of understanding the functionality of this module, a test action-server was written. Feel free to use it as a template for your own action server.

Simply call the launch file of the test action server first

```bash
roslaunch se_order_generator example_order_action_server.launch
```
and then the launch file for the action client.

```bash
roslaunch se_order_generator order_action_client_read.launch
```
A total of 3 launch files exist, which call the order action client, which itself uses the actual order generator. 
The functionality of the order generator consists of three main functions. 
- Read order from .json file
- Create custom order and save into .json file
- Create random order and save into .json file

To create custom order, call 

```bash
roslaunch se_order_generator order_generator_custom.launch
```
and for a random order, 

```bash
roslaunch se_order_generator order_generator_random.launch
```
Note that for the creation of an order, it is not necessary to previously call the action server launch file.

Since the action is started via launch files, some parameters can get adjusted, like the file path and name from which the order is acquired. To change the values of the parameters you may change the values of the arguments, either by changing them in the file directly or by calling one of the three launch files from a launch file made by you, whilst passing the desired values to the specified arguments. To achieve this, look into [Using arguments and parameters](https://campus-rover.gitbook.io/lab-notebook/campusrover-lab-notebook/faq/using-args-params-roslaunch).

In the following the launchfile for the action client is shown.

### Action-client launchfile (order_action_client_read.launch)

```xml
<launch>
  <!--Arguments-->
  <arg name="file_path" default="$(find se_order_generator)/json"/>
  <arg name="file_name" default="order"                          />
  <!--10 seconds is the default max time. Make sure to always change the time in your launch file-->
  <arg name="max_time"            default="10"  /> 
  <arg name="max_time_minutes"    default="0"   />
  <arg name="max_time_seconds"    default="10"  />
  <!--Box dimensions-->
  <arg name="height_large_boxes"  default="15"  />
  <arg name="width_large_boxes"   default="5"   />
  <arg name="length_large_boxes"  default="5"   />
  <arg name="height_small_boxes"  default="10"  />
  <arg name="width_small_boxes"   default="5"   />
  <arg name="length_small_boxes"  default="5"   />

  <!--Parameters-->
  <param name="file_path"         value="$(arg file_path)"    type="str"/> <!--file path for .json file-->
  <param name="file_name"         value="$(arg file_name)"    type="str"/> <!--json file name used to read or write order-->
  <!--The next parameters define the maximum amount of time that an order should take-->
  <param name="max_time"            value="$(arg max_time)"             type="string"/> <!--in seconds-->
  <param name="max_time_minutes"    value="$(arg max_time_minutes)"     type="int"   />
  <param name="max_time_seconds"    value="$(arg max_time_seconds)"     type="int"   />
  <!--Box dimensions-->
  <param name="height_large_boxes"  value="$(arg height_large_boxes)"   type="double"/>
  <param name="width_large_boxes"   value="$(arg width_large_boxes)"    type="double"/>
  <param name="length_large_boxes"  value="$(arg length_large_boxes)"   type="double"/>
  <param name="height_small_boxes"  value="$(arg height_small_boxes)"   type="double"/>
  <param name="width_small_boxes"   value="$(arg width_small_boxes)"    type="double"/>
  <param name="length_small_boxes"  value="$(arg length_small_boxes)"   type="double"/>

  <!--Start node-->
  <node pkg="se_order_generator" type="order_generator_client.py" name="order_action_client" output="screen"/>
</launch>
```
A brief description of the parameters may be useful for understanding the working method of the action-client:
- file_path - The path where the json files are stored
- file_name - Name of the json file. For example if the functionality is switched to read, then it is the file where the order is acquired.

An order should have an upper bound of time:
- max_time         - Maximum time allowed (in seconds)
- max_time_minutes - Maximum minutes allowed
- max_time_seconds - Maximum seconds allowed

Information about the boxes is defined in this launch file, with the given names being self explanatory.

In addition, the two remaining launch files can contain the next arguments:

- max_num_of_boxes - The total number of boxes per order.
- max_num_orders - One file can contain more than one order. This parameter sets the number of orders to be created. Naturally, it only works for the creation of orders.

To establish which arguments are actually used, check the according launch file.