### Controlling Rotor RPMs
This fork of ardupilot_gazebo provides ROS interface to control rotor speed of each rotor individually. This version is tied to `models/iris_with_ardupilot` which is configured to set rotation speed to be in between -838(MIN) to 838(MAX) inclusive. As such these values are baked into header and cpp files.

### ROS API
This plugin exposes channel ids with respect to iris model name(See models/iris\_with_ardupilot/model.sdf) as in

```
/iris_demo/chan_0
/iris_demo/chan_1
/iris_demo/chan_2
/iris_demo/chan_3
```

By default, the plugin is under sitl control and preserves the normal behavior. User can take control of rotor speeds by publishing a value for each rotor within MIN AND MAX. Publishing a value outside of MIN and MAX range will cause the plugin to be back under SITL control.

Ex: To set rotor speed for chanel 0 to 200 rpms, send following command

```
rostopic pub -1 /iris_demo/chan_0 std_msgs/Int32 200
```

To hand control of channel 0 back to SITL, 

```
rostopic pub -1 /iris_demo/chan_0 std_msgs/Int32 839
```
