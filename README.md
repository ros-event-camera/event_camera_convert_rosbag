# event_camera_convert_rosbag

Conversion tool to change the message type of ROS/ROS2 bags from
``event_array_msgs::EventArray`` to
``event_camera_msgs::EventPacket``.

This package is built like any other ROS package (copy in workspace
under src and rebuild workspace).

To convert bag:
```
rosrun event_camera_convert_rosbag convert -i <name_of_input_bag> -o <name_of_converted_bag>
```

## License

This software is issued under the Apache License Version 2.0.
