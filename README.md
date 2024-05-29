# event_camera_convert_rosbag

Conversion tool to change the message type of ROS/ROS2 bags from
``event_array_msgs::EventArray`` to
``event_camera_msgs::EventPacket``.

## How to build

Set the following shell variables:
```bash
repo=event_camera_convert_rosbag
url=https://github.com/ros-event-camera/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)


## How to use

To convert bag:
```
rosrun event_camera_convert_rosbag convert -i <name_of_input_bag> -o <name_of_converted_bag>
```

## License

This software is issued under the Apache License Version 2.0.
