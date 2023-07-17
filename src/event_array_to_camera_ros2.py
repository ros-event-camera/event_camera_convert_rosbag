#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

"""
event_array_to_camera.

converts message type
from event_array_msgs::msg::EventArray
to event_camera_msgs::msg::EventPacket
"""

import time
import rosbag2_py
import argparse


source_type = 'event_array_msgs/msg/EventArray'
target_type = 'event_camera_msgs/msg/EventPacket'


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def main(args):
    in_bag_name = str(args.in_bag)
    out_bag_name = str(args.out_bag)

    print(f'converting bag {in_bag_name} to {out_bag_name}')

    storage_options, converter_options = get_rosbag_options(in_bag_name)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    writer = rosbag2_py.SequentialWriter()
    writer_storage_options = rosbag2_py._storage.StorageOptions(
        uri=out_bag_name,
        storage_id=storage_options.storage_id)
    writer.open(writer_storage_options, converter_options)

    # simply modifying the writer topic type string seems to do the trick
    for topic in topic_types:
        writer_topic = rosbag2_py._storage.TopicMetadata(
            name=topic.name,
            type=(target_type if topic.type == source_type else topic.type),
            serialization_format=topic.serialization_format)
        if topic.type == source_type:
            print(f'converting topic: {topic.name} to type {target_type}')
        writer.create_topic(writer_topic)

    start_time = time.time()
    num_msgs = 0

    while reader.has_next():
        (topic, data, t_rec) = reader.read_next()
        num_msgs += 1
        writer.write(topic, data, t_rec)

    dt = time.time() - start_time
    print(f'total time: {dt:.2f}s, processed {num_msgs / dt:.2f} msgs/s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='convert rosbag event_array_msgs -> event_camera_msgs')
    parser.add_argument('--in_bag', '-i', action='store', default=None,
                        required=True, help='bag file to read events from')
    parser.add_argument('--out_bag', '-o', action='store', default=None,
                        required=True, help='bag file to write events to')
    parser.set_defaults(use_sensor_time=False)
    main(parser.parse_args())
