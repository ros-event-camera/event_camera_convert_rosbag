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
from event_array_msgs::EventArray
to event_camera_msgs::EventPacket
"""

import time
import argparse
import rosbag


source_type = 'event_array_msgs/EventArray'
target_type = 'event_camera_msgs/EventPacket'


def main(args):
    in_bag_name = str(args.in_bag)
    out_bag_name = str(args.out_bag)

    print(f'converting bag {in_bag_name} to {out_bag_name}')

    in_bag = rosbag.Bag(in_bag_name)
    out_bag = rosbag.Bag(out_bag_name, 'w')
    topics = in_bag.get_type_and_topic_info()[1]
    print('changing type for topics: ',
          list(dict(filter(lambda p:  p[1][0] == source_type, topics.items())).keys()))

    start_time = time.time()
    num_msgs = 0

    for topic, msg, t in in_bag.read_messages(raw=True):
        new_msg = (target_type, ) + msg[1:] if msg[0] == source_type else msg
        out_bag.write(topic, new_msg, t=t, raw=True)
        num_msgs += 1
    in_bag.close()
    out_bag.close()
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
