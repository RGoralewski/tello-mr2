# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction

import pandas as pd
import numpy as np
import cv2
import time
from datetime import datetime
from cv_bridge import CvBridge
from pathlib import Path


class TelloAccTester(Node):

    def __init__(self, out_dir):
        super().__init__('tello_acc_tester')
        self.subscription = self.create_subscription(
            FlightData,
            'flight_data',
            self.listener_callback,
            10)
        self.sub_image = self.create_subscription(
            Image,
            'image_raw',
            self.img_listener_callback,
            10)

        self.buffer = []
        self.traj_state = ''

        self.cli = self.create_client(TelloAction, 'tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

        self.out_dir = out_dir
        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")
        self.filenames = f'tello_acc_data_{dt_string}'

        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        path = self.out_dir / Path(f'{self.filenames}.mp4')
        self.video_out = cv2.VideoWriter(str(path), fourcc, 30, (960, 720))

        self.last_video_frame_time = time.time()
        self.frames_counter = 0

    def send_request(self, cmd):
        self.traj_state = cmd

        self.req.cmd = cmd
        self.future = self.cli.call_async(self.req)

    def listener_callback(self, msg):
        self.get_logger().info(f'acc_x: {msg.agx}, acc_y: {msg.agy}, acc_z: {msg.agz}, height: {msg.h}')

        timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        ntp_time = time.time()
        one_raw = [ntp_time, timestamp, msg.agx, msg.agy, msg.agz, msg.h, self.traj_state, msg.roll, msg.pitch, msg.yaw]
        self.buffer.append(one_raw)

    def img_listener_callback(self, msg):

        timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        self.get_logger().info(f'{timestamp} - ({msg.height},{msg.width}), encoding: {msg.encoding}, is bigendian: {msg.is_bigendian}, step: {msg.step}')

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = np.array(cv_image)
        print(f"{img.shape=}")
        self.video_out.write(cv_image)

        self.frames_counter += 1
        time_elapsed = time.time() - self.last_video_frame_time
        if time_elapsed >= 1.0:
            print(f'video fps: {self.frames_counter}')
            self.frames_counter = 0
            self.last_video_frame_time = time.time()

    def dump_data_to_csv(self):
        buffer_df = pd.DataFrame(self.buffer, columns=['ntp_time', 'timestamp', 'acc_x', 'acc_y', 'acc_z', 'height', 'traj_state', 'roll', 'pitch', 'yaw'])

        # dist_x = input('Type X drift distance: ')
        # dist_y = input('Type Y drift distance: ')
        # buffer_df['drift_x'] = dist_x
        # buffer_df['drift_y'] = dist_y
        buffer_df.set_index('timestamp', inplace=True)
        filename = self.filenames + '.csv'
        path = self.out_dir / filename
        buffer_df.to_csv(path)
        print(f'Data saved to {filename}')

        self.video_out.release()


def main(args=None):
    rclpy.init(args=args)

    out_dir = Path('drift_tests_video_mocap_19-05-22/tello_rg')
    # out_dir = Path('tello_drift_optitrack/tello_drift_ot_moving')
    out_dir.mkdir(parents=True, exist_ok=True)

    tello_acc_tester = TelloAccTester(out_dir)

    trajectory_hanging = [(0, 'takeoff'), (10, 'land')]
    trajectory_moving = [(0, 'takeoff'), (10, 'rc -10 -10 0 0')]
    trajectory = trajectory_hanging

    current_state = 0
    additional_time_for_landing = 4
    start_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(tello_acc_tester)

        if current_state >= len(trajectory):
            if (time.time() - start_time) > (trajectory[-1][0] + additional_time_for_landing):
                break

        elif (time.time() - start_time) > trajectory[current_state][0]:
            tello_acc_tester.send_request(trajectory[current_state][1])
            current_state += 1

        if tello_acc_tester.future.done():
            try:
                response = tello_acc_tester.future.result()
            except Exception as e:
                tello_acc_tester.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                tello_acc_tester.get_logger().info(f'Result of cmd: {response.rc}')

    tello_acc_tester.dump_data_to_csv()
    tello_acc_tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
