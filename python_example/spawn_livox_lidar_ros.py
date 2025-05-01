#!/usr/bin/env python3

import carla
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class LivoxLidarPublisher(Node):
    def __init__(self, client, world):
        super().__init__('livox_lidar_node')
        self.publisher_ = self.create_publisher(PointCloud2, '/livox/points', 10)

        self.client = client
        self.world = world
        self.get_logger().info('[DEBUG] Connected to CARLA')

        blueprint_library = self.world.get_blueprint_library()
        lidar_bp = blueprint_library.find("sensor.lidar.ray_cast_livox")
        self.get_logger().info('[test6]')
        # 센서 속성 설정
        lidar_bp.set_attribute("channels", "1")
        lidar_bp.set_attribute("range", "260")
        lidar_bp.set_attribute("points_per_second", "1000000")
        lidar_bp.set_attribute("dropoff_intensity_limit", "0")
        lidar_bp.set_attribute("dropoff_zero_intensity", "0")
        lidar_bp.set_attribute("dropoff_general_rate", "0.3")
        lidar_bp.set_attribute("noise_stddev", "0")
        lidar_bp.set_attribute("decay_time", "1")
        lidar_bp.set_attribute("lidar_type", "0")

        spawn_point = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=3.0),
            carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0)
        )

        self.lidar = self.world.spawn_actor(lidar_bp, spawn_point)
        if self.lidar is None:
            self.get_logger().error("Livox LiDAR spawn failed.")
            return

        self.get_logger().info("Livox LiDAR sensor spawned at (0,0,3)")
        self.lidar.listen(self.lidar_callback)

    def lidar_callback(self, data):
        header = Header()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.frame_id = "livox_lidar"

        points = []
        for i in range(0, len(data.raw_data), 16*4):
            x, y, z, intensity = struct.unpack_from("ffff", data.raw_data, offset=i)
            points.append([x, y, z, intensity])

        pointcloud_msg = point_cloud2.create_cloud(
            header,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ],
            points=points
        )

        self.publisher_.publish(pointcloud_msg)

    def destroy_node_and_sensor(self):
        self.get_logger().info("Shutting down...")
        if self.lidar:
            self.lidar.stop()
            self.lidar.destroy()
        self.destroy_node()

def wait_for_world(client):
    for i in range(10):
        try:
            world = client.get_world()
            print("[PRE-RCLPY] Connected to world")
            return world
        except RuntimeError as e:
            print("[PRE-RCLPY] World not ready:", e)
            time.sleep(1)
    raise RuntimeError("CARLA world not available after retries.")

def main(args=None):
    rclpy.init(args=args)

    # ROS init 전에 CARLA world가 준비되었는지 확인
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = wait_for_world(client)

    # 노드 실행 시 미리 확보한 client, world 전달
    node = LivoxLidarPublisher(client, world)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node_and_sensor()
        rclpy.shutdown()
if __name__ == '__main__':


    main()
