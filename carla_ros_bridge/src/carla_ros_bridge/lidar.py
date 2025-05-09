#!/usr/bin/env python

#
# Copyright (c) 2018, Willow Garage, Inc.
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import numpy

from carla_ros_bridge.sensor import Sensor, create_cloud

from sensor_msgs.msg import PointCloud2, PointField


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Lidar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.lidar_publisher = node.new_publisher(PointCloud2,
                                                  self.get_topic_prefix(),
                                                  qos_profile=10)
        self.listen()

    def destroy(self):
        super(Lidar, self).destroy()
        self.node.destroy_publisher(self.lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        # we take the opposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data[:, 1] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data)
        self.lidar_publisher.publish(point_cloud_msg)


class SemanticLidar(Sensor):

    """
    Actor implementation details for semantic lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(SemanticLidar, self).__init__(uid=uid,
                                            name=name,
                                            parent=parent,
                                            relative_spawn_pose=relative_spawn_pose,
                                            node=node,
                                            carla_actor=carla_actor,
                                            synchronous_mode=synchronous_mode)

        self.semantic_lidar_publisher = node.new_publisher(
            PointCloud2,
            self.get_topic_prefix(),
            qos_profile=10)
        self.listen()

    def destroy(self):
        super(SemanticLidar, self).destroy()
        self.node.destroy_publisher(self.semantic_lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received semantic lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='CosAngle', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ObjIdx', offset=16, datatype=PointField.UINT32, count=1),
            PointField(name='ObjTag', offset=20, datatype=PointField.UINT32, count=1)
        ]

        lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data),
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))

        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data.tolist())
        self.semantic_lidar_publisher.publish(point_cloud_msg)

# class LivoxLidar(Sensor):

#     """
#     Actor implementation details for livox lidar
#     """

#     def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
#         """
#         Constructor

#         :param uid: unique identifier for this object
#         :type uid: int
#         :param name: name identiying this object
#         :type name: string
#         :param parent: the parent of this
#         :type parent: carla_ros_bridge.Parent
#         :param relative_spawn_pose: the spawn pose of this
#         :type relative_spawn_pose: geometry_msgs.Pose
#         :param node: node-handle
#         :type node: CompatibleNode
#         :param carla_actor: carla actor object
#         :type carla_actor: carla.Actor
#         :param synchronous_mode: use in synchronous mode?
#         :type synchronous_mode: bool
#         """
#         super(LivoxLidar, self).__init__(uid=uid,
#                                     name=name,
#                                     parent=parent,
#                                     relative_spawn_pose=relative_spawn_pose,
#                                     node=node,
#                                     carla_actor=carla_actor,
#                                     synchronous_mode=synchronous_mode) # default: synchronous_mode
#         print("[DEBUG] LivoxLidar class initialized for", self.name)

#         self.livoxlidar_publisher = node.new_publisher(PointCloud2,
#                                                   self.get_topic_prefix(),
#                                                   qos_profile=10)
#         self.listen()

#     def destroy(self):
#         super(LivoxLidar, self).destroy()
#         self.node.destroy_publisher(self.livoxlidar_publisher)

#     # pylint: disable=arguments-differ
#     def sensor_data_updated(self, carla_lidar_measurement):
#         print("[DEBUG] LivoxLidar sensor_data_updated called for", self.name)

#         print("Livox raw_data length:", len(carla_lidar_measurement.raw_data))
#         print("First 16 bytes:", list(carla_lidar_measurement.raw_data[:16]))

#         header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#             PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
#         ]

#         try:
#             print(f"[Livox Debug] raw_data length: {len(carla_lidar_measurement.raw_data)}")
#             print(f"[Livox Debug] raw_data preview (first 20 bytes): {list(carla_lidar_measurement.raw_data[:20])}")

#             HEADER_BYTES = 12  # ← LivoxLidarSerializer 기준 계산
#             payload = carla_lidar_measurement.raw_data[HEADER_BYTES:]

#             lidar_data = numpy.frombuffer(payload, dtype=numpy.float32)

#             if lidar_data.shape[0] % 4 != 0:
#                 print(f"[ERROR] raw_data length ({lidar_data.shape[0]}) is not divisible by 4!")
#                 return

#             lidar_data = lidar_data.reshape((-1, 4))
#         except Exception as e:
#             print(f"[LivoxLidar] Failed to parse raw_data: {e}")
#             return

#         lidar_data[:, 1] *= -1  # Flip y-axis for ROS coord

#         point_cloud_msg = create_cloud(header, fields, lidar_data)
#         self.livoxlidar_publisher.publish(point_cloud_msg)

class LivoxLidar(Sensor):

    """
    Actor implementation details for livox lidar
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(LivoxLidar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode) # default: synchronous_mode

        self.livoxlidar_publisher = node.new_publisher(PointCloud2,
                                                  self.get_topic_prefix(),
                                                  qos_profile=10)
        self.listen()

    def destroy(self):
        super(LivoxLidar, self).destroy()
        self.node.destroy_publisher(self.livoxlidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1) # this intensity is livox-type. The integer part is the line number and the decimal part is the scan timestamp
        ]
       
        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        

        lidar_data[:, 1] *= -1 

        point_cloud_msg = create_cloud(header, fields, lidar_data) 
        self.livoxlidar_publisher.publish(point_cloud_msg) 