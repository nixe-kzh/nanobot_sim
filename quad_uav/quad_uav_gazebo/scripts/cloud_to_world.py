#!/usr/bin/env python3
"""Transform lidar PointCloud2 data to world coordinates."""

import math
from typing import Iterable, Tuple

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


def _vec3(value: Iterable[float], name: str) -> Tuple[float, float, float]:
    values = tuple(float(item) for item in value)
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    return values


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _quat_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1.0e-24:
        raise ValueError("zero-length transform quaternion")

    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


class PcToWorld:
    def __init__(self) -> None:
        self.in_topic = rospy.get_param("~input_topic", "/velodyne_points")
        self.out_topic = rospy.get_param(
            "~output_topic", "/velodyne_points_world"
        )
        self.target = str(rospy.get_param("~target_frame", "world")).lstrip("/")
        self.body = str(rospy.get_param("~body_frame", "base_link")).lstrip("/")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/mavros/local_position/odom"
        )
        self.max_age = float(rospy.get_param("~max_odom_age", 0.10))
        self.filter_radius = float(rospy.get_param("~body_filter_radius", 1.0))
        self.use_tf = bool(rospy.get_param("~use_tf", False))
        if self.filter_radius < 0.0:
            raise ValueError("~body_filter_radius must be non-negative")

        pos = _vec3(
            rospy.get_param("~static_translation", [0.10, 0.0, 0.15]),
            "~static_translation",
        )
        rpy = _vec3(
            rospy.get_param("~static_rpy", [0.0, 0.785398, 0.0]),
            "~static_rpy",
        )
        self.sensor_pos = np.asarray(pos, dtype=np.float64)
        self.sensor_rot = _rpy_matrix(*rpy)
        self.odom = None
        self.tf_buf = None
        self.tf_listener = None

        if self.use_tf:
            self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        else:
            self.odom_sub = rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self._on_odom,
                queue_size=10,
                tcp_nodelay=True,
            )

        self.pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        self.cloud_sub = rospy.Subscriber(
            self.in_topic,
            PointCloud2,
            self._on_cloud,
            queue_size=1,
            buff_size=64 * 1024 * 1024,
            tcp_nodelay=True,
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    @staticmethod
    def _xyz_fields(msg: PointCloud2):
        fields = {field.name: field for field in msg.fields}
        missing = [name for name in ("x", "y", "z") if name not in fields]
        if missing:
            raise ValueError(f"PointCloud2 missing fields: {', '.join(missing)}")

        xyz = tuple(fields[name] for name in ("x", "y", "z"))
        types = {field.datatype for field in xyz}
        if len(types) != 1 or next(iter(types)) not in (
            PointField.FLOAT32,
            PointField.FLOAT64,
        ):
            raise ValueError("x, y and z must share a floating-point type")
        if any(field.count != 1 for field in xyz):
            raise ValueError("x, y and z fields must have count=1")
        return xyz

    @staticmethod
    def _field_view(data, msg: PointCloud2, field: PointField) -> np.ndarray:
        kind = "f4" if field.datatype == PointField.FLOAT32 else "f8"
        order = ">" if msg.is_bigendian else "<"
        return np.ndarray(
            shape=(msg.height, msg.width),
            dtype=np.dtype(order + kind),
            buffer=data,
            offset=field.offset,
            strides=(msg.row_step, msg.point_step),
        )

    def _tf(self, target: str, msg: PointCloud2):
        source = msg.header.frame_id.lstrip("/")
        if not source:
            raise ValueError("input cloud has an empty frame_id")
        transform = self.tf_buf.lookup_transform(
            target, source, msg.header.stamp, rospy.Duration(0.05)
        ).transform
        quat = transform.rotation
        pos = transform.translation
        rot = _quat_matrix(quat.x, quat.y, quat.z, quat.w)
        return rot, np.array([pos.x, pos.y, pos.z], dtype=np.float64)

    def _world_tf(self, msg: PointCloud2):
        if self.use_tf:
            return self._tf(self.target, msg)
        if self.odom is None:
            raise ValueError(f"no odometry received on {self.odom_topic}")

        odom_stamp = self.odom.header.stamp
        cloud_stamp = msg.header.stamp
        if not odom_stamp.is_zero() and not cloud_stamp.is_zero():
            age = abs((cloud_stamp - odom_stamp).to_sec())
            if age > self.max_age:
                raise ValueError(
                    f"odometry age {age:.3f}s exceeds {self.max_age:.3f}s"
                )

        pose = self.odom.pose.pose
        quat = pose.orientation
        body_rot = _quat_matrix(quat.x, quat.y, quat.z, quat.w)
        body_pos = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float64,
        )
        return (
            body_rot.dot(self.sensor_rot),
            body_rot.dot(self.sensor_pos) + body_pos,
        )

    def _body_tf(self, msg: PointCloud2):
        if self.use_tf:
            return self._tf(self.body, msg)
        return self.sensor_rot, self.sensor_pos

    def _transform(
        self,
        msg: PointCloud2,
        rot: np.ndarray,
        pos: np.ndarray,
        body_rot: np.ndarray,
        body_pos: np.ndarray,
    ) -> PointCloud2:
        x_field, y_field, z_field = self._xyz_fields(msg)
        out_data = bytearray(msg.data)
        src_x = self._field_view(msg.data, msg, x_field).astype(
            np.float64, copy=True
        )
        src_y = self._field_view(msg.data, msg, y_field).astype(
            np.float64, copy=True
        )
        src_z = self._field_view(msg.data, msg, z_field).astype(
            np.float64, copy=True
        )

        valid = np.isfinite(src_x) & np.isfinite(src_y) & np.isfinite(src_z)
        if self.filter_radius > 0.0:
            body_x = (
                body_rot[0, 0] * src_x
                + body_rot[0, 1] * src_y
                + body_rot[0, 2] * src_z
                + body_pos[0]
            )
            body_y = (
                body_rot[1, 0] * src_x
                + body_rot[1, 1] * src_y
                + body_rot[1, 2] * src_z
                + body_pos[1]
            )
            body_z = (
                body_rot[2, 0] * src_x
                + body_rot[2, 1] * src_y
                + body_rot[2, 2] * src_z
                + body_pos[2]
            )
            with np.errstate(invalid="ignore", over="ignore"):
                radius_sq = self.filter_radius**2
                valid &= body_x**2 + body_y**2 + body_z**2 >= radius_sq

        out_x = self._field_view(out_data, msg, x_field)
        out_y = self._field_view(out_data, msg, y_field)
        out_z = self._field_view(out_data, msg, z_field)
        out_x[...] = (
            rot[0, 0] * src_x
            + rot[0, 1] * src_y
            + rot[0, 2] * src_z
            + pos[0]
        )
        out_y[...] = (
            rot[1, 0] * src_x
            + rot[1, 1] * src_y
            + rot[1, 2] * src_z
            + pos[1]
        )
        out_z[...] = (
            rot[2, 0] * src_x
            + rot[2, 1] * src_y
            + rot[2, 2] * src_z
            + pos[2]
        )

        records = np.ndarray(
            shape=(msg.height, msg.width, msg.point_step),
            dtype=np.uint8,
            buffer=out_data,
            strides=(msg.row_step, msg.point_step, 1),
        )
        data = records[valid].tobytes()

        out = PointCloud2()
        out.header.seq = msg.header.seq
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target
        out.height = 1
        out.width = int(np.count_nonzero(valid))
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.width * msg.point_step
        out.data = data
        out.is_dense = True
        return out

    def _on_cloud(self, msg: PointCloud2) -> None:
        try:
            rot, pos = self._world_tf(msg)
            body_rot, body_pos = self._body_tf(msg)
            self.pub.publish(self._transform(msg, rot, pos, body_rot, body_pos))
        except (
            ValueError,
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(2.0, "cloud_to_world: %s", error)


def main() -> None:
    rospy.init_node("cloud_to_world")
    try:
        PcToWorld()
    except (TypeError, ValueError) as error:
        rospy.logfatal("cloud_to_world: %s", error)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
