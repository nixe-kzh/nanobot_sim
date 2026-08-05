#!/usr/bin/env python3
"""Rotate MAVROS odometry velocity from body to world axes."""

import numpy as np
import rospy
from nav_msgs.msg import Odometry


def _quat_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1.0e-24:
        raise ValueError("zero-length odometry quaternion")

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


def to_world_odom(msg: Odometry) -> Odometry:
    quat = msg.pose.pose.orientation
    rot = _quat_matrix(quat.x, quat.y, quat.z, quat.w)

    body_twist = np.array(
        [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ],
        dtype=np.float64,
    )
    rot6 = np.zeros((6, 6), dtype=np.float64)
    rot6[:3, :3] = rot
    rot6[3:, 3:] = rot
    world_twist = rot6.dot(body_twist)

    body_cov = np.asarray(msg.twist.covariance, dtype=np.float64).reshape(6, 6)
    world_cov = rot6.dot(body_cov).dot(rot6.T)

    out = Odometry()
    out.header.seq = msg.header.seq
    out.header.stamp = msg.header.stamp
    out.header.frame_id = "world"
    out.child_frame_id = msg.child_frame_id or "base_link"
    out.pose = msg.pose
    out.twist.twist.linear.x = world_twist[0]
    out.twist.twist.linear.y = world_twist[1]
    out.twist.twist.linear.z = world_twist[2]
    out.twist.twist.angular.x = world_twist[3]
    out.twist.twist.angular.y = world_twist[4]
    out.twist.twist.angular.z = world_twist[5]
    out.twist.covariance = world_cov.reshape(36).tolist()
    return out


class VelToWorld:
    def __init__(self) -> None:
        self.in_topic = str(
            rospy.get_param("~input_topic", "/mavros/local_position/odom")
        )
        self.out_topic = str(
            rospy.get_param("~output_topic", "/Odometry_high_rate")
        )
        if self.in_topic == self.out_topic:
            raise ValueError("~output_topic must differ from ~input_topic")

        self.pub = rospy.Publisher(self.out_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(
            self.in_topic,
            Odometry,
            self._on_odom,
            queue_size=10,
            tcp_nodelay=True,
        )

    def _on_odom(self, msg: Odometry) -> None:
        try:
            self.pub.publish(to_world_odom(msg))
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "vel_to_world: %s", error)


def main() -> None:
    rospy.init_node("vel_to_world")
    try:
        VelToWorld()
    except (TypeError, ValueError) as error:
        rospy.logfatal("vel_to_world: %s", error)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
