#!/usr/bin/env python3
"""Republish MAVROS odometry with twist rotated from body to world frame."""

import numpy as np
import rospy
from nav_msgs.msg import Odometry


def _quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1.0e-24:
        raise ValueError("received a zero-length odometry quaternion")

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


def odom_with_world_velocity(message: Odometry) -> Odometry:
    """Copy odometry and rotate twist and its covariance into world axes."""
    q = message.pose.pose.orientation
    rotation = _quaternion_matrix(q.x, q.y, q.z, q.w)

    body_twist = np.array(
        [
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        ],
        dtype=np.float64,
    )
    twist_rotation = np.zeros((6, 6), dtype=np.float64)
    twist_rotation[:3, :3] = rotation
    twist_rotation[3:, 3:] = rotation
    world_twist = twist_rotation.dot(body_twist)

    body_covariance = np.asarray(
        message.twist.covariance, dtype=np.float64
    ).reshape(6, 6)
    world_covariance = twist_rotation.dot(body_covariance).dot(twist_rotation.T)

    output = Odometry()
    output.header = message.header
    output.child_frame_id = message.header.frame_id or "world"
    output.pose = message.pose
    output.twist.twist.linear.x = world_twist[0]
    output.twist.twist.linear.y = world_twist[1]
    output.twist.twist.linear.z = world_twist[2]
    output.twist.twist.angular.x = world_twist[3]
    output.twist.twist.angular.y = world_twist[4]
    output.twist.twist.angular.z = world_twist[5]
    output.twist.covariance = world_covariance.reshape(36).tolist()
    return output


class VelocityToWorld:
    def __init__(self) -> None:
        self.input_topic = str(
            rospy.get_param("~input_topic", "/mavros/local_position/odom")
        )
        self.output_topic = str(
            rospy.get_param("~output_topic", "/Odometry_high_rate")
        )
        if self.input_topic == self.output_topic:
            raise ValueError("~output_topic must differ from ~input_topic")

        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            Odometry,
            self._odom_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            "vel2world: body-frame velocity %s -> world-frame %s",
            self.input_topic,
            self.output_topic,
        )

    def _odom_callback(self, message: Odometry) -> None:
        try:
            self.publisher.publish(odom_with_world_velocity(message))
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "vel2world: %s", error)


def main() -> None:
    rospy.init_node("vel2world")
    try:
        VelocityToWorld()
    except (TypeError, ValueError) as error:
        rospy.logfatal("vel2world configuration error: %s", error)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
