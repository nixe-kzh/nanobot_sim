#!/usr/bin/env python3
"""Publish Gazebo ground truth to PX4 through MAVROS."""

import math
from typing import List

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped


def _covariance(pos_var: float, rot_var: float) -> List[float]:
    cov = [0.0] * 36
    for index in (0, 7, 14):
        cov[index] = pos_var
    for index in (21, 28, 35):
        cov[index] = rot_var
    return cov


class GtToPx4:
    def __init__(self) -> None:
        self.model = str(rospy.get_param("~model_name", "iris"))
        self.service = str(
            rospy.get_param("~model_state_service", "/gazebo/get_model_state")
        )
        self.ref_frame = str(rospy.get_param("~reference_frame", "world"))
        self.pose_topic = str(
            rospy.get_param("~pose_topic", "/mavros/vision_pose/pose_cov")
        )
        self.vel_topic = str(
            rospy.get_param(
                "~velocity_topic", "/mavros/vision_speed/speed_twist_cov"
            )
        )
        self.frame = str(rospy.get_param("~frame_id", "odom"))
        self.pub_vel = bool(rospy.get_param("~publish_velocity", True))
        self.rate = float(rospy.get_param("~publish_rate", 30.0))
        timeout = float(rospy.get_param("~service_timeout", 30.0))

        pos_std = float(rospy.get_param("~position_stddev", 0.001))
        rot_std = float(rospy.get_param("~orientation_stddev", 0.001))
        vel_std = float(rospy.get_param("~linear_velocity_stddev", 0.001))
        ang_std = float(rospy.get_param("~angular_velocity_stddev", 0.001))

        if not self.model:
            raise ValueError("~model_name must not be empty")
        if not math.isfinite(self.rate) or self.rate <= 0.0:
            raise ValueError("~publish_rate must be finite and positive")
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("~service_timeout must be finite and positive")
        for name, value in (
            ("~position_stddev", pos_std),
            ("~orientation_stddev", rot_std),
            ("~linear_velocity_stddev", vel_std),
            ("~angular_velocity_stddev", ang_std),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

        self.pose_cov = _covariance(pos_std**2, rot_std**2)
        self.vel_cov = _covariance(vel_std**2, ang_std**2)
        self.pose_pub = rospy.Publisher(
            self.pose_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.vel_pub = None
        if self.pub_vel:
            self.vel_pub = rospy.Publisher(
                self.vel_topic, TwistWithCovarianceStamped, queue_size=10
            )

        rospy.wait_for_service(self.service, timeout=timeout)
        self._connect()
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.rate), self._poll
        )

    def _connect(self) -> None:
        self.state_client = rospy.ServiceProxy(
            self.service, GetModelState, persistent=True
        )

    def _poll(self, _event) -> None:
        try:
            state = self.state_client(self.model, self.ref_frame)
        except (rospy.ServiceException, rospy.ROSException) as error:
            rospy.logwarn_throttle(5.0, "Model-state query failed: %s", error)
            self._connect()
            return

        if not state.success:
            rospy.logwarn_throttle(
                5.0, "Cannot read model '%s': %s", self.model, state.status_message
            )
            return
        self._publish(state)

    def _publish(self, state: GetModelStateResponse) -> None:
        pose = state.pose
        quat = pose.orientation
        norm = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if norm < 1.0e-12 or not math.isfinite(norm):
            rospy.logwarn_throttle(5.0, "Invalid model quaternion")
            return

        stamp = state.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame
        pose_msg.pose.pose.position = pose.position
        pose_msg.pose.pose.orientation.x = quat.x / norm
        pose_msg.pose.pose.orientation.y = quat.y / norm
        pose_msg.pose.pose.orientation.z = quat.z / norm
        pose_msg.pose.pose.orientation.w = quat.w / norm
        pose_msg.pose.covariance = self.pose_cov
        self.pose_pub.publish(pose_msg)

        if self.vel_pub is None:
            return
        vel_msg = TwistWithCovarianceStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = self.frame
        vel_msg.twist.twist = state.twist
        vel_msg.twist.covariance = self.vel_cov
        self.vel_pub.publish(vel_msg)


def main() -> None:
    rospy.init_node("gt_to_px4")
    try:
        GtToPx4()
    except (rospy.ROSException, TypeError, ValueError) as error:
        rospy.logfatal("gt_to_px4: %s", error)
        return
    rospy.spin()


if __name__ == "__main__":
    main()
