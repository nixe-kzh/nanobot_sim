#!/usr/bin/env python3
"""Send Gazebo Classic model ground truth to PX4 through MAVROS.

The model state is polled through ``/gazebo/get_model_state`` at a bounded
rate.  This avoids subscribing to ``/gazebo/model_states``, which can make
Gazebo serialize every model at the 1000 Hz physics update rate and slow down
PX4 lockstep simulation.

Gazebo's world pose is expressed in the ROS ENU frame. MAVROS' vision plugins
expect ROS conventions and perform the ENU/FLU to PX4 NED/FRD conversion
themselves, so this node must not swap or negate axes.

The node publishes:

* ``/mavros/vision_pose/pose_cov`` for position and attitude;
* ``/mavros/vision_speed/speed_twist_cov`` for world-frame velocity.

PX4 must still be configured to fuse external-vision measurements.  The
relevant estimator parameters depend on the PX4 release (for example,
``EKF2_EV_CTRL`` on newer releases or ``EKF2_AID_MASK`` on older releases).
"""

import math
from typing import List

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped


def _diagonal_covariance(linear_variance: float, angular_variance: float) -> List[float]:
    """Return a row-major 6x6 covariance for [x, y, z, roll, pitch, yaw]."""
    covariance = [0.0] * 36
    for index in (0, 7, 14):
        covariance[index] = linear_variance
    for index in (21, 28, 35):
        covariance[index] = angular_variance
    return covariance


class GazeboTruthToPx4:
    def __init__(self) -> None:
        # rspx4.sh uses the launch file's default vehicle name, "iris". Gazebo's
        # spawn_model -model argument overrides the name stored in the custom SDF.
        self.model_name = str(rospy.get_param("~model_name", "iris"))
        self.model_state_service = str(
            rospy.get_param("~model_state_service", "/gazebo/get_model_state")
        )
        self.reference_frame = str(rospy.get_param("~reference_frame", "world"))
        self.pose_topic = str(
            rospy.get_param("~pose_topic", "/mavros/vision_pose/pose_cov")
        )
        self.velocity_topic = str(
            rospy.get_param(
                "~velocity_topic", "/mavros/vision_speed/speed_twist_cov"
            )
        )
        self.frame_id = str(rospy.get_param("~frame_id", "odom"))
        self.publish_velocity = bool(rospy.get_param("~publish_velocity", True))
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        service_timeout = float(rospy.get_param("~service_timeout", 30.0))

        position_stddev = float(rospy.get_param("~position_stddev", 0.001))
        orientation_stddev = float(rospy.get_param("~orientation_stddev", 0.001))
        linear_velocity_stddev = float(
            rospy.get_param("~linear_velocity_stddev", 0.001)
        )
        angular_velocity_stddev = float(
            rospy.get_param("~angular_velocity_stddev", 0.001)
        )

        if not self.model_name:
            raise ValueError("~model_name must not be empty")
        if self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be greater than zero")
        if not math.isfinite(service_timeout) or service_timeout <= 0.0:
            raise ValueError("~service_timeout must be finite and greater than zero")
        for name, value in (
            ("~position_stddev", position_stddev),
            ("~orientation_stddev", orientation_stddev),
            ("~linear_velocity_stddev", linear_velocity_stddev),
            ("~angular_velocity_stddev", angular_velocity_stddev),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("{} must be finite and non-negative".format(name))

        self.pose_covariance = _diagonal_covariance(
            position_stddev**2, orientation_stddev**2
        )
        self.velocity_covariance = _diagonal_covariance(
            linear_velocity_stddev**2, angular_velocity_stddev**2
        )

        self.pose_publisher = rospy.Publisher(
            self.pose_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.velocity_publisher = None
        if self.publish_velocity:
            self.velocity_publisher = rospy.Publisher(
                self.velocity_topic, TwistWithCovarianceStamped, queue_size=10
            )

        rospy.loginfo(
            "gazebo_truth_to_px4: waiting for service %s",
            self.model_state_service,
        )
        rospy.wait_for_service(self.model_state_service, timeout=service_timeout)
        self._connect_model_state_service()
        self.poll_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.publish_rate), self._poll_model_state
        )

        rospy.loginfo(
            "gazebo_truth_to_px4: polling model '%s' from %s -> %s at %.1f Hz",
            self.model_name,
            self.model_state_service,
            self.pose_topic,
            self.publish_rate,
        )
        if self.publish_velocity:
            rospy.loginfo("gazebo_truth_to_px4: velocity -> %s", self.velocity_topic)

    def _connect_model_state_service(self) -> None:
        self.model_state_client = rospy.ServiceProxy(
            self.model_state_service, GetModelState, persistent=True
        )

    def _poll_model_state(self, _event) -> None:
        try:
            response = self.model_state_client(self.model_name, self.reference_frame)
        except (rospy.ServiceException, rospy.ROSException) as error:
            rospy.logwarn_throttle(
                5.0,
                "gazebo_truth_to_px4: model-state query failed: %s",
                error,
            )
            # Gazebo restart invalidates a persistent service connection.
            self._connect_model_state_service()
            return

        if not response.success:
            rospy.logwarn_throttle(
                5.0,
                "gazebo_truth_to_px4: cannot read model '%s': %s",
                self.model_name,
                response.status_message,
            )
            return

        self._publish_model_state(response)

    def _publish_model_state(self, response: GetModelStateResponse) -> None:
        pose = response.pose
        quaternion_norm = math.sqrt(
            pose.orientation.x**2
            + pose.orientation.y**2
            + pose.orientation.z**2
            + pose.orientation.w**2
        )
        if quaternion_norm < 1.0e-12 or not math.isfinite(quaternion_norm):
            rospy.logwarn_throttle(
                5.0, "gazebo_truth_to_px4: ignoring an invalid model quaternion"
            )
            return

        stamp = response.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()

        pose_message = PoseWithCovarianceStamped()
        pose_message.header.stamp = stamp
        pose_message.header.frame_id = self.frame_id
        pose_message.pose.pose.position = pose.position
        pose_message.pose.pose.orientation.x = pose.orientation.x / quaternion_norm
        pose_message.pose.pose.orientation.y = pose.orientation.y / quaternion_norm
        pose_message.pose.pose.orientation.z = pose.orientation.z / quaternion_norm
        pose_message.pose.pose.orientation.w = pose.orientation.w / quaternion_norm
        pose_message.pose.covariance = self.pose_covariance
        self.pose_publisher.publish(pose_message)

        if self.velocity_publisher is not None:
            velocity_message = TwistWithCovarianceStamped()
            velocity_message.header.stamp = stamp
            velocity_message.header.frame_id = self.frame_id
            velocity_message.twist.twist = response.twist
            velocity_message.twist.covariance = self.velocity_covariance
            self.velocity_publisher.publish(velocity_message)


def main() -> None:
    rospy.init_node("gazebo_truth_to_px4")
    try:
        GazeboTruthToPx4()
    except (rospy.ROSException, TypeError, ValueError) as error:
        rospy.logfatal("gazebo_truth_to_px4: invalid configuration: %s", error)
        rospy.signal_shutdown(str(error))
        return
    rospy.spin()


if __name__ == "__main__":
    main()
