import numpy as np
import rclpy.time
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sam_diving_controller.controllers.DiveControllerInterface import DiveControllerInterface
from sam_diving_controller.controllers.ONNXManager import ONNXManager, norm_move, norm_align
from scipy.spatial.transform import Rotation as R

from sam_diving_controller import TransformUtils
from sam_diving_controller.IDivePub import MissionStates, ActuatorStates
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_pose, tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_inverse, quaternion_matrix
import numpy as np


class DiveControllerONNX(DiveControllerInterface):

    def __init__(self, node, dive_pub, dive_sub, param, rate=0.2):
        super().__init__(node, dive_pub, dive_sub, param, rate)
        np.set_printoptions(precision=2, suppress=True)
        # Convenience Topics
        self._current_state = None
        self._current_state_in_odom = None
        self._current_state_in_mocap = None
        self._current_control = None
        self._ref = None
        self._error = None
        self.waypoint = None

        self.onnx_manager_move = ONNXManager("PoolMove")
        self.onnx_manager_align = ONNXManager("PoolAlign")
        self.onnx_manager_move.normalization = norm_move
        self.onnx_manager_align.normalization = norm_align
        self.manager = self.onnx_manager_move


        self.wp_pub = node.create_publisher(PoseStamped, "wp_test", 10)
        self.baselink_pub = node.create_publisher(Odometry, "baselink_test", 10)

        self._loginfo("ONNX Dive Controller created")

    def update(self):
        mission_state = self._dive_sub.get_mission_state()

        if mission_state == MissionStates.RECEIVED or mission_state == MissionStates.COMPLETED or mission_state == MissionStates.CANCELLED:
            self._loginfo_once(f"Mission not running. State: {mission_state}")
            self._set_actuators_neutral()
            return
        # Engage actuators in case they were off before.
        self._dive_pub.set_actuator_states(ActuatorStates.ENGAGED, "DP")

        baselink = self._dive_sub.get_states()
        if baselink is None:
            self._loginfo(f"No state available yet.")
            return

        waypoint = self._dive_sub.get_waypoint()
        if waypoint is None:
            self._loginfo(f"waypoint is None")
            return

        target_frame_id = "KTHTank/map"

        baselink_to_map = self._dive_sub.lookup_transform(target_frame=target_frame_id, source_frame=baselink.header.frame_id)
        baselink.pose.pose = tf2_geometry_msgs.do_transform_pose(baselink.pose.pose, baselink_to_map)
        baselink.header.frame_id = target_frame_id
        baselink_in_map = baselink

        waypoint_to_map = self._dive_sub.lookup_transform(target_frame=target_frame_id, source_frame=waypoint.header.frame_id)
        waypoint_in_map = tf2_geometry_msgs.do_transform_pose_stamped(waypoint, waypoint_to_map)

        waypoint_in_body = self._dive_sub.lookup_transform_pose(waypoint, baselink.child_frame_id) # Keep this for sanity check
        self._loginfo(f"waypoint in body tf {waypoint_in_body}")

        position = TransformUtils.transform_point_to_child(baselink_in_map, waypoint_in_map.pose.position)
        orientation = TransformUtils.rotate_quat_to_child(baselink_in_map, waypoint_in_map.pose.orientation)
        self._loginfo(f"manual tf waypoint {position}   {orientation}")

        baselink_enu_flu_map = baselink_in_map
        waypoint_enu_body = waypoint_in_body

        control_input = self._dive_sub.get_control_input()
        onnx_input = self.manager.prepare_state((baselink_enu_flu_map,
                                                 waypoint_enu_body,
                                                 control_input))

        control_output = self.manager.get_control(onnx_input)
        control_output = self.manager.rescale_outputs(control_output)

        self.set_publishers(control_output)
        self.baselink_pub.publish(baselink_enu_flu_map)
        self.wp_pub.publish(waypoint_enu_body)


    def set_publishers(self, outputs):
        """
        Set the corresponding publishers for the actuators and convenience topics
        """
        u_rpm1 = outputs[0]
        u_rpm2 = outputs[0]
        u_aileron = outputs[1]
        u_rudder = outputs[2]
        u_vbs = outputs[3]
        u_lcg = outputs[4]

        # Publish the control input
        self._dive_pub.set_vbs(u_vbs)
        self._dive_pub.set_lcg(u_lcg)
        self._dive_pub.set_thrust_vector(u_rudder, u_aileron)
        self._dive_pub.set_rpm(u_rpm1, u_rpm2)



def convert_pose_frd_to_enu(odometry_frd):
    out = Odometry()
    out.header = odometry_frd.header
    out.child_frame_id = odometry_frd.child_frame_id

    p = odometry_frd.pose.pose.position
    p_enu = frd_vec_to_enu([p.x, p.y, p.z])
    out.pose.pose.position.x = float(p_enu[0])
    out.pose.pose.position.y = float(p_enu[1])
    out.pose.pose.position.z = float(p_enu[2])

    # --- orientation ---
    q = odometry_frd.pose.pose.orientation
    q_enu = frd_quat_to_enu([q.x, q.y, q.z, q.w])
    out.pose.pose.orientation.x = float(q_enu[0])
    out.pose.pose.orientation.y = float(q_enu[1])
    out.pose.pose.orientation.z = float(q_enu[2])
    out.pose.pose.orientation.w = float(q_enu[3])

    return out


def frd_vec_to_enu(v):
    """FRD (x forward, y right, z down) -> ENU (x east, y north, z up)."""
    return np.array([v[0], v[1], -v[2]], dtype=float)


def frd_quat_to_enu(q_xyzw):
    """
    Convert orientation from FRD frame convention to ENU.

    Assumes quaternion q describes body orientation in the FRD world frame.
    We convert by applying the basis transform to the rotation matrix:
        R_enu = T * R_frd * T
    where T = diag(1, 1, -1).
    """
    T = np.diag([1.0, 1.0, -1.0])

    R_frd = R.from_quat(q_xyzw).as_matrix()  # scipy expects [x, y, z, w]
    R_enu = T @ R_frd @ T

    q_enu = R.from_matrix(R_enu).as_quat()  # returns [x, y, z, w]
    return q_enu


def invert_transform(t: TransformStamped) -> TransformStamped:
    q = [
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w,
    ]

    # Invert rotation
    q_inv = quaternion_inverse(q)

    # Invert translation:  -R^T * t
    R_inv = quaternion_matrix(q_inv)[0:3, 0:3]
    t_vec = np.array([
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z,
    ])
    t_inv_vec = -R_inv @ t_vec

    t_inv = TransformStamped()
    t_inv.header.stamp = t.header.stamp
    t_inv.header.frame_id = t.child_frame_id
    t_inv.child_frame_id = t.header.frame_id

    t_inv.transform.translation.x = t_inv_vec[0]
    t_inv.transform.translation.y = t_inv_vec[1]
    t_inv.transform.translation.z = t_inv_vec[2]

    t_inv.transform.rotation.x = q_inv[0]
    t_inv.transform.rotation.y = q_inv[1]
    t_inv.transform.rotation.z = q_inv[2]
    t_inv.transform.rotation.w = q_inv[3]

    return t_inv
