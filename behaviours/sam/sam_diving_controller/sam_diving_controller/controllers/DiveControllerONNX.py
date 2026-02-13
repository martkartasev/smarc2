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
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_pose


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

        self._loginfo("ONNX Dive Controller created")

    def update(self):
        mission_state = self._dive_sub.get_mission_state()

        if mission_state == MissionStates.RECEIVED or mission_state == MissionStates.COMPLETED or mission_state == MissionStates.CANCELLED:
            self._loginfo_once(f"Mission not running. State: {mission_state}")
            self._set_actuators_neutral()
            return
        # Engage actuators in case they were off before.
        self._dive_pub.set_actuator_states(ActuatorStates.ENGAGED, "DP")

        waypoint_mocap_frd = self._get_waypoint()
        if waypoint_mocap_frd is None:
            self._loginfo_once(f"No waypoint available yet.")
            return

        odom_mocap_frd_flu = self._dive_sub.get_states_in_mocap()
        if odom_mocap_frd_flu is None:
            self._loginfo_once(f"No state available yet.")
            return

        odom_enu_flu = convert_pose_frd_to_enu(odom_mocap_frd_flu) #TODO: Unnecessary?
        waypoint_enu = convert_pose_frd_to_enu(waypoint_mocap_frd)

        transform_odom_to_enu = self._dive_sub.lookup_transform(source_frame=odom_enu_flu.header.frame_id, target_frame="KTHTank/map")
        transform_waypoint_to_enu = self._dive_sub.lookup_transform(source_frame=waypoint_enu.header.frame_id, target_frame="KTHTank/map")

        odom_enu_flu_map = transform_odom_pose(odom_enu_flu, transform_odom_to_enu)
        waypoint_enu_map = transform_odom_pose(waypoint_enu, transform_waypoint_to_enu)

        waypoint_enu_body = self.convert_to_body(odom_target=odom_enu_flu_map, odom_to_covert=waypoint_enu_map)

        control_input = self._dive_sub.get_control_input()
        # self.manager = self.onnx_manager_align if np.linalg.norm(TransformUtils.vector_to_list(waypoint_enu_body.pose.pose.position)) < 0.5 and self.manager == self.onnx_manager_move \
        #                                           or np.linalg.norm(TransformUtils.vector_to_list(waypoint_enu_body.pose.pose.position)) < 1 and self.manager == self.onnx_manager_align \
        #     else self.onnx_manager_move

        onnx_input = self.manager.prepare_state((odom_enu_flu_map,
                                                 waypoint_enu_body,
                                                 control_input))

        # pos = odometry_frd.pose.pose.position
        # self._loginfo(f'Odometry_frd: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')
        # pos = waypoint_mocap_frd.pose.pose.position
        # self._loginfo(f'Waypoint_mocap_frd: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')
        self._loginfo(f'Vec: {onnx_input[0, 13:17]}')

        control_output = self.manager.get_control(onnx_input)
        control_output = self.manager.rescale_outputs(control_output)

        self.set_publishers(control_output)

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

    def _get_waypoint(self):
        if not self._dive_sub.has_waypoint():
            return None

        waypoint_in_mocap = self._dive_sub.get_waypoint()
        # FIXME: This might be useless.
        if waypoint_in_mocap is None:
            self._loginfo(f"waypoint_in_mocap is None")
            return False

        odometry = self.convert_wp_to_odometry(waypoint_in_mocap)
        return odometry

    def convert_wp_to_odometry(self, wp_msg):
        """
        Returns waypoint as Odometry
        """
        odom_wp = Odometry()

        if isinstance(wp_msg, PoseStamped):
            odom_wp.header.frame_id = wp_msg.header.frame_id
            odom_wp.header.stamp = wp_msg.header.stamp

            odom_wp.pose.pose = wp_msg.pose

        elif isinstance(wp_msg, Pose):
            odom_wp.header.frame_id = '/mocap'
            odom_wp.header.stamp = self._node.get_clock().now().to_msg()

            odom_wp.pose.pose.position = wp_msg.position
            odom_wp.pose.pose.orientation = wp_msg.orientation

        elif isinstance(wp_msg, Odometry):
            odom_wp = wp_msg

        else:
            return None

        return odom_wp

    def convert_to_body(self, odom_target: Odometry, odom_to_covert: Odometry):
        odom = Odometry()

        odom.child_frame_id = ""
        odom.header.frame_id = "base_link"
        odom.header.stamp = self._node.get_clock().now().to_msg()

        odom.pose.pose.position = TransformUtils.transform_point_to_child(odom_target, odom_to_covert.pose.pose.position)
        odom.pose.pose.orientation = TransformUtils.rotate_quat_to_child(odom_target, odom_to_covert.pose.pose.orientation)

        return odom

def transform_odom_pose(source: Odometry, transform):
    out = Odometry()
    out.header = source.header
    out.child_frame_id = source.child_frame_id
    out.twist = source.twist

    pose_in = PoseStamped()
    pose_in.header = source.header
    pose_in.pose = source.pose.pose

    pose_out = do_transform_pose_stamped(pose_in, transform)

    out.pose.pose = pose_out.pose
    out.pose.covariance = source.pose.covariance
    return out



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
