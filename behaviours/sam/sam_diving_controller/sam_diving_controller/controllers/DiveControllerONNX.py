import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from sam_diving_controller.controllers.DiveControllerInterface import DiveControllerInterface
from sam_diving_controller.controllers.ONNXManager import ONNXManager, norm_move, norm_align
from scipy.spatial.transform import Rotation as R

from sam_diving_controller import TransformUtils
from sam_diving_controller.IDivePub import MissionStates, ActuatorStates


class DiveControllerONNX(DiveControllerInterface):

    def __init__(self, node, dive_pub, dive_sub, param, rate=0.2):
        super().__init__(node, dive_pub, dive_sub, param, rate)

        # Convenience Topics
        self._current_state = None
        self._current_state_in_odom = None
        self._current_state_in_mocap = None
        self._current_control = None
        self._ref = None
        self._error = None
        self.waypoint = None

        self.onnx_manager_move = ONNXManager("SAMAlign")
        self.onnx_manager_align = ONNXManager("SAMMove")
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

        # Get the current states
        current_state_in_mocap = self._dive_sub.get_states_in_mocap()

        if current_state_in_mocap is None:
            self._loginfo_once(f"No state available yet.")
            return

        odometry_mocap_frd = self.convert_flu_to_frd(current_state_in_mocap, convert_pos=False)
        odometry_body_frd = self.convert_to_body(current_state_in_mocap, odometry_mocap_frd)
        waypoint_body_frd =  self.convert_flu_to_frd(self.convert_to_body(current_state_in_mocap, waypoint_mocap_frd), True)
        control_input = self._dive_sub.get_control_input()

        self.manager = self.onnx_manager_align if np.linalg.norm(TransformUtils.vector_to_list(waypoint_body_frd.pose.pose.position)) < 0.5 and self.manager == self.onnx_manager_move \
                                                  or np.linalg.norm(TransformUtils.vector_to_list(waypoint_body_frd.pose.pose.position)) < 1 and self.manager == self.onnx_manager_align \
            else self.onnx_manager_move

        pos = odometry_body_frd.twist.twist.linear
        self._loginfo(f'Vec: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')

        onnx_input = self.manager.prepare_state((odometry_mocap_frd,
                                                 odometry_body_frd,
                                                 waypoint_body_frd,
                                                 control_input))
        control_output = self.manager.get_control_scaled(onnx_input)

        self.set_publishers(control_output)

    def convert_flu_to_frd(self, flu_msg, convert_pos=True):
        """
        If convert_state, it converts an odometry message from FLU to FRD

        """
        frd_odometry = Odometry()
        frd_odometry.header.frame_id = flu_msg.header.frame_id
        frd_odometry.header.stamp = flu_msg.header.stamp
        if convert_pos:
            frd_odometry.pose.pose.position.x = flu_msg.pose.pose.position.x # We currently dont convert position on purpose.
            frd_odometry.pose.pose.position.y = -flu_msg.pose.pose.position.y
            frd_odometry.pose.pose.position.z = -flu_msg.pose.pose.position.z
        else:
            frd_odometry.pose.pose.position.x = flu_msg.pose.pose.position.x  # We currently dont convert position on purpose.
            frd_odometry.pose.pose.position.y = flu_msg.pose.pose.position.y
            frd_odometry.pose.pose.position.z = flu_msg.pose.pose.position.z

        quat = self.quat_flu_to_frd([flu_msg.pose.pose.orientation.w,
                                     flu_msg.pose.pose.orientation.x,
                                     flu_msg.pose.pose.orientation.y,
                                     flu_msg.pose.pose.orientation.z])
        frd_odometry.pose.pose.orientation.x = quat[1]
        frd_odometry.pose.pose.orientation.y = quat[2]
        frd_odometry.pose.pose.orientation.z = quat[3]
        frd_odometry.pose.pose.orientation.w = quat[0]

        frd_odometry.twist.twist.linear.x = flu_msg.twist.twist.linear.x
        frd_odometry.twist.twist.linear.y = -flu_msg.twist.twist.linear.y
        frd_odometry.twist.twist.linear.z = -flu_msg.twist.twist.linear.z
        frd_odometry.twist.twist.angular.x = flu_msg.twist.twist.angular.x
        frd_odometry.twist.twist.angular.y = -flu_msg.twist.twist.angular.y
        frd_odometry.twist.twist.angular.z = -flu_msg.twist.twist.angular.z


        return frd_odometry

    def quat_flu_to_frd(self, q_flu):
        """
        quat_flu = [q0, q1, q2, q3], with q0 the scalar part
        """
        quat_flu = np.array([q_flu[1], q_flu[2], q_flu[3], q_flu[0]])

        rot = R.from_euler('x', 180, degrees=True)
        r_flu = R.from_quat(quat_flu)  # Convert ENU quaternion to rotation object, assumes scalar last
        r_frd = r_flu.as_matrix() @ rot.as_matrix()
        quat_frd = R.from_matrix(r_frd).as_quat()  # Convert back to quaternion with scalar last
        quat_frd_right_order = np.array([quat_frd[3],  # w
                                         quat_frd[0],  # x
                                         quat_frd[1],  # y
                                         quat_frd[2]  # z
                                         ])
        return quat_frd_right_order

    def set_publishers(self, outputs):
        """
        Set the corresponding publishers for the actuators and convenience topics
        """
        u_rpm1 = outputs[0]
        u_rpm2 = outputs[0]
        u_stern = outputs[1]
        u_rudder = outputs[2]
        u_vbs = outputs[3]
        u_lcg = outputs[4]

        # Publish the control input
        self._dive_pub.set_vbs(u_vbs)
        self._dive_pub.set_lcg(u_lcg)
        self._dive_pub.set_thrust_vector(u_rudder, u_stern)
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

    def convert_to_body(self, target_frame: Odometry, odometry: Odometry):
        odom = Odometry()

        odom.child_frame_id = ""
        odom.header.frame_id = "base_link"
        odom.header.stamp = self._node.get_clock().now().to_msg()

        odom.pose.pose.position = TransformUtils.transform_point_to_child(target_frame, odometry.pose.pose.position)
        odom.pose.pose.orientation = TransformUtils.rotate_quat_to_child(target_frame, odometry.pose.pose.orientation)

        odom.twist.twist = odometry.twist.twist
        # odom.twist.twist.linear = TransformUtils.rotate_vector_to_child(target_frame, odometry.twist.twist.linear)
        # odom.twist.twist.angular = TransformUtils.rotate_vector_to_child(target_frame, odometry.twist.twist.angular)

        return odom
