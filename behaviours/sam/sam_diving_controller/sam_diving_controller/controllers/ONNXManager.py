#!/usr/bin/python3
import os

import numpy as np
import onnxruntime as ort
from sam_diving_controller.TransformUtils import vector_to_list, limit_vector, range_normalize
from ament_index_python import get_package_share_directory


class ONNXManager():
    """
    Simple ONNX inference session: https://onnxruntime.ai/docs/get-started/with-python.html
    Max values based on training configuration.
    If you are not sure what values were used for training, do not touch this.
    """

    def __init__(self,
                 model_resource: str = "DR_temp",
                 rpm_max: float = 1000,
                 aileron_angle_max: float = 0.2,
                 rudder_angle_max: float = 0.2,
                 vbs_max: float = 100,
                 lcg_max: float = 100,
                 ):
        # options = ort.SessionOptions()
        # options.use_deterministic_compute = True

        pkg_share = get_package_share_directory("sam_diving_controller")
        onnx_path = os.path.join(pkg_share, "resource", f"{model_resource}.onnx")

        self.onnx_inferenceSession = ort.InferenceSession(onnx_path,
                                                          # sess_options=options
                                                          )  # TODO: If non-determinisim cant be solved, might need to load it as a torch model

        self.rpm_max = rpm_max
        self.rudder_angle_max = rudder_angle_max
        self.aileron_angle_max = aileron_angle_max
        self.vbs_max = vbs_max
        self.lcg_max = lcg_max
        self.normalization = lambda x: x

    def get_control_scaled(self, x):
        return self.rescale_outputs(self.get_control(x))

    def get_control(self, x):
        """
        Inputs (1,27):
            x[0-3] = Orientation. Mocap frame. NED, Quaternion
            x[4-6] = Linear velocity. Body Frame, FLU, Vector3
            x[7-9] = Angular velocity. Body Frame, FLU, Vector3
            x[10-12] = Relative vector to waypoint. Body Frame, NED, Vector3
            x[13-16] = Relative orientation of waypoint w.r.p body. Body Frame, NED, Quaternion
            x[17] = Target velocity magnitude. Between 0.1 - 0.5.
            x[18-20] = Absolute position. Mocap frame, NED, Vector3
            x[21-25] = Previous/current "action" vector.
            x[26] = LCG feedback. Normalized to [0, 1] (Divide percentage by 100)
            x[27] = VBS feedback. Normalized to [0, 1] (Divide percentage by 100)

        Outputs:
            y[0] = rpm1 // rpm2
            y[1] = Aileron
            y[2] = Rudder
            y[3] = VBS
            y[4] = LCG
        """

        controls = self.onnx_inferenceSession.run(["action_mean"], {'obs': x})
        return np.array(controls[0], dtype=np.float32).flatten()

    def prepare_state(self, state):
        odom = state[0]
        waypoint = state[1]
        control = state[2]

        x = np.zeros((1, 28), dtype=np.float32)

        orientation = force_positive_quat(odom.pose.pose.orientation)
        # x[0-3] = Orientation. Mocap frame. Quaternion
        x[0, 0] = orientation.x
        x[0, 1] = orientation.y
        x[0, 2] = orientation.z
        x[0, 3] = orientation.w

        # x[4-6] = Linear velocity. Body Frame, Vector3
        linear = limit_vector(np.array(vector_to_list(odom.twist.twist.linear)) * 1.5)
        x[0, 4:7] = linear

        # x[7-9] = Angular velocity. Body Frame, Vector3
        angular = limit_vector(np.array(vector_to_list(odom.twist.twist.angular)) * 3)
        x[0, 7:10] = angular

        # x[10-12] = Relative vector to waypoint. Body Frame, Vector3
        x[0, 10] = waypoint.pose.position.x
        x[0, 11] = waypoint.pose.position.y
        x[0, 12] = waypoint.pose.position.z

        waypoint_orientation = force_positive_quat(waypoint.pose.orientation)
        # x[13-16] = Relative orientation of waypoint w.r.p body. Body Frame, Quaternion
        x[0, 13] = waypoint_orientation.x
        x[0, 14] = waypoint_orientation.y
        x[0, 15] = waypoint_orientation.z
        x[0, 16] = waypoint_orientation.w

        # x[17] = Target velocity magnitude. Between 0.1 - 0.5. Normalized to 0.2 - 1
        x[0, 17] = 1

        # x[18-20] = Absolute position. Mocap frame, Vector3
        x[0, 18] = range_normalize(odom.pose.pose.position.x, 0.8, 8.2)
        x[0, 19] = range_normalize(odom.pose.pose.position.y, 1.5, -1.5)
        x[0, 20] = range_normalize(odom.pose.pose.position.z, 0, 2.8)

        # x[21-25] = Previous/current "action" vector.
        x[0, 21] = control['rpm1'] / 1000
        x[0, 22] = control['stern'] / 0.2
        x[0, 23] = control['rudder'] / 0.2
        x[0, 24] = ((control['vbs'] / 100) + 1) / 2
        x[0, 25] = ((control['lcg'] / 100) + 1) / 2

        # x[26] = LCG feedback. Normalized to [0, 1] (Divide percentage by 100)
        # x[27] = VBS feedback. Normalized to [0, 1] (Divide percentage by 100)
        x[0, 26] = control['lcg'] / 100  # Normalized differently, unfortunately
        x[0, 27] = control['vbs'] / 100 #TODO: Need actual feedback

        x = self.normalization(x)
        return np.clip(x, -1, 1)

    def rescale_outputs(self, y):
        """
        Rescale NN outputs to values actually used by SAM.
        Note, this depends on how the NN was configured during training.
        Must be cross-referenced to the training configuration in Unity.
        """
        y = np.array(y, dtype=np.float32)

        y = np.clip(y, -1, 1)
        y[0] = y[0] * self.rpm_max
        y[1] = y[1] * self.aileron_angle_max
        y[2] = y[2] * self.rudder_angle_max
        y[3] = ((y[3] + 1) * 0.5) * self.vbs_max
        y[4] = ((y[4] + 1) * 0.5) * self.lcg_max
        return y

def force_positive_quat(quaternion):
    if quaternion.w < 0:
        quaternion.x = -quaternion.x
        quaternion.y = -quaternion.y
        quaternion.z = -quaternion.z
        quaternion.w = -quaternion.w
    return quaternion

def norm_move(x):
    x[:, 10:13] = limit_vector(x[:, 10:13] / 10)
    return x


def norm_align(x):
    x[:, 10:13] = limit_vector(x[:, 10:13] / 2)
    return x