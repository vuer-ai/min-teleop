"""G1-29 Arm IK Solver (14 DoF).

Simplified from xr_teleoperate/teleop/robot_control/robot_arm_ik.py.
Uses Pinocchio + CasADi for constrained nonlinear IK optimization.
"""

import logging
import os

import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin

log = logging.getLogger(__name__)

_ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets", "g1")


# ---------------------------------------------------------------------------
# Weighted Moving Filter (inlined from xr_teleoperate/teleop/utils)
# ---------------------------------------------------------------------------
class WeightedMovingFilter:
    def __init__(self, weights, data_size=14):
        self._window_size = len(weights)
        self._weights = np.array(weights)
        assert np.isclose(np.sum(self._weights), 1.0)
        self._data_size = data_size
        self._filtered_data = np.zeros(self._data_size)
        self._data_queue = []

    def _apply_filter(self):
        if len(self._data_queue) < self._window_size:
            return self._data_queue[-1]
        data_array = np.array(self._data_queue)
        temp = np.zeros(self._data_size)
        for i in range(self._data_size):
            temp[i] = np.convolve(data_array[:, i], self._weights, mode="valid")[-1]
        return temp

    def add_data(self, new_data):
        assert len(new_data) == self._data_size
        if self._data_queue and np.array_equal(new_data, self._data_queue[-1]):
            return
        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)
        self._data_queue.append(new_data)
        self._filtered_data = self._apply_filter()

    @property
    def filtered_data(self):
        return self._filtered_data


# ---------------------------------------------------------------------------
# G1 29-DoF Arm IK  (reduced to 14 DoF by locking legs/waist/hands)
# ---------------------------------------------------------------------------
class G1ArmIK:
    def __init__(self, urdf_dir=None):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        model_dir = urdf_dir or _ASSETS_DIR
        urdf_path = os.path.join(model_dir, "g1_body29_hand14.urdf")

        log.info("Loading G1-29 URDF from %s ...", urdf_path)
        robot = pin.RobotWrapper.BuildFromURDF(urdf_path, model_dir)

        joints_to_lock = [
            # Legs (12)
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            # Waist (3)
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            # Hands (14)
            "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
            "left_hand_middle_0_joint", "left_hand_middle_1_joint",
            "left_hand_index_0_joint", "left_hand_index_1_joint",
            "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
            "right_hand_index_0_joint", "right_hand_index_1_joint",
            "right_hand_middle_0_joint", "right_hand_middle_1_joint",
        ]

        self.reduced_robot = robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=np.array([0.0] * robot.model.nq),
        )

        # Add end-effector frames
        self.reduced_robot.model.addFrame(
            pin.Frame("L_ee",
                       self.reduced_robot.model.getJointId("left_wrist_yaw_joint"),
                       pin.SE3(np.eye(3), np.array([0.05, 0, 0])),
                       pin.FrameType.OP_FRAME)
        )
        self.reduced_robot.model.addFrame(
            pin.Frame("R_ee",
                       self.reduced_robot.model.getJointId("right_wrist_yaw_joint"),
                       pin.SE3(np.eye(3), np.array([0.05, 0, 0])),
                       pin.FrameType.OP_FRAME)
        )

        # CasADi symbolic model
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        nq = self.reduced_robot.model.nq
        self.cq = casadi.SX.sym("q", nq, 1)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        L_id = self.reduced_robot.model.getFrameId("L_ee")
        R_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [casadi.vertcat(
                self.cdata.oMf[L_id].translation - self.cTf_l[:3, 3],
                self.cdata.oMf[R_id].translation - self.cTf_r[:3, 3],
            )],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [casadi.vertcat(
                cpin.log3(self.cdata.oMf[L_id].rotation @ self.cTf_l[:3, :3].T),
                cpin.log3(self.cdata.oMf[R_id].rotation @ self.cTf_r[:3, :3].T),
            )],
        )

        # Optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(nq)
        self.var_q_last = self.opti.parameter(nq)
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)

        trans_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        rot_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        reg_cost = casadi.sumsqr(self.var_q)
        smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit,
        ))
        self.opti.minimize(50 * trans_cost + rot_cost + 0.02 * reg_cost + 0.1 * smooth_cost)

        self.opti.solver("ipopt", {
            "expand": True,
            "detect_simple_bounds": True,
            "calc_lam_p": False,
            "print_time": False,
            "ipopt.sb": "yes",
            "ipopt.print_level": 0,
            "ipopt.max_iter": 30,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 5e-4,
            "ipopt.acceptable_iter": 5,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.derivative_test": "none",
            "ipopt.jacobian_approximation": "exact",
        })

        self.init_data = np.zeros(nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14)

        log.info("G1ArmIK ready (nq=%d)", nq)

    def solve_ik(self, left_wrist, right_wrist):
        """Solve IK for both arms.

        Args:
            left_wrist:  4x4 homogeneous target for left end-effector.
            right_wrist: 4x4 homogeneous target for right end-effector.

        Returns:
            (joint_q[14], torques[14]) or None on failure.
        """
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            self.opti.solve()
            sol_q = self.opti.value(self.var_q)
        except Exception as e:
            log.warning("IK solve failed: %s", e)
            sol_q = self.opti.debug.value(self.var_q)

        self.smooth_filter.add_data(sol_q)
        sol_q = self.smooth_filter.filtered_data

        v = np.zeros(self.reduced_robot.model.nv)
        sol_tauff = pin.rnea(
            self.reduced_robot.model, self.reduced_robot.data,
            sol_q, v, np.zeros(self.reduced_robot.model.nv),
        )

        self.init_data = sol_q
        return sol_q, sol_tauff
