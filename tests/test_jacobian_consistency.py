import numpy as np
import pytest

pytest.importorskip("pinocchio")

from nero.kinematics.analytic_IK_solver import Pinocchio_Solver
from nero.kinematics.debug_tools import (
    DEFAULT_NERO_JOINT_LIMITS,
    numerical_jacobian,
    sample_random_q,
)


def test_pinocchio_jacobian_matches_finite_difference():
    rng = np.random.default_rng(11)
    solver = Pinocchio_Solver(
        joint_limits=DEFAULT_NERO_JOINT_LIMITS,
        dt=0.05,
        max_iterations=80,
    )
    q = sample_random_q(rng, DEFAULT_NERO_JOINT_LIMITS, num_samples=1, margin=0.15)[0]

    analytic_J = solver.jacobian_matrix(q)
    numeric_J = numerical_jacobian(solver.fk_matrix, q, eps=1e-6)

    assert analytic_J.shape == (6, 7)
    assert numeric_J.shape == (6, 7)

    translational_inf_err = float(np.max(np.abs(analytic_J[:3] - numeric_J[:3])))
    rotational_inf_err = float(np.max(np.abs(analytic_J[3:] - numeric_J[3:])))

    assert translational_inf_err < 1e-4
    assert rotational_inf_err < 1e-3
