import numpy as np
import pytest
from nav_msgs.msg import Odometry

from sam_diving_controller.controllers.ONNXManager import ONNXManager, limit_vector, range_normalize

sut: ONNXManager


@pytest.fixture(autouse=True)
def test_before_after():
    # Code that will run before your test
    global sut

    sut = ONNXManager("DR_temp")

    yield  # A test function will be run at this point
    # Code that will run after your test


def test_model_load():
    assert sut.onnx_inferenceSession is not None, 'Interference session not created.'


def test_get_control_dummy():
    control = sut.get_control(np.zeros((1, 28), dtype=np.float32))

    assert control is not None, 'Output is None'
    assert control.shape == (5,), 'Output shape is incorrect.'''


def test_get_control_scaled():
    control = sut.get_control_scaled(np.ones((1, 28), dtype=np.float32) / 2)

    assert control is not None, 'Output is None'
    assert control.shape == (5,), 'Output shape is incorrect.'


def test_rescale_outputs_zeros():
    outputs = sut.rescale_outputs(np.array([0, 0, 0, 0, 0]))

    assert outputs.shape == (5,), 'Output shape is incorrect.'
    assert outputs[0] == 0
    assert outputs[1] == 0
    assert outputs[2] == 0
    assert outputs[3] == 50
    assert outputs[4] == 50


def test_rescale_outputs_outofrange_min():
    outputs = sut.rescale_outputs(np.array([-2, -2, -2, -2, -2]))

    assert outputs.shape == (5,), 'Output shape is incorrect.'
    assert outputs[0] == -sut.rpm_max
    assert outputs[1] == pytest.approx(-sut.aileron_angle_max, 0.0001)
    assert outputs[2] == pytest.approx(-sut.rudder_angle_max, 0.0001)
    assert outputs[3] == 0
    assert outputs[4] == 0


def test_rescale_outputs_outofrange_max():
    outputs = sut.rescale_outputs(np.array([2, 2, 2, 2, 2]))

    assert outputs.shape == (5,), 'Output shape is incorrect.'
    assert outputs[0] == sut.rpm_max
    assert outputs[1] == pytest.approx(sut.aileron_angle_max, 0.0001)
    assert outputs[2] == pytest.approx(sut.rudder_angle_max, 0.0001)
    assert outputs[3] == 100
    assert outputs[4] == 100


def test_rescale_outputs_testset():
    outputs = sut.rescale_outputs(np.array([0.75, -0.5, 0.2, -0.3, -0.4]))

    assert outputs.shape == (5,), 'Output shape is incorrect.'
    assert outputs[0] == 750
    assert outputs[1] == pytest.approx(-0.1, 0.0001)
    assert outputs[2] == pytest.approx(0.04, 0.0001)
    assert outputs[3] == 35
    assert outputs[4] == pytest.approx(30, 0.0001)


def test_rescale_outputs_customconfig_testset():
    sut.rpm_max = 600
    sut.rudder_angle_max = 0.35
    sut.aileron_angle_max = 0.66
    outputs = sut.rescale_outputs(np.array([0.75, -0.5, 0.2, -0.3, -0.4]))

    assert outputs.shape == (5,), 'Output shape is incorrect.'
    assert outputs[0] == 450
    assert outputs[1] == pytest.approx(-0.33, 0.0001)
    assert outputs[2] == pytest.approx(0.07, 0.0001)
    assert outputs[3] == 35
    assert outputs[4] == pytest.approx(30, 0.0001)


def test_prepare_state():
    state = sut.prepare_state((Odometry(), Odometry(), Odometry(), {
        "rpm1": 1000,
        "stern": 0.2,
        "rudder": 0,
        "vbs": 50,
        "lcg": 50,
    }))

    assert state.shape == (1, 28), 'Output shape is incorrect.'


def test_limit_vector():
    v = np.array([3.0, 4.0])
    vector = limit_vector(v)

    assert vector[0] == 0.6
    assert vector[1] == 0.8

    v2 = np.array([0.3, 0.4])
    vector = limit_vector(v2)
    assert vector[0] == 0.3
    assert vector[1] == 0.4

def test_range_normalize():

    assert range_normalize(5, 0, 10) == 0

    values = np.array([0, 5, 10])
    expected = np.array([-1.0, 0.0, 1.0])
    assert np.allclose(range_normalize(values, 0, 10), expected)