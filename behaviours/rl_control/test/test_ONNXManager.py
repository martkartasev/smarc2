import numpy as np
from ament_copyright.main import main
import pytest

from behaviours.rl_control.rl_control.ONNXManager import ONNXManager

sut: ONNXManager


@pytest.fixture(autouse=True)
def test_before_after():
    # Code that will run before your test, for example:
    global sut
    sut = ONNXManager("../resource/SAMSimple-9499700.onnx")
    # A test function will be run at this point
    yield
    # Code that will run after your test


def test_model_load():
    assert sut.onnx_inferenceSession is not None, 'Interference session not created.'


def test_get_control_dummy():
    control = sut.get_control(np.zeros((1, 14), dtype=np.float32))

    assert control is not None, 'Output is None'
    assert control.shape == (6,), 'Output shape is incorrect.'''


def test_get_control_scaled():
    control = sut.get_control_scaled(np.ones((1, 14), dtype=np.float32) / 2)

    assert control is not None, 'Output is None'
    assert control.shape == (6,), 'Output shape is incorrect.'


def test_rescale_outputs_zeros():
    outputs = sut.rescale_outputs(np.array([0, 0, 0, 0, 0, 0]))

    assert outputs.shape == (6,), 'Output shape is incorrect.'
    assert outputs[0] == 0
    assert outputs[1] == 0
    assert outputs[2] == 50
    assert outputs[3] == 0
    assert outputs[4] == 0


def test_rescale_outputs_outofrange_min():
    outputs = sut.rescale_outputs(np.array([-2, -2, -2, -2, -2, -2]))

    assert outputs.shape == (6,), 'Output shape is incorrect.'
    assert outputs[0] == -sut.rpm_max
    assert outputs[1] == -sut.rpm_max
    assert outputs[2] == 0
    assert outputs[3] == -sut.aileron_angle_max
    assert outputs[4] == -sut.rudder_angle_max
    assert outputs[5] == 0


def test_rescale_outputs_outofrange_max():
    outputs = sut.rescale_outputs(np.array([2, 2, 2, 2, 2, 2]))

    assert outputs.shape == (6,), 'Output shape is incorrect.'
    assert outputs[0] == sut.rpm_max
    assert outputs[1] == sut.rpm_max
    assert outputs[2] == 100
    assert outputs[3] == sut.aileron_angle_max
    assert outputs[4] == sut.rudder_angle_max
    assert outputs[5] == 100


def test_rescale_outputs_testset():
    outputs = sut.rescale_outputs(np.array([0.75, -0.3, -0.5, 0.2, -0.4, 0.5]))

    assert outputs.shape == (6,), 'Output shape is incorrect.'
    assert outputs[0] == 750
    assert outputs[1] == -300
    assert outputs[2] == 25
    assert outputs[3] == pytest.approx(0.04, 0.0001)
    assert outputs[4] == pytest.approx(-0.08, 0.0001)
    assert outputs[5] == pytest.approx(75, 0.0001)


def test_rescale_outputs_customconfig_testset():
    sut.rpm_max = 600
    sut.rudder_angle_max = 0.35
    sut.aileron_angle_max = 0.66
    outputs = sut.rescale_outputs(np.array([0.75, -0.3, -0.5, 0.2, -0.4, 0.5]))

    assert outputs.shape == (6,), 'Output shape is incorrect.'
    assert outputs[0] == 450
    assert outputs[1] == -180
    assert outputs[2] == 25
    assert outputs[3] == pytest.approx(0.132, 0.0001)
    assert outputs[4] == pytest.approx(-0.14, 0.0001)
    assert outputs[5] == pytest.approx(75, 0.0001)