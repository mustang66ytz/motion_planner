import pytest
from robots.dummyrobot import Dummyrobot

@pytest.fixture(scope="function")
def robot_obj():
    robot_obj = Dummyrobot()
    yield robot_obj
    # TODO release or whatever?

def test_robot_getHand(robot_obj):
    assert robot_obj.get_hand("right").name == 'RightArm'
    assert robot_obj.get_hand("left").name == 'LeftArm'
    # case sensitive
    assert robot_obj.get_hand("left").name != 'Leftarm'

def test_robot_getStatesForHand(robot_obj):
    expected = [10, 0, 0, 0, 0, 0, 0]
    rightArm = robot_obj.get_hand("right")
    rightArm.set_state(expected)

    actualState = rightArm.get_state()
    actual = actualState.getStates()
    print("expected: ", expected, " returned: ", actual)
    assert (expected == actual)