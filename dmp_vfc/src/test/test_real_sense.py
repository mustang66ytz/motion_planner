import pytest
from cameras.realsense import realsense


@pytest.fixture(scope="function")
def camera_obj():
	cam_obj = realsense()
	yield cam_obj
	cam_obj.destroy()


def test_librealsense_get_frame(camera_obj):
	assert camera_obj.get_frame()

