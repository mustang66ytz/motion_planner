import pytest
from cameras.web_camera import web_camera


@pytest.fixture(scope="function")
def camera_obj():
	cam_obj = web_camera()
	yield cam_obj
	cam_obj.destroy()

def test_webcam_get_frame(camera_obj):
    assert camera_obj.get_frame()