import pytest
from cameras.localvideo import localvideo


@pytest.fixture(scope="function")
def camera_obj():
	cam_obj = localvideo()
	yield cam_obj
	cam_obj.destroy()

def test_webcam_get_frame(camera_obj):
    assert camera_obj.get_frame()