# import visual_servoing_utils_main as utils
from autolab_core import rigid_transformations as rt
from yumipy import YuMiState

class YumiConstants:

    T_gripper_gripperV = rt.RigidTransform(rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
                                           from_frame='gripper', to_frame='obj')

    T_rightH_yumi_1 = rt.RigidTransform(rotation=[[0, 0, 1], [1, 0, 0], [0, 1, 0]],
                                        translation=[0.6256, -0.15060002, 0.3616],
                                        from_frame='home', to_frame='yumi')

    T_rightH_yumi_2 = rt.RigidTransform(rotation=[[0, 0, 1], [1, 0, 0], [0, 1, 0]],
                                        translation=[0.6256 - 0.1, -0.15060002 + 0.1, 0.3616],
                                        from_frame='home', to_frame='yumi')

    T_rightH_yumi_3 = rt.RigidTransform(rotation=[[0, 0, 1], [1, 0, 0], [0, 1, 0]],
                                        translation=[0.6256 - 0.1, -0.15060002 + 0.1, 0.3616 - 0.05],
                                        from_frame='home', to_frame='yumi')

    T_leftH_yumi_1 = rt.RigidTransform(rotation=[[1, 0, 0], [0, 0, -1], [0, 1, 0]],
                                       translation=[0.52070004, 0.07340001, 0.3574],
                                       from_frame='home', to_frame='yumi')

    T_leftH_yumi_2 = rt.RigidTransform(rotation=[[1, 0, 0], [0, 0, -1], [0, 1, 0]],
                                       translation=[0.67080003 - 0.15, -0.12650001 + 0.2, 0.35720003],
                                       from_frame='home', to_frame='yumi')

    T_board_yumi = rt.RigidTransform(translation=[0.3984, 0, 0.0837],
                                     from_frame='board', to_frame='yumi')


    board_center = rt.RigidTransform(rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
                                     translation=[0.42971, -0.004, -0.057],
                                     from_frame='yumi', to_frame='world')

    T_rightH_yumi = rt.RigidTransform(rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
                                      translation=[0.3984, 0 - 8 * 0.0375, 0.0837],
                                      from_frame='home', to_frame='yumi')

    T_leftH_yumi = rt.RigidTransform(rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
                                     translation=[0.3984, 0 + 8 * 0.0375, 0.0837],
                                     # translation=[0.3984, 0 + 8*0.0375, 0.0837],
                                     from_frame='home', to_frame='yumi')

    right_threading_home = YuMiState([101.34, -83.3, 54.01, -44.34, -82.32, -26.22, -76.76])
    left_threading_home = YuMiState([-74.73, -70.63, 9.62, 15.86, 65.74, -169.18, 50.61])

    right_pickup_home = YuMiState([80.92, -118.47, 39.2, -139.35, 107.91, 4.83, -26.93])
    left_pickup_home = YuMiState([-75.32, -114.45, 37.59, 134.52, 102.66, -8.73, 42.77])


