class YumiUtils(object):
    def __init__(self, yc):
        self.yplay = yc
        # self.yplay.get_hand('right').close_gripper(force=10)
        # self.yplay.get_hand('left').close_gripper(force=10)
        # self.yplay.get_hand('right').goto_pose(board_center.as_frames('yumi', 'world'), relative=True)

    def control_hand(self, verbose=False):
        print 'Press G to get control info ...'
        print 'Start moving ...'
        unit = 0.01

        while True:
            try:
                left_end_effector = self.yplay.get_hand('left').get_pose().matrix
                left_end_effector[0:3, -1] = left_end_effector[0:3, -1] * 100
                right_end_effector = self.yplay.get_hand('left').get_pose().matrix
                right_end_effector[0:3, -1] = right_end_effector[0:3, -1] * 100

                key = raw_input()
                if key == 'w':
                    self.yplay.move_delta('left', trans=[unit, 0, 0], rotation=None)
                elif key == 's':
                    self.yplay.move_delta('left', trans=[-unit, 0, 0], rotation=None)
                elif key == 'a':
                    self.yplay.move_delta('left', trans=[0, unit, 0], rotation=None)
                elif key == 'd':
                    self.yplay.move_delta('left', trans=[0, -unit, 0], rotation=None)
                elif key == 'q':
                    self.yplay.move_delta('left', trans=[0, 0, unit], rotation=None)
                elif key == 'e':
                    self.yplay.move_delta('left', trans=[0, 0, -unit], rotation=None)
                elif key == 'z':
                    self.yplay.rotate_x_axis(hand='left', theta=10 * unit)
                elif key == 'c':
                    self.yplay.rotate_x_axis(hand='left', theta=-10 * unit)

                elif key == 'i':
                    self.yplay.move_delta('right', trans=[unit, 0, 0], rotation=None)
                elif key == 'k':
                    self.yplay.move_delta('right', trans=[-unit, 0, 0], rotation=None)
                elif key == 'j':
                    self.yplay.move_delta('right', trans=[0, unit, 0], rotation=None)
                elif key == 'l':
                    self.yplay.move_delta('right', trans=[0, -unit, 0], rotation=None)
                elif key == 'u':
                    self.yplay.move_delta('right', trans=[0, 0, unit], rotation=None)
                elif key == 'o':
                    self.yplay.move_delta('right', trans=[0, 0, -unit], rotation=None)
                elif key == 'm':
                    self.yplay.rotate_x_axis(hand='right', theta=10 * unit)
                elif key == '.':
                    self.yplay.rotate_x_axis(hand='right', theta=-10 * unit)

                if verbose:
                    # print('Left pose :', self.yplay.get_pose('left'))
                    # print('Left state :', self.yplay.get_state('left'))
                    print('Right pose :', self.yplay.get_pose('right'))
                    # print('Right state :', self.yplay.get_state('right'))

                if key == 'g':
                    print('Left arm : \n'
                          'W : Forward     S : Backward \n'
                          'A : Left        D : Right    \n'
                          'Q : Up          E : Down     \n')
                    print('Right arm : \n'
                          'I : Forward     K : Backward \n'
                          'J : Left        L : Right    \n'
                          'U : Up          O : Down     \n')
            except:
                pass

