import pickle
import os
#import visualization

left_home_trans = [-0.1856695794148949, -0.2124832623176604, -0.016995409634506024]
left_home_quaternion = [0.00022654249216508434, -0.0005469729404355046, 0.00017302118047832443, 0.9999998210313703]
right_home_trans = [-0.18567598453464002, 0.21242531723787053, -0.017007879232618376]
right_home_quaternion = [1.754584572893577e-05, 0.0007713534482236648, -0.0004400999228857989, 0.9999996167589579]

left_goal_trans = [0.203012626405, -0.10336711683, 0.367727894224]
left_goal_quaternion = [-0.146324317422, 0.813324406607, -0.0763794300819, 0.557905735165]
right_goal_trans = [0.201876243218, 0.105297188055, 0.368928875097]
right_goal_quaternion = [0.150617178566, 0.80436764101, 0.0693176313726, 0.570528047014]

left_goal_trans1 = [0.16916795403606658, -0.364444400295356, 0.33720842643207116]
left_goal_quaternion1 = [-0.32831267882010345, 0.5721403113873615, 0.18591296942035118, 0.7282188138048439]

# hard-coded part to be removed when the vision unit is ready to use:
left_home_pos = left_home_quaternion + left_home_trans
right_home_pos = right_home_quaternion + right_home_trans

end_left_config = left_goal_quaternion1+left_goal_trans1
end_right_config = right_goal_quaternion+right_goal_trans

# TODO: fill in the second primitive start position
primitive2_left_start = []
primitive2_right_start = []

waypoints_right = [end_right_config]
waypoints_left = [end_left_config, left_home_pos]

# this function reformats the ginger quaternion order to fit the demo_data collected from yumi
# Do not use this function if demo data is collected from ginger robot
def reformat_ginger_quaternion():
    global left_home_quaternion
    global left_goal_quaternion
    global right_home_quaternion
    global right_goal_quaternion
    global left_home_pos
    global right_home_pos
    global end_left_config
    global end_right_config
    global waypoints_right
    global waypoints_left

    left_home_quaternion = left_home_quaternion[-3:]+[left_home_quaternion[0]]
    left_goal_quaternion = left_goal_quaternion[-3:]+[left_goal_quaternion[0]]
    right_home_quaternion = right_home_quaternion[-3:]+[right_home_quaternion[0]]
    right_goal_quaternion = right_goal_quaternion[-3:]+[right_goal_quaternion[0]]

    left_home_pos = left_home_quaternion+left_home_trans
    right_home_pos = right_home_quaternion+right_home_trans
    end_left_config = left_goal_quaternion+left_goal_trans
    end_right_config = right_goal_quaternion+right_goal_trans

    waypoints_left = [end_left_config]
    waypoints_right = [end_right_config]

def visual_recording(primitive_folder, save_to_file=True, load_transformation_file=False):
    if save_to_file:
        print "Currently under construction"
        #visualization.entry_point()
        return True
    else:
        return False

# this function is to test the dmp using hard-coded motion primitive waypoints substituting the visual recording
def get_waypoints_from_camera(primitive_folder, arm, save_to_file=True, load_transformation_file=False):
    if load_transformation_file:
        g_camera_base = load_pickle_file("camera_to_base.pickle") # camera_to_base should be a 4X4 matrix

    primitive_filenames = os.listdir(primitive_folder)
    primitive_filenames.sort()
    del primitive_filenames[-1]
    mp_way_points = []

    #reformat_ginger_quaternion()

    # assign the home position as the first item in the motion primitive way point
    if arm == "left":
        mp_way_points.append(left_home_pos)
    if arm == "right":
        mp_way_points.append(right_home_pos)

    for counter, primitives in enumerate(primitive_filenames):
        # add vision part here to generate a way point
        if arm == "left":
            temp_waypoint = waypoints_left[counter]
        if arm == "right":
            temp_waypoint = waypoints_right[counter]
        mp_way_points.append(temp_waypoint)

    if save_to_file:
        # store the motion primitives' starting waypoints into a pickle file
        pickling_on = open("waypoint_targets/mp_way_points.pickle", "wb")
        pickle.dump(mp_way_points, pickling_on)
        pickling_on.close()
        return True
    else:
        return mp_way_points

def load_pickle_file(filename):
    pickle_off = open(filename, "rb")
    result = pickle.load(pickle_off)
    pickle_off.close()
    return result


if __name__ == "__main__":
    folder_name = "mp1"
    arm_controlled = "right"
    get_waypoints_from_camera(folder_name, arm_controlled)
