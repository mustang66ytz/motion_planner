import pickle
import os

def get_mp_gripper_states(primitive_folder):
    primitive_filenames = os.listdir(primitive_folder)
    primitive_filenames.sort()
    del primitive_filenames[-1]
    gripper_states = []

    gripper_states.append(raw_input("Enter the state for the "+str(0)+" gripper state, close or open?"))
    for counter, primitives in enumerate(primitive_filenames):
        gripper_states.append(raw_input("Enter the state for the "+str(counter+1)+" gripper state, close or open?"))

    pickling_on = open(primitive_folder+"/dmp_params/gripper_states.pickle", "wb")
    pickle.dump(gripper_states, pickling_on)
    pickling_on.close()


def read_pickle(filename):
    pickle_off = open(filename, "rb")
    result = pickle.load(pickle_off)
    pickle_off.close()
    return result


if __name__ == "__main__":
    folder_name = "mp1"
    get_mp_gripper_states(folder_name)
