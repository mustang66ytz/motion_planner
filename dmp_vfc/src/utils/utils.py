import os
import errno


def assert_for_log(condition, error_message):
    assert condition, error_message


def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def check_arg_name(args):
    ''' Raise error if obsolete arg names are present. '''
    # Mapping - key: old name, value: new name
    name_dict = {'old_name':'new_name'}
    for old_name, new_name in name_dict.items():
        assert_for_log(old_name not in args,
                      "Error: Attempting to load old arg name [%s], please update to new name [%s]" %
                      (old_name,name_dict[old_name]))


def check_arg_required(args, required_list=[]):
  ''' Raise error if required but not set. '''
  for arg_name in required_list:
    assert_for_log(arg_name in args,
                   "Error: Arg [%s] is required but not set" % (arg_name,))


class Object(object):
  pass


def get_object():
  return Object()
