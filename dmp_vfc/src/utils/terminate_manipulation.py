#!/usr/bin/env python

from helper import *

if __name__ == '__main__':
  logging.getLogger().setLevel(logging.INFO)
  tmp_process_list = get_process_by_name("manipulation")
  terminate_processes_by_inst(tmp_process_list)
