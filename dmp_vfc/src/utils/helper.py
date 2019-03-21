#!/usr/bin/env python

import subprocess
import psutil
import logging
import time
import os


config_inst = None
helper_logger = None


def get_process_by_name(process_name, ignore_current_process=True):
  logging.info("get_process_by_name: Enter for process_name %s" % process_name)
  process_list = []
  for proc in psutil.process_iter():
    cmd_line = ' '.join(proc.cmdline())
    if ignore_current_process and proc.pid == os.getpid():
      continue
    if process_name in cmd_line:
      logging.info("get_process_by_name: found %s" % cmd_line)
      process_list.append(proc)
  return process_list


def terminate_processes_by_inst(process_list):
  for process_inst in process_list:
    logging.info("terminate_process_by_inst: terminate %s : %s" %
                 (process_inst.pid, ' '.join(process_inst.cmdline())))
    process_inst.terminate()
    try:
      process_inst.wait(timeout=2)
    except psutil.TimeoutExpired:
      process_inst.kill()
    logging.info("terminate_process_by_inst: finish one")


def start_process(args):
  p = psutil.Popen(args, shell=True)
  return p


def restart_actuator():
  tmp_process_list = get_process_by_name("actuator_bridge")
  terminate_processes_by_inst(tmp_process_list)
  actuator_bridge_args = "rosrun xr1controllerol actuator_bridge"
  p = start_process(actuator_bridge_args)
  return p


def init_logging():
  FORMAT = '%(asctime)-15s %(message)s'
  logging.basicConfig(format=FORMAT)
  logging.getLogger().setLevel(logging.INFO)


def set_config_inst(inst):
    global config_inst
    config_inst = inst


def get_config_inst():
    global config_inst

    if config_inst is None:
        raise ValueError("config not parsed yet?")

    return config_inst


def setup_logging():
    global helper_logger, log_config, config_inst

    args = config_inst

    if helper_logger is None:
        log_dir = args.log_dir

        error_file_handler_idx = 2
        file_handler_idx = 1

        if "error_file_handler" in args.log_handlers:
          log_config["handlers"]["error_file_handler"]["filename"] = os.path.join(log_dir, "app_error.log")
          log_config["handlers"]["error_file_handler"]["backupCount"] = args.log_backupCount
        else:
          del log_config["handlers"]["error_file_handler"]
          log_config["root"]["handlers"].remove("error_file_handler")
          error_file_handler_idx = -1

        if "file_handler" in args.log_handlers:
          log_config["handlers"]["file_handler"]["filename"] = os.path.join(log_dir, "app.log")
          log_config["handlers"]["file_handler"]["backupCount"] = args.log_backupCount
        else:
          del log_config["handlers"]["file_handler"]
          log_config["root"]["handlers"].remove("file_handler")
          file_handler_idx = -1
          error_file_handler_idx -= 1

        logging.config.dictConfig(log_config)
        root_logger = logging.getLogger()
        if error_file_handler_idx >= 0 and \
            os.path.exists(log_config["handlers"]["error_file_handler"]["filename"]):
            root_logger.handlers[error_file_handler_idx].doRollover()
        if file_handler_idx >= 0 and \
            os.path.exists(log_config["handlers"]["file_handler"]["filename"]):
              root_logger.handlers[file_handler_idx].doRollover()

        root_logger.info("App start logging")


def set_console_logger_level(lvl):
    root_logger = logging.getLogger()
    for handler in root_logger.handlers:
        if not isinstance(handler, logging.handlers.RotatingFileHandler):
            handler.setLevel(lvl)


def halt(description="halt"):
    logging.error("Enter halt: %s" % description)

    raise Exception(description)


log_config = {
  "version": 1,
  "disable_existing_loggers": False,
  "formatters": {
    "brief": {
      "class": "logging.Formatter",
      "format": "%(asctime)s  %(message)s"
    },
    "normal": {
      "class": "logging.Formatter",
      "format": "%(asctime)s  %(levelname)-8s [%(name)s] %(module)s:%(funcName)s:%(lineno)d: %(message)s"
    },
    "verbose": {
      "class": "logging.Formatter",
      "format": "%(asctime)s  %(levelname)-8s [%(name)s] %(module)s:%(funcName)s:%(lineno)d: [%(process)d]: %(threadName)s: %(message)s"
    }
  },
  "handlers": {
    "console":{
      "level": "INFO",
      "class": "logging.StreamHandler",
      "formatter": "brief",
      "stream" : "ext://sys.stdout"
    },
    "file_handler": {
      "class": "logging.handlers.RotatingFileHandler",
      "level": "INFO",
      "formatter": "brief",
      "filename": "app.log",
      "maxBytes": 10485760,
      "backupCount": 20,
      "encoding": "utf8"
    },
    "error_file_handler": {
      "class": "logging.handlers.RotatingFileHandler",
      "level": "ERROR",
      "formatter": "verbose",
      "filename": "app_error.log",
      "maxBytes": 10485760,
      "backupCount": 20,
      "encoding": "utf8"
    }
  },
  "loggers": { },
  "root": {
    "handlers": ["console", "file_handler", "error_file_handler"],
    "level": "INFO"
  }
}

if __name__ == '__main__':
  init_logging()
  if True:
    tmp_process_list = get_process_by_name("actuator_bridge")
    terminate_processes_by_inst(tmp_process_list)
    actuator_bridge_args = "rosrun xr1controllerol actuator_bridge"
    start_process(actuator_bridge_args)
