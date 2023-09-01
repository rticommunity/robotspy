# (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
from itertools import chain
import os

import logging

class LvlFilter(logging.Filter):
  LVL_NAMES = {
    logging.DEBUG: "D",
    logging.INFO: "I",
    logging.WARNING: "W",
    logging.ERROR: "E",
    logging.CRITICAL: "C",
  }

  def filter(self, record: logging.LogRecord) -> bool:
    record.lvlname = self.LVL_NAMES[record.levelno]
    return True

class CustomFormatter(logging.Formatter):
  blue = "\x1b[34;20m"
  magenta = "\x1b[35;20m"
  cyan = "\x1b[36;20m"
  grey = "\x1b[38;20m"
  yellow = "\x1b[33;20m"
  red = "\x1b[31;20m"
  bold_red = "\x1b[31;1m"
  reset = "\x1b[0m"
  format = '[%(lvlname)s] %(message)s'

  FORMATS = {
    logging.DEBUG: magenta + format + reset,
    logging.INFO: cyan + format + reset,
    logging.WARNING: yellow + format + reset,
    logging.ERROR: red + format + reset,
    logging.CRITICAL: bold_red + format + reset
  }

  def format(self, record):
    log_fmt = self.FORMATS[record.levelno]
    formatter = logging.Formatter(log_fmt)
    return formatter.format(record)

_log_level = os.getenv("LOG_LEVEL") or "ERROR"
_global_logger = None
_loggers = {}

def log_level(lvl: str):
  global _log_level
  for l in chain(
      [_global_logger] if _global_logger is not None else [],
      _loggers.values()):
    l.setLevel(lvl)
    l.handlers[0].setLevel(lvl)
  _log_level = lvl

def logger(module : str = None) -> logging.Logger:
  global _loggers, _global_logger
  if module is not None:
    logger = _loggers.get(module)
    if logger is not None:
      return logger
    logger = logging.getLogger(module)
  else:
    if _global_logger is not None:
      return _global_logger
    logger = logging.getLogger()
  log_level = getattr(logging, _log_level)
  logger.addFilter(LvlFilter())
  logger.setLevel(log_level)
  # Disable propagation to parent loggers to avoid duplicate messages
  logger.propagate = False
  # create console handler with a higher log level
  ch = logging.StreamHandler()
  ch.setLevel(log_level)
  ch.setFormatter(CustomFormatter())
  logger.addHandler(ch)
  logger.handlers = [ch]
  if module is not None:
    _loggers[module] = logger
  else:
    _global_logger = logger
  return logger
