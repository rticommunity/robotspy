# (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
import signal
import sys
import argparse
from pathlib import Path
import traceback
import threading

from .output_emitter import (
  CombinedOutputEmitter,
  IdlTypesEmitter,
  ListOnlyEmitter,
  TopicsListEmitter
)

from .types_scraper import TypesScraper
from .log import logger, log_level
log = logger()

_TYPES_SCRAPER = None
def exit_handler(sig, frame):
  global _TYPES_SCRAPER
  if _TYPES_SCRAPER is not None:
    # m = _TYPES_SCRAPER
    _TYPES_SCRAPER.stop()

class TimeExpr:
  def __init__(self, val : str) -> None:
    if len(val) == 0:
      raise RuntimeError("invalid empty time expression")
    if val[-1].lower() in ("s", "m", "h"):
      unit = val[-1].lower()
      self._val = int(val[0:-1]) * (
        1 if unit == "s" else
        60 if unit == "m" else
        (60*60)
      )
      self._unit = unit
    else:
      self._val = int(val)
      self._unit = "s"
    self._timer = None
  
  def schedule(self, event):
    self._timer = threading.Timer(float(self._val), event)
    self._timer.start()

  def cancel(self):
    if self._timer is not None:
      self._timer.cancel()

  def __str__(self) -> str:
    if self._unit == "s":
      return f"{self._val} seconds"
    elif self._unit == "m":
      return f"{int(self._val/60)} minutes"
    elif self._unit == "h":
      return f"{int(self._val/(60*60))} hours"

class TypesScraperCli:
  @staticmethod
  def parse_args():
    parser = argparse.ArgumentParser(
      description="Inspect a DDS domain (or your local filesystem), and dump all detected ROS types to IDL.",
      add_help=True)
    in_opts = parser.add_argument_group("Input Options")
    in_opts.add_argument("-d", "--domain",
      help="Join the specified DOMAIN and detect types from DDS discovery information. Repeat to join multiple domains.",
      metavar="DOMAIN-ID[/QOS-PROFILE]",
      action="append",
      default=[])
    in_opts.add_argument("-i", "--input",
      metavar="FILE",
      action="append",
      help="Read type names from the specified FILE (or standard input if - is used). Repeat to read from multiple files.",
      type=Path,
      default=[])
    in_opts.add_argument("-p", "--parse",
      metavar="FILE",
      action="append",
      help="Parse output previously generated by types_scraper_cpp.",
      type=Path,
      default=[])
    in_opts.add_argument("-D", "--directory",
      metavar="DIR",
      action="append",
      help="Determine type names by searching for .msg files in the specified directory. Deduce package names from the directory structure.",
      type=Path,
      default=[])
    in_opts.add_argument("-f", "--filter",
      help="Only consider ROS types whose name matches the provided regular expression. The expression is matched against a type's \"canonical\" form, i.e. <package>::(msg|srv)::<type>.",
      default=None)
    in_opts.add_argument("-F", "--raw-filter",
      help="Only consider types whose name matches the provided regular expression. The expression will be matched against the \"raw\" type (e.g. including underscores and the \"dds[_]\" module).",
      default=None)
    out_opts = parser.add_argument_group("Output Options")
    out_opts.add_argument("-o", "--output",
      help="Dump output to the specified file instead of standard out.",
      type=Path,
      default=None)
    out_opts.add_argument("--split",
      help="Split each detected type into a separate file. Files will be placed in subdirectories based on their package names. Use --output to specify the base path for all generated files.",
      action="store_true",
      default=False)
    out_opts.add_argument("--flat",
      help="When splitting into multiple files, place all files in a single directory. The package name will be encoded in the file name",
      action="store_true",
      default=False)
    file_write_opts = out_opts.add_mutually_exclusive_group()
    file_write_opts.add_argument("-a", "--append",
      help="Append detected types to output file. When splitting into multiple files, allow new files to be added to existing directories.",
      action="store_true",
      default=False)
    file_write_opts.add_argument("-O", "--overwrite",
      help="Overwrite output files and directories if they already exists. Existing directories will be deleted and recreated. ",
      action="store_true",
      default=False)
    out_type_opts = out_opts.add_mutually_exclusive_group()
    out_type_opts.add_argument("-l", "--list",
      help="Only generate a list of detected type names instead of their IDL.",
      action="store_true",
      default=False)
    out_type_opts.add_argument("-m", "--mangle",
      help="Output ROS types using their \"mangled\" type name (e.g. \"my_types::msg::dds_::MyType_\").",
      action="store_true",
      default=False)
    out_opts.add_argument("-T", "--topics-list",
      metavar="FILE",
      help="Generate a list of all detected topics in JSON/YAML format.",
      type=Path,
      default=None)
    out_opts.add_argument("--topics-only",
      action="store_true",
      help="Only generate list of all detected topics in JSON/YAML format and print it to stdout.",
      default=False)
    fmt_opts = parser.add_argument_group("Formatting Options")
    fmt_opts.add_argument("--no-indent",
      help="Do not indent the generated IDL.",
      action="store_true",
      default=False)
    fmt_opts.add_argument("--indent-depth",
      metavar="DEPTH",
      help="Indentation depth to add to the generated IDL. Default: %(default)s.",
      type=int,
      default=0)
    fmt_opts.add_argument("--indent-step",
      metavar="DEPTH",
      help="Size of each indentation step added to the generated IDL. Default: %(default)s.",
      type=int,
      default=2)
    fmt_opts.add_argument("--yaml",
      help="Generate YAML instead of JSON.",
      action="store_true",
      default=False)
    other_opts = parser.add_argument_group("Other Options")
    other_opts.add_argument("--exit-after",
      metavar="TIME",
      help="Exit after the specified amount of time. TIME is interpreted as seconds, unless one of the following prefixes: s, m, h. E.g. \"10m\"",
      type=TimeExpr,
      default=None)
    other_opts.add_argument("-v", "--verbose",
      help="Produce more logging output. Repeat to increase even more",
      action="count",
      default=0)
    other_opts.add_argument("--debug",
      help=argparse.SUPPRESS,
      action="store_true",
      default=False)

    # Additional options from C++ scraper
    other_opts.add_argument("--compatibility-mode",
      help=argparse.SUPPRESS,
      choices=("rmw_connext_cpp", "rmw_cyclonedds_cpp"),
      default=None)
    other_opts.add_argument("--request-reply-mapping",
      help=argparse.SUPPRESS,
      choices=("basic", "extended"),
      default=None)

    return parser.parse_args()

  def __init__(self) -> None:
    self.args = self.parse_args()

  def main(self):
    try:
      if sys.stdin.isatty() and Path("-")  in self.args.input:
        raise RuntimeError("pipe something into the process to read from stdin")

      log_level("DEBUG" if self.args.verbose > 1
        else "INFO" if self.args.verbose > 0
        else "WARNING")

      no_domain = len(self.args.domain) == 0
      no_input = len(self.args.input) == 0
      no_dirs = len(self.args.directory) == 0
      no_parse = len(self.args.parse) == 0
      no_input = no_domain and no_input and no_dirs and no_parse
      if no_input:
        log.info("no input specified, joining DDS domain 0")
        self.args.domain.append("0")

      emitters = []
      if self.args.list:
        emitters.append(ListOnlyEmitter(
          topics_only=self.args.topics_only,
          output_path=self.args.output))
      else:
        if self.args.topics_list:
          emitters.append(TopicsListEmitter(
            output_path=self.args.topics_list,
            yaml=self.args.yaml,
            no_indent=self.args.no_indent,
            indent_step=self.args.indent_step,
            indent_depth=self.args.indent_depth,
            overwrite=self.args.overwrite,
            split=self.args.split))
        emitters.append(IdlTypesEmitter(
          flat=self.args.flat,
          split=self.args.split,
          output_path=self.args.output,
          no_indent=self.args.no_indent,
          indent_step=self.args.indent_step,
          indent_depth=self.args.indent_depth,
          overwrite=self.args.overwrite))

      scraper = TypesScraper(
        CombinedOutputEmitter(*emitters),
        pregenerated=self.args.parse,
        directories=self.args.directory,
        domains=self.args.domain,
        input_files=self.args.input,
        filter=self.args.filter,
        verbosity=self.args.verbose,
        mangle_ros_names=self.args.mangle,
        raw_filter=self.args.raw_filter,
        compatibility_mode=self.args.compatibility_mode,
        req_reply_mapping=self.args.request_reply_mapping,
        debug=self.args.debug)

      # Setup a signal handler for SIGNINT (i.e. CTRL+C)
      global _TYPES_SCRAPER
      _TYPES_SCRAPER = scraper
      signal.signal(signal.SIGINT, exit_handler)

      # Schedule a callback to the exit handler if running for a limited time
      if self.args.exit_after is not None:
        log.info(f"running for {self.args.exit_after}")
        def _scheduled_exit():
          log.info(f"terminating after {self.args.exit_after}")
          scraper.stop()
        self.args.exit_after.schedule(_scheduled_exit)

      scraper.start()
      scraper.run()
      scraper.stop()

      return 0
    except Exception as e:
      log.error(f"{e}")
      if self.args.verbose > 1 or self.args.debug:
        traceback.print_exc()
      return 1
