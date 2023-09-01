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
import sys
from pathlib import Path
from typing import Callable, Iterable, Optional


from .detected import DetectedTopic, DetectedType
from .monitored_process import MonitoredProcess
from .output_parser import OutputParser
from .output_emitter import OutputEmitter

from .log import logger
log = logger()

def _scan_directories(directories: Iterable[str]) -> dict:
  detected = []
  for search_dir in directories:
    for (dirpath, _, filenames) in os.walk(search_dir):
      dirpath = Path(dirpath)
      for f in filter(lambda f: f.endswith(".msg"), filenames):
        detected.append(dirpath / f)
  result = []
  for f in detected:
    if len(f.parts) - 1 < 3:
      continue
    package, type_class, type_name = f.parts[-3:]
    type_name = type_name.replace(".msg","")
    type_fqname = "::".join((package, type_class, type_name))
    if type_fqname not in result:
      log.debug(f"found: {f} [{type_fqname}]")
      result.append(type_fqname)
  return result

# Convert to a set and preserve values
def _unique_list(seq: Iterable) -> list:
  seen = set()
  return [x for x in seq if not (x in seen or seen.add(x))]

def _find_cpp_exec() -> Path:
  ld_lib_var = "LD_LIBRARY_PATH"
  ld_lib_sep = ":"
  ld_lib_path = os.getenv(ld_lib_var, "").split(ld_lib_sep)
  search_path = _unique_list(chain(map(Path, ld_lib_path), [Path(os.getcwd())]))
  log.debug(f"executable search path: {search_path}")
  for search_dir in search_path:
    for (dirpath, _, filenames) in os.walk(search_dir):
      if "types_scraper_cpp" in filenames:
        ld_lib = Path(dirpath) / "types_scraper_cpp"
        log.debug(f"found executable: {ld_lib}")
        return ld_lib
  raise RuntimeError("failed to find types_scraper_cpp")

def _cpp_scraper_command(
    domains=tuple(),
    input_files=tuple(),
    swap_outputs=True,
    open_stdin=False,
    filter=None,
    raw_filter=None,
    mangle_ros_names=False,
    verbosity=0,
    compatibility_mode=None,
    req_reply_mapping=None) -> Iterable[str]:
  cpp_exec = _find_cpp_exec()

  cpp_cmd = [str(cpp_exec)]
  if swap_outputs:
    cpp_cmd.extend(["-W"])
  if open_stdin:
    cpp_cmd.extend(["-i", "-"])
  if filter is not None:
    cpp_cmd.extend(["-f", filter])
  if raw_filter is not None:
    cpp_cmd.extend(["-F", raw_filter])
  if verbosity > 0:
    cpp_cmd.extend("-v" for i in range(verbosity))
  if compatibility_mode is not None:
    cpp_cmd.extend(["--compatibility-mode", compatibility_mode])
  if req_reply_mapping is not None:
    cpp_cmd.extend(["--request-reply-mapping", req_reply_mapping])
  if mangle_ros_names:
    cpp_cmd.extend(["-m"])
  cpp_cmd.extend(chain.from_iterable((["-d", str(d)] for d in domains)))
  cpp_cmd.extend(chain.from_iterable(
    (["-i", str(i)] for i in input_files if i != Path("-"))))

  # log.info("starting CPP scraper")

  return cpp_cmd

class TypesScraper:
  def __init__(self,
      output_emitter: OutputEmitter,
      pregenerated: Iterable[Path] = tuple(),
      directories: Iterable[Path] = tuple(),
      domains: dict = {},
      input_files: Iterable[Path] = tuple(),
      filter: Optional[str] = None,
      raw_filter: Optional[str] = None,
      mangle_ros_names: bool = False,
      verbosity: int = 0,
      compatibility_mode: Optional[str] = None,
      req_reply_mapping: Optional[str] = None,
      debug: bool = False) -> None:
    self._output_emitter = output_emitter
    self._cpp_scraper = CppScraper(
      domains=domains,
      input_files=input_files,
      filter=filter,
      raw_filter=raw_filter,
      mangle_ros_names=mangle_ros_names,
      verbosity=verbosity - 1 if verbosity > 0 else 0,
      compatibility_mode=compatibility_mode,
      req_reply_mapping=req_reply_mapping,
      debug=debug)

    self._directories = set(directories)
    self._pregenerated = set(pregenerated)
    log.info(f"pregenerated output: {', '.join(map(str, self._pregenerated))}")
    log.info(f"scanning directories: {', '.join(map(str, self._directories))}")
  
    self._detected_types = {}
    self._detected_topics = {}

  def start(self) -> None:
    self._output_emitter.open()
    

  def stop(self) -> None:
    self._output_emitter.close(
      list(self._detected_types.values()),
      list(self._detected_topics.values()))
    self._cpp_scraper.stop()

  @property
  def scanned_types(self):
    if not hasattr(self, "_scanned_types"):
      self._scanned_types = _scan_directories(self._directories)
    return self._scanned_types

  @property
  def pregenerated_text(self):
    return chain.from_iterable(
      f.read_text().split("\n") for f in self._pregenerated)

  def _on_detected(self,
      d_type: Optional[DetectedType],
      d_topic: Optional[DetectedTopic]) -> None:
    if d_type is not None:
      self._detected_types[d_type.fqname] = d_type
      self._output_emitter.detected_type(d_type)
    if d_topic is not None:
      self._detected_topics[d_topic.name] = d_topic
      self._output_emitter.detected_topic(d_topic)
  
  def _on_log(self, log_line : str) -> None:
    log.debug(f"scraper: {log_line[:-1]}")

  def run(self) -> None:
    self._cpp_scraper.communicate(
      self._on_detected,
      self._on_log,
      pregenerated=self.pregenerated_text,
      scanned_types=self.scanned_types)

class CppScraper:
  def __init__(self,
      domains: Iterable[str] = tuple(),
      input_files: Iterable[Path] = tuple(),
      filter: Optional[str] = None,
      raw_filter: Optional[str] = None,
      mangle_ros_names: bool = False,
      verbosity: int = 0,
      compatibility_mode: Optional[str] = None,
      req_reply_mapping: Optional[str] = None,
      debug: bool = False) -> None:
    self._cpp_scraper = None
    self._domains = set(domains)
    self._input_files = set(input_files)
    self._filter = filter
    self._raw_filter = raw_filter
    self._mangle_ros_names = mangle_ros_names
    self._verbosity = verbosity
    self._debug = debug
    self._pipe_stdin = Path("-") in self._input_files
    if compatibility_mode not in ("rmw_cyclonedds_cpp", "rmw_connextdds_cpp", None):
      raise RuntimeError(f"invalid compatibility mode: {compatibility_mode}")
    self._compatibility_mode = compatibility_mode
    if req_reply_mapping not in ("basic", "extended", None):
      raise RuntimeError(f"invalid compatibility mode: {compatibility_mode}")
    self._req_reply_mapping = req_reply_mapping

  def _process_lines(self,
      output_parser: OutputParser,
      on_detected: Callable,
      lines : Iterable[str] = tuple()):
    for line in lines:
      detected_type, detected_topic = output_parser.parse_line(line)
      if detected_type is not None:
        detected_type = DetectedType(detected_type)
      if detected_topic is not None:
        detected_topic = DetectedTopic(detected_topic)
      on_detected(detected_type, detected_topic)

  def stop(self):
    if self._cpp_scraper is not None:
      self._cpp_scraper.request_stop()

  def communicate(self,
      on_detected: Callable,
      on_log: Callable,
      pregenerated: Iterable[str]=tuple(),
      scanned_types: Iterable[str]=tuple()):
    output_parser = OutputParser()

    self._process_lines(output_parser, on_detected, pregenerated)

    cpp_cmd = _cpp_scraper_command(
      swap_outputs=True,
      domains=self._domains,
      input_files=self._input_files,
      open_stdin=self._pipe_stdin or len(scanned_types) > 0,
      filter=self._filter,
      mangle_ros_names=self._mangle_ros_names,
      raw_filter=self._raw_filter,
      verbosity=self._verbosity,
      compatibility_mode=self._compatibility_mode,
      req_reply_mapping=self._req_reply_mapping)

    log.info(f"scraper command: {' '.join(cpp_cmd)}")

    self._cpp_scraper = MonitoredProcess(
      cpp_cmd, pipe_stdin=self._pipe_stdin, debug=self._debug)

    for scanned_t in scanned_types:
      stdin_line = "".join((str(scanned_t), "\n"))
      self._cpp_scraper.pipe_stdin(stdin_line)
    self._cpp_scraper.pipe_stdin(None)
    if not self._pipe_stdin:
      sys.stdin.close()
    
    for stdout_line, stderr_line in self._cpp_scraper.monitor():
      if stderr_line is not None:
        self._process_lines(output_parser, on_detected, [stderr_line])
      if stdout_line is not None:
        on_log(stdout_line)
    
    self._cpp_scraper = None
