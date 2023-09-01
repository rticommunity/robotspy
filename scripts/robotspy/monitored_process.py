# (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
import queue
import signal
import subprocess
import sys
import errno
import threading
import traceback

from .iter_mux import IterMux

def _stdin_thread(self: "MonitoredProcess"):
  stdin_mux_iter = iter(self._stdin_mux.read())
  while True:
    stdin_lines = next(stdin_mux_iter, None)
    if stdin_lines is None:
      break;
    try:
      for line in stdin_lines:
        if line is None or line=="\n":
          continue
        if self._debug:
          print(
            f"({self.pid})(merged-stdin)>>>", "\n",
            line,
            f"({self.pid})(merged-stdin)<<<",
            file=sys.stderr)
        self._process.stdin.write(line)
        self._process.stdin.flush()
    except IOError as e:
      if e.errno == errno.EPIPE or e.errno == errno.EINVAL:
        break
      else:
        # Raise any other error.
        self._process.stdin.close()
        raise
    except:
      self._process.stdin.close()
      raise
  self._process.stdin.close()


class MonitoredProcess:
  def __init__(self, process_cmd,
      wait_timeout: int = None,
      pipe_stdin: bool = False,
      debug: bool = False) -> None:
    self._process_lock = threading.Lock()
    self._process = subprocess.Popen(
      process_cmd,
      stdin=subprocess.PIPE,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      universal_newlines=True)

    self._debug = debug
    self._pipe_stdin = pipe_stdin
    self._mux = IterMux(str(self.pid) + "-out",
      muxed={
        "stdout": iter(self._process.stdout.readline, ""),
        "stderr": iter(self._process.stderr.readline, ""),
      },
      wait_timeout=wait_timeout,
      debug=debug)

    muxed_stdin = {
      "stdin-pipe": iter(self._piped_stdin())
    }
    if pipe_stdin:
      muxed_stdin["stdin"] = iter(self._process_stdin())
    self._stdin_mux = IterMux(str(self.pid) + "-in",
      muxed=muxed_stdin,
      wait_timeout=wait_timeout,
      debug=debug)
    
    self._stdin_queue = queue.SimpleQueue()
    self._stdin_thread = threading.Thread(
      target=_stdin_thread, args=(self,))

    self._mux.start()
    self._stdin_mux.start()
    self._stdin_thread.start()

  @property
  def pid(self):
    return self._process.pid

  def pipe_stdin(self, line):
    self._stdin_queue.put(line)

  def _piped_stdin(self):
    while True:
      next_in = self._stdin_queue.get()
      if next_in is None:
        # yield None
        return
      else:
        yield next_in

  def _process_stdin(self):
    if sys.stdin.isatty():
      # Don't try to read from stdin when
      # running in an interactive terminal
      return
    while True:
      next_line = sys.stdin.readline()
      if len(next_line) == 0:
        break
      yield next_line

  def request_stop(self):
    self._process.stdin.close()
    self._process.send_signal(signal.SIGINT)
    self._stdin_queue.put(None)
    self._stdin_mux.request_stop()
    self._mux.request_stop()

  def monitor(self):
    if self._process is None:
      raise RuntimeError("process already terminated")

    mux_iter = iter(self._mux.read())
    while True:
      muxed_input = next(mux_iter, None)
      if muxed_input is None:
        break
      yield muxed_input

    return_code = self._process.wait()
    if return_code != 0 and return_code != (-signal.SIGINT):
      raise subprocess.CalledProcessError(return_code, self._process)

    self._stdin_thread.join()
