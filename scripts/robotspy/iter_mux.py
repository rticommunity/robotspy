# (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
import threading
import queue
import sys
from typing import Iterator, Mapping

class IterMux:
  def __init__(self,
      id: str,
      muxed: Mapping[str, Iterator],
      wait_timeout: int = None,
      debug: bool = False) -> None:
    self._id = id
    self._wait_timeout = wait_timeout
    self._debug = debug
    self._queue_ready = threading.Semaphore(0)
    self._active = True

    self._muxed = {
      k: [
        m,
        queue.SimpleQueue(),
        threading.Thread(
          target=IterMux._monitor_thread,
          args=(self, k))
      ]
      for k, m in muxed.items()
    }

  def start(self):
    for m in self._muxed.values():
      m[2].start()

  @staticmethod
  def _monitor_thread(self, id):
    stream_iter, stream_queue, self_thread = self._muxed[id]
    while self._active:
      if self._debug:
          print(f"({self._id})({id})>>> reading next...", file=sys.stderr)
      stream_next = next(stream_iter, None)
      if stream_next is None:
        if self._debug:
          print(f"({self._id})({id})>>> done reading <<<", file=sys.stderr)
        self._queue_ready.release()
        return
      if self._debug:
        print(f"({self._id})({id})>>>" + "\n" + stream_next + f"({self._id})({id})<<<", file=sys.stderr)
      stream_queue.put(stream_next)
      self._queue_ready.release()

  def request_stop(self):
    self._active = False
    self._queue_ready.release()

  def _queued_messages(self):
    return next(filter(lambda m: not m[1].empty(), self._muxed.values()), None) is not None

  def _active_threads(self) -> bool:
    return next(filter(threading.Thread.is_alive,
      map(lambda m: m[2], self._muxed.values())), None) is not None

  def read(self):

    def _get_next(m):
      try:
        return m[1].get_nowait()
      except queue.Empty:
        return None

    while self._active and (self._active_threads() or self._queued_messages()):
      self._queue_ready.acquire(timeout=self._wait_timeout)
      if not self._active:
        break
      result = tuple(map(_get_next, self._muxed.values()))
      has_line = next(filter(lambda r: r is not None, result), None) is not None
      if has_line:
        yield result
    for m in self._muxed.values():
      m[2].join()

