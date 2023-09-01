# (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
from typing import Optional

class OutputParser:
  BEGIN_TYPE = ">>> type"
  END_TYPE = "<<< type"
  BEGIN_TOPIC = ">>> topic"
  END_TOPIC = "<<< topic"

  SECTION_RESET = object()
  SECTION_TYPE = "type"
  SECTION_TOPIC = "topic"

  def __init__(self) -> None:
    self._current_section = None
    self._section_buffer = []
    self._current_topic = None

  def _retval(self, section=None, detected_type=None, detected_topic=None):
    if section is None:
      section = self._current_section
    if section is OutputParser.SECTION_RESET:
      section = None
    self._current_section = section
    return detected_type, detected_topic

  def _enter_section(self, section: Optional[str]):
    self._section_buffer.clear();
    return self._retval(section)
  
  def _in_section(self, line: str):
    self._section_buffer.append(line)
    return self._retval()
  
  def _exit_section(self):
    section = "\n".join(self._section_buffer)
    if self._current_section == OutputParser.SECTION_TYPE:
      return self._retval(OutputParser.SECTION_RESET, section)
    else:
      return self._retval(OutputParser.SECTION_RESET, None, section)

  def parse_line(self, line):
    # remove trailing '\n' if present
    if line[-1] == "\n":
      line = line[:-1]      
    # ignore empty lines
    if len(line) == 0:
      return self._retval()

    if line.startswith(OutputParser.BEGIN_TYPE):
      if self._current_section is not None:
        self.log_error("unexpected new type declaration started by monitor")
      return self._enter_section(OutputParser.SECTION_TYPE)
    elif line.startswith(OutputParser.END_TYPE):
      if self._current_section != OutputParser.SECTION_TYPE:
        self.log_error("unexpected end of type declaration by monitor")
        # reset state machine
        return self._retval(OutputParser.SECTION_RESET)
      return self._exit_section()
    elif line.startswith(OutputParser.BEGIN_TOPIC):
      if self._current_section is not None:
        self.log_error("unexpected new topic declaration started by monitor")
      return self._enter_section(OutputParser.SECTION_TOPIC)
    elif line.startswith(OutputParser.END_TOPIC):
      if self._current_section is None:
        self.log_error("unexpected end of topic declaration by monitor")
        # reset state machine
        return self._retval(OutputParser.SECTION_RESET)
      return self._exit_section()
    elif self._current_section is not None:
      # Accumulate lines if within a section otherwise ignore them
      return self._in_section(line)
    else:
      return self._retval()

  def parse_monitor_output(self, output_iter):
    while True:
      # Read next line from the stderr iterator
      stderr_next = next(output_iter, None)

      # Exit if the iterator is done
      if stderr_next is None:
        break

      detected_type, detected_topic = self._process_stderr(stderr_next)
      if detected_type is not None or detected_topic is not None:
        yield detected_type, detected_topic
