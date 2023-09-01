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
import json
import os
from pathlib import Path
from typing import Iterable, TextIO
from .detected import DetectedTopic, DetectedType
from .log import logger
log = logger()

class OutputEmitter:
  def detected_type(self, d_type: DetectedType) -> None:
    log.info(f"detected type: {d_type}")
  def detected_topic(self, d_topic: DetectedTopic) -> None:
    log.info(f"detected topic: {d_topic}")
  def open(self) -> None:
    pass
  def close(self,
      detected_types: Iterable[DetectedType],
      detected_topic: Iterable[DetectedTopic]) -> None:
    pass

class CombinedOutputEmitter(OutputEmitter):
  def __init__(self, *combined: Iterable[OutputEmitter]):
    super().__init__()
    self._combined = list(combined)

  def detected_topic(self, d_topic: DetectedTopic) -> None:
    super().detected_topic(d_topic)
    for c in self._combined:
      c.detected_topic(d_topic)

  def detected_type(self, d_type: DetectedType) -> None:
    super().detected_type(d_type)
    for c in self._combined:
      c.detected_type(d_type)

  def open(self) -> None:
    super().open()
    for c in self._combined:
      c.open()

  def close(self, detected_types: Iterable[DetectedType], detected_topic: Iterable[DetectedTopic]) -> None:
    super().close(detected_types, detected_topic)
    for c in self._combined:
      c.close(detected_types, detected_topic)

class FileEmitter(OutputEmitter):
  def __init__(self,
      output_path: Path,
      append: bool = False,
      overwrite: bool = False,
      split: bool = False,
      flat: bool = False,) -> None:
    self._append = append
    self._overwrite = overwrite
    self._split = split
    self._flat = flat
    self._output_path = output_path
    self._output_file = None
    self._output_dir = None
    self._updated_files = set()
  
  def open(self) -> None:
    if self._output_path is not None or self._split: 
      output_path = self._output_path or Path(os.getcwd())
      self._output_file = self.open_output_file(
        output_path,
        is_dir=self._split,
        append=self._append,
        overwrite=self._overwrite)
      if self._split:
        self._output_dir = output_path

  def close(self,
      detected_types: Iterable[DetectedType],
      detected_topic: Iterable[DetectedTopic]) -> None:
    if self._output_file is not None:
      self._output_file.close()
      self._output_file = None

  def write_output_file(self, filename: str, file_content: str) -> None:
    t_file = Path(f"{self._output_dir}/{filename}")
    with self.open_output_file(t_file,
        overwrite=self._overwrite,
        append=False) as t_file_fd:
      t_file_fd.write(file_content)
      t_file_fd.flush()
    log.info(f"created file : {t_file}")

  def write_output(self, output: str) -> None:
    if self._output_file is not None:
      self._output_file.write(output)
      self._output_file.flush()
    else:
      print(output)
  
  def open_output_file(self,
      output_path: Path,
      is_dir: bool = False,
      append: bool = False,
      overwrite: bool = False) -> TextIO:
    log.info(f"open output file: {output_path}{' (append)' if append else ''}")
    if output_path.exists():
      if not overwrite and not append and not output_path in self._updated_files:
        raise RuntimeError(f"output {'directory' if is_dir else 'file'} already exists: {output_path}")
    if output_path not in self._updated_files:
      self._updated_files.add(output_path)
    if is_dir:
      output_path.mkdir(parents=True, exist_ok=True)
    else:
      output_path.parent.mkdir(parents=True, exist_ok=True)
    if not is_dir:
      return open(output_path, "a" if append else "w")
    else:
      return None

class IdlTypesEmitter(FileEmitter):
  def __init__(self,
      no_indent: bool = False,
      indent_depth: int = 0,
      indent_step: int = 2,
      **kwargs) -> None:
    super().__init__(**kwargs)
    self._no_indent = bool(no_indent)
    self._indent_depth = int(indent_depth)
    self._indent_step = int(indent_step)

  def detected_type(self, d_type: DetectedType) -> None:
      super().detected_type(d_type)

      to_idl_args = {
        "indent": not self._no_indent,
        "indent_depth": self._indent_depth,
        "indent_step": self._indent_step,
        "add_includes": self._split,
        "flat_includes": self._flat,
      }
      idl_str = d_type.to_idl(**to_idl_args)

      if self._split:
        sep = "_" if self._flat else "/"
        t_filename = sep.join(chain(d_type.modules,[d_type.name]))
        t_filename = f"{t_filename}.idl"
        self.write_output_file(t_filename, idl_str)
        return
      
      self.write_output(idl_str)
  
  def detected_topic(self, d_topic: DetectedTopic) -> None:
    super().detected_topic(d_topic)

    self.write_output(f"// {d_topic.type} @ \"{d_topic.name}\"")

class ListOnlyEmitter(FileEmitter):
  def __init__(self,
      topics_only: bool = False,
      **kwargs) -> None:
    super().__init__(**kwargs)
    self._topics_only = topics_only

  def detected_topic(self, d_topic: DetectedTopic) -> None:
    super().detected_topic(d_topic)
    self.write_output(f"{d_topic.type}@{d_topic.name}")

  def detected_type(self, d_type: DetectedType) -> None:
    super().detected_type(d_type)
    if not self._topics_only:
      self.write_output(d_type.fqname)

class TopicsListEmitter(FileEmitter):
  def __init__(self,
      no_indent: bool = False,
      indent_depth: int = 0,
      indent_step: int = 2,
      yaml: bool = False,
      **kwargs) -> None:
    super().__init__(**kwargs)
    self._no_indent = bool(no_indent)
    self._indent_depth = int(indent_depth)
    self._indent_step = int(indent_step)
    self._yaml = bool(yaml)
    self._detected = []

  def detected_topic(self, d_topic: DetectedTopic) -> None:
    super().detected_topic(d_topic)
    self._detected.append(d_topic)

  def close(self,
      detected_types: Iterable[DetectedType],
      detected_topic: Iterable[DetectedTopic]) -> None:
    if self._output_file is None:
      return
    topics_list = self.generate_topics_list(self._detected)
    self.write_output(topics_list)
    super().close(detected_types, detected_topic)

  def generate_topics_list(self, detected_topics):
    topics_map = {t.name: t.type.fqname for t in detected_topics}
    if self._yaml:
      import yaml
      topics_json = yaml.safe_dump(topics_map)
    else:
      json_args = {
        "sort_keys": True
      }
      if not self._no_indent:
        json_args["indent"] = self._indent_step
      topics_json = json.dumps(topics_map, **json_args)
    return topics_json