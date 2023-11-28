# (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
import json
import re
from typing import Iterable

class UnsupportedType(Exception):
  def __init__(self, *args: object) -> None:
    super().__init__(*args)

def parse_sequence_member_type(m_type):
  remaining = m_type
  while len(remaining) > 0:
    next_seq_type_start = len("sequence<")
    next_seq_type_end = remaining.rfind(",")
    remaining = m_type[next_seq_type_start: next_seq_type_end].strip()
    if not remaining.startswith("sequence<"):
      break
  return remaining

class DetectedType:
  def __init__(self, json_str: str, json_obj: dict=None, **defaults) -> None:
    if json_str is not None:
      json_obj = json.loads(json_str)
    if json_obj is not None:
      self.fqname = json_obj["fqname"]
      self.idl = json_obj["idl"]
    else:
      for k, v in defaults.items():
        setattr(self, k, v)
    
    self.parsed = False
    parsed_fqname = self.fqname
    try:
      ( self.modules,
        self.name,
        self.parsed_idl) = self._parse_idl()
      self.parsed = True
    except UnsupportedType:
      self.parsed = False
      self.modules = []
      self.name = self.fqname
      self.parsed_idl = self.idl
    if parsed_fqname != self.fqname:
      raise RuntimeError(f"invalid parsed fqname: {parsed_fqname} != {self.fqname}")

  def _fix_unbounded_members(self):
    self.idl = self.idl.replace("<2147483647>", "").replace(",2147483647>", ">")

  def __str__(self) -> str:
    return self.fqname

  def __eq__(self, o: object) -> bool:
    if not isinstance(o, DetectedType):
      return NotImplemented
    return self.fqname == o.fqname

  def __lt__(self, o: object) -> bool:
    if not isinstance(o, DetectedType):
      return NotImplemented
    return self.fqname < o.fqname

  def __hash__(self) -> int:
    return hash(self.fqname)

  def _parse_idl(self):
    self._fix_unbounded_members()
    name_parts = self.fqname.split("::")
    modules = name_parts[:-1]
    name = name_parts[-1]

    if self.idl.startswith("typedef "):
      "".rfind()
      type_name_pos = self.idl.rfind(self.fqname)
    else:
      type_name_pos = self.idl.find(self.fqname)
    if type_name_pos == -1:
      raise RuntimeError("failed to find type name in idl string", self.fqname, self.idl)
    parsed_idl = self.idl[:type_name_pos] + name + self.idl[type_name_pos+len(self.fqname):]

    return (
      modules,
      name,
      parsed_idl
    )

  @property
  def references(self) -> Iterable[str]:
    referenced = []
    for m in filter(lambda m: "::" in m, self.members):
      m_type = m.split(" ")[0].strip()
      if m.startswith("sequence<"):
        referenced.append(parse_sequence_member_type(m_type))
      else:
        referenced.append(m_type)
    return referenced

  def to_json(self):
    return json.dumps({
      "fqname": self.fqname,
      "name": self.name,
      "kind": self.kind,
      "modules": self.modules,
      "parsed_idl": self.parsed_idl,
      # "members": self.members,
      # "annotations": self.annotations,
      "parsed": self.parsed,
      "idl": self.idl,
    }, indent=2)

  def to_idl(self,
      indent: bool = True,
      indent_depth: int = 0,
      indent_step = 2) -> str:

    if not self.parsed:
      return self.idl

    result = []
    indent_step_str = " " * indent_step
    indent_str = indent_step_str * indent_depth
    # indent_depth = 1

    def _append(*args, extra_indent=0, extra_depth=1):
      indent and result.append(indent_str * extra_depth)
      indent and result.append(indent_step_str * extra_indent)
      result.extend(args)

    def _new_line(*args, extra_indent=0, extra_depth=1):
      _append(*args, extra_indent=extra_indent, extra_depth=extra_depth)
      result.append("\n")

    for i, m in enumerate(self.modules):
      _new_line(f"module {m} ", "{", extra_indent=i)

    for l in self.parsed_idl.split("\n"):
      _new_line(l, extra_indent=len(self.modules))

    for i, m in enumerate(reversed(self.modules)):
      _new_line("}; ", f"// module {m}", extra_indent=(len(self.modules)-1-i))
    if self.modules:
      _new_line()

    return "".join(result)

class DetectedTopic:
  def __init__(self, json_str: str) -> None:
    json_obj = json.loads(json_str)
    self.name = json_obj["name"]
    self.type = DetectedType(None,
      fqname=json_obj["type_name"],
      idl=json_obj["idl"])

  def __str__(self) -> str:
    return self.name

  def __eq__(self, o: object) -> bool:
    if not isinstance(o, DetectedType):
      return NotImplemented
    return self.name == o.name

  def __lt__(self, o: object) -> bool:
    if not isinstance(o, DetectedType):
      return NotImplemented
    return self.name < o.name

  def __hash__(self) -> int:
    return hash(self.name)