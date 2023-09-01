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
      ( parsed_fqname,
        self.name,
        self.modules,
        self.members,
        self.annotations ) = self._parse_idl()
      self.parsed = True
    except UnsupportedType:
      self.parsed = False
      self.name = self.fqname
      self.modules = []
      self.members = []
      self.annotations = []
    
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

    type_start_re = re.compile(r"\n.*(struct|valuetype) ")
    
    type_start_m = type_start_re.search(self.idl)
    if type_start_m is None:
      raise RuntimeError(f"unsupported type: '{self.idl}'")

    type_annotations = self.idl[:type_start_m.start()].strip()
    type_annotations = list(
      filter(len, map(str.strip, type_annotations.replace("\n"," ").split("@"))))

    remaining = self.idl[type_start_m.end():].strip()    
    el_end = remaining.find("{")
    type_fqname = remaining[:el_end].strip()

    remaining = remaining[el_end + 1:]

    if "::" not in type_fqname:
      type_name = type_fqname
      type_modules = []
    else:
      type_name = type_fqname.split("::")[-1].strip()
      type_modules = list(map(str.strip, type_fqname.split("::")[:-1]))

    type_members = remaining[:remaining.find("};")]
    type_members = list(filter(len, map(str.strip, type_members.replace("\n"," ").split(";"))))

    return type_fqname, type_name, type_modules, type_members, type_annotations

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
      "modules": self.modules,
      "members": self.members,
      "annotations": self.annotations,
      "parsed": self.parsed,
      "idl": self.idl,
    }, indent=2)

  def to_idl(self,
      indent: bool = True,
      indent_depth: int = 0,
      indent_step = 2,
      add_includes: bool = False,
      flat_includes: bool = False) -> str:

    if not self.parsed:
      return self.idl

    result = []
    indent_step_str = " " * indent_step
    indent_str = indent_step_str * indent_depth
    # indent_depth = 1

    def _new_line(*args, extra_indent=0, extra_depth=1):
      indent and result.append(indent_str * extra_depth)
      indent and result.append(indent_step_str * extra_indent)
      result.extend(args)
      result.append("\n")

    if add_includes:
      for reft in self.references:
        inc_guard = reft.replace("::","_")
        if flat_includes:
          inc_file = inc_guard
        else:
          inc_file = reft.replace("::","/")
        _new_line("#ifndef ", inc_guard)
        _new_line("#define ", inc_guard)
        _new_line("#include \"", inc_file, ".idl\"")
        _new_line("#endif  // ", inc_guard)

    for i, m in enumerate(self.modules):
      _new_line(f"module {m} ", "{", extra_indent=i)

    _new_line("struct ", self.name, " {", extra_indent=len(self.modules))
    # indent_depth += 1
    for m in self.members:
      _new_line(m, ";", extra_indent=len(self.modules) + 1, extra_depth=2)
    # indent_depth -= 1
    _new_line("};", extra_indent=len(self.modules))

    for i, m in enumerate(reversed(self.modules)):
      _new_line("}; ", f"// module {m}", extra_indent=(len(self.modules)-1-i))

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