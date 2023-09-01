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
import re

def idl_fix(idl_text: str, array_typedefs=None):
  idl_text = _remove_empty_lines(idl_text)
  idl_text = _fix_includes(idl_text)
  idl_text = _fix_primitive_array_typedefs(idl_text, typedefs=array_typedefs)
  idl_text = _remove_comments(idl_text)
  # idl_text = _remote_units(idl_text)
  idl_text = _fix_invalid_member_names(idl_text)
  return "".join(idl_text)

def _remove_empty_lines(idl_text: str):
  for line in idl_text.split("\n") if isinstance(idl_text, str) else idl_text:
    if line.endswith("\n"):
      line = line[:-1]
    line = line.strip()
    if len(line) == 0:
      continue
    yield _new_line(line)

def _new_line(*elements):
  return "".join(chain(elements, "\n"))

def _fix_includes(idl_text: str):
  in_guard = False
  for line in idl_text.split("\n") if isinstance(idl_text, str) else idl_text:
    if line.endswith("\n"):
      line = line[:-1]

    if line.startswith("#ifndef "):
      in_guard = True
      yield _new_line(line)
      continue
    elif line.startswith("#ifndef "):
      in_guard = False
      yield _new_line(line)
      continue
    elif not line.startswith("#include") or in_guard:
      yield _new_line(line)
      continue

    included = line.split(" \"")[1][:-1]
    inc_guard = included[:-4].replace("/", "_") + "_"
    yield _new_line("#ifndef ", inc_guard)
    yield _new_line("#define ", inc_guard)
    yield _new_line(line)
    yield _new_line("#endif  // ", inc_guard)
    # yield _new_line()

def _fix_primitive_array_typedefs(idl_text: str, typedefs=None):
  array_typedef_re = re.compile(
    r"^[ ]*typedef ([a-z][a-z0-9]*) ([a-z][a-z0-9]*)__([1-9][0-9]*)\[([1-9][0-9]*)\];")
  typedefs = typedefs or {}
  for line in idl_text.split("\n") if isinstance(idl_text, str) else idl_text:
    if line.endswith("\n"):
      line = line[:-1]

    m_typedef = array_typedef_re.match(line)

    if m_typedef is not None:
      prim_t = m_typedef.group(2)
      array_len = m_typedef.group(3)
      array_t = f"{prim_t}__{array_len}"
      if array_t not in typedefs:
        array_t_re = re.compile(f"([ ]*)({array_t}) ([a-z_]+);")
        typedefs[array_t] = [prim_t, array_len, array_t_re]
      continue

    for prim_t, array_len, array_t_re in typedefs.values():
      m_array_t = array_t_re.match(line)
      if m_array_t is None:
        continue
      line = f"{m_array_t.group(1)}{prim_t} {m_array_t.group(3)}[{array_len}];"
      break

    yield _new_line(line)

def _remove_comments(idl_text: str):
  comment_start_re = re.compile(r"[ ]*@verbatim[ ]*\(language=\"comment\", text=")
  comment_body_re = re.compile(r"[ ]*\"(.*)\" \"\\n\"")
  comment_end_re = re.compile(r"[ ]*\"(.*)\"\)")
  for line in idl_text.split("\n") if isinstance(idl_text, str) else idl_text:
    if line.endswith("\n"):
      line = line[:-1]

    m_comment_start = comment_start_re.match(line)
    if m_comment_start is not None:
      continue
    m_comment_body = comment_body_re.match(line)
    if m_comment_body is not None:
      continue
    m_comment_end = comment_end_re.match(line)
    if m_comment_end is not None:
      continue
    yield _new_line(line)

def _remote_units(idl_text: str):
  unit_re = re.compile(r"[ ]*@unit[ ]*\(.*")
  for line in idl_text.split("\n") if isinstance(idl_text, str) else idl_text:
    if line.endswith("\n"):
      line = line[:-1]

    m_unit = unit_re.match(line)
    if m_unit is not None:
      continue
    yield _new_line(line)

def _fix_invalid_member_names(idl_text: str):
  member_re = re.compile(r"([ ]*)([^ ]*) ([^;]*);")
  for line in idl_text.split("\n") if isinstance(idl_text, str) else idl_text:
    if line.endswith("\n"):
      line = line[:-1]

    m_member = member_re.match(line)
    if m_member is None:
      yield _new_line(line)
      continue
  
    member_name = m_member.group(3)

    if member_name in ("sequence", "struct", ):
      member_name = f"{member_name}_"
    
    line = f"{m_member.group(1)}{m_member.group(2)} {member_name};"
    yield _new_line(line)

