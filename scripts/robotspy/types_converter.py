# (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
import argparse
from itertools import chain
from functools import reduce
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
from typing import Iterable, Optional

from .idl_fix import idl_fix

from .log import logger, log_level
log = logger()
log_level("DEBUG")

def merge_scanned_types(to_types: dict, from_types: dict):
  for p, p_times in from_types.items():
    to_p_items = to_types.get(p, {})
    for t, t_file in p_times.items():
      to_p_items[t] = t_file
    to_types[p] = to_p_items


def scan_directories(directories: Iterable[str], file_type: str = "msg") -> Iterable[Path]:
  for search_dir in directories:
    for (dirpath, _, filenames) in os.walk(search_dir):
      dirpath = Path(dirpath)
      for f in filter(lambda f: f.endswith(f".{file_type}"), filenames):
        yield dirpath / f


def scan_for_ros_message_types(directories: Iterable[str], file_type: str = "msg") -> dict:
  result = {}
  log.debug(f"scannig for ROS types ({file_type}): {', '.join(map(str, directories))}")
  for f in scan_directories(directories, file_type):
    if len(f.parts) - 1 < 3:
      continue
    package, type_class, type_name = f.parts[-3:]
    type_name = type_name[:-(len(file_type) + 1)]
    type_fqname = "::".join((package, type_class, type_name))
    package_items = result.get(package, {})
    if type_fqname not in package_items:
      log.debug(f"found: {f} [{type_fqname}]")
      package_items[type_fqname] = f
    result[package] = package_items
  return result


def scan_for_ros_service_types(directories: Iterable[str]):
  return scan_for_ros_message_types(directories, file_type="srv")


def scan_for_ros_action_types(directories: Iterable[str]):
  return scan_for_ros_message_types(directories, file_type="action")


def scan_for_raw_message_types(directories: Iterable[str]) -> dict:
  def _find_root(f: Path):
    for d in directories:
      if str(f).startswith(str(d)):
        return d
    raise RuntimeError(f"failed to find root for file: {f}")

  log.debug(f"scannig for raw message types: {', '.join(map(str, directories))}")
  result = {}
  for f in scan_directories(directories, file_type="idl"):
    root = _find_root(f)
    rel_f = f.relative_to(root)
    module, type_name = "::".join(rel_f.parts[:-1]), rel_f.parts[-1][:-4]
    type_fqname = "::".join([module, type_name])
    module_items = result.get(module, {})
    if type_fqname not in module_items:
      log.debug(f"found: {f} [{type_fqname}]")
      module_items[type_fqname] = f
    result[module] = module_items
  return result


def translate_ros_message_types_to_idl(
    scanned_types: dict,
    output_path: Path,
    input_format: str = "msg"):
  from rosidl_cli.command.translate.api import translate

  assert(input_format in ["msg", "srv", "action"])

  include_paths = {
    f.parent.parent
      for p in scanned_types.values() for f in p.values()}

  for p, p_items in scanned_types.items():
    interfaces = list(f"{f_pkgdir}:{f_relpath}"
      for f in p_items.values() if f.name.endswith(f".{input_format}")
        for f_pkgdir in [f.parent.parent]
          for f_relpath in [f.relative_to(f_pkgdir)]
            for f_type in [str(f_relpath).split("/")[0]]
              if f_type == input_format)
    
    translate(
      package_name=p,
      interface_files=interfaces,
      include_paths=include_paths,
      input_format=input_format,
      output_format='idl',
      output_path=Path(output_path) / p,)


def filter_scanned_types(scanned_types: dict, filter: re.Pattern):
  for p, p_items in scanned_types.items():
    for t in p_items.keys():
      if filter.match(t) is not None:
        yield t


def resolve_idl_dependencies(types: Iterable[str], scanned_types: dict, processed: Optional[set] = None):
  processed = processed or set()
  include_re = re.compile(r"#include \"(.+)\.idl\"")
  result = []
  for top_t in types:
    package = top_t.split("::")[0]
    top_t_file = scanned_types[package][top_t]
    log.debug(f"resolve dependencies: {top_t} ({package}) ({top_t_file})")
    dep_count = 0
    for line in top_t_file.read_text().split("\n"):
      m_include = include_re.match(line)
      if m_include is None:
        continue
      included = m_include.group(1)
      included_t = included.replace("/", "::")
      if included_t in processed:
        continue
      log.debug(f"detected include: {m_include.group(1)}")
      processed.add(included_t)
      for dep in resolve_idl_dependencies(
        [included_t], scanned_types, processed):
        if dep not in result:
          result.append(dep)
          dep_count += 1
          yield dep
      dep_count += 1
      yield included_t
    log.debug(f"resolved dependencies: {dep_count} {top_t} ({package}) ({top_t_file})")
    yield top_t


def copy_raw_message_types(directories: Iterable[str], output_dir: Path):
  scanned_types = scan_for_raw_message_types(directories)
  result = {}
  for m, m_types in scanned_types:
    out_dir = output_dir / "/".join(m.split("::"))
    m_result = result.get(m, {})
    for t, t_file in m_types.items():
      t_name = t.split("::")[-1]
      t_out_file = out_dir / t_name
      t_out_file.parent.mkdir(exists_ok=True, parents=True)
      t_out_file.write_text(t_file.read_text())
      log.debug(f"copied: {t_out_file}")
      m_result[t] = t_out_file
    result[m] = m_result
  return result


def convert_ros_message_types(
    input_dirs: Iterable[Path],
    output_dir: Path,
    filter: re.Pattern = re.compile(".*")):
  scanned_types = scan_for_ros_message_types(input_dirs, file_type="msg")
  if len(scanned_types) == 0:
    raise RuntimeError("no .msg ROS types found")

  with tempfile.TemporaryDirectory() as tempdir:
    translate_ros_message_types_to_idl(scanned_types, output_path=tempdir)

    idl_scanned_types = scan_for_ros_message_types([tempdir], file_type="idl")
    if len(idl_scanned_types) == 0:
      raise RuntimeError("no .idl ROS types found")

    return convert_idl_message_types(
      output_dir=output_dir,
      scanned_types=idl_scanned_types,
      filter=filter)


def convert_ros_service_types(
    input_dirs: Iterable[Path],
    output_dir: Path,
    filter: re.Pattern = re.compile(".*"),
    scanned_msg: Optional[dict] = None):
  scanned_types = scan_for_ros_service_types(input_dirs)
  service_types_len = len(scanned_types)
  if service_types_len == 0:
    log.debug("no .srv ROS types found")
    return {}
  merge_scanned_types(scanned_types, scanned_msg)

  with tempfile.TemporaryDirectory() as tempdir:
    translate_ros_message_types_to_idl(
      scanned_types, output_path=tempdir, input_format="srv")

    idl_scanned_types = scan_for_ros_message_types([tempdir], file_type="idl")
    if len(idl_scanned_types) != service_types_len:
      raise RuntimeError(f"unexpected number of converted .idl ROS service types found, expected: {service_types_len}, found: {len(idl_scanned_types)}")

    merge_scanned_types(idl_scanned_types, scanned_msg)

    return convert_idl_message_types(
      output_dir=output_dir,
      scanned_types=idl_scanned_types,
      filter=filter)


def convert_ros_action_types(
    input_dirs: Iterable[Path],
    output_dir: Path,
    filter: re.Pattern = re.compile(".*"),
    scanned_msg: Optional[dict] = None,
    scanned_srv: Optional[dict] = None):
  scanned_types = scan_for_ros_action_types(input_dirs)
  action_types_len = len(scanned_types)
  if action_types_len == 0:
    log.debug("no .action ROS types found")
    return {}
  merge_scanned_types(scanned_types, scanned_msg)
  merge_scanned_types(scanned_types, scanned_srv)

  with tempfile.TemporaryDirectory() as tempdir:
    translate_ros_message_types_to_idl(
      scanned_types, output_path=tempdir, input_format="action")

    idl_scanned_types = scan_for_ros_message_types([tempdir], file_type="idl")
    if len(idl_scanned_types) != action_types_len:
      raise RuntimeError(f"unexpected number of converted .idl ROS action types found, expected: {action_types_len}, found: {len(idl_scanned_types)}")

    merge_scanned_types(idl_scanned_types, scanned_msg)
    merge_scanned_types(idl_scanned_types, scanned_srv)

    return convert_idl_message_types(
      output_dir=output_dir,
      scanned_types=idl_scanned_types,
      filter=filter)
  

def convert_idl_message_types(
    output_dir: Path,
    filter: re.Pattern = re.compile(".*"),
    input_dirs: Optional[Iterable[Path]] = None,
    scanned_types: Optional[dict] = None):
  if scanned_types is None:
    scanned_types = scan_for_ros_message_types(input_dirs, file_type="idl")
    if len(scanned_types) == 0:
      raise RuntimeError("no .idl ROS types found")

  filtered_types = filter_scanned_types(scanned_types, filter)
  resolved_types = resolve_idl_dependencies(filtered_types, scanned_types)

  result = {}
  for ros_type in resolved_types:
    package, type_class, type_name = ros_type.split("::")
    p_result = result.get(package, {})
    idl_file = scanned_types[package][ros_type]
    idl_text = idl_fix(idl_file.read_text())
    idl_out_file = output_dir / package / type_class / f"{type_name}.idl"
    idl_out_file.parent.mkdir(exist_ok=True, parents=True)
    idl_out_file.write_text(idl_text)
    p_result[ros_type] = idl_out_file
    result[package] = p_result
  return result

def generate_jumpstart_config(j_file: Path, scanned_types):
  yml_cfg = {
    "_config": {
      "types": scanned_types,
      "generate-command": " ".join(chain([Path(sys.argv[0]).name], sys.argv[1:])),
      "generate-gitignore": True,
    }
  }
  import yaml
  cfg_str = yaml.safe_dump(yml_cfg)
  j_file.parent.mkdir(parents=True, exist_ok=True)
  j_file.write_text(cfg_str)
  log.debug(f"jumpstart config: {j_file}")

def run_jumpstart_generate(j_file: Path):
  from jumpstart import Jumpstart
  from ament_index_python import get_package_share_directory
  pkg_dir = get_package_share_directory("robotspy")
  workflow_dir = Path(pkg_dir) / "resource" / "jumpstart" / "scraped_types_lib"
  log.debug(f"running jumpstart to generate {j_file}")
  js = Jumpstart(
    config=j_file,
    workflows=[workflow_dir],
    force=True,
    verbosity=0)
  js.run(command="generate")

class TypesConverter:
  @staticmethod
  def parse_args():
    parser = argparse.ArgumentParser(
      description="Scan the filesystem for ROS message types and generate IDL files for Connext DDS.",
      add_help=True)
    in_opts = parser.add_argument_group("Input Options")
    in_opts.add_argument("-r", "--ros-input",
      help="Input directory to scan for ROS .msg files.",
      metavar="DIR",
      action="append",
      default=[])
    in_opts.add_argument("-i", "--idl-input",
      help="Input directory to scan for ROS .idl files.",
      metavar="DIR",
      action="append",
      default=[])
    in_opts.add_argument("-R", "--raw-input",
      help="Input directory to scan for .idl files that don't need more processing.",
      metavar="PREFIX:FILE",
      action="append",
      default=[])
    out_opts = parser.add_argument_group("Output Options")
    out_opts.add_argument("OUTPUT_DIR",
      help="Output directory where all files will be generated.",
      nargs="?",
      type=Path,
      default=Path(os.getcwd()) / "robotspy_scraped_types")
    out_opts.add_argument("-j","--jumpstart",
      metavar="FILE",
      # help="Generate a YAML configuration file to be used with jumpstart and the 'scraped_types_lib' workflow.",
      help=argparse.SUPPRESS,
      type=Path,
      default=None)
    in_opts.add_argument("-f", "--filter",
      help="Only consider ROS types whose name matches the provided regular expression. The expression is matched against a type's \"canonical\" form, i.e. <package>::(msg|srv)::<type>.",
      default=".*")
    return parser.parse_args()

  def __init__(self,
      input_dirs_msg: Iterable[Path],
      input_dirs_idl: Iterable[Path],
      input_dirs_raw: Iterable[Path],
      output_dir: Path,
      filter: str = ".*",
      ) -> None:
    self._output_dir = output_dir
    self._input_dirs_msg = set(input_dirs_msg)
    self._input_dirs_idl = set(input_dirs_idl)
    self._input_dirs_raw = set(input_dirs_raw)
    self._filter = re.compile(filter)

    log.debug(f"output dir: {self._output_dir}")
    log.debug(f"input dir (msg): {self._input_dirs_msg}")
    log.debug(f"input dir (idl): {self._input_dirs_idl}")
    log.debug(f"input dir (raw): {self._input_dirs_raw}")
    log.debug(f"filter: {self._filter}")

  @staticmethod
  def main():

    args = TypesConverter.parse_args()

    self = TypesConverter(
      input_dirs_msg=args.ros_input,
      input_dirs_idl=args.idl_input,
      input_dirs_raw=args.raw_input,
      filter=args.filter,
      output_dir=args.OUTPUT_DIR)

    with tempfile.TemporaryDirectory() as tmpdir:
      scanned_raw = copy_raw_message_types(self._input_dirs_raw,
        output_dir=Path(tmpdir)) if len(self._input_dirs_raw ) > 0 else {}

      scanned_msg = convert_ros_message_types(self._input_dirs_msg,
        output_dir=Path(tmpdir),
        filter=self._filter) if len(self._input_dirs_msg) > 0 else {}

      scanned_srv = convert_ros_service_types(self._input_dirs_msg,
        output_dir=Path(tmpdir),
        filter=self._filter,
        scanned_msg=scanned_msg) if len(self._input_dirs_msg) > 0 else {}

      scanned_action = convert_ros_action_types(self._input_dirs_msg,
        output_dir=Path(tmpdir),
        filter=self._filter,
        scanned_msg=scanned_msg,
        scanned_srv=scanned_srv) if len(self._input_dirs_msg) > 0 else {}

      scanned_idl = convert_idl_message_types(
        output_dir=Path(tmpdir),
        input_dirs=self._input_dirs_idl,
        filter=self._filter) if len(self._input_dirs_idl) > 0 else {}

      shutil.copytree(tmpdir, self._output_dir, dirs_exist_ok=True)

      for m, m_types in scanned_raw.items():
        for t, t_file in m_types.items():
          log.debug(f"raw: {t} [{t_file.relative_to(tmpdir)}]")
      for m, m_types in scanned_msg.items():
        for t, t_file in m_types.items():
          log.debug(f"msg: {t} [{t_file.relative_to(tmpdir)}]")
      for m, m_types in scanned_srv.items():
        for t, t_file in m_types.items():
          log.debug(f"srv: {t} [{t_file.relative_to(tmpdir)}]")
      for m, m_types in scanned_action.items():
        for t, t_file in m_types.items():
          log.debug(f"action: {t} [{t_file.relative_to(tmpdir)}]")
      for m, m_types in scanned_idl.items():
        for t, t_file in m_types.items():
          log.debug(f"idl: {t} [{t_file.relative_to(tmpdir)}]")
      
      scanned_types = list(set(chain(
        map(str, chain.from_iterable(map(dict.keys, scanned_raw.values()))),
        map(str, chain.from_iterable(map(dict.keys, scanned_msg.values()))),
        map(str, chain.from_iterable(map(dict.keys, scanned_srv.values()))),
        map(str, chain.from_iterable(map(dict.keys, scanned_action.values()))),
        map(str, chain.from_iterable(map(dict.keys, scanned_idl.values()))),
      )))
      scanned_types.sort()
    
      if args.jumpstart is not None:
        generate_jumpstart_config(args.jumpstart, scanned_types)

    return 0
