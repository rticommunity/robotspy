# robotspy

`robotspy` is a collection of command-line utilities to extract information
from ROS 2/DDS systems, and facilitate their integration with RTI Connext DDS:

- [`types_scraper`](#types-scraper): extract type definitions from a ROS 2/DDS
  system, and generate `.idl` files for `rtiddsgen`.

- [`types_converter`](#types-converter): collect type definition files from disk,
  and convert them to `.idl` files for `rtiddsgen`.

## Building

`robotspy` must be built as a ROS 2 package, and thus requires
ROS 2 to be installed and loaded in the environment (refer to the [ROS 2 documentation](https://docs.ros.org/en/rolling/Installation.html)
for more information on how to do this).

You must also have RTI Connext DDS 6 (or later) installed on your system. The Connext
installation must be loaded in the environment via variable `CONNEXTDDS_DIR` (or `NDDSHOME`).

For example:

```sh
# Load ROS 2, e.g. humble
source /opt/ros/humble/setup.bash

# Load RTI Connext DDS 6+, e.g. 6.1.2
source rti_connext_dds-6.1.2/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash

# Clone and build the robotspy package
git clone https://github.com/rticommunity/robotspy.git
colcon build --symlink-install

# Load package into environment
source install/setup.bash
```

## Types Scraper

`types_scraper` is a C++/Python utlity which takes a list of ROS 2 type names (in their "canonical" form,
i.e. `<package>::(msg|srv)::<type>`), loads their definitions, and generates `.idl` files
which can then be processed by `rtiddsgen` for use with RTI Connext DDS.

Type names can be extracted by monitoring the discovery traffic of one or more live ROS 2/DDS systems,
or by searching the file system for ROS 2 type definition files (i.e. `.msg` files).

For every detected type, `types_scraper` will load its definition from either:

- a DDS Databus, if remote applications are sharing type definitions through discovery (default behavior if they are
  running with RTI Connext DDS and/or `rmw_connextdds`). In this case, the name of discovered topics will also be
  printed in output (as IDL comments).

- the C++ support library which ROS 2 automatically generates for every message type.
  In this case, all types must have been compiled/installed locally, and loaded, before running `types_scraper`.

### Live System Scraping Examples

- Join DDS domain 0, and save generated IDL in a file, exit after 10 seconds:

  ```sh
  ros2 run robotspy types_scraper \
    --exit-after 10 \
    -o my_project/idl/my_types.idl
  ```

  When connecting to a DDS Databus, `types_scraper` will run indefinitely,
  unless the `--exit-after` option is used to specify a maximum run time.

- Join multiple DDS domains:

  ```sh
  ros2 run robotspy types_scraper \
    -d 0 \
    -d 1 \
    -d 2/MyQoSLibrary::MyCustomQosProfile \
    --exit-after 10 \
    -o my_project/idl/my_types.idl
  ```

  The custom Qos profiles specified through the `--domain` option must be loaded via
  one or more XML QoS Configuration files, e.g. by creating `USER_QOS_PROFILES.xml` in
  the current directory, or by specifying the file paths through variable `NDDS_QOS_PROFILES`.

### Local Filesystem Scraping Examples

- Search the local ROS installation for type definitions, save all
  types in a single `.idl` file:

  ```sh
  ros2 run robotspy types_scraper \
    -D /opt/ros/${ROS_DISTRO} \
    -o ros_${ROS_DISTRO}.idl
  ```

- Search the local ROS installation for types from the `visualization_msgs` and `geometry_msgs` packages:

  ```sh
  ros2 run robotspy types_scraper \
    -D /opt/ros/${ROS_DISTRO} \
    -o ros_${ROS_DISTRO}.idl \
    -f '^(visualization|geometry)_msgs::.*'
  ```

  This command will generate a subtree with directories for every referenced package, similar to the following:

  ```sh
  ros_${ROS_DISTRO}/
  ├── builtin_interfaces
  │   └── msg
  │       ├── Duration.idl
  │       └── Time.idl
  ├── geometry_msgs
  │   └── msg
  │       ├── Accel.idl
  │       ├── ...
  │       └── WrenchStamped.idl
  ├── sensor_msgs
  │   └── msg
  │       └── CompressedImage.idl
  ├── std_msgs
  │   └── msg
  │       ├── ColorRGBA.idl
  │       └── Header.idl
  └── visualization_msgs
      ├── msg
      │   ├── ImageMarker.idl
      │   ├── ...
      │   └── UVCoordinate.idl
      └── srv
          ├── GetInteractiveMarkers_Request.idl
          └── GetInteractiveMarkers_Response.idl
  ```

### Scraping Output

By default, `types_scraper` will dump the generated IDL to standard output, so you can just pipe it into a file to save it:

```sh
ros2 run robotspy types_scraper > my_types.idl
```

You can also have `types_scraper` dump the IDL directly to a file using the `--output` option:

```sh
ros2 run robotspy types_scraper --output my_package/idl/my_types.idl
```

`types_scraper` will attempt to create any non-existing directory
included in the output file's path, but it will not overwrite the file if it already exists
(unless options `--overwrite` or `--append` are used).

### Input Filtering

By default, all detected ROS types will be dumped to output.

When scraping type names from a live ROS 2/DDS system, `types_scraper` will search for DDS topics whose name
follows the ROS 2 naming conventions, and "demangle" their type name into the ROS 2 "canonical" form
(i.e. from `<package>::(msg|srv)::dds_::<type>_` to `<package>::(msg|srv)::<type>`).

When searching the filesystem for type names, `types_scraper` will search for `.msg` files, and expect them
to be located under `<package>/(msg|srv)/<type>.msg`. This assumption is used to determine their
"canonical" names and load their definitions.

You can restrict the tool to only a subset of the detected types by using a regular expression, which will be applied
to the "canonical" type names. For example:

For example, to only include types from package `rcl_interfaces`:

```sh
# Only include types from package `rcl_interfaces`:
ros2 run robotspy types_scraper -f "^rcl_interfaces::.*"

# Only include service interfaces from packages `rcl_interfaces` and `visualization_msgs`:
ros2 run robotspy types_scraper -f "^(rcl_interfaces|visualization_msgs)::srv::.*"
```

### C++ implementation and Python wrapper

The `types_scraper` utlity is implemented in C++ (`types_scraper_cpp`), but it is typically run using a Python wrapper (`types_scraper`).

The purpose of the Python wrapper is to parse the output of the C++ executable into a more "human-friendly" format (but also
consumable by `rtiddsgen`). This is because the C++ executable will output custom JSON data structures interleaved with "control tags",
more suitable for machine processing.

The Python wrapper also implements scraping of the local filesystem to identify type names, which it then passes to the C++
executable in list format through the process' standard input.

## Types Converter

`types_converter` is Python utility which provides functionality similar to `types_scraper`, with its main purpose being to
generate `.idl` files containing ROS 2 message types to be processed by `rtiddsgen` for use with RTI Connext DDS.

The main difference between `types_converter` and `types_scraper` is that `types_converter` does not require the types to
have been previously compiled into libraries.

The utility performs the following actions:

- Scan the filesystem for supported ROS 2 type definition files (`.msg`, `.srv`, `.action`).

- Invoke the ROS 2 type tranformation framework and normalize all input types into `.idl` files.

- Apply some textual transformations on the `.idl` files to make them compatible with `rtiddsgen`.

- Collect all generated files into a single directory, with subdirectories named after each referenced package.

### Supported Type Definition Files

`types_converter` supports multiple types of input files:

- ROS 2 message definition files (`.msg`).

- ROS 2 service definition files (`.srv`).

- ROS 2 action definition files (`.action`).

- ROS 2 IDL type definition files (`.idl`). These files are automatically generated by ROS 2 when type definition files are "compiled".

- DDS IDL type definition files (`.idl`). These files will not be processed in any way, and just copied to the output directory.

The utility will search for:

- `.msg`, `.srv`, and `.action` files in directories specified with the `--ros-input` option.

- ROS 2 `.idl` files in directories specified with the `--idl-input` option.

- DDS `.idl` files in directories specified with the `--raw-input` option.

### Types Converter Examples

- Search the local ROS installation for `.msg`, `.srv`, and `.action` files; save generated `.idl` files in directory `ros_${ROS_DISTRO}`:

  ```sh
  ros2 run robotspy types_converter \
    -r /opt/ros/${ROS_DISTRO} \
    ros_${ROS_DISTRO}
  ```

  Type definition files must follow the ROS conventions, and be located in a subdirectory named after their
  extension.

- Search the local ROS installation for `.idl` files, convert them, and save the result in directory `ros_${ROS_DISTRO}`:

  ```sh
  ros2 run robotspy types_converter \
    -i /opt/ros/${ROS_DISTRO} \
    ros_${ROS_DISTRO}
  ```
