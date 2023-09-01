// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#include <condition_variable>
#include <iostream>
#include <sstream>

#include "robotspy/base_type_monitor.hpp"
#include "robotspy/base_output_emitter.hpp"
#include "robotspy/dds_input_emitter.hpp"
#include "robotspy/cli.hpp"
#include "robotspy/log_default.hpp"

using namespace robotspy;

const char * const VERSION = "types_scraper_cpp 0.1.0";

static
std::string
program_name(const std::string & prog)
{
  auto slash_pos = prog.rfind("/");
  if (slash_pos == std::string::npos) {
    return prog;
  } else {
    return prog.substr(slash_pos);
  }
}

void
help_menu(const std::string & prog)
{
  using namespace std;
  cout
    << "Usage:" << endl
    << "  " << program_name(prog) << " [OPTIONS]" << endl
    << endl
    << "Input Options:" << endl
    << "  -d, --domain DOMAIN-ID[/QOS-PROFILE]" << endl
    << "    Join the specified DOMAIN and detect types from DDS discovery information." << endl
    << "    Repeat to join multiple domains." << endl
    << "  -i, --input [FILE|-]" << endl
    << "    Read type names from the specified FILE (or standard input)." << endl
    << "    Repeat to read from multiple files." << endl
    << "  -f, --filter" << endl
    << "      Only consider types whose name matches the provided regular expression." << endl
    << endl
    << "Output Options:" << endl
    << "  -m, --mangle" << endl
    << "      Output ROS types using their \"mangled\" type name (e.g. \"my_types::msg::dds_::MyType_\")." << endl
    << "  -o, --output FILE" << endl
    << "      Dump output to the specified file instead of standard out." << endl
    << "  -a, --append" << endl
    << "      Append content to output file instead of truncating it." << endl
    << "  -O, --overwrite" << endl
    << "      Overwrite output file if it already exists." << endl
    // << "  --keep-dds-namespace" << endl
    // << "      Keep the \"dds\" namespace in the name of detected ROS types." << endl
    << endl
    << "Other Options:" << endl
    << "  -v, --verbose" << endl
    << "      Produce more logging output. Repeat to increase." << endl
    << "  -V, --version" << endl
    << "      Print version information and exit." << endl
    << endl
    << "Advanced Options:" << endl
    << "  --compatibility-mode (rmw_connext_cpp|rmw_cyclonedds_cpp)" << endl
    << "      rmw_connext_cpp" << endl
    << "        Generate types which are compatible with rmw_connext_cpp, by adding _ to " << endl
    << "        the end of every member name." << endl
    << "        Requires '--request-reply-mapping extended'." << endl
    << "      rmw_cyclonedds_cpp:"
    << "        Generate types which are compatible with rmw_cyclonedds_cpp, by adding a " << endl
    << "        compatible inline header for request/reply messages." << endl
    << "        Requires '--request-reply-mapping basic'." << endl
    << "  --request-reply-mapping [extended|basic]" << endl
    << "      Select how to correlate the types for services and clients. The default " << endl
    << "      \"extended\" mode relies on DDS sample metadata, while the \"basic\" mode " << endl
    << "      uses an inline header that is automatically added to the payload of every" << endl
    << "      request/reply message." << endl
    << endl;
}

void invalid_args(
  const std::string & prog,
  const std::string & msg)
{
  using namespace std;
  cout
    << "invalid arguments detected: " << msg << endl
    << endl;
  help_menu(prog);
}

// source https://stackoverflow.com/questions/9237216/removing-duplicates-in-a-vector-of-strings
#include <algorithm>
#include <map>

template<typename T>
void unique_elements(std::vector<T> & vec)
{
  std::map<T, int> m;
  for (auto p : vec) {
    ++m[p];
  }
  vec.clear();
  for (const auto & p : m) {
    // if (p.second == 1) {
    //   vec.push_back(p.first);
    // }
    vec.push_back(p.first);
  }
}

int
parse_args(
  const int argc,
  const char ** const argv,
  std::vector<std::pair<const int32_t, const std::string>> & participant_configs,
  DefaultLoggerOptions & log_options,
  DDSInputEmitterOptions & input_options,
  BaseOutputEmitterOptions & output_options,
  BaseTypeMonitorOptions & options)
{
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      help_menu(argv[0]);
      return -1;
    } else if (arg == "-V" || arg == "--version") {
      std::cout << VERSION << std::endl;
      return -1;
    } else if (arg == "-d" || arg == "--domain") {
      if (i == argc - 1) {
        invalid_args(argv[0], "missing domain ID.");
        return 1;
      }
      std::string domain_arg = argv[i + 1];
      std::string domain_id_str = argv[i + 1];
      std::string qos_profile;
      auto slash_pos = domain_arg.find("/");
      if (slash_pos != std::string::npos) {
        if (slash_pos == domain_arg.size() - 1) {
          invalid_args(argv[0], "empty QoS profile name");
        }
        domain_id_str = domain_arg.substr(0, slash_pos);
        qos_profile = domain_arg.substr(slash_pos + 1);
        if (qos_profile.size() == 0) {
          invalid_args(argv[0], "empty QoS profile name");
        }
      }
      try {
        int32_t domain_id;
        std::istringstream(domain_id_str) >> domain_id;
        participant_configs.emplace_back(std::make_pair(domain_id, qos_profile));
      } catch (std::exception & e) {
        invalid_args(argv[0], "failed to parse domain ID.");
        return 1;
      }
      i += 1;
    } else if (arg == "-i" || arg == "--input") {
      if (i == argc - 1) {
        invalid_args(argv[0], "missing input file path.");
        return 1;
      }
      input_options.input_files.emplace_back(argv[i + 1]);
      std::cout << "command line input file (" << input_options.input_files.size() << "): " << argv[i + 1] << std::endl;
      i += 1;
    } else if (arg == "-o" || arg == "--output") {
      if (i == argc - 1) {
        invalid_args(argv[0], "missing output file path.");
        return 1;
      }
      output_options.output_file = argv[i + 1];
      i += 1;
    } else if (arg == "-a" || arg == "--append") {
      output_options.append = true;
    } else if (arg == "-f" || arg == "--filter") {
      if (i == argc - 1) {
        invalid_args(argv[0], "missing filter value.");
        return 1;
      }
      options.type_filter = argv[i + 1];
      i += 1;
    } else if (arg == "-F" || arg == "--raw-filter") {
      if (i == argc - 1) {
        invalid_args(argv[0], "missing raw filter value.");
        return 1;
      }
      options.raw_type_filter = argv[i + 1];
      i += 1;
    } else if (arg == "-O" || arg == "--overwrite") {
      output_options.overwrite = true;
    } else if (arg == "-v" || arg == "--verbose") {
      log_options.verbosity += 1;
    } else if (arg == "-m" || arg == "--mangle") {
      options.cache.demangle_ros_names = false;
    } else if (arg == "--compatibility-mode") {
      if (i == argc - 1) {
        invalid_args(argv[0], "missing compatibility mode.");
        return 1;
      }
      if (strcmp("rmw_cyclonedds_cpp", argv[i + 1]) == 0) {
        options.cache.cyclone_compatible = true;
      } else if (strcmp("rmw_connext_cpp", argv[i + 1]) == 0) {
        options.cache.legacy_rmw_compatible = true;
      } else {
        invalid_args(argv[0], "unsupported compatibility mode");
        return 1;
      }
      i += 1;
    } else if (arg == "--request-reply-mapping") {
      if (i == argc - 1) {
        invalid_args(argv[0], "missing mapping id.");
        return 1;
      }
      try {
        options.cache.request_reply_mapping =
          request_reply_mapping_from_string(argv[i + 1]);
      } catch (std::exception & e) {
        invalid_args(argv[0], "failed to parse request-reply mapping.");
        return 1;
      }
      i += 1;
    } else if (arg == "-W" || arg == "--swap-outputs") {
      output_options.swap_outputs = true;
      log_options.swap_outputs = true;
    } else {
      invalid_args(argv[0], arg);
      return 1;
    }
  }

  // Normalize lists to contain unique entries
  unique_elements(participant_configs);
  unique_elements(input_options.input_files);
  return 0;
}

void
scraper_thread(BaseTypeMonitor * scraper)
{
  try {
    scraper->consume_input();
  } catch (std::exception & e) {
    LOG(ERROR) << "an error occurred: " << e.what() << std::endl;
  }
  force_exit();
}


int main(int argc, const char ** const argv)
{
  int rc = 0;
  DefaultLoggerOptions log_options;
  DDSInputEmitterOptions input_options;
  BaseOutputEmitterOptions output_options;
  BaseTypeMonitorOptions options;
  std::vector<std::pair<const int32_t, const std::string>> participant_configs;
  rc = parse_args(argc, argv, participant_configs,
    log_options, input_options, output_options, options);
  if (-1 == rc) {
    return 0;
  } else if (0 != rc) {
    return rc;
  }
  log_init_default(log_options);
  try {
    if (participant_configs.size() == 0) {
      LOG(INFO) << "no DDS domains specified" << std::endl;
    }
    for (const auto & dp_config : participant_configs) {
      LOG(INFO) << "creating DDS DomainParticipant for domain " << dp_config.first;
      if (dp_config.second.size() > 0) {
        LOG(INFO) << " with QoS profile " << dp_config.second;
      }
      LOG(INFO) << std::endl;
      input_options.participants.emplace_back(
        create_participant(dp_config.first, dp_config.second));
    }
    LOG(INFO) << "(cli) input files: " << input_options.input_files.size();
    auto input = std::make_shared<DDSInputEmitter>(input_options);
    auto output = std::make_shared<BaseOutputEmitter>(output_options);
    BaseTypeMonitor scraper(input, output, options);
    scraper.start();
    for (auto & participant : input_options.participants) {
      LOG(INFO) << "enabling DDS DomainParticipant(" << participant->domain_id() << ")" << std::endl;
      participant->enable();
    }
    register_exit_handlers(nullptr);

    std::thread scraper_t(scraper_thread, &scraper);

    wait_for_exit();
    scraper.stop();

    if (scraper_t.joinable()) {
      scraper_t.join();
    }
  } catch (std::exception & e) {
    LOG(ERROR) << "an error occurred: " << e.what() << std::endl;
    return -1;
  }
  return rc;
}
