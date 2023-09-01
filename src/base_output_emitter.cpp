// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#include "robotspy/base_output_emitter.hpp"
#include "robotspy/log.hpp"

const std::string fv_prefix_begin_type = ">>> type";
const std::string fv_prefix_end_type = "<<< type";
const std::string fv_prefix_begin_topic = ">>> topic";
const std::string fv_prefix_end_topic = "<<< topic";

namespace robotspy
{
BaseOutputEmitter::BaseOutputEmitter(const BaseOutputEmitterOptions & options)
: options_(options)
{
  LOG(DEBUG) << "output file: " <<
  ((options_.output_file.size()) ?
      options_.output_file :
      ((options_.swap_outputs) ? "stdout" : "stderr")) << std::endl;
  LOG(DEBUG) << "overwrite: " << options_.overwrite << std::endl;
  LOG(DEBUG) << "append: " << options_.append << std::endl;
}

BaseOutputEmitter::~BaseOutputEmitter()
{

}

void
BaseOutputEmitter::open()
{
  if (options_.output_file.size() == 0) {
    LOG(DEBUG) << "no output file specified, using stdout." << std::endl;
    return;
  }
  // Check if the file exists. If it does, we won't overwrite it
  // unless the user requested to do so.
  FILE * const outfile = fopen(options_.output_file.c_str(), "r");
  if (nullptr != outfile) {
    fclose(outfile);
    if (!options_.overwrite && !options_.append) {
      throw std::runtime_error("output file already exists.");
    }
  }
  LOG(INFO) << "opening output: " << options_.output_file;
  auto open_f = std::ios_base::out;
  if (options_.append) {
    open_f |= std::ios_base::app;
    LOG(INFO) << " (append)";
  }
  LOG(INFO) << std::endl;
  output_stream_.open(options_.output_file, open_f);
  if (!output_stream_.good()) {
    throw std::runtime_error("failed to open output file for writing");
  }
}

void
BaseOutputEmitter::close()
{
  if (options_.output_file.size() > 0) {
    LOG(INFO) << "closing output: " << options_.output_file;
    output_stream_.close();
  }
}

void
BaseOutputEmitter::emit_type(const DDS_TypeCode * const type)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string tc_name = DDS_TypeCode_name(type, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  std::string formatted = format_type(tc_name, type);
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    stdout() << fv_prefix_begin_type << std::endl;
    stdout() << formatted << std::endl;
    stdout() << fv_prefix_end_type << std::endl;
  }
}

void
BaseOutputEmitter::emit_topic(
  const std::string & topic_name,
  const DDS_TypeCode * const topic_type)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string tc_name = DDS_TypeCode_name(topic_type, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  std::string formatted = format_topic(topic_name, tc_name, topic_type);
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    stdout() << fv_prefix_begin_topic << std::endl;
    stdout() << formatted << std::endl;
    stdout() << fv_prefix_end_topic << std::endl;
  }
}

static
void
json_begin(std::ostringstream & ss)
{
  ss << "{ ";
}
static
std::string
json_end(std::ostringstream & ss)
{
  ss << " }";
  return ss.str();
}
static
void
json_string(std::ostringstream & ss, const std::string & value)
{
  std::string escaped = std::regex_replace(value, std::regex("[\"]"), "\\\"");
  escaped = std::regex_replace(value, std::regex("\r\n"), "\\n");
  escaped = std::regex_replace(value, std::regex("\n"), "\\n");
  ss << "\"" << escaped << "\"";
}
static
void
json_field(
  std::ostringstream & ss,
  const std::string & key,
  const std::string & value,
  const bool last = false)
{
  json_string(ss, key);
  ss << ": ";
  json_string(ss, value);
  if (!last) {
    ss << ", ";
  }
}

static
std::string
print_idl(const DDS_TypeCode * const tc)
{
  DDS_ExceptionCode_t ex;
  DDS_UnsignedLong print_len;
  DDS_TypeCode_to_string(tc, nullptr, &print_len, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to determine printed typecode length");
  }
  std::string print_buf;
  print_buf.resize(print_len - 1);
  DDS_UnsignedLong printed_len = print_len;
  DDS_TypeCode_to_string(tc, &(print_buf[0]), &printed_len, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to print typecode");
  }
  if (print_len != printed_len || printed_len != print_buf.size() + 1) {
    throw std::runtime_error("unexpected printed typecode length");
  }
  return print_buf;
}

// static
// std::string
// print_idl(const std::vector<DDS_TypeCode *> & tc_cache)
// {
//   std::ostringstream ss;
//   for (auto const & tc : tc_cache) {
//     DDS_ExceptionCode_t ex;
//     if (DDS_TK_STRUCT != DDS_TypeCode_kind(tc, &ex)) {
//       continue;
//     }
//     ss << print_idl(tc) << std::endl;
//   }
//   return ss.str();
// }

std::string
BaseOutputEmitter::format_type(
  const std::string type_fqname, const DDS_TypeCode * const type)
{
  std::ostringstream ss;
  json_begin(ss);
  json_field(ss, "fqname", type_fqname);
  json_field(ss, "idl", print_idl(type), true);
  return json_end(ss);
}

std::string
BaseOutputEmitter::format_topic(
  const std::string topic_name,
  const std::string topic_type_name,
  const DDS_TypeCode * const topic_type)
{
  std::ostringstream ss;
  json_begin(ss);
  json_field(ss, "name", topic_name);
  json_field(ss, "type_name", topic_type_name);
  json_field(ss, "idl", print_idl(topic_type), true);
  return json_end(ss);
}

}  // namespace robotspy
