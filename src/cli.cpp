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

#include "robotspy/cli.hpp"

namespace robotspy
{

bool exit_flag = false;
std::shared_ptr<std::mutex> exit_mutex;
std::shared_ptr<std::condition_variable> exit_cond;
exit_handler_fn exit_handler = nullptr;


void force_exit()
{
  auto mutex = exit_mutex;
  if (nullptr != mutex) {
    std::unique_lock lk(*mutex);
    if (nullptr != exit_handler) {
      exit_handler();
    }
    exit_flag = true;
    exit_cond->notify_all();
  }
}

static
void force_exit_handler(int signal)
{
  (void)signal;
  force_exit();
}

#if defined(_WIN32)
BOOL exit_on_signal(DWORD cntrlType)
{
  force_exit_handler(0);
  return TRUE;
}
#else // Linux + macOS

#include <signal.h>

#if _POSIX_C_SOURCE
int
set_exit_on_signal(int sig)
{
  int rc = 1;
  struct sigaction new_action;
  struct sigaction old_action;
  memset(&new_action, 0, sizeof(struct sigaction));
  new_action.sa_handler = force_exit_handler;
  /* Temporarily block all other signals during handler execution */
  sigfillset(&new_action.sa_mask);
  if (sigaction(sig, NULL, &old_action) != 0) {
    goto done;
  }
  /* Honor inherited SIG_IGN that's set by some shell's */
  if (old_action.sa_handler != SIG_IGN) {
    if (sigaction(sig, &new_action, NULL) != 0) {
      goto done;
    }
  }
  rc = 0;
done:
  return rc;
}
#else
int
set_exit_on_signal(int sig)
{
  signal(sig, force_exit_handler);
  return 0;
}
#endif /* _XOPEN_SOURCE >= 700 */
#endif // defined(_WIN32)

int
register_exit_handlers(exit_handler_fn handler)
{
  exit_handler = handler;

#if !defined(_WIN32)
  sigset_t set, oset;
#define MAX_SIGNALS     5
  int signals[MAX_SIGNALS] = {SIGTERM, SIGHUP, SIGINT, SIGABRT, SIGPIPE};
  int i = 0;
#endif
  exit_mutex = std::make_shared<std::mutex>();
  exit_cond = std::make_shared<std::condition_variable>();

#if defined(_WIN32)
  if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)exit_on_signal, TRUE)) {
    return 2;
  }
#else
#if _XOPEN_SOURCE
  /*
   * Mask all signals during startup.
   */
  if (sigfillset(&set)) {
    return 2;
  }
  if (pthread_sigmask(SIG_SETMASK, &set, &oset)) {
    return 3;
  }
#endif /* _XOPEN_SOURCE */

  /*
   * Install signals handlers
   */
  for (i = 0; i < MAX_SIGNALS; i++) {
    int sig = signals[i];
    int rc = set_exit_on_signal(sig);
    if (0 != rc) {
      return rc;
    }
  }
#endif
  return 0;
}

void
wait_for_exit()
{
  std::unique_lock lock(*exit_mutex);
  exit_cond->wait(lock, []() {return exit_flag;});
}

dds::domain::DomainParticipant
create_participant(const int32_t domain_id, const std::string & qos_profile)
{
  dds::domain::qos::DomainParticipantFactoryQos dpf_qos;
  dpf_qos << dds::core::policy::EntityFactory::ManuallyEnable();
  dds::domain::DomainParticipant::participant_factory_qos(dpf_qos);
  dds::domain::qos::DomainParticipantQos dp_qos;
  if (qos_profile.size() == 0) {
    dp_qos = dds::core::QosProvider::Default().participant_qos();
  } else {
    dp_qos = dds::core::QosProvider::Default().participant_qos(qos_profile);
  }
  rti::core::policy::Database db_policy;
  db_policy.shutdown_timeout(dds::core::Duration::from_millisecs(100));
  db_policy.shutdown_cleanup_period(dds::core::Duration::from_millisecs(100));
  dp_qos << db_policy;
  return dds::domain::DomainParticipant(domain_id, dp_qos);
}
}  // namespace robotspy
