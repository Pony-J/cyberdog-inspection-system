#include <atomic>
#include <cstdlib>
#include <csignal>
#include <iostream>

#include "cyberdog_inspection/local_inspection.hpp"

namespace {
std::atomic<bool> g_running{true};
cyberdog_inspection::LocalInspectionService * g_service = nullptr;

void signal_handler(int) {
  g_running = false;
  if (g_service) {
    g_service->shutdown();
  }
}
}  // namespace

int main(int argc, char ** argv) {
  std::string config_path = argc > 1 ? argv[1] : "config/inspection_config.yaml";

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    auto config = cyberdog_inspection::load_config(config_path);
    if (std::getenv("CYBERDOG_NS") == nullptr) {
      setenv("CYBERDOG_NS", config.ns.c_str(), 0);
    }
    cyberdog_inspection::LocalInspectionService service(config);
    g_service = &service;
    service.run();
    g_service = nullptr;
  } catch (const std::exception & e) {
    std::cerr << "[cyberdog_inspection] Fatal error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
