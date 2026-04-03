/**
 * @file ranger_base_node.cpp
 * @date 2026-02-04
 * @brief Main entry point for Ranger DORA node
 *
 * @copyright Copyright (c) 2021 AgileX Robotics
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */
extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <csignal>
#include <memory>

#include "ranger_base/ranger_messenger.hpp"
#include <nlohmann/json.hpp>
// using json = nlohmann::json;

using namespace westonrobot;

std::shared_ptr<RangerDoraMessenger> messenger;

void SignalHandler(int signal)
{
  std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
  messenger.reset();
  exit(0);
}

int main()
{
  std::cout << "========================================" << std::endl;
  std::cout << "Ranger DORA Node" << std::endl;
  std::cout << "========================================" << std::endl;

  // Setup signal handlers
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  try
  {
    // Initialize DORA context

    auto dora_context = init_dora_context_from_env();
    // if (dora_context == NULL)
    // {
    //   std::cerr << "Failed to initialize DORA context" << std::endl;
    //   return -1;
    // }

    std::cout << "DORA context initialized" << std::endl;

    // Create messenger
    messenger = std::make_shared<RangerDoraMessenger>(dora_context);

    std::cout << "Starting main loop..." << std::endl;
    std::cout << "========================================" << std::endl;

    // Run main loop
    messenger->Run();

    // Cleanup
    free_dora_context(dora_context);
    messenger.reset();

    std::cout << "Ranger DORA node stopped" << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
