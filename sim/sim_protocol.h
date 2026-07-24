#pragma once

#include <string>

#include "sim_controller.h"

class SimProtocol
{
  public:
    explicit SimProtocol(SimController &controller);

    std::string HandleLine(const std::string &line);

  private:
    SimController &mController;
};
