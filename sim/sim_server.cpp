#include "sim_server.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>

#include "sim_protocol.h"

int RunServer(SimController &controller)
{
    SimProtocol protocol(controller);
    std::string line;
    int responseFd = dup(STDOUT_FILENO);
    FILE *responseOut = fdopen(responseFd, "w");

    dup2(STDERR_FILENO, STDOUT_FILENO);

    while (std::getline(std::cin, line))
    {
        if (line.empty())
        {
            continue;
        }

        fprintf(responseOut, "%s\n", protocol.HandleLine(line).c_str());
        fflush(responseOut);
    }

    controller.Shutdown();

    fclose(responseOut);

    return 0;
}
