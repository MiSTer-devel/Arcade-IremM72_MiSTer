#include "sim_core.h"
#include "sim_controller.h"
#include "sim_server.h"
#include "sim_ui.h"
#include "sim_video.h"
#include "sim_hierarchy.h"
#include "games.h"
#include "M72.h"
#include "M72___024root.h"
#include "imgui_wrap.h"
#include "sim_sdram.h"
#include "verilated_fst_c.h"

#include <cstdio>
#include <memory>
#include <SDL.h>

int main(int argc, char **argv)
{
    // Make Verilog plusargs available on the model's private context.
    gSimCore.SetCommandArgs(argc, argv);

    bool serverMode = false;
    std::string target;

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--server" || arg == "--control-stdio")
        {
            serverMode = true;
        }
        else if (!arg.empty() && arg[0] != '-')
        {
            target = arg;
        }
    }

    if (serverMode)
    {
        return RunServer(gSimController);
    }

    UiInit("M72 Simulator");

    auto initResult = gSimController.Initialize(false);
    if (!initResult.ok)
    {
        fprintf(stderr, "Failed to initialize simulator: %s\n", initResult.errorMessage.c_str());
        ImguiShutdown();
        return -1;
    }

    if (target.empty())
    {
        target = "hharryu";
    }

    if (target.length() > 4 && target.substr(target.length() - 4) == ".mra")
    {
        auto loadResult = gSimController.LoadMra(target);
        if (!loadResult.ok)
        {
            fprintf(stderr, "Failed to load MRA: %s\n", loadResult.errorMessage.c_str());
            gSimController.Shutdown();
            ImguiShutdown();
            return -1;
        }
    }
    else
    {
        auto loadResult = gSimController.LoadGame(target);
        if (!loadResult.ok)
        {
            fprintf(stderr, "Failed to load game: %s\n", loadResult.errorMessage.c_str());
            gSimController.Shutdown();
            ImguiShutdown();
            return -1;
        }
        gSimController.Reset(100);
    }

    UiInitWindows();
    UiGameChanged();

    const Uint8 *keyboardState = SDL_GetKeyboardState(NULL);
    bool screenshotKeyPressed = false;

    bool running = true;
    while (running)
    {
        if (!UiBeginFrame())
        {
            running = false;
            break;
        }

        if (keyboardState && keyboardState[SDL_SCANCODE_F12] && !screenshotKeyPressed)
        {
            std::string filename = gSimCore.mVideo->GenerateScreenshotFilename("sim");
            gSimCore.mVideo->SaveScreenshot(filename.c_str());
            screenshotKeyPressed = true;
        }
        else if (keyboardState && !keyboardState[SDL_SCANCODE_F12])
        {
            screenshotKeyPressed = false;
        }

        // Push inputs into the model.  ImguiGetButtons packs joystick bits in
        // the low word (right/left/down/up = bits 0-3, buttons 1-4 = bits 4-7),
        // start at bit 16 and coin at bit 18.
        gSimCore.mTop->pause = gSimCore.mSystemPause;
        uint32_t buttons = ImguiGetButtons();
        gSimCore.mTop->p1_joystick = buttons & 0xf;
        gSimCore.mTop->p1_buttons = (buttons >> 4) & 0xf;
        gSimCore.mTop->start = (buttons >> 16) & 0x1;
        gSimCore.mTop->coin = (buttons >> 18) & 0x1;

        if (gSimCore.mSimulationRun || gSimCore.mSimulationStep)
        {
            if (gSimCore.mSimulationStepVblank)
            {
                gSimCore.TickUntil([&] { return gSimCore.mTop->vblank == 0; }, 0);
                gSimCore.TickUntil([&] { return gSimCore.mTop->vblank != 0; }, 0);
            }
            else
            {
                gSimCore.Tick(gSimCore.mSimulationStepSize);
            }
            gSimCore.mVideo->UpdateTexture();
        }
        gSimCore.mSimulationStep = false;

        UiDraw();
        UiEndFrame();
    }

    gSimController.Shutdown();
    ImguiShutdown();
    return 0;
}
