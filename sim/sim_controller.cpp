#include "sim_controller.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>

#include "file_search.h"
#include "games.h"
#include "imgui_wrap.h"
#include "sim_hierarchy.h"
#include "sim_state.h"
#include "sim_video.h"
#include "testrom_gui.h"

#include "M72.h"
#include "M72___024root.h"
#include "verilated.h"

#include "vltstd/vpi_user.h"

namespace
{

const char *RunStopReasonToString(RunStopReason reason)
{
    switch (reason)
    {
    case RunStopReason::COMPLETED:
        return "completed";
    case RunStopReason::CONDITION_MET:
        return "condition_met";
    case RunStopReason::WATCHPOINT_HIT:
        return "watchpoint_hit";
    case RunStopReason::TIMEOUT:
        return "timeout";
    case RunStopReason::ERROR:
        return "error";
    }
    return "error";
}

const char *VpiTypeToKind(PLI_INT32 type)
{
    switch (type)
    {
    case vpiModule:
        return "module";
    case vpiReg:
        return "reg";
    case vpiNet:
        return "net";
    case vpiMemoryWord:
        return "memory_word";
    default:
        return "signal";
    }
}

void CollectVpiSignalsRecursive(vpiHandle moduleHandle, std::vector<SignalInfo> &signals, std::set<std::string> &seen)
{
    if (moduleHandle == nullptr)
        return;

    if (vpiHandle regIter = vpi_iterate(vpiReg, moduleHandle))
    {
        while (vpiHandle regHandle = vpi_scan(regIter))
        {
            const char *fullName = vpi_get_str(vpiFullName, regHandle);
            if (fullName == nullptr)
                continue;

            std::string name = fullName;
            if (!seen.insert(name).second)
                continue;

            SignalInfo info;
            info.mName = name;
            info.mWidth = std::max(0, vpi_get(vpiSize, regHandle));
            info.mKind = VpiTypeToKind(vpi_get(vpiType, regHandle));
            info.mSource = "vpi";
            signals.push_back(std::move(info));
        }
    }

    if (vpiHandle moduleIter = vpi_iterate(vpiModule, moduleHandle))
    {
        while (vpiHandle childModule = vpi_scan(moduleIter))
        {
            CollectVpiSignalsRecursive(childModule, signals, seen);
        }
    }
}
} // namespace

SimController gSimController;

ControllerResult<EmptyResult> SimController::EnsureInitialized() const
{
    if (!mInitialized)
    {
        return ControllerResult<EmptyResult>::Failure("not_initialized", "Simulator is not initialized");
    }
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::Initialize(bool headless)
{
    if (mInitialized)
    {
        return ControllerResult<EmptyResult>::Success({});
    }

    mHeadless = headless;

    gSimCore.Init();
    gFileSearch.AddSearchPath(".");

    mStateManager = new SimState();

    if (mHeadless)
    {
        gSimCore.mVideo->Init(384, 256, nullptr);
    }
    else
    {
        gSimCore.mVideo->Init(384, 256, ImguiGetRenderer());
    }

    Verilated::traceEverOn(true);
    gSimCore.mTop->dipswitch = mDipSwitch;

    mInitialized = true;
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::Shutdown()
{
    if (mStateManager)
    {
        delete mStateManager;
        mStateManager = nullptr;
    }

    if (mInitialized)
    {
        gSimCore.Shutdown();
    }

    mInitialized = false;
    mVpiHandleCache.clear();
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::LoadGame(const std::string &name)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    Game game = GameFind(name.c_str());
    if (game == GAME_INVALID)
    {
        return ControllerResult<EmptyResult>::Failure("unknown_game", "Unknown game: " + name);
    }

    if (!GameInit(game))
    {
        return ControllerResult<EmptyResult>::Failure("load_failed", "Failed to load game: " + name);
    }

    mStateManager->SetGameName(GameLoadedShortName());
    mDipSwitch = GameDipDefaults();
    gSimCore.mTop->dipswitch = mDipSwitch;
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::LoadMra(const std::string &path)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    if (!GameInitMra(path.c_str()))
    {
        return ControllerResult<EmptyResult>::Failure("load_failed", "Failed to load MRA: " + path);
    }

    mStateManager->SetGameName(gSimCore.GetGameName());
    mDipSwitch = GameDipDefaults();
    gSimCore.mTop->dipswitch = mDipSwitch;
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::Reset(uint64_t cycles)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    gSimCore.mTop->reset = 1;
    gSimCore.Tick(cycles);
    gSimCore.mTop->reset = 0;
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<RunResult> SimController::RunCycles(uint64_t cycles)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<RunResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    ApplyInputState();
    TickResult tickResult = gSimCore.Tick(static_cast<int>(cycles));
    RunResult runResult;
    runResult.mReason = ConvertTickStopReason(tickResult.mReason);
    runResult.mTicksExecuted = tickResult.mTicksExecuted;
    return ControllerResult<RunResult>::Success(runResult);
}

ControllerResult<RunResult> SimController::RunFrames(uint64_t frames)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<RunResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    RunResult runResult;
    for (uint64_t i = 0; i < frames; i++)
    {
        ApplyInputState();
        TickResult lowResult = gSimCore.TickUntil([&] { return gSimCore.mTop->vblank == 0; }, 10000000);
        runResult.mTicksExecuted += lowResult.mTicksExecuted;
        if (!lowResult.Succeeded())
        {
            runResult.mReason = ConvertTickStopReason(lowResult.mReason);
            return ControllerResult<RunResult>::Success(runResult);
        }

        TickResult highResult = gSimCore.TickUntil([&] { return gSimCore.mTop->vblank != 0; }, 10000000);
        runResult.mTicksExecuted += highResult.mTicksExecuted;
        if (!highResult.Succeeded())
        {
            runResult.mReason = ConvertTickStopReason(highResult.mReason);
            return ControllerResult<RunResult>::Success(runResult);
        }

        runResult.mFramesExecuted++;
    }

    runResult.mReason = RunStopReason::COMPLETED;
    return ControllerResult<RunResult>::Success(runResult);
}

ControllerResult<RunResult> SimController::RunUntil(const RunUntilRequest &request)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<RunResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    if (EvaluateCondition(request.mCondition))
    {
        RunResult result;
        result.mReason = RunStopReason::CONDITION_MET;
        return ControllerResult<RunResult>::Success(result);
    }

    uint64_t ticks = 0;
    while (true)
    {
        if (request.mTimeoutCycles > 0 && ticks >= request.mTimeoutCycles)
        {
            RunResult result;
            result.mReason = RunStopReason::TIMEOUT;
            result.mTicksExecuted = ticks;
            return ControllerResult<RunResult>::Success(result);
        }

        ApplyInputState();
        TickResult tickResult = gSimCore.Tick(1);
        ticks += tickResult.mTicksExecuted;
        if (tickResult.mReason != TickStopReason::COMPLETED)
        {
            RunResult result;
            result.mReason = ConvertTickStopReason(tickResult.mReason);
            result.mTicksExecuted = ticks;
            return ControllerResult<RunResult>::Success(result);
        }

        if (EvaluateCondition(request.mCondition))
        {
            RunResult result;
            result.mReason = RunStopReason::CONDITION_MET;
            result.mTicksExecuted = ticks;
            return ControllerResult<RunResult>::Success(result);
        }
    }
}

ControllerResult<SimStatus> SimController::GetStatus() const
{
    SimStatus status;
    status.mInitialized = mInitialized;
    status.mRunning = gSimCore.mSimulationRun;
    status.mPaused = gSimCore.mSystemPause;
    status.mTraceActive = gSimCore.IsTraceActive();
    status.mHeadless = mHeadless;
    status.mTotalTicks = gSimCore.GetTotalTicks();
    status.mGameName = mInitialized ? gSimCore.GetGameName() : "";
    return ControllerResult<SimStatus>::Success(status);
}

ControllerResult<CpuState> SimController::GetCpuState() const
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<CpuState>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    CpuState state;
    state.mPc = gSimCore.GetCpuLinearPc();
    state.mRegisters.push_back(gSimCore.mTop->dbg_cpu_cs);
    state.mRegisters.push_back(gSimCore.mTop->dbg_cpu_ip);
    state.mRegisters.push_back(gSimCore.mTop->dbg_cpu_opcode);
    return ControllerResult<CpuState>::Success(state);
}

ControllerResult<MemoryRegion> SimController::ParseRegion(const std::string &name) const
{
    static const std::vector<std::pair<std::string, MemoryRegion>> kRegions = {
        {"CPU_ROM", MemoryRegion::CPU_ROM},
        {"SPRITE_ROM", MemoryRegion::SPRITE_ROM},
        {"BG_A_ROM", MemoryRegion::BG_A_ROM},
        {"BG_B_ROM", MemoryRegion::BG_B_ROM},
        {"WORK_RAM", MemoryRegion::WORK_RAM},
        {"SPRITE_RAM", MemoryRegion::SPRITE_RAM},
        {"SOUND_ROM", MemoryRegion::SOUND_ROM},
        {"SAMPLE_ROM", MemoryRegion::SAMPLE_ROM},
        {"MCU_RAM", MemoryRegion::MCU_RAM},
        {"MCU_ROM", MemoryRegion::MCU_ROM},
        {"MCU_SHARED_RAM", MemoryRegion::MCU_SHARED_RAM},
        {"VRAM_A", MemoryRegion::VRAM_A},
        {"VRAM_B", MemoryRegion::VRAM_B},
    };

    for (const auto &entry : kRegions)
    {
        if (entry.first == name)
        {
            return ControllerResult<MemoryRegion>::Success(entry.second);
        }
    }

    return ControllerResult<MemoryRegion>::Failure("invalid_region", "Unknown memory region: " + name);
}

ControllerResult<MemoryReadResult> SimController::ReadMemory(const std::string &region, uint32_t address, uint32_t size) const
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<MemoryReadResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    auto regionResult = ParseRegion(region);
    if (!regionResult.ok)
    {
        return ControllerResult<MemoryReadResult>::Failure(regionResult.errorCode, regionResult.errorMessage);
    }

    MemoryReadResult result;
    result.mRegion = region;
    result.mAddress = address;
    result.mData.resize(size);
    gSimCore.Memory(regionResult.value).Read(address, size, result.mData.data());
    return ControllerResult<MemoryReadResult>::Success(result);
}

ControllerResult<EmptyResult> SimController::WriteMemory(const std::string &region, uint32_t address, const std::vector<uint8_t> &data)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    auto regionResult = ParseRegion(region);
    if (!regionResult.ok)
    {
        return ControllerResult<EmptyResult>::Failure(regionResult.errorCode, regionResult.errorMessage);
    }

    gSimCore.Memory(regionResult.value).Write(address, static_cast<uint32_t>(data.size()), data.data());
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<std::vector<std::string>> SimController::ListRegions() const
{
    return ControllerResult<std::vector<std::string>>::Success(
        {"CPU_ROM", "SPRITE_ROM", "BG_A_ROM", "BG_B_ROM", "WORK_RAM", "SPRITE_RAM", "SOUND_ROM", "SAMPLE_ROM", "MCU_RAM",
         "MCU_ROM", "MCU_SHARED_RAM", "VRAM_A", "VRAM_B"});
}

ControllerResult<EmptyResult> SimController::DebugLinkStart(uint32_t commsWordAddr)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;
    gSimCore.DebugLinkStart(commsWordAddr);
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::DebugLinkStop()
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;
    gSimCore.DebugLinkStop();
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::DebugLinkWrite(const std::vector<uint8_t> &data, uint64_t timeoutCyclesPerByte)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;
    if (!gSimCore.DebugLinkWrite(data, timeoutCyclesPerByte))
        return ControllerResult<EmptyResult>::Failure("debug_link_timeout", "Timed out writing debug-link data");
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<DebugLinkReadResult> SimController::DebugLinkRead(uint32_t maxBytes, uint32_t minBytes, uint64_t timeoutCycles)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return ControllerResult<DebugLinkReadResult>::Failure(initResult.errorCode, initResult.errorMessage);
    DebugLinkReadResult result;
    result.mData = gSimCore.DebugLinkRead(maxBytes, minBytes, timeoutCycles);
    result.mAvailable = static_cast<uint32_t>(result.mData.size());
    return ControllerResult<DebugLinkReadResult>::Success(result);
}

ControllerResult<SignalReadResult> SimController::ReadSignal(const std::string &signal) const
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<SignalReadResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    return ReadSignalValue(signal);
}

ControllerResult<SignalListResult> SimController::ListSignals() const
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<SignalListResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    SignalListResult result;
    result.mSignals.push_back({"vblank", 1, "alias", "builtin"});
    result.mSignals.push_back({"hblank", 1, "alias", "builtin"});
    result.mSignals.push_back({"reset", 1, "alias", "builtin"});
    result.mSignals.push_back({"rom_load_busy", 1, "alias", "builtin"});
    result.mSignals.push_back({"cpu_cs", 16, "alias", "builtin"});
    result.mSignals.push_back({"cpu_ip", 16, "alias", "builtin"});
    result.mSignals.push_back({"cpu_opcode", 8, "alias", "builtin"});

    auto vpiResult = ListSignalsVpi();
    if (!vpiResult.ok)
    {
        return vpiResult;
    }

    result.mSignals.insert(result.mSignals.end(), vpiResult.value.mSignals.begin(), vpiResult.value.mSignals.end());
    std::sort(result.mSignals.begin(), result.mSignals.end(),
              [](const SignalInfo &a, const SignalInfo &b) { return a.mName < b.mName; });
    return ControllerResult<SignalListResult>::Success(result);
}

ControllerResult<EmptyResult> SimController::SetDipSwitch(uint8_t switchIndex, bool enabled)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    if (switchIndex >= 16)
    {
        return ControllerResult<EmptyResult>::Failure("invalid_dipswitch", "DIP switch index must be 0..15");
    }

    uint16_t mask = static_cast<uint16_t>(1u << switchIndex);
    if (enabled)
        mDipSwitch |= mask;
    else
        mDipSwitch &= static_cast<uint16_t>(~mask);

    gSimCore.mTop->dipswitch = mDipSwitch;
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::SetDipSwitches(uint16_t value)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    mDipSwitch = value;
    gSimCore.mTop->dipswitch = mDipSwitch;
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<InputStateResult> SimController::GetInputState() const
{
    InputStateResult result;
    result.mButtons = ImguiGetButtons();
    return ControllerResult<InputStateResult>::Success(result);
}

ControllerResult<uint32_t> SimController::ParseInputBits(const std::string &name) const
{
    if (name == "left")
        return ControllerResult<uint32_t>::Success(0x0002);
    if (name == "right")
        return ControllerResult<uint32_t>::Success(0x0001);
    if (name == "down")
        return ControllerResult<uint32_t>::Success(0x0004);
    if (name == "up")
        return ControllerResult<uint32_t>::Success(0x0008);
    if (name == "button1" || name == "btn1" || name == "a")
        return ControllerResult<uint32_t>::Success(0x0010);
    if (name == "button2" || name == "btn2" || name == "b")
        return ControllerResult<uint32_t>::Success(0x0020);
    if (name == "button3" || name == "btn3" || name == "x")
        return ControllerResult<uint32_t>::Success(0x0040);
    if (name == "button4" || name == "btn4" || name == "y")
        return ControllerResult<uint32_t>::Success(0x0080);
    if (name == "start")
        return ControllerResult<uint32_t>::Success(0x00010000);
    if (name == "coin")
        return ControllerResult<uint32_t>::Success(0x00040000);

    return ControllerResult<uint32_t>::Failure("invalid_input", "Unknown input: " + name);
}

void SimController::ApplyInputState() const
{
    if (!mInitialized)
        return;

    uint32_t buttons = ImguiGetButtons();
    gSimCore.mTop->p1_joystick = buttons & 0xf;
    gSimCore.mTop->p1_buttons = (buttons >> 4) & 0xf;
    gSimCore.mTop->start = (buttons >> 16) & 0x1;
    gSimCore.mTop->coin = (buttons >> 18) & 0x1;
}

ControllerResult<EmptyResult> SimController::SetInput(const std::string &name, bool pressed)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    auto bitResult = ParseInputBits(name);
    if (!bitResult.ok)
        return ControllerResult<EmptyResult>::Failure(bitResult.errorCode, bitResult.errorMessage);

    if (pressed)
        ImguiSetButtonBits(bitResult.value);
    else
        ImguiClearButtonBits(bitResult.value);

    ApplyInputState();
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::ClearInput(const std::string &name)
{
    return SetInput(name, false);
}

ControllerResult<RunResult> SimController::PressInput(const std::string &name)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return ControllerResult<RunResult>::Failure(initResult.errorCode, initResult.errorMessage);

    auto bitResult = ParseInputBits(name);
    if (!bitResult.ok)
        return ControllerResult<RunResult>::Failure(bitResult.errorCode, bitResult.errorMessage);

    RunResult totalResult;

    ImguiSetButtonBits(bitResult.value);
    ApplyInputState();

    auto pressedResult = RunFrames(2);
    if (!pressedResult.ok)
        return pressedResult;
    totalResult.mTicksExecuted += pressedResult.value.mTicksExecuted;
    totalResult.mFramesExecuted += pressedResult.value.mFramesExecuted;
    if (pressedResult.value.mReason != RunStopReason::COMPLETED)
    {
        totalResult.mReason = pressedResult.value.mReason;
        ImguiClearButtonBits(bitResult.value);
        ApplyInputState();
        return ControllerResult<RunResult>::Success(totalResult);
    }

    ImguiClearButtonBits(bitResult.value);
    ApplyInputState();

    auto releasedResult = RunFrames(2);
    if (!releasedResult.ok)
        return releasedResult;
    totalResult.mTicksExecuted += releasedResult.value.mTicksExecuted;
    totalResult.mFramesExecuted += releasedResult.value.mFramesExecuted;
    totalResult.mReason = releasedResult.value.mReason;

    return ControllerResult<RunResult>::Success(totalResult);
}

uint16_t SimController::GetDipSwitches() const
{
    return mDipSwitch;
}

ControllerResult<StateListResult> SimController::ListStates() const
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<StateListResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    StateListResult result;
    result.mStates = mStateManager->GetStateFiles();
    return ControllerResult<StateListResult>::Success(result);
}

ControllerResult<EmptyResult> SimController::SaveState(const std::string &filename)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    if (!mStateManager->SaveState(filename.c_str()))
    {
        return ControllerResult<EmptyResult>::Failure("unsupported", SimState::UnsupportedReason());
    }
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::LoadState(const std::string &filename)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    if (!mStateManager->RestoreState(filename.c_str()))
    {
        return ControllerResult<EmptyResult>::Failure("unsupported", SimState::UnsupportedReason());
    }
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::SaveNvram(const std::string &filename)
{
    (void)filename;
    return ControllerResult<EmptyResult>::Failure("unsupported", "nvram/hiscore is not wired up in the M72 sim yet");
}

ControllerResult<EmptyResult> SimController::LoadNvram(const std::string &filename)
{
    (void)filename;
    return ControllerResult<EmptyResult>::Failure("unsupported", "nvram/hiscore is not wired up in the M72 sim yet");
}

ControllerResult<EmptyResult> SimController::StartTrace(const std::string &filename, int depth)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    gSimCore.StartTrace(filename.c_str(), depth);
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::StopTrace()
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    gSimCore.StopTrace();
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::StartAudioCapture(const std::string &filename)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    if (!gSimCore.StartAudioCapture(filename.c_str()))
    {
        return ControllerResult<EmptyResult>::Failure("audio_capture_failed", "Failed to start audio capture: " + filename);
    }

    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<EmptyResult> SimController::StopAudioCapture()
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
        return initResult;

    gSimCore.StopAudioCapture();
    return ControllerResult<EmptyResult>::Success({});
}

ControllerResult<ScreenshotResult> SimController::SaveScreenshot(const std::string &path)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<ScreenshotResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    gSimCore.mVideo->UpdateTexture();
    if (!gSimCore.mVideo->SaveScreenshot(path.c_str()))
    {
        return ControllerResult<ScreenshotResult>::Failure("screenshot_failed", "Failed to save screenshot: " + path);
    }

    ScreenshotResult result;
    result.mPath = path;
    return ControllerResult<ScreenshotResult>::Success(result);
}

ControllerResult<FlipResult> SimController::SetFlip(bool flipX, bool flipY)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<FlipResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    // The video window's checkboxes drive mTop->flip_x/flip_y each tick, so set these and
    // the next Tick() applies them to the RTL global_flip_x/global_flip_y inputs.
    gSimCore.mVideo->mFlipX = flipX;
    gSimCore.mVideo->mFlipY = flipY;

    FlipResult result;
    result.mFlipX = flipX;
    result.mFlipY = flipY;
    return ControllerResult<FlipResult>::Success(result);
}

ControllerResult<GuiStateResult> SimController::GetGuiState() const
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<GuiStateResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    GuiStateResult result;
    TestRomGuiState guiState = GetTestRomGuiWindow().GetState();
    result.mAvailable = guiState.mAvailable;
    result.mAddress = guiState.mAddress;
    result.mLastSyncTicks = guiState.mLastSyncTicks;
    result.mEntries.reserve(guiState.mEntries.size());
    for (const auto &entry : guiState.mEntries)
    {
        GuiEntryState out;
        out.mIndex = entry.mIndex;
        out.mLabel = entry.mLabel;
        out.mType = entry.mType;
        out.mTypeName = TestRomGuiWindow::TypeName(entry.mType);
        out.mValue = entry.mValue;
        out.mOverrideValue = entry.mOverrideValue;
        result.mEntries.push_back(std::move(out));
    }

    return ControllerResult<GuiStateResult>::Success(result);
}

ControllerResult<GuiOverrideResult> SimController::SetGuiOverrideByIndex(uint32_t index, uint16_t value, bool pulse)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<GuiOverrideResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    bool applied = false;
    std::string error;
    if (!GetTestRomGuiWindow().SetOverrideByIndex(index, value, pulse, &applied, &error))
    {
        return ControllerResult<GuiOverrideResult>::Failure("gui_unavailable", error);
    }

    GuiOverrideResult result;
    result.mApplied = applied;
    return ControllerResult<GuiOverrideResult>::Success(result);
}

ControllerResult<GuiOverrideResult> SimController::SetGuiOverrideByLabel(const std::string &label, uint16_t value, bool pulse)
{
    auto initResult = EnsureInitialized();
    if (!initResult.ok)
    {
        return ControllerResult<GuiOverrideResult>::Failure(initResult.errorCode, initResult.errorMessage);
    }

    bool applied = false;
    std::string error;
    if (!GetTestRomGuiWindow().SetOverrideByLabel(label, value, pulse, &applied, &error))
    {
        return ControllerResult<GuiOverrideResult>::Failure("gui_unavailable", error);
    }

    GuiOverrideResult result;
    result.mApplied = applied;
    return ControllerResult<GuiOverrideResult>::Success(result);
}

ControllerResult<SignalReadResult> SimController::ReadSignalValue(const std::string &signal) const
{
    auto builtinResult = ReadSignalValueBuiltin(signal);
    if (builtinResult.ok)
    {
        return builtinResult;
    }

    return ReadSignalValueVpi(signal);
}

ControllerResult<SignalReadResult> SimController::ReadSignalValueBuiltin(const std::string &signal) const
{
    SignalReadResult result;
    result.mSignal = signal;
    result.mWidth = 1;

    if (signal == "vblank")
    {
        result.mValue = gSimCore.mTop->vblank;
    }
    else if (signal == "hblank")
    {
        result.mValue = gSimCore.mTop->hblank;
    }
    else if (signal == "reset")
    {
        result.mValue = gSimCore.mTop->reset;
    }
    else if (signal == "rom_load_busy")
    {
        result.mValue = gSimCore.mTop->ioctl_download;
    }
    else if (signal == "cpu_cs")
    {
        result.mValue = gSimCore.mTop->dbg_cpu_cs;
        result.mWidth = 16;
    }
    else if (signal == "cpu_ip")
    {
        result.mValue = gSimCore.mTop->dbg_cpu_ip;
        result.mWidth = 16;
    }
    else if (signal == "cpu_opcode")
    {
        result.mValue = gSimCore.mTop->dbg_cpu_opcode;
        result.mWidth = 8;
    }
    else
    {
        return ControllerResult<SignalReadResult>::Failure("invalid_signal", "Unknown built-in signal: " + signal);
    }

    std::ostringstream valueHex;
    valueHex << std::hex << result.mValue;
    result.mValueHex = valueHex.str();
    return ControllerResult<SignalReadResult>::Success(result);
}

vpiHandle SimController::LookupVpiHandle(const std::string &signal) const
{
    auto it = mVpiHandleCache.find(signal);
    if (it != mVpiHandleCache.end())
    {
        return it->second;
    }

    const std::string candidates[] = {
        signal,
        signal.rfind("TOP.", 0) == 0 ? signal : "TOP." + signal,
        signal.rfind("sim_top.", 0) == 0 ? signal : "sim_top." + signal,
        signal.rfind("TOP.", 0) == 0 || signal.rfind("sim_top.", 0) == 0 ? signal : "TOP.sim_top." + signal,
    };

    for (const auto &candidate : candidates)
    {
        vpiHandle handle = vpi_handle_by_name(const_cast<PLI_BYTE8 *>(candidate.c_str()), nullptr);
        if (handle != nullptr)
        {
            mVpiHandleCache[signal] = handle;
            return handle;
        }
    }

    return nullptr;
}

ControllerResult<SignalReadResult> SimController::ReadSignalValueVpi(const std::string &signal) const
{
    vpiHandle handle = LookupVpiHandle(signal);
    if (handle == nullptr)
    {
        return ControllerResult<SignalReadResult>::Failure(
            "invalid_signal", "Unknown signal: " + signal + " (also tried TOP." + signal + ")");
    }

    int width = vpi_get(vpiSize, handle);
    if (width <= 0)
    {
        return ControllerResult<SignalReadResult>::Failure("signal_read_failed", "Failed to determine signal width: " + signal);
    }
    if (width > 64)
    {
        return ControllerResult<SignalReadResult>::Failure("signal_too_wide", "Signal is wider than 64 bits: " + signal);
    }

    s_vpi_value value;
    value.format = vpiHexStrVal;
    vpi_get_value(handle, &value);
    if (value.value.str == nullptr)
    {
        return ControllerResult<SignalReadResult>::Failure("signal_read_failed", "Failed to read signal value: " + signal);
    }

    std::string valueHex = value.value.str;
    for (char c : valueHex)
    {
        if (c == 'x' || c == 'X' || c == 'z' || c == 'Z')
        {
            return ControllerResult<SignalReadResult>::Failure("signal_has_unknown_bits", "Signal has X/Z bits: " + signal);
        }
    }

    uint64_t parsedValue = 0;
    std::stringstream stream;
    stream << std::hex << valueHex;
    stream >> parsedValue;
    if (stream.fail())
    {
        return ControllerResult<SignalReadResult>::Failure("signal_read_failed", "Failed to parse signal value: " + signal);
    }

    SignalReadResult result;
    result.mSignal = signal;
    result.mValue = parsedValue;
    result.mWidth = static_cast<uint32_t>(width);
    result.mValueHex = valueHex;
    return ControllerResult<SignalReadResult>::Success(result);
}

ControllerResult<SignalListResult> SimController::ListSignalsVpi() const
{
    SignalListResult result;
    std::set<std::string> seen;

    vpiHandle moduleIter = vpi_iterate(vpiModule, nullptr);
    if (moduleIter == nullptr)
    {
        return ControllerResult<SignalListResult>::Failure("signal_list_failed", "Failed to enumerate VPI modules");
    }

    while (vpiHandle moduleHandle = vpi_scan(moduleIter))
    {
        CollectVpiSignalsRecursive(moduleHandle, result.mSignals, seen);
    }

    return ControllerResult<SignalListResult>::Success(result);
}

bool SimController::EvaluateCondition(const Condition &condition) const
{
    switch (condition.mType)
    {
    case Condition::Type::SIGNAL_EQUALS:
    {
        auto signalResult = ReadSignalValue(condition.mSignal);
        return signalResult.ok && signalResult.value.mValue == condition.mValue;
    }
    case Condition::Type::SIGNAL_NOT_EQUALS:
    {
        auto signalResult = ReadSignalValue(condition.mSignal);
        return signalResult.ok && signalResult.value.mValue != condition.mValue;
    }
    case Condition::Type::SIGNAL_LESS_THAN:
    {
        auto signalResult = ReadSignalValue(condition.mSignal);
        return signalResult.ok && signalResult.value.mValue < condition.mValue;
    }
    case Condition::Type::SIGNAL_LESS_EQUAL:
    {
        auto signalResult = ReadSignalValue(condition.mSignal);
        return signalResult.ok && signalResult.value.mValue <= condition.mValue;
    }
    case Condition::Type::SIGNAL_GREATER_THAN:
    {
        auto signalResult = ReadSignalValue(condition.mSignal);
        return signalResult.ok && signalResult.value.mValue > condition.mValue;
    }
    case Condition::Type::SIGNAL_GREATER_EQUAL:
    {
        auto signalResult = ReadSignalValue(condition.mSignal);
        return signalResult.ok && signalResult.value.mValue >= condition.mValue;
    }
    case Condition::Type::CPU_PC_EQUALS:
        return gSimCore.GetCpuLinearPc() == condition.mValue;
    case Condition::Type::CPU_PC_IN_RANGE:
    {
        uint32_t pc = gSimCore.GetCpuLinearPc();
        return pc >= condition.mValue && pc < condition.mValue2;
    }
    case Condition::Type::CPU_PC_OUT_OF_RANGE:
    {
        uint32_t pc = gSimCore.GetCpuLinearPc();
        return pc < condition.mValue || pc >= condition.mValue2;
    }
    case Condition::Type::AND:
        for (const auto &child : condition.mChildren)
        {
            if (!EvaluateCondition(child))
                return false;
        }
        return true;
    case Condition::Type::OR:
        for (const auto &child : condition.mChildren)
        {
            if (EvaluateCondition(child))
                return true;
        }
        return false;
    case Condition::Type::NOT:
        return !condition.mChildren.empty() && !EvaluateCondition(condition.mChildren[0]);
    }
    return false;
}

RunStopReason SimController::ConvertTickStopReason(TickStopReason reason) const
{
    switch (reason)
    {
    case TickStopReason::COMPLETED:
        return RunStopReason::COMPLETED;
    case TickStopReason::WATCHPOINT_HIT:
        return RunStopReason::WATCHPOINT_HIT;
    case TickStopReason::CONDITION_MET:
        return RunStopReason::CONDITION_MET;
    case TickStopReason::TIMEOUT:
        return RunStopReason::TIMEOUT;
    }
    return RunStopReason::ERROR;
}
