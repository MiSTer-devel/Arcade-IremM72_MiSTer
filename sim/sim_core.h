#ifndef SIM_CORE_H
#define SIM_CORE_H

#include <functional>
#include <memory>
#include <vector>
#include <cstdint>
#include <deque>

#include "games.h"
#include "sim_memory.h"

class VerilatedContext;
class M72;
class VerilatedFstC;
class SimSDRAM;
class SimDDR;
class SimVideo;
class SimAudioCapture;

enum class MemoryRegion : int
{
    // SDRAM-backed
    CPU_ROM,
    SPRITE_ROM,
    BG_A_ROM,
    BG_B_ROM,
    WORK_RAM,

    // BRAM-backed (verilated model internals)
    SPRITE_RAM,
    SOUND_ROM,
    SAMPLE_ROM,
    MCU_RAM,
    MCU_ROM,
    MCU_SHARED_RAM,
    VRAM_A,
    VRAM_B,

    COUNT
};

enum class TickStopReason
{
    COMPLETED,
    WATCHPOINT_HIT,
    CONDITION_MET,
    TIMEOUT
};

struct TickResult
{
    TickStopReason mReason;
    int mTicksExecuted;

    bool Succeeded() const
    {
        return mReason == TickStopReason::COMPLETED || mReason == TickStopReason::CONDITION_MET;
    }
};

class SimCore
{
  public:
    // Public members that external code needs access to
    M72 *mTop;
    std::unique_ptr<SimVideo> mVideo;
    std::unique_ptr<SimSDRAM> mSDRAM;
    std::unique_ptr<SimDDR> mDDRMemory; // savestate slot window
    std::unique_ptr<SimAudioCapture> mAudioCapture;

    // Simulation state
    uint64_t mTotalTicks;
    bool mSimulationRun;
    bool mSimulationStep;
    int mSimulationStepSize;
    bool mSimulationStepVblank;
    bool mSystemPause;
    bool mSimulationWpSet;
    int mSimulationWpAddr;
    bool mTraceActive;
    char mTraceFilename[64];
    int mTraceDepth;

    // Constructor/Destructor
    SimCore();
    ~SimCore();

    // Main simulation methods
    void Init();
    TickResult Tick(int count = 1);
    TickResult TickUntil(std::function<bool()> until, int limit);
    void Shutdown();
    void SetSignalWatchpointCallback(std::function<bool()> callback);

    // Trace control methods
    void StartTrace(const char *filename, int depth = 1);
    void StopTrace();
    bool IsTraceActive() const
    {
        return mTraceActive;
    }

    // Audio capture control methods
    bool StartAudioCapture(const char *filename, uint64_t simClockHz = 32'000'000ull);
    void StopAudioCapture();
    bool IsAudioCaptureActive() const;

    // PicoROM/debug-link emulator for simulator-side TestROM control.
    // commsWordAddr is the CPU-ROM word address of the comms block
    // (testroms comms.c uses linear 0x3F000 = word 0x1F800).
    void DebugLinkStart(uint32_t commsWordAddr = 0x1F800);
    void DebugLinkStop();
    bool DebugLinkEnabled() const;
    bool DebugLinkWrite(const std::vector<uint8_t> &data, uint64_t timeoutCyclesPerByte = 2000000);
    std::vector<uint8_t> DebugLinkRead(uint32_t maxBytes, uint32_t minBytes = 0, uint64_t timeoutCycles = 2000000);

    // IOCTL methods
    bool SendIOCTLData(uint8_t index, const std::vector<uint8_t> &data);

    // Stats
    uint64_t GetTotalTicks() const
    {
        return mTotalTicks;
    }

    void SetGame(Game game);
    Game GetGame() const;
    const char *GetGameName() const;

    // Current V30 linear PC ((cs << 4) + ip) from the debug taps
    uint32_t GetCpuLinearPc() const;

    MemoryInterface &Memory(MemoryRegion region)
    {
        return *mMemoryRegion[(int)region];
    }

    // Last completed ch3 (CPU) SDRAM transaction, for watchpoints/DebugLink
    uint32_t mLastCpuSdrAddr = 0xffffffff;
    bool mLastCpuSdrRead = false;
    bool mLastCpuSdrIsCode = false;
    bool mLastCpuSdrValid = false;

  private:
    // Verilator context and top module
    VerilatedContext *mContextp;
    std::unique_ptr<VerilatedFstC> mTfp;
    std::function<bool()> mSignalWatchpointCallback;

    std::unique_ptr<MemoryInterface> mMemoryRegion[(int)MemoryRegion::COUNT];

    Game mLoadedGame = GAME_INVALID;

    bool mDebugLinkEnabled = false;
    uint32_t mDebugLinkBaseByte = 0;
    uint8_t mDebugLinkInSeq = 0;
    uint8_t mDebugLinkOutSeq = 0;
    bool mDebugLinkPrevInByteRead = false;
    bool mDebugLinkPrevOutRead = false;
    bool mDebugLinkTxOutstanding = false;
    std::deque<uint8_t> mDebugLinkTx;
    std::deque<uint8_t> mDebugLinkRx;

    TickResult TickOneCycle();
    void ServiceSdramChannels();
    void DebugLinkTick();
    void DebugLinkPrimeTx();
    void DebugLinkWriteByte(uint32_t offset, uint8_t value);

    // IOCTL helper methods
    void WaitForIOCTLReady();

    void SetMemory(MemoryRegion region, std::unique_ptr<MemoryInterface> &&memory)
    {
        mMemoryRegion[(int)region].swap(memory);
    }
};

// Global instance
extern SimCore gSimCore;

#endif // SIM_CORE_H
