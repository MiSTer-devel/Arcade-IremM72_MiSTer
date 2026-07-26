
#include "sim_core.h"
#include "games.h"
#include "sim_hierarchy.h"
#include "M72.h"
#include "M72___024root.h"
#include "M72__Syms.h"
#include "verilated.h"
#include "verilated_fst_c.h"
#include "sim_sdram.h"
#include "sim_ddr.h"
#include "sim_video.h"
#include "sim_audio_capture.h"
#include "testrom_gui.h"

#include <cstring>
#include <cstdio>
#include <algorithm>

// Global instance
SimCore gSimCore;

namespace
{
bool gPrevVblank = false;

// PicoROM comms block layout (testroms comms.c CommsRegisters, 0x800 bytes).
// Fields are u8_rom (byte in a 16-bit word) / u32_rom (byte in 4 words):
//   +0x000  magic "PICO"  (bytes at 0, 2, 4, 6)
//   +0x008  active        (u32_rom, value byte at +8)
//   +0x010  pending
//   +0x018  in_seq
//   +0x020  out_seq
//   +0x028  tick_count
//   +0x200  tick_reset
//   +0x400  in_byte
//   +0x600  out_area[256] (word stride)
constexpr uint32_t DEBUG_LINK_ROM_MASK = (1024 * 1024) - 1;
constexpr uint32_t DEBUG_LINK_ACTIVE_OFF = 0x08;
constexpr uint32_t DEBUG_LINK_PENDING_OFF = 0x10;
constexpr uint32_t DEBUG_LINK_IN_SEQ_OFF = 0x18;
constexpr uint32_t DEBUG_LINK_OUT_SEQ_OFF = 0x20;
constexpr uint32_t DEBUG_LINK_IN_BYTE_OFF = 0x400;
constexpr uint32_t DEBUG_LINK_OUT_AREA_OFF = 0x600;
}

// SimCore implementation
SimCore::SimCore()
    : mTop(nullptr), mVideo(nullptr), mSDRAM(nullptr), mContextp(nullptr), mTotalTicks(0), mTraceActive(false),
      mTraceDepth(1), mSimulationRun(false), mSimulationStep(false), mSimulationStepSize(100000), mSimulationStepVblank(false),
      mSystemPause(false), mSimulationWpSet(false), mSimulationWpAddr(0), mSignalWatchpointCallback()
{
    strcpy(mTraceFilename, "sim.fst");
}

SimCore::~SimCore()
{
    Shutdown();
}

void SimCore::Init()
{
    mContextp = new VerilatedContext;
    mTop = new M72{mContextp};
    mTfp = nullptr;

    for (int i = 0; i < (int)MemoryRegion::COUNT; i++)
    {
        SetMemory((MemoryRegion)i, std::make_unique<MemoryNull>());
    }

    // Create memory subsystems
    mSDRAM = std::make_unique<SimSDRAM>(32 * 1024 * 1024);
    // 64-bit DDR window used by the savestate streamer (slots at 0x3E000000)
    mDDRMemory = std::make_unique<SimDDR>(0x30000000, 256 * 1024 * 1024);
    mVideo = std::make_unique<SimVideo>();
    mAudioCapture = std::make_unique<SimAudioCapture>();

    mDebugLinkEnabled = false;
    mDebugLinkBaseByte = 0;
    mDebugLinkInSeq = 0;
    mDebugLinkOutSeq = 0;
    mDebugLinkPrevInByteRead = false;
    mDebugLinkPrevOutRead = false;
    mDebugLinkTxOutstanding = false;
    mDebugLinkTx.clear();
    mDebugLinkRx.clear();
    GetTestRomGuiWindow().Reset();
    gPrevVblank = false;

    // Debug enables default on (mirrors the OSD debug page defaults)
    mTop->en_layer_a = 1;
    mTop->en_layer_b = 1;
    mTop->en_sprites = 1;
    mTop->en_layer_palette = 1;
    mTop->en_sprite_palette = 1;
    mTop->en_audio_filters = 1;
    mTop->sprite_freeze = 0;
    mTop->video_timing_in = 0; // 55Hz

    auto *syms = mTop->rootp->vlSymsp;

    // SDRAM-backed regions (rtl/m72_pkg.sv LOAD_REGIONS)
    SetMemory(MemoryRegion::CPU_ROM, std::make_unique<MemorySlice>(*mSDRAM, CPU_ROM_SDR_BASE, 1024 * 1024));
    SetMemory(MemoryRegion::SPRITE_ROM, std::make_unique<MemorySlice>(*mSDRAM, SPRITE_ROM_SDR_BASE, 1024 * 1024));
    SetMemory(MemoryRegion::BG_A_ROM, std::make_unique<MemorySlice>(*mSDRAM, BG_A_ROM_SDR_BASE, 1024 * 1024));
    SetMemory(MemoryRegion::BG_B_ROM, std::make_unique<MemorySlice>(*mSDRAM, BG_B_ROM_SDR_BASE, 1024 * 1024));
    SetMemory(MemoryRegion::WORK_RAM,
              std::make_unique<Memory16w>(syms->TOP__sim_top__m72_inst__work_ram.ram.m_storage, 128 * 1024));

    // BRAM-backed regions (verilated dpramv instances)
    SetMemory(MemoryRegion::SPRITE_RAM,
              std::make_unique<Memory16b>(syms->TOP__sim_top__m72_inst__sprite__ram_l.ram.m_storage,
                                          syms->TOP__sim_top__m72_inst__sprite__ram_h.ram.m_storage, 1024));
    SetMemory(MemoryRegion::SOUND_ROM,
              std::make_unique<Memory8b>(syms->TOP__sim_top__m72_inst__sound__sound_rom_ram.ram.m_storage, 65536));
    SetMemory(MemoryRegion::SAMPLE_ROM,
              std::make_unique<Memory8b>(syms->TOP__sim_top__m72_inst__sample_rom__sample_rom.ram.m_storage, 131072));
    SetMemory(MemoryRegion::MCU_RAM,
              std::make_unique<Memory8b>(syms->TOP__sim_top__m72_inst__mcu__nu8051__u_iram.mem.m_storage, 256));
    SetMemory(MemoryRegion::MCU_ROM,
              std::make_unique<Memory8b>(syms->TOP__sim_top__m72_inst__mcu__prom.ram.m_storage, 8192));
    SetMemory(MemoryRegion::MCU_SHARED_RAM,
              std::make_unique<Memory16b>(syms->TOP__sim_top__m72_inst__mcu_shared_ram__ram_0.ram.m_storage,
                                          syms->TOP__sim_top__m72_inst__mcu_shared_ram__ram_1.ram.m_storage, 4096));
    // VRAM layers: 4-way byte-interleaved dpramv banks; exposed as raw banks
    // for now (proper interleaved view when the gfx debug tooling lands).
    SetMemory(MemoryRegion::VRAM_A,
              std::make_unique<Memory16b>(syms->TOP__sim_top__m72_inst__board_b_d__layer_a__ram_00.ram.m_storage,
                                          syms->TOP__sim_top__m72_inst__board_b_d__layer_a__ram_01.ram.m_storage, 8192));
    SetMemory(MemoryRegion::VRAM_B,
              std::make_unique<Memory16b>(syms->TOP__sim_top__m72_inst__board_b_d__layer_b__ram_00.ram.m_storage,
                                          syms->TOP__sim_top__m72_inst__board_b_d__layer_b__ram_01.ram.m_storage, 8192));
}

// One 96MHz-domain service step: models the 3-channel SDRAM controller.
// Must run before the posedge eval so a 1-cycle rdy pulse is seen exactly once.
void SimCore::ServiceSdramChannels()
{
    // ch1: background, 32-bit reads
    {
        uint32_t dout;
        uint8_t rdy;
        mSDRAM->UpdateChannelPulse32(0, 3, mTop->sdr_bg_addr, mTop->sdr_bg_req, &dout, &rdy);
        if (rdy)
            mTop->sdr_bg_dout = dout;
        mTop->sdr_bg_rdy = rdy;
    }

    // ch2: sprites, 64-bit reads
    {
        uint64_t dout;
        uint8_t rdy;
        mSDRAM->UpdateChannelPulse64(1, 3, mTop->sdr_sprite_addr, mTop->sdr_sprite_req, &dout, &rdy);
        if (rdy)
            mTop->sdr_sprite_dout = dout;
        mTop->sdr_sprite_rdy = rdy;
    }

    // ch3: cpu r/w + rom download, 16-bit
    {
        uint16_t dout;
        uint8_t rdy;
        mSDRAM->UpdateChannelPulse16(2, 3, mTop->sdr_ch3_addr, mTop->sdr_ch3_req, mTop->sdr_ch3_rnw, mTop->sdr_ch3_be,
                                     mTop->sdr_ch3_din, &dout, &rdy);
        if (rdy)
        {
            if (mTop->sdr_ch3_rnw)
                mTop->sdr_ch3_dout = dout;
            mLastCpuSdrAddr = mSDRAM->mPulseCh[2].addr;
            mLastCpuSdrRead = mSDRAM->mPulseCh[2].rnw != 0;
            mLastCpuSdrIsCode = mTop->dbg_sdr_cpu_code != 0;
            mLastCpuSdrValid = true;
        }
        mTop->sdr_ch3_rdy = rdy;
    }
}

TickResult SimCore::TickOneCycle()
{
    mTotalTicks++;

    // Service the savestate DDR window once per CLK_32M cycle (memory_stream
    // runs in the 32MHz domain).
    {
        uint64_t rdata = mTop->ddr_rdata;
        uint8_t busy = 0, readComplete = 0;
        mDDRMemory->Clock(mTop->ddr_addr, mTop->ddr_wdata, rdata, mTop->ddr_read != 0, mTop->ddr_write != 0,
                          busy, readComplete, mTop->ddr_burstcnt, mTop->ddr_byteenable);
        mTop->ddr_rdata = rdata;
        mTop->ddr_busy = busy;
        mTop->ddr_read_complete = readComplete;
    }

    // One tick = one CLK_32M period = three CLK_96M periods.  Both clocks come
    // from the same PLL, phase aligned: they rise together at step 0.
    for (int step = 0; step < 3; step++)
    {
        ServiceSdramChannels();

        mContextp->timeInc(1);
        mTop->clk_96m = 1;
        if (step == 0)
            mTop->clk_32m = 1;
        mTop->eval();
        if (mTfp)
            mTfp->dump(mContextp->time());

        mContextp->timeInc(1);
        mTop->clk_96m = 0;
        if (step == 1)
            mTop->clk_32m = 0;
        mTop->eval();
        if (mTfp)
            mTfp->dump(mContextp->time());
    }

    mVideo->Clock(mTop->ce_pixel != 0, mTop->hblank != 0, mTop->vblank != 0, mTop->red, mTop->green, mTop->blue);

    if (mAudioCapture)
    {
        mAudioCapture->Tick(mTotalTicks, true,
                            static_cast<int16_t>(mTop->audio_l),
                            static_cast<int16_t>(mTop->audio_r));
    }
    DebugLinkTick();

    const bool vblank = mTop->vblank != 0;
    if (vblank && !gPrevVblank)
    {
        GetTestRomGuiWindow().TickVblank();
    }
    gPrevVblank = vblank;

    if (mSimulationWpSet && mLastCpuSdrValid && (int)mLastCpuSdrAddr == mSimulationWpAddr)
    {
        mLastCpuSdrValid = false;
        mSimulationRun = false;
        mSimulationStep = false;
        return {TickStopReason::WATCHPOINT_HIT, 1};
    }

    if (mSignalWatchpointCallback && mSignalWatchpointCallback())
    {
        mSimulationRun = false;
        mSimulationStep = false;
        return {TickStopReason::WATCHPOINT_HIT, 1};
    }

    return {TickStopReason::COMPLETED, 1};
}

uint32_t SimCore::GetCpuLinearPc() const
{
    if (!mTop)
        return 0;
    return ((uint32_t)mTop->dbg_cpu_cs << 4) + mTop->dbg_cpu_ip;
}

void SimCore::DebugLinkWriteByte(uint32_t offset, uint8_t value)
{
    if (!mDebugLinkEnabled)
        return;
    Memory(MemoryRegion::CPU_ROM).Write((mDebugLinkBaseByte + offset) & DEBUG_LINK_ROM_MASK, 1, &value);
}

void SimCore::DebugLinkStart(uint32_t commsWordAddr)
{
    // The PicoROM API names the V30 ROM word address.  The TestROM reads this
    // through the normal program-ROM bus (every read is an SDRAM ch3
    // transaction; the V30 has no cache), so the simulator emulates PicoROM by
    // patching the CPU ROM slice and watching ch3 reads.
    mDebugLinkBaseByte = (commsWordAddr << 1) & DEBUG_LINK_ROM_MASK;
    mDebugLinkEnabled = true;
    mDebugLinkInSeq = 0;
    mDebugLinkOutSeq = 0;
    mDebugLinkPrevInByteRead = false;
    mDebugLinkPrevOutRead = false;
    mDebugLinkTxOutstanding = false;
    mDebugLinkTx.clear();
    mDebugLinkRx.clear();

    const uint8_t magic[4] = {'P', 'I', 'C', 'O'};
    for (uint32_t i = 0; i < sizeof(magic); i++)
        DebugLinkWriteByte(i * 2, magic[i]); // u8_rom: byte per 16-bit word
    DebugLinkWriteByte(DEBUG_LINK_ACTIVE_OFF, 1);
    DebugLinkWriteByte(DEBUG_LINK_PENDING_OFF, 0);
    DebugLinkWriteByte(DEBUG_LINK_IN_SEQ_OFF, mDebugLinkInSeq);
    DebugLinkWriteByte(DEBUG_LINK_OUT_SEQ_OFF, mDebugLinkOutSeq);
    DebugLinkWriteByte(DEBUG_LINK_IN_BYTE_OFF, 0);
}

void SimCore::DebugLinkStop()
{
    if (mDebugLinkEnabled)
        DebugLinkWriteByte(DEBUG_LINK_ACTIVE_OFF, 0);
    mDebugLinkEnabled = false;
    mDebugLinkPrevInByteRead = false;
    mDebugLinkPrevOutRead = false;
    mDebugLinkTxOutstanding = false;
    mDebugLinkTx.clear();
    mDebugLinkRx.clear();
}

bool SimCore::DebugLinkEnabled() const
{
    return mDebugLinkEnabled;
}

void SimCore::DebugLinkPrimeTx()
{
    if (!mDebugLinkEnabled || mDebugLinkTxOutstanding || mDebugLinkTx.empty())
        return;
    const uint8_t value = mDebugLinkTx.front();
    mDebugLinkTx.pop_front();
    DebugLinkWriteByte(DEBUG_LINK_IN_BYTE_OFF, value);
    mDebugLinkInSeq++;
    DebugLinkWriteByte(DEBUG_LINK_IN_SEQ_OFF, mDebugLinkInSeq);
    mDebugLinkTxOutstanding = true;
}

void SimCore::DebugLinkTick()
{
    if (!mDebugLinkEnabled || !mTop)
        return;
    if (!mLastCpuSdrValid || !mLastCpuSdrRead)
        return;
    // Only data reads (BS==MEMR) drive the comms handshake. Instruction
    // prefetches (BS==CODE) can touch the same ROM addresses without meaning a
    // read, so skip them to avoid spurious acks/byte advances.
    if (mLastCpuSdrIsCode)
        return;

    const uint32_t byteAddr = mLastCpuSdrAddr & DEBUG_LINK_ROM_MASK;
    mLastCpuSdrValid = false;

    const uint32_t inByteAddr = (mDebugLinkBaseByte + DEBUG_LINK_IN_BYTE_OFF) & ~1u;
    const bool inByteRead = (byteAddr & ~1u) == inByteAddr;
    if (inByteRead && !mDebugLinkPrevInByteRead)
    {
        if (mDebugLinkTxOutstanding)
            mDebugLinkTxOutstanding = false;
        DebugLinkPrimeTx();
    }
    mDebugLinkPrevInByteRead = inByteRead;

    const uint32_t outAreaByte = (mDebugLinkBaseByte + DEBUG_LINK_OUT_AREA_OFF) & DEBUG_LINK_ROM_MASK;
    const bool outRead = (byteAddr >= outAreaByte) && (byteAddr < outAreaByte + 512) && (((byteAddr - outAreaByte) & 1u) == 0);
    if (outRead && !mDebugLinkPrevOutRead)
    {
        const uint8_t value = static_cast<uint8_t>((byteAddr - outAreaByte) >> 1);
        mDebugLinkRx.push_back(value);
        mDebugLinkOutSeq++;
        DebugLinkWriteByte(DEBUG_LINK_OUT_SEQ_OFF, mDebugLinkOutSeq);
    }
    mDebugLinkPrevOutRead = outRead;
}

bool SimCore::DebugLinkWrite(const std::vector<uint8_t> &data, uint64_t timeoutCyclesPerByte)
{
    if (!mDebugLinkEnabled)
        DebugLinkStart();

    const uint64_t timeoutCycles = timeoutCyclesPerByte * std::max<size_t>(data.size(), 1);
    uint64_t elapsed = 0;

    for (uint8_t value : data)
        mDebugLinkTx.push_back(value);
    DebugLinkPrimeTx();

    while (!mDebugLinkTx.empty() || mDebugLinkTxOutstanding)
    {
        if (timeoutCycles && elapsed >= timeoutCycles)
            return false;
        TickResult tickResult = Tick(64);
        elapsed += tickResult.mTicksExecuted;
        if (!tickResult.Succeeded())
            return false;
    }
    return true;
}

std::vector<uint8_t> SimCore::DebugLinkRead(uint32_t maxBytes, uint32_t minBytes, uint64_t timeoutCycles)
{
    if (!mDebugLinkEnabled)
        DebugLinkStart();
    if (minBytes > maxBytes)
        minBytes = maxBytes;

    uint64_t elapsed = 0;
    while (mDebugLinkRx.size() < minBytes)
    {
        if (timeoutCycles && elapsed >= timeoutCycles)
            break;
        TickResult tickResult = Tick(64);
        elapsed += tickResult.mTicksExecuted;
        if (!tickResult.Succeeded())
            break;
    }

    const uint32_t count = std::min<uint32_t>(maxBytes, static_cast<uint32_t>(mDebugLinkRx.size()));
    std::vector<uint8_t> out;
    out.reserve(count);
    for (uint32_t i = 0; i < count; i++)
    {
        out.push_back(mDebugLinkRx.front());
        mDebugLinkRx.pop_front();
    }
    return out;
}

TickResult SimCore::Tick(int count)
{
    TickResult result{TickStopReason::COMPLETED, 0};

    for (int i = 0; i < count; i++)
    {
        TickResult tickResult = TickOneCycle();
        result.mTicksExecuted += tickResult.mTicksExecuted;
        if (tickResult.mReason != TickStopReason::COMPLETED)
        {
            result.mReason = tickResult.mReason;
            return result;
        }
    }

    return result;
}

TickResult SimCore::TickUntil(std::function<bool()> until, int limit)
{
    int count = 0;
    while (!until())
    {
        count++;
        if (count == limit)
        {
            return {TickStopReason::TIMEOUT, count - 1};
        }

        TickResult tickResult = Tick(1);
        if (tickResult.mReason != TickStopReason::COMPLETED)
        {
            tickResult.mTicksExecuted = count;
            return tickResult;
        }
    }
    return {TickStopReason::CONDITION_MET, count};
}

void SimCore::Shutdown()
{
    if (mTfp)
    {
        mTfp->close();
        mTfp.reset();
    }

    if (mTop)
    {
        mTop->final();
        delete mTop;
        mTop = nullptr;
    }

    if (mContextp)
    {
        delete mContextp;
        mContextp = nullptr;
    }

    // Reset member objects
    mAudioCapture.reset();
    mSDRAM.reset();
    mVideo.reset();
    mSignalWatchpointCallback = nullptr;
    GetTestRomGuiWindow().Reset();
    gPrevVblank = false;
}

void SimCore::SetSignalWatchpointCallback(std::function<bool()> callback)
{
    mSignalWatchpointCallback = std::move(callback);
}

void SimCore::StartTrace(const char *filename, int depth)
{
    if (!mContextp || !mTop)
        return;

    if (mTfp)
    {
        mTfp->close();
        mTfp.reset();
    }

    strcpy(mTraceFilename, filename);
    mTraceDepth = depth;

    mTfp = std::make_unique<VerilatedFstC>();
    mTop->trace(mTfp.get(), mTraceDepth);
    mTfp->open(mTraceFilename);
    mTraceActive = true;
}

void SimCore::StopTrace()
{
    if (mTfp)
    {
        mTfp->close();
        mTfp.reset();
    }
    mTraceActive = false;
}

bool SimCore::StartAudioCapture(const char *filename, uint64_t simClockHz)
{
    if (!mAudioCapture)
    {
        mAudioCapture = std::make_unique<SimAudioCapture>();
    }
    return mAudioCapture->Start(filename, simClockHz);
}

void SimCore::StopAudioCapture()
{
    if (mAudioCapture)
    {
        mAudioCapture->Stop();
    }
}

bool SimCore::IsAudioCaptureActive() const
{
    return mAudioCapture && mAudioCapture->IsActive();
}

bool SimCore::SendIOCTLData(uint8_t index, const std::vector<uint8_t> &data)
{
    if (!mTop)
    {
        return false;
    }

    printf("Starting ioctl download (index=%d, size=%zu)\n", (int)index, data.size());

    // Start download sequence
    mTop->reset = 1;
    mTop->ioctl_download = 1;
    mTop->ioctl_index = index;
    mTop->ioctl_wr = 0;
    mTop->ioctl_dout = 0;

    // Clock to let the core see download start
    Tick(1);

    // Send each byte.  ioctl_wr must be a single clk_sys pulse: rom_loader
    // consumes a byte on every 32M edge that samples wr high.
    for (size_t i = 0; i < data.size(); i++)
    {
        mTop->ioctl_dout = data[i];
        mTop->ioctl_wr = 1;
        Tick(1);
        mTop->ioctl_wr = 0;
        WaitForIOCTLReady();

        // Progress indicator every 256KB
        if ((i & 0x3FFFF) == 0)
        {
            printf("  Sent %zu/%zu bytes\n", i, data.size());
        }
    }

    // End download sequence
    mTop->ioctl_download = 0;
    mTop->reset = 0;
    Tick(1);

    printf("ioctl download complete\n");
    return true;
}

void SimCore::WaitForIOCTLReady()
{
    int timeout = 1000; // Prevent infinite loops

    while (mTop->ioctl_wait && timeout > 0)
    {
        Tick(1);
        timeout--;
    }

    if (timeout == 0)
    {
        printf("Warning: ioctl_wait timeout\n");
    }
}

void SimCore::SetGame(Game game)
{
    mLoadedGame = game;
}

Game SimCore::GetGame() const
{
    return mLoadedGame;
}

const char *SimCore::GetGameName() const
{
    return GameName(GetGame());
}
