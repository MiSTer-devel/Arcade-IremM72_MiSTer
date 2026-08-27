#include "sim_state.h"
#include "M72.h"
#include "sim_ddr.h"
#include "sim_core.h"
#include "M72___024root.h"
#include "M72__Syms.h"

#include <dirent.h>
#include <algorithm>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <sstream>
#include <iomanip>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <functional>

namespace
{
uint64_t GetEnvU64(const char *name, uint64_t defaultValue)
{
    const char *value = std::getenv(name);
    if (!value || !*value)
        return defaultValue;

    char *end = nullptr;
    uint64_t parsed = std::strtoull(value, &end, 0);
    return end != value ? parsed : defaultValue;
}

const char *SaveStateStateName(uint8_t state)
{
    switch (state)
    {
    case 0: return "IDLE";
    case 1: return "SAVE_WAIT_PAUSE";
    case 2: return "SAVE_SETTLE";
    case 3: return "SAVE_WAIT_WRITE";
    case 6: return "RESTORE_WAIT_PAUSE";
    case 7: return "RESTORE_WAIT_READ";
    case 8: return "RESTORE_DRAIN";
    default: return "UNKNOWN";
    }
}

const char *MemoryStreamStateName(uint32_t state)
{
    switch (state)
    {
    case 0: return "IDLE";
    case 1: return "READ_MEM_REQ";
    case 2: return "READ_MEM_WAIT";
    case 3: return "READ_STREAM";
    case 4: return "WRITE_GATHER";
    case 5: return "WRITE_MEM_REQ";
    case 6: return "WRITE_MEM_FINAL_REQ";
    case 7: return "WRITE_MEM_FINAL_WAIT";
    case 8: return "WRITE_MEM_WAIT";
    case 9: return "QUERY_GATHER_FIRST";
    case 10: return "QUERY_GATHER_NEXT";
    case 11: return "QUERY_GATHER_WAIT";
    case 12: return "QUERY_SCATTER_WAIT";
    case 13: return "READ_HEADER";
    case 14: return "READ_HEADER_WAIT";
    case 15: return "WRITE_HEADER";
    case 16: return "WRITE_HEADER_WAIT";
    default: return "UNKNOWN";
    }
}

void PrintStateProgress(const char *op, const char *phase, M72 *top, uint64_t elapsedTicks, const char *prefix = "progress")
{
    auto &m72 = top->rootp->vlSymsp->TOP__sim_top__m72_inst;
    const uint8_t ssState = top->ss_state_out;
    const uint32_t streamState = m72.__PVT__save_state_data__DOT__memory_stream__DOT__state;

    std::fprintf(stderr,
                 "[sim-state] %s %s: %s elapsed=%" PRIu64
                 " ss=%s(%u) pause=%u paused=%u read=%u write=%u v30_quiet=%u ss_err=%u"
                 " stream=%s(%u) is_reading=%u rd_req=%u wr_req=%u query=%u"
                 " chunk_idx=%u chunk_addr=0x%08x remaining=%u width=%u word=%u"
                 " current=0x%08x end=0x%08x ddr_busy=%u ddr_read_done=%u\n",
                 op,
                 phase,
                 prefix,
                 elapsedTicks,
                 SaveStateStateName(ssState),
                 ssState,
                 m72.ss_pause,
                 m72.paused,
                 m72.ss_read,
                 m72.ss_write,
                 m72.v30_ss_quiet,
                 m72.v30__DOT__ss_err,
                 MemoryStreamStateName(streamState),
                 streamState,
                 m72.__PVT__save_state_data__DOT__memory_stream__DOT__is_reading,
                 m72.save_state_data__DOT____Vcellout__memory_stream__read_req,
                 m72.save_state_data__DOT____Vcellout__memory_stream__write_req,
                 m72.save_state_data__DOT____Vcellout__memory_stream__query_req,
                 m72.__PVT__save_state_data__DOT__memory_stream__DOT__chunk_index,
                 m72.save_state_data__DOT____Vcellout__memory_stream__chunk_address,
                 m72.__PVT__save_state_data__DOT__memory_stream__DOT__chunk_remaining,
                 m72.__PVT__save_state_data__DOT__memory_stream__DOT__chunk_width,
                 m72.__PVT__save_state_data__DOT__memory_stream__DOT__word_counter,
                 m72.__PVT__save_state_data__DOT__memory_stream__DOT__current_addr,
                 m72.__PVT__save_state_data__DOT__memory_stream__DOT__end_addr,
                 top->ddr_busy,
                 top->ddr_read_complete);
    std::fflush(stderr);
}

bool TickUntilStateCondition(M72 *top, const char *op, const char *phase, const std::function<bool()> &condition)
{
    const uint64_t timeoutTicks = GetEnvU64("M72_STATE_TIMEOUT_TICKS", 20'000'000ull);
    const uint64_t progressTicks = GetEnvU64("M72_STATE_PROGRESS_TICKS", 500'000ull);
    const uint64_t startTicks = gSimCore.GetTotalTicks();
    uint64_t lastProgressTicks = startTicks;
    uint8_t lastSsState = top->ss_state_out;

    PrintStateProgress(op, phase, top, 0, "start");

    // Set M72_STATE_SECTION_TRACE=1 to log each savestate section as memory_stream
    // writes (save) or parses (restore) it - useful for diagnosing stream desyncs.
    const bool sectionTrace = GetEnvU64("M72_STATE_SECTION_TRACE", 0) != 0;
    uint32_t prevStreamState = 0xffffffff;

    while (!condition())
    {
        TickResult tickResult = gSimCore.Tick(1);
        const uint64_t nowTicks = gSimCore.GetTotalTicks();
        const uint64_t elapsedTicks = nowTicks - startTicks;
        const uint8_t ssState = top->ss_state_out;
        const bool changed = ssState != lastSsState;

        if (sectionTrace)
        {
            auto &m72 = top->rootp->vlSymsp->TOP__sim_top__m72_inst;
            const uint32_t ms = m72.__PVT__save_state_data__DOT__memory_stream__DOT__state;
            // QUERY_SCATTER_WAIT(12): a section header was just parsed during restore.
            if (ms == 12 && prevStreamState != 12)
            {
                std::fprintf(stderr,
                    "[section] idx=%u width=%u remaining=%u current=0x%08x\n",
                    m72.__PVT__save_state_data__DOT__memory_stream__DOT__chunk_index,
                    m72.__PVT__save_state_data__DOT__memory_stream__DOT__chunk_width,
                    m72.__PVT__save_state_data__DOT__memory_stream__DOT__chunk_remaining,
                    m72.__PVT__save_state_data__DOT__memory_stream__DOT__current_addr);
            }
            prevStreamState = ms;
        }

        if (tickResult.mReason != TickStopReason::COMPLETED)
        {
            PrintStateProgress(op, phase, top, elapsedTicks, "stopped");
            return false;
        }

        if (changed || (nowTicks - lastProgressTicks) >= progressTicks)
        {
            PrintStateProgress(op, phase, top, elapsedTicks);
            lastProgressTicks = nowTicks;
            lastSsState = ssState;
        }

        if (elapsedTicks >= timeoutTicks)
        {
            PrintStateProgress(op, phase, top, elapsedTicks, "timeout");
            return false;
        }
    }

    PrintStateProgress(op, phase, top, gSimCore.GetTotalTicks() - startTicks, "done");
    return true;
}
} // namespace

SimState::SimState(M72 *top, SimDDR *memory, int offset, int size)
    : mTop(top), mMemory(memory), mOffset(offset), mSize(size), mGameName("unknown")
{
}

void SimState::SetGameName(const char *gameName)
{
    mGameName = gameName ? gameName : "";
    EnsureStateDirectory();
}

void SimState::EnsureStateDirectory()
{
    mkdir("states", 0755);
    if (!mGameName.empty())
    {
        std::string gameDir = "states/" + mGameName;
        mkdir(gameDir.c_str(), 0755);
    }
}

std::string SimState::GetStatePath(const char *filename)
{
    if (mGameName.empty())
        return std::string("states/") + filename;
    return "states/" + mGameName + "/" + filename;
}

bool SimState::SaveState(const char *filename)
{
    std::string fullPath = GetStatePath(filename);
    std::fprintf(stderr, "[sim-state] save begin: %s\n", fullPath.c_str());
    std::fflush(stderr);

    mTop->ss_index = 0;
    mTop->ss_do_save = 1;
    if (!TickUntilStateCondition(mTop, "save", "request accepted", [&] { return mTop->ss_state_out != 0; }))
    {
        mTop->ss_do_save = 0;
        return false;
    }

    mTop->ss_do_save = 0;
    if (!TickUntilStateCondition(mTop, "save", "state machine idle", [&] { return mTop->ss_state_out == 0; }))
        return false;

    if (!mMemory->SaveData(fullPath.c_str(), mOffset, mSize))
        return false;

    std::fprintf(stderr, "[sim-state] save complete: %s\n", fullPath.c_str());
    std::fflush(stderr);
    return true;
}

bool SimState::RestoreState(const char *filename)
{
    std::string fullPath = GetStatePath(filename);
    std::fprintf(stderr, "[sim-state] restore begin: %s\n", fullPath.c_str());
    std::fflush(stderr);

    struct stat st;
    if (stat(fullPath.c_str(), &st) == 0)
    {
        std::fprintf(stderr, "[sim-state] restore file size: %lld bytes\n", static_cast<long long>(st.st_size));
        std::fflush(stderr);
    }

    if (!mMemory->LoadData(fullPath.c_str(), mOffset, 1)) // Pass stride=1 explicitly
        return false;

    mTop->ss_index = 0;
    mTop->ss_do_restore = 1;
    if (!TickUntilStateCondition(mTop, "restore", "request accepted", [&] { return mTop->ss_state_out != 0; }))
    {
        mTop->ss_do_restore = 0;
        return false;
    }

    mTop->ss_do_restore = 0;
    if (!TickUntilStateCondition(mTop, "restore", "state machine idle", [&] { return mTop->ss_state_out == 0; }))
        return false;

    if (mTop->rootp->vlSymsp->TOP__sim_top__m72_inst.v30__DOT__ss_err)
    {
        std::fprintf(stderr,
                     "[sim-state] restore rejected: incompatible V30 savestate map\n");
        std::fflush(stderr);
        return false;
    }

    {
        uint64_t fileVer = mTop->rootp->vlSymsp->TOP__sim_top__m72_inst.ss_restored_version;
        // SV string literals are packed big-endian (first char in the high byte);
        // decode MSB-first and skip leading nulls from the zero-extension.
        char ascii[9] = {0};
        int n = 0;
        for (int i = 7; i >= 0; i--)
        {
            char c = static_cast<char>((fileVer >> (8 * i)) & 0xff);
            if (c == 0 && n == 0)
                continue;
            ascii[n++] = c;
        }
        std::fprintf(stderr, "[sim-state] restored build version=\"%s\"\n",
                     ascii);
    }

    std::fprintf(stderr, "[sim-state] restore complete: %s\n", fullPath.c_str());
    std::fflush(stderr);
    return true;
}

std::vector<std::string> SimState::GetStateFiles()
{
    std::vector<std::string> files;

    std::string dirPath = mGameName.empty() ? "states" : ("states/" + mGameName);
    DIR *dir = opendir(dirPath.c_str());
    if (!dir)
        return files;

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string filename = entry->d_name;
        if (filename.size() > 9 && filename.substr(filename.size() - 9) == ".m72state")
        {
            files.push_back(filename);
        }
    }
    closedir(dir);

    std::sort(files.begin(), files.end());
    return files;
}

std::string SimState::GenerateNextStateName()
{
    std::vector<std::string> existingFiles = GetStateFiles();

    for (int nextNum = 0; nextNum < 1000; nextNum++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(3) << nextNum << ".m72state";
        if (std::find(existingFiles.begin(), existingFiles.end(), ss.str()) == existingFiles.end())
        {
            return ss.str();
        }
    }

    return "999.m72state";
}
