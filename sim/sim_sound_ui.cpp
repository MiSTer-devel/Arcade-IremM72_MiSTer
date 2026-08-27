#include "imgui_wrap.h"
#include "sim_sound_ui.h"
#include "sim_core.h"
#include "M72.h"
#include "M72___024root.h"
#include "M72__Syms.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <utility>
#include <string>

// Sound command monitor.
//
// The main CPU has no direct access to the YM2151: it writes a single command
// byte to I/O port 0x00, which rtl/pal.sv decodes as `snd_latch1_wr` and
// rtl/sound.sv latches into `snd_latch1` (raising `snd_latch1_ready`, which
// pulls the Z80's INT).  The Z80 sound driver reads the byte back on port 0x02
// and does everything else itself, so the whole main-CPU-side protocol is that
// one byte - commands carry no parameters.  M72 boards also have a second
// latch at port 0xC0 (`snd_latch2_wr`) wired to the Z80's NMI; R-Type never
// uses it, but it is logged here for the games that do.
//
// The decode below reads the driver's own command table out of the shared
// sound RAM, so what the window shows is what the Z80 would actually do with
// the byte.  See docs/rtype_sound_commands.md for how the table was derived.
//
// The window can also inject a command (`SoundCommandsInject`, `sound.send`
// over the JSON server): it pokes `snd_latch1`/`snd_latch1_ready` in
// rtl/sound.sv exactly as the SND strobe would, so the Z80 takes the same
// RST 18h path it takes for a real one.  `M72_YM_LOG=1` traces the YM2151
// register writes the driver makes in response, which is what you diff
// against the reference emulator when a sound plays but plays wrong.

namespace
{

// ---------------------------------------------------------------------------
// R-Type sound driver
// ---------------------------------------------------------------------------

// The driver is uploaded into the shared sound RAM by the V30 at boot (R-Type
// has no sound ROM of its own).  Fingerprint: the reset vector at 0x0000 is
// `DI / IM 0 / JP 0x00A1`, and the command table at 0x1000 starts with the
// "unused command" entry at 0x1100.
constexpr uint16_t kRTypeCmdTable = 0x1000;
constexpr uint16_t kRTypePendingCmd = 0xF800; // 0xFF = idle, else pending command
constexpr uint16_t kRTypeActiveTable = 0xFE00; // one byte per command id, 0xFF = not playing
constexpr uint32_t kMaxCommands = 0x80;        // driver masks the latch byte with 0x7F

// Entry kinds, from the high nibble of the table entry's header byte.
enum SoundEntryKind
{
    KIND_PLAY = 0x00, // start n tracks unconditionally
    KIND_COND = 0x10, // start n tracks, but only if sound `ref` is playing
    KIND_MOD = 0x20,  // attach a modifier voice to playing sound `ref`
};

struct SoundEntry
{
    bool mValid = false;
    uint16_t mAddr = 0;
    uint8_t mHeader = 0;
    uint8_t mKind = 0;
    uint8_t mTracks = 0;
    uint8_t mRef = 0;
    uint16_t mTrack[8] = {};
    uint8_t mPrio[8] = {};
    uint8_t mModB = 0;
    uint8_t mModC = 0;
};

// Names for R-Type's command ids.  The music ids come in triplets - N starts a
// song, N+1 stops it (a conditional entry keyed on N), N+2 attaches a modifier
// voice to it - and were identified by driving MAME through a full playthrough
// and correlating the latch writes with the screen.  The SFX ids listed here
// were pinned down the same way; the rest are left to the structural decode.
struct SoundCommandName
{
    uint8_t mCommand;
    const char *mName;
};

const SoundCommandName kRTypeNames[] = {
    {0x00, "Reset driver / stop all sound"},

    {0x01, "BGM: Stage 1 (restart)"},
    {0x02, "BGM stop: Stage 1 (restart)"},
    {0x03, "BGM modifier: Stage 1 (restart)"},
    {0x04, "BGM: Stage 2"},
    {0x05, "BGM stop: Stage 2"},
    {0x06, "BGM modifier: Stage 2"},
    {0x07, "BGM: Stage 3"},
    {0x08, "BGM stop: Stage 3"},
    {0x09, "BGM modifier: Stage 3"},
    {0x0A, "BGM: Stage 4"},
    {0x0B, "BGM stop: Stage 4"},
    {0x0C, "BGM modifier: Stage 4"},
    {0x0D, "BGM: Stage 6"},
    {0x0E, "BGM stop: Stage 6"},
    {0x0F, "BGM modifier: Stage 6"},
    {0x10, "BGM: Stage 5"},
    {0x11, "BGM stop: Stage 5"},
    {0x12, "BGM modifier: Stage 5"},
    {0x13, "BGM: Stage 7"},
    {0x14, "BGM stop: Stage 7"},
    {0x15, "BGM modifier: Stage 7"},
    {0x16, "BGM: Stage 8"},
    {0x17, "BGM stop: Stage 8"},
    {0x18, "BGM modifier: Stage 8"},
    {0x19, "BGM: Boss"},
    {0x1A, "BGM stop: Boss"},
    {0x1B, "BGM modifier: Boss"},
    {0x1C, "BGM: Stage clear"},
    {0x1D, "BGM stop: Stage clear"},
    {0x1E, "BGM modifier: Stage clear"},
    {0x1F, "BGM: Stage 1 (game start)"},
    {0x20, "BGM stop: Stage 1 (game start)"},
    {0x21, "BGM modifier: Stage 1 (game start)"},
    {0x22, "BGM: Game over"},
    {0x23, "BGM stop: Game over"},
    {0x25, "BGM: Attract mode"},
    {0x26, "BGM stop: Attract mode"},
    {0x27, "BGM modifier: Attract mode"},
    {0x28, "BGM: unidentified (never observed)"},
    {0x29, "BGM stop: unidentified"},
    {0x2A, "BGM modifier: unidentified"},
    {0x2B, "BGM: Ending"},
    {0x2C, "BGM stop: Ending"},
    {0x2D, "BGM modifier: Ending"},

    {0x30, "SFX: Player shot"},
    {0x31, "SFX: Charged beam fired"},
    {0x32, "SFX: Beam charging (loop)"},
    {0x33, "SFX stop: Beam charging"},
    {0x35, "SFX: Player death"},
    {0x50, "SFX: Hit / explosion"},
    {0x63, "SFX: Coin inserted"},

    {0x4C, "BGM: Ending, track 1 alone"},
    {0x4D, "BGM: Ending, track 2 alone"},
    {0x4E, "BGM: Ending, track 3 alone"},
    {0x4F, "BGM: Ending, track 4 alone"},

    {0x70, "SFX: Continue countdown 9"},
    {0x71, "SFX: Continue countdown 8"},
    {0x72, "SFX: Continue countdown 7"},
    {0x73, "SFX: Continue countdown 6"},
    {0x74, "SFX: Continue countdown 5"},
    {0x75, "SFX: Continue countdown 4"},
    {0x76, "SFX: Continue countdown 3"},
    {0x77, "SFX: Continue countdown 2"},
    {0x78, "SFX: Continue countdown 1"},
    {0x79, "SFX: Continue countdown 0"},
};

const char *RTypeCommandName(uint8_t command)
{
    for (const auto &entry : kRTypeNames)
    {
        if (entry.mCommand == command)
            return entry.mName;
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Sound RAM access
// ---------------------------------------------------------------------------

uint8_t SoundRamByte(uint16_t address)
{
    uint8_t value = 0;
    gSimCore.Memory(MemoryRegion::SOUND_ROM).Read(address, 1, &value);
    return value;
}

uint16_t SoundRamWord(uint16_t address)
{
    uint8_t value[2] = {};
    gSimCore.Memory(MemoryRegion::SOUND_ROM).Read(address, 2, value);
    return static_cast<uint16_t>(value[0] | (value[1] << 8));
}

bool RTypeDriverPresent()
{
    static const uint8_t kReset[] = {0xF3, 0xED, 0x46, 0xC3, 0xA1, 0x00}; // DI / IM 0 / JP 0x00A1
    uint8_t reset[sizeof(kReset)] = {};
    gSimCore.Memory(MemoryRegion::SOUND_ROM).Read(0, sizeof(reset), reset);
    for (size_t i = 0; i < sizeof(kReset); i++)
    {
        if (reset[i] != kReset[i])
            return false;
    }
    return SoundRamWord(kRTypeCmdTable) == 0x1100;
}

SoundEntry DecodeRTypeEntry(uint8_t command)
{
    SoundEntry entry;
    if (command >= kMaxCommands)
        return entry;

    entry.mAddr = SoundRamWord(static_cast<uint16_t>(kRTypeCmdTable + command * 2));
    entry.mHeader = SoundRamByte(entry.mAddr);

    // The driver rejects anything with a header >= 0x30; the unused slots all
    // point at one shared 0xF0 entry.
    if (entry.mHeader >= 0x30)
        return entry;

    entry.mValid = true;
    entry.mKind = entry.mHeader & 0xF0;
    entry.mTracks = static_cast<uint8_t>((entry.mHeader & 7) + 1);

    uint16_t cursor = static_cast<uint16_t>(entry.mAddr + 1);
    if (entry.mKind == KIND_MOD)
    {
        entry.mRef = SoundRamByte(cursor);
        entry.mModB = SoundRamByte(static_cast<uint16_t>(cursor + 1));
        entry.mModC = SoundRamByte(static_cast<uint16_t>(cursor + 2));
        entry.mTracks = 0;
        return entry;
    }

    if (entry.mKind == KIND_COND)
    {
        entry.mRef = SoundRamByte(cursor);
        cursor++;
    }

    for (uint8_t i = 0; i < entry.mTracks; i++)
    {
        entry.mTrack[i] = SoundRamWord(cursor);
        entry.mPrio[i] = SoundRamByte(entry.mTrack[i]);
        cursor = static_cast<uint16_t>(cursor + 2);
    }

    return entry;
}

std::string DescribeRTypeEntry(uint8_t command, const SoundEntry &entry)
{
    char text[256];

    if (command == 0x00)
        return "reset: clear channel state, silence all 8 FM channels";

    if (!entry.mValid)
    {
        snprintf(text, sizeof(text), "unused slot (entry %04X, header %02X)", entry.mAddr, entry.mHeader);
        return text;
    }

    if (entry.mKind == KIND_MOD)
    {
        snprintf(text, sizeof(text), "modify sound %02X (vol %02X, pitch %02X) @%04X", entry.mRef, entry.mModB,
                 entry.mModC, entry.mAddr);
        return text;
    }

    std::string result;
    if (entry.mKind == KIND_COND)
    {
        snprintf(text, sizeof(text), "if sound %02X playing: ", entry.mRef);
        result += text;
    }

    snprintf(text, sizeof(text), "%u track%s", static_cast<unsigned>(entry.mTracks), entry.mTracks == 1 ? "" : "s");
    result += text;

    for (uint8_t i = 0; i < entry.mTracks; i++)
    {
        snprintf(text, sizeof(text), "%s %04X(p%02X)", i == 0 ? "" : ",", entry.mTrack[i], entry.mPrio[i]);
        result += text;
    }

    return result;
}

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------

struct SoundCommandEvent
{
    uint64_t mTick = 0;
    uint32_t mFrame = 0;
    uint8_t mValue = 0;
    bool mLatch2 = false;
    bool mInjected = false;
    uint32_t mRepeat = 1;
    std::string mName;
    std::string mDetail;
};

constexpr size_t kMaxEvents = 8192;

std::deque<SoundCommandEvent> gEvents;
uint32_t gFrame = 0;

bool gPrevLatch1Wr = false;
bool gPrevLatch2Wr = false;
bool gPrevVblank = false;

bool gCoalesceRepeats = true;
bool gAutoScroll = true;
bool gCapture = true;

// Pending injections, applied one per tick so a burst can't overwrite a byte
// the Z80 has not read yet.
std::deque<std::pair<uint8_t, bool>> gPending;
int gInjectValue = 0x34;

// YM2151 bus trace (M72_YM_LOG=1).  The driver writes the register number
// with a0=0 and the value with a0=1; log the pair.
bool gPrevYmWrite = false;
uint8_t gYmAddr = 0;

void RecordCommand(uint8_t value, bool latch2, bool injected = false)
{
    SoundCommandEvent event;
    event.mTick = gSimCore.mTotalTicks;
    event.mFrame = gFrame;
    event.mValue = value;
    event.mLatch2 = latch2;
    event.mInjected = injected;

    if (!latch2 && RTypeDriverPresent())
    {
        // The interrupt handler masks the latch byte with 0x7F before storing
        // it, so bit 7 never reaches the command table.
        const uint8_t command = value & 0x7F;
        const SoundEntry entry = DecodeRTypeEntry(command);
        const char *name = RTypeCommandName(command);
        event.mName = name ? name : (entry.mValid ? "SFX: unidentified" : "(unused)");
        event.mDetail = DescribeRTypeEntry(command, entry);
        if (value & 0x80)
            event.mDetail += "  [bit7 masked off]";
    }
    else if (latch2)
    {
        event.mName = "latch 2 (Z80 NMI)";
    }
    else
    {
        event.mName = "(no decode for this game)";
    }

    // M72_SOUND_LOG=1 mirrors every command to stderr - before the repeat
    // coalescing below, so headless/server runs get the unabridged stream.
    static const bool kLogToStderr = getenv("M72_SOUND_LOG") != nullptr;
    if (kLogToStderr)
    {
        fprintf(stderr, "sndcmd frame=%u tick=%llu %s%s%02X  %s  %s\n", event.mFrame,
                static_cast<unsigned long long>(event.mTick), event.mInjected ? "inject " : "",
                event.mLatch2 ? "L2:" : "", event.mValue, event.mName.c_str(), event.mDetail.c_str());
    }

    if (gCoalesceRepeats && !gEvents.empty())
    {
        SoundCommandEvent &last = gEvents.back();
        if (last.mValue == event.mValue && last.mLatch2 == event.mLatch2 && last.mInjected == event.mInjected)
        {
            last.mRepeat++;
            last.mTick = event.mTick;
            last.mFrame = event.mFrame;
            return;
        }
    }

    gEvents.push_back(std::move(event));
    while (gEvents.size() > kMaxEvents)
        gEvents.pop_front();
}

// ---------------------------------------------------------------------------
// jt51 slot trace (M72_JT51_LOG=1)
// ---------------------------------------------------------------------------
//
// jt51 time-multiplexes 32 operator slots per sample: `cycles` is the slot
// counter, `cur_ch = cycles[2:0]`, `cur_op = cycles[4:3]` (M1,M2,C1,C2).
// `eg_XI` is the operator's TOTAL attenuation (EG + TL + AM, 0 = loudest,
// 0x3FF = silent) and `ph_X` the top 10 bits of its phase.  Both are pipeline
// stages, so a slot's value shows up a fixed number of cycles after the slot
// index it belongs to - hence the whole 32-entry vector is dumped rather than
// guessing the skew, and the active slots are identified from the data.
//
// Dumped every kJt51Decim samples so a second of audio stays readable.

constexpr uint32_t kJt51Decim = 256;

uint16_t gSlotEg[32];
uint16_t gSlotPh[32];
uint16_t gSlotRawEg[32];
uint8_t gSlotRate[32];
uint8_t gSlotCfg[32];
uint8_t gSlotState[32];
uint8_t gSlotKsh[32];
uint8_t gSlotCnt[32];
uint8_t gSlotCntOut[32];
uint32_t gSumUps[32];
uint32_t gSteps[32];
uint32_t gEgCnt;
// Bucketed by rate_V so no slot arithmetic is involved: sum_up/step_VI are one
// pipeline stage after rate_V, so they pair with the previous cen's rate.
uint32_t gCensByRate[64];
uint32_t gSumByRate[64];
uint32_t gStepByRate[64];
uint8_t gPrevRateV = 0;
uint8_t gPrevCycles = 0xFF;
uint32_t gSampleCount = 0;

void Jt51Trace()
{
    auto &ym = gSimCore.mTop->rootp->vlSymsp->TOP__sim_top__m72_inst__sound__ym2151;

    const uint8_t cycles = static_cast<uint8_t>(ym.cycles & 0x1F);
    if (cycles == gPrevCycles)
        return;
    gPrevCycles = cycles;

    auto &eg = gSimCore.mTop->rootp->vlSymsp->TOP__sim_top__m72_inst__sound__ym2151__u_eg;
    gSlotEg[cycles] = static_cast<uint16_t>(ym.eg_XI);
    gSlotPh[cycles] = static_cast<uint16_t>(ym.ph_X);
    gSlotRawEg[cycles] = static_cast<uint16_t>(eg.eg_VII);
    gSlotRate[cycles] = static_cast<uint8_t>(eg.rate_V);
    gSlotCfg[cycles] = static_cast<uint8_t>(eg.cfg_III);
    gSlotState[cycles] = static_cast<uint8_t>(eg.state_in_V);
    gSlotKsh[cycles] = static_cast<uint8_t>(eg.kshift_III);
    gSlotCnt[cycles] = static_cast<uint8_t>(eg.cnt_V);
    gSlotCntOut[cycles] = static_cast<uint8_t>(eg.cnt_out);
    gEgCnt = static_cast<uint32_t>(eg.eg_cnt);
    {
        const uint8_t rv = static_cast<uint8_t>(eg.rate_V & 0x3F);
        gCensByRate[gPrevRateV]++;
        if (eg.sum_up)
            gSumByRate[gPrevRateV]++;
        if (eg.sum_up && eg.step_VI)
            gStepByRate[gPrevRateV]++;
        gPrevRateV = rv;
    }

    // sum_up / step_VI are stage-VI regs: attribute them to slot cycles-5.
    const uint8_t evSlot = static_cast<uint8_t>((cycles + 32 - 5) & 0x1F);
    if (eg.sum_up)
        gSumUps[evSlot]++;
    if (eg.sum_up && eg.step_VI)
        gSteps[evSlot]++;

    if (cycles != 0)
        return;

    if ((gSampleCount++ % kJt51Decim) != 0)
        return;

    auto dump = [&](const char *tag, const char *fmt, const void *base, int stride) {
        char line[32 * 4 + 1] = {};
        for (int i = 0; i < 32; i++)
        {
            const uint32_t v = stride == 2 ? static_cast<const uint16_t *>(base)[i]
                                           : static_cast<const uint8_t *>(base)[i];
            snprintf(line + i * 4, 5, fmt, v);
        }
        fprintf(stderr, "jt51 tick=%llu %s %s\n",
                static_cast<unsigned long long>(gSimCore.mTotalTicks), tag, line);
    };

    fprintf(stderr, "jt51 tick=%llu pm=%02X am=%02X\n",
            static_cast<unsigned long long>(gSimCore.mTotalTicks), ym.pm, ym.am);
    dump("eg=", "%03X ", gSlotEg, 2);
    dump("ph=", "%03X ", gSlotPh, 2);
    dump("rw=", "%03X ", gSlotRawEg, 2);
    dump("rt=", "%3u ", gSlotRate, 1);
    dump("cf=", "%3u ", gSlotCfg, 1);
    dump("st=", "%3u ", gSlotState, 1);
    dump("ks=", "%3u ", gSlotKsh, 1);
    dump("cv=", "%3u ", gSlotCnt, 1);
    dump("co=", "%3u ", gSlotCntOut, 1);
    {
        char line[32 * 8 + 1] = {};
        for (int i = 0; i < 32; i++)
            snprintf(line + i * 8, 9, "%7u ", gSumUps[i]);
        fprintf(stderr, "jt51 tick=%llu su= %s\n", static_cast<unsigned long long>(gSimCore.mTotalTicks), line);
        for (int i = 0; i < 32; i++)
            snprintf(line + i * 8, 9, "%7u ", gSteps[i]);
        fprintf(stderr, "jt51 tick=%llu sp= %s\n", static_cast<unsigned long long>(gSimCore.mTotalTicks), line);
        fprintf(stderr, "jt51 tick=%llu egcnt=%u\n", static_cast<unsigned long long>(gSimCore.mTotalTicks), gEgCnt);
        for (int r = 0; r < 64; r++)
        {
            if (gCensByRate[r] == 0)
                continue;
            fprintf(stderr, "jt51 tick=%llu byrate %2d cens=%u sumup=%u step=%u\n",
                    static_cast<unsigned long long>(gSimCore.mTotalTicks), r, gCensByRate[r],
                    gSumByRate[r], gStepByRate[r]);
        }
    }
}

// ---------------------------------------------------------------------------
// Window
// ---------------------------------------------------------------------------

class SoundCommandWindow : public Window
{
  public:
    SoundCommandWindow() : Window("Sound Commands")
    {
    }

    void Init() override
    {
    }

    void Draw() override
    {
        if (!gSimCore.mTop)
            return;

        DrawDriverState();
        ImGui::Separator();
        DrawControls();
        ImGui::Separator();
        DrawLog();
    }

  private:
    void DrawDriverState()
    {
        if (!RTypeDriverPresent())
        {
            ImGui::TextUnformatted("Sound driver: unrecognised (raw byte logging only)");
            return;
        }

        const uint8_t pending = SoundRamByte(kRTypePendingCmd);
        if (pending == 0xFF)
            ImGui::TextUnformatted("Sound driver: R-Type (M72)    pending: idle");
        else
            ImGui::Text("Sound driver: R-Type (M72)    pending: %02X", pending);

        std::string active;
        char text[16];
        for (uint32_t command = 0; command < kMaxCommands; command++)
        {
            if (SoundRamByte(static_cast<uint16_t>(kRTypeActiveTable + command)) == 0xFF)
                continue;
            snprintf(text, sizeof(text), "%s%02X", active.empty() ? "" : " ", static_cast<unsigned>(command));
            active += text;
        }
        ImGui::Text("Playing: %s", active.empty() ? "-" : active.c_str());
    }

    void DrawControls()
    {
        ImGui::Checkbox("Capture", &gCapture);
        ImGui::SameLine();
        ImGui::Checkbox("Coalesce repeats", &gCoalesceRepeats);
        ImGui::SameLine();
        ImGui::Checkbox("Auto-scroll", &gAutoScroll);
        ImGui::SameLine();
        if (ImGui::Button("Clear"))
            gEvents.clear();
        ImGui::SameLine();
        ImGui::Text("%zu logged", gEvents.size());

        // Inject a command straight into the latch, so a sound can be
        // auditioned without playing the game to the point that triggers it.
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputInt("##inject", &gInjectValue, 1, 16, ImGuiInputTextFlags_CharsHexadecimal);
        gInjectValue = std::min(std::max(gInjectValue, 0), 0xFF);
        ImGui::SameLine();
        if (ImGui::Button("Send"))
            SoundCommandsInject(static_cast<uint8_t>(gInjectValue), false);
        ImGui::SameLine();
        if (ImGui::Button("Stop all (00)"))
            SoundCommandsInject(0x00, false);
        ImGui::SameLine();
        ImGui::TextUnformatted("(injected rows are marked *)");
    }

    void DrawLog()
    {
        const ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                      ImGuiTableFlags_SizingFixedFit;
        if (!ImGui::BeginTable("sound_commands", 5, flags))
            return;

        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Frame");
        ImGui::TableSetupColumn("Cmd");
        ImGui::TableSetupColumn("x");
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Decode", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(gEvents.size()));
        while (clipper.Step())
        {
            for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; row++)
            {
                const SoundCommandEvent &event = gEvents[static_cast<size_t>(row)];
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%u", event.mFrame);
                ImGui::TableNextColumn();
                ImGui::Text("%s%s%02X", event.mInjected ? "*" : "", event.mLatch2 ? "L2:" : "", event.mValue);
                ImGui::TableNextColumn();
                if (event.mRepeat > 1)
                    ImGui::Text("%u", event.mRepeat);
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(event.mName.c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(event.mDetail.c_str());
            }
        }

        if (gAutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::EndTable();
    }
};

SoundCommandWindow gSoundCommandWindow;

} // namespace

void SoundCommandsTick()
{
    if (!gSimCore.mTop)
        return;

    const bool vblank = gSimCore.mTop->vblank != 0;
    if (vblank && !gPrevVblank)
        gFrame++;
    gPrevVblank = vblank;

    // The m72 instance keeps its own class, so reach the latch taps (made
    // public in verilator.vlt) through the symbol table rather than the
    // flattened-root M72_SIGNAL macros.
    const auto &m72 = gSimCore.mTop->rootp->vlSymsp->TOP__sim_top__m72_inst;

    // IOWR is a level for the whole write cycle, so latch on the rising edge
    // and sample the CPU write data bus with it.
    const bool latch1Wr = m72.snd_latch1_wr != 0;
    const bool latch2Wr = m72.snd_latch2_wr != 0;

    if (gCapture)
    {
        if (latch1Wr && !gPrevLatch1Wr)
            RecordCommand(static_cast<uint8_t>(m72.cpu_dout & 0xFF), false);
        if (latch2Wr && !gPrevLatch2Wr)
            RecordCommand(static_cast<uint8_t>(m72.cpu_dout & 0xFF), true);
    }

    gPrevLatch1Wr = latch1Wr;
    gPrevLatch2Wr = latch2Wr;

    static const bool kJt51Log = getenv("M72_JT51_LOG") != nullptr;
    if (kJt51Log)
        Jt51Trace();

    auto &snd = gSimCore.mTop->rootp->vlSymsp->TOP__sim_top__m72_inst__sound;

    // YM2151 writes: SCS is a level for the whole IO cycle, so take the edge.
    static const bool kYmLog = getenv("M72_YM_LOG") != nullptr;
    if (kYmLog)
    {
        const bool ymWrite = snd.SCS && !snd.SWR_N;
        if (ymWrite && !gPrevYmWrite)
        {
            if (snd.SA0)
                fprintf(stderr, "ym frame=%u tick=%llu reg %02X <- %02X\n", gFrame,
                        static_cast<unsigned long long>(gSimCore.mTotalTicks), gYmAddr, snd.SD_IN);
            else
                gYmAddr = snd.SD_IN;
        }
        gPrevYmWrite = ymWrite;
    }

    // Apply one queued injection per tick.  Wait until the Z80 has consumed
    // the previous byte so an injected command cannot swallow a real one.
    if (!gPending.empty() && !snd.snd_latch1_ready && !snd.snd_latch2_ready)
    {
        const auto pending = gPending.front();
        gPending.pop_front();
        if (pending.second)
        {
            snd.snd_latch2 = pending.first;
            snd.snd_latch2_ready = 1;
        }
        else
        {
            snd.snd_latch1 = pending.first;
            snd.snd_latch1_ready = 1;
        }
        RecordCommand(pending.first, pending.second, true);
    }
}

void SoundCommandsInject(uint8_t value, bool latch2)
{
    gPending.push_back({value, latch2});
}

void SoundCommandsReset()
{
    gPending.clear();
    gEvents.clear();
    gFrame = 0;
    gPrevLatch1Wr = false;
    gPrevLatch2Wr = false;
    gPrevVblank = false;
}
