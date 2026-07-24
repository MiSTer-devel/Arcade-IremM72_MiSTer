#include "imgui_wrap.h"
#include "sim_ui.h"
#include "imgui_wrap.h"
#include "third_party/imgui_memory_editor.h"
#include "sim_controller.h"
#include "sim_core.h"
#include "sim_hierarchy.h"
#include "M72.h"
#include "M72___024root.h"
#include "verilated_fst_c.h"
#include "sim_sdram.h"

#include <algorithm>
#include <cstring>
#include <sstream>

namespace
{
struct WatchedSignal
{
    std::string mName;
    bool mWatchpoint = false;
    bool mInitialized = false;
    uint64_t mLastValue = 0;
    uint32_t mWidth = 0;
    std::string mValueHex;
    std::string mError;
};

std::vector<WatchedSignal> gWatchedSignals;
std::string gSignalWatchStatus;
std::vector<SignalInfo> gAvailableSignals;
bool gRefreshAvailableSignals = true;

void RefreshAvailableSignals()
{
    if (!gRefreshAvailableSignals)
        return;

    auto result = gSimController.ListSignals();
    gAvailableSignals = result.ok ? result.value.mSignals : std::vector<SignalInfo>{};
    gRefreshAvailableSignals = false;
}

void AddWatchedSignal(const std::string &name)
{
    if (name.empty())
        return;

    auto existing = std::find_if(gWatchedSignals.begin(), gWatchedSignals.end(), [&](const auto &signal) {
        return signal.mName == name;
    });
    if (existing == gWatchedSignals.end())
    {
        gWatchedSignals.push_back({name});
    }
}

void RemoveWatchedSignal(const std::string &name)
{
    auto it = std::remove_if(gWatchedSignals.begin(), gWatchedSignals.end(), [&](const auto &signal) { return signal.mName == name; });
    gWatchedSignals.erase(it, gWatchedSignals.end());
}

static void *SignalSettingsReadOpen(ImGuiContext *, ImGuiSettingsHandler *, const char *name)
{
    return strcmp(name, "Signals") == 0 ? reinterpret_cast<void *>(1) : nullptr;
}

static void SignalSettingsReadLine(ImGuiContext *, ImGuiSettingsHandler *, void *, const char *line)
{
    char signalName[256] = {};
    int watch = 0;
    if (sscanf(line, "Watch=%255[^,],%d", signalName, &watch) == 2)
    {
        AddWatchedSignal(signalName);
        auto it = std::find_if(gWatchedSignals.begin(), gWatchedSignals.end(), [&](const auto &signal) { return signal.mName == signalName; });
        if (it != gWatchedSignals.end())
        {
            it->mWatchpoint = watch != 0;
            it->mInitialized = false;
        }
        return;
    }

    if (sscanf(line, "Signal=%255[^\n]", signalName) == 1)
    {
        AddWatchedSignal(signalName);
    }
}

static void SignalSettingsWriteAll(ImGuiContext *, ImGuiSettingsHandler *handler, ImGuiTextBuffer *buf)
{
    buf->appendf("[%s][Signals]\n", handler->TypeName);
    for (const auto &signal : gWatchedSignals)
    {
        buf->appendf("Watch=%s,%d\n", signal.mName.c_str(), signal.mWatchpoint ? 1 : 0);
    }
    buf->append("\n");
}

void RegisterSignalSettingsHandler()
{
    ImGuiSettingsHandler iniHandler;
    iniHandler.TypeName = "SimSignals";
    iniHandler.TypeHash = ImHashStr("SimSignals");
    iniHandler.ClearAllFn = nullptr;
    iniHandler.ReadOpenFn = SignalSettingsReadOpen;
    iniHandler.ReadLineFn = SignalSettingsReadLine;
    iniHandler.ApplyAllFn = nullptr;
    iniHandler.WriteAllFn = SignalSettingsWriteAll;
    ImGui::AddSettingsHandler(&iniHandler);
}

bool CheckSignalWatchpoints()
{
    for (auto &signal : gWatchedSignals)
    {
        if (!signal.mWatchpoint)
            continue;

        auto result = gSimController.ReadSignal(signal.mName);
        if (!result.ok)
        {
            signal.mError = result.errorMessage;
            continue;
        }

        signal.mError.clear();
        signal.mWidth = result.value.mWidth;
        signal.mValueHex = result.value.mValueHex;

        if (!signal.mInitialized)
        {
            signal.mLastValue = result.value.mValue;
            signal.mValueHex = result.value.mValueHex;
            signal.mInitialized = true;
            continue;
        }

        if (signal.mLastValue != result.value.mValue)
        {
            std::string previousHex = signal.mValueHex;
            gSignalWatchStatus = signal.mName + " changed from 0x" + previousHex + " to 0x" + result.value.mValueHex;
            signal.mLastValue = result.value.mValue;
            signal.mValueHex = result.value.mValueHex;
            return true;
        }
    }

    return false;
}
} // namespace

class MemoryInterfaceEditor : public MemoryEditor
{
  public:
    MemoryInterfaceEditor(MemoryInterface &mem) : MemoryEditor(), mMemory(mem)
    {
        ReadFn = ReadMem;
        WriteFn = WriteMem;
        UserData = this;
    }

    void DrawContents()
    {
        MemoryEditor::DrawContents(nullptr, mMemory.GetSize());
    }

    static ImU8 ReadMem(const ImU8 *, size_t off, void *user)
    {
        MemoryInterfaceEditor *selfPtr = (MemoryInterfaceEditor *)user;
        ImU8 data;
        selfPtr->mMemory.Read(off, 1, &data);
        return data;
    }

    static void WriteMem(ImU8 *, size_t off, ImU8 data, void *user)
    {
        MemoryInterfaceEditor *selfPtr = (MemoryInterfaceEditor *)user;
        selfPtr->mMemory.Write(off, 1, &data);
    }

    MemoryInterface &mMemory;
};

void UiInit(const char *title)
{
    ImguiInit(title);
    RegisterSignalSettingsHandler();
}

void UiInitWindows()
{
    ImguiInitWindows();
    gSimCore.SetSignalWatchpointCallback(CheckSignalWatchpoints);
    gRefreshAvailableSignals = true;
}

bool UiBeginFrame()
{
    return ImguiBeginFrame();
}

void UiEndFrame()
{
    ImguiEndFrame();
}

static bool gRefreshStateFiles = true;
static void RefreshMemoryWindows();

void UiGameChanged()
{
    char title[64];
    const char *name = gSimCore.GetGameName();

    snprintf(title, sizeof(title), "M72 - %s", name);
    ImguiSetTitle(title);
    RefreshMemoryWindows();
    gRefreshStateFiles = true;
    gRefreshAvailableSignals = true;
}

void UiDraw()
{
    if (ImGui::Begin("Simulation Control"))
    {
        ImGui::LabelText("Ticks", "%llu", gSimCore.mTotalTicks);
        ImGui::Checkbox("Run", &gSimCore.mSimulationRun);
        if (ImGui::Button("Step"))
        {
            gSimCore.mSimulationStep = true;
            gSimCore.mSimulationRun = false;
        }
        ImGui::InputInt("Step Size", &gSimCore.mSimulationStepSize);
        ImGui::Checkbox("Step Frame", &gSimCore.mSimulationStepVblank);

        ImGui::Checkbox("WP Set", &gSimCore.mSimulationWpSet);
        ImGui::SameLine();
        ImGui::InputInt("##wpaddr", &gSimCore.mSimulationWpAddr, 0, 0, ImGuiInputTextFlags_CharsHexadecimal);

        if (ImGui::Button("Reset"))
        {
            gSimController.Reset(100);
        }

        ImGui::SameLine();
        ImGui::Checkbox("Pause", &gSimCore.mSystemPause);

        if (!gSignalWatchStatus.empty())
        {
            ImGui::TextWrapped("Signal watchpoint: %s", gSignalWatchStatus.c_str());
        }

        ImGui::Separator();

        // Save/Restore State Section
        ImGui::Text("Save/Restore State");

        static char sStateFilename[256] = "state.m72state";
        ImGui::InputText("State Filename", sStateFilename, sizeof(sStateFilename));

        static std::vector<std::string> sStateFiles;
        static int sSelectedStateFile = -1;

        // Auto-generate filename when file list is loaded/updated
        if (gRefreshStateFiles)
        {
            auto listResult = gSimController.ListStates();
            sStateFiles = listResult.ok ? listResult.value.mStates : std::vector<std::string>{};
            std::string autoName = sStateFiles.size() < 1000 ? "000.m72state" : "999.m72state";
            if (!sStateFiles.empty())
            {
                for (int i = 0; i < 1000; i++)
                {
                    char candidate[32];
                    snprintf(candidate, sizeof(candidate), "%03d.m72state", i);
                    if (std::find(sStateFiles.begin(), sStateFiles.end(), candidate) == sStateFiles.end())
                    {
                        autoName = candidate;
                        break;
                    }
                }
            }
            strncpy(sStateFilename, autoName.c_str(), sizeof(sStateFilename) - 1);
            sStateFilename[sizeof(sStateFilename) - 1] = '\0';
            gRefreshStateFiles = false;
        }

        if (ImGui::Button("Save State"))
        {
            // Ensure filename has .m72state extension
            std::string filename = sStateFilename;
            if (filename.size() < 9 || filename.substr(filename.size() - 9) != ".m72state")
            {
                filename += ".m72state";
                strncpy(sStateFilename, filename.c_str(), sizeof(sStateFilename) - 1);
                sStateFilename[sizeof(sStateFilename) - 1] = '\0';
            }

            if (gSimController.SaveState(sStateFilename).ok)
            {
                // Update file list after successfully saving
                auto listResult = gSimController.ListStates();
                sStateFiles = listResult.ok ? listResult.value.mStates : std::vector<std::string>{};
                // Try to select the newly saved file
                for (size_t i = 0; i < sStateFiles.size(); i++)
                {
                    if (sStateFiles[i] == sStateFilename)
                    {
                        sSelectedStateFile = i;
                        break;
                    }
                }
                // Auto-generate next filename after successful save
                std::string autoName = sStateFilename;
                for (int i = 0; i < 1000; i++)
                {
                    char candidate[32];
                    snprintf(candidate, sizeof(candidate), "%03d.m72state", i);
                    if (std::find(sStateFiles.begin(), sStateFiles.end(), candidate) == sStateFiles.end())
                    {
                        autoName = candidate;
                        break;
                    }
                }
                strncpy(sStateFilename, autoName.c_str(), sizeof(sStateFilename) - 1);
                sStateFilename[sizeof(sStateFilename) - 1] = '\0';
            }
        }

        // Show list of state files
        if (sStateFiles.size() > 0)
        {
            ImGui::Text("Available State Files:");
            ImGui::BeginChild("StateFiles", ImVec2(0, 100), true);
            for (size_t i = 0; i < sStateFiles.size(); i++)
            {
                if (ImGui::Selectable(sStateFiles[i].c_str(), sSelectedStateFile == (int)i, ImGuiSelectableFlags_AllowDoubleClick))
                {
                    sSelectedStateFile = (int)i;
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    {
                        gSimController.LoadState(sStateFiles[i].c_str());
                    }
                }
            }
            ImGui::EndChild();
        }
        else
        {
            ImGui::Text("No state files found (*.m72state)");
        }

        ImGui::Separator();

        ImGui::PushItemWidth(100);
        if (ImGui::InputInt("Trace Depth", &gSimCore.mTraceDepth, 1, 10,
                            gSimCore.mTraceActive ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None))
        {
            gSimCore.mTraceDepth = std::min(std::max(gSimCore.mTraceDepth, 1), 99);
        }
        ImGui::PopItemWidth();
        ImGui::InputText("Filename", gSimCore.mTraceFilename, sizeof(gSimCore.mTraceFilename),
                         gSimCore.IsTraceActive() ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None);
        if (ImGui::Button(gSimCore.IsTraceActive() ? "Stop Tracing###TraceBtn" : "Start Tracing###TraceBtn"))
        {
            if (gSimCore.IsTraceActive())
            {
                gSimCore.StopTrace();
            }
            else
            {
                if (strlen(gSimCore.mTraceFilename) > 0)
                {
                    gSimCore.StartTrace(gSimCore.mTraceFilename, gSimCore.mTraceDepth);
                }
            }
        }
    }

    ImGui::End();
}

class DipswitchWindow : public Window
{
  public:
    DipswitchWindow() : Window("Dipswitches")
    {
    }

    void Init() {};
    void Draw()
    {
        uint32_t dipSwitches = gSimController.GetDipSwitches();
        bool changed = false;

        if (ImGui::BeginTable("dipswitches", 9))
        {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
            for (int i = 0; i < 8; i++)
            {
                char n[2];
                n[0] = '1' + i;
                n[1] = 0;
                ImGui::TableSetupColumn(n, ImGuiTableColumnFlags_WidthFixed);
            }
            ImGui::TableHeadersRow();
            for (int bank = 0; bank < 2; bank++)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("DIP %d", bank + 1);
                for (int i = 0; i < 8; i++)
                {
                    ImGui::TableNextColumn();
                    ImGui::PushID(bank * 8 + i);
                    changed |= ImGui::CheckboxFlags("##dip", &dipSwitches, ((uint32_t)1 << (bank * 8 + i)));
                    ImGui::PopID();
                }
            }
            ImGui::EndTable();
        }

        if (changed)
            gSimController.SetDipSwitches(static_cast<uint16_t>(dipSwitches));
    }
};

DipswitchWindow gDipswitchWindow;

class SignalsWindow : public Window
{
  public:
    SignalsWindow() : Window("Signals")
    {
    }

    void Init() override
    {
        gSignalWatchStatus.clear();
        for (auto &signal : gWatchedSignals)
        {
            signal.mInitialized = false;
            signal.mError.clear();
        }
    }

    void Draw() override
    {
        static char sNewSignalName[256] = "";
        static int sSelectedSignalIndex = -1;

        RefreshAvailableSignals();

        ImGui::InputText("Signal", sNewSignalName, sizeof(sNewSignalName));
        ImGui::SameLine();
        if (ImGui::Button("Add") && sNewSignalName[0] != '\0')
        {
            AddWatchedSignal(sNewSignalName);
            sNewSignalName[0] = '\0';
        }

        if (ImGui::BeginCombo("Available", sSelectedSignalIndex >= 0 && sSelectedSignalIndex < (int)gAvailableSignals.size()
                                               ? gAvailableSignals[sSelectedSignalIndex].mName.c_str()
                                               : "Select signal"))
        {
            for (int i = 0; i < (int)gAvailableSignals.size(); i++)
            {
                bool selected = sSelectedSignalIndex == i;
                if (ImGui::Selectable(gAvailableSignals[i].mName.c_str(), selected))
                {
                    sSelectedSignalIndex = i;
                    strncpy(sNewSignalName, gAvailableSignals[i].mName.c_str(), sizeof(sNewSignalName) - 1);
                    sNewSignalName[sizeof(sNewSignalName) - 1] = '\0';
                }
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        ImGui::SameLine();
        if (ImGui::Button("Refresh"))
        {
            gRefreshAvailableSignals = true;
            RefreshAvailableSignals();
        }

        if (!gSignalWatchStatus.empty())
        {
            ImGui::TextWrapped("Last watchpoint hit: %s", gSignalWatchStatus.c_str());
            if (ImGui::Button("Clear Status"))
            {
                gSignalWatchStatus.clear();
            }
        }

        ImGui::Separator();

        if (ImGui::BeginTable("signals_table", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("Watch", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("Signal");
            ImGui::TableSetupColumn("Value");
            ImGui::TableSetupColumn("Width", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableHeadersRow();

            for (size_t i = 0; i < gWatchedSignals.size();)
            {
                auto &signal = gWatchedSignals[i];
                auto result = gSimController.ReadSignal(signal.mName);
                if (result.ok)
                {
                    signal.mError.clear();
                    signal.mWidth = result.value.mWidth;
                    signal.mValueHex = result.value.mValueHex;
                    if (!signal.mWatchpoint)
                    {
                        signal.mLastValue = result.value.mValue;
                        signal.mInitialized = false;
                    }
                }
                else
                {
                    signal.mError = result.errorMessage;
                }

                ImGui::TableNextRow();

                ImGui::TableNextColumn();
                ImGui::PushID(static_cast<int>(i));
                bool watchpoint = signal.mWatchpoint;
                if (ImGui::Checkbox("##watch", &watchpoint))
                {
                    signal.mWatchpoint = watchpoint;
                    signal.mInitialized = false;
                }

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(signal.mName.c_str());

                ImGui::TableNextColumn();
                if (signal.mError.empty())
                    ImGui::Text("0x%s", signal.mValueHex.c_str());
                else
                    ImGui::TextUnformatted(signal.mError.c_str());

                ImGui::TableNextColumn();
                if (signal.mError.empty())
                    ImGui::Text("%u", signal.mWidth);

                ImGui::TableNextColumn();
                bool remove = ImGui::Button("X");
                ImGui::PopID();

                if (remove)
                {
                    RemoveWatchedSignal(signal.mName);
                    continue;
                }

                i++;
            }

            ImGui::EndTable();
        }
    }
};

SignalsWindow gSignalsWindow;

class ROMWindow : public Window
{
  public:
    struct Tab
    {
        const char *mName;
        std::unique_ptr<MemoryInterfaceEditor> mEditor;

        Tab(const char *name, MemoryInterface &memory) : mName(name), mEditor(std::make_unique<MemoryInterfaceEditor>(memory))
        {
        }
    };

    std::vector<Tab> mTabs;

    ROMWindow() : Window("ROM View")
    {
    }

    void Init()
    {
        if (!gSimCore.mTop)
        {
            mTabs.clear();
            return;
        }

        mTabs.clear();
        mTabs.emplace_back("CPU", gSimCore.Memory(MemoryRegion::CPU_ROM));
        mTabs.emplace_back("Sprite", gSimCore.Memory(MemoryRegion::SPRITE_ROM));
        mTabs.emplace_back("BG A", gSimCore.Memory(MemoryRegion::BG_A_ROM));
        mTabs.emplace_back("BG B", gSimCore.Memory(MemoryRegion::BG_B_ROM));
        mTabs.emplace_back("Sound", gSimCore.Memory(MemoryRegion::SOUND_ROM));
        mTabs.emplace_back("Samples", gSimCore.Memory(MemoryRegion::SAMPLE_ROM));
        mTabs.emplace_back("MCU", gSimCore.Memory(MemoryRegion::MCU_ROM));
    }

    void Draw()
    {
        if (ImGui::BeginTabBar("rom_tabs"))
        {
            for (auto &it : mTabs)
            {
                if (ImGui::BeginTabItem(it.mName))
                {
                    it.mEditor->DrawContents();
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
    }
};

ROMWindow gRomWindow;

class RAMWindow : public Window
{
  public:
    struct Tab
    {
        const char *mName;
        std::unique_ptr<MemoryInterfaceEditor> mEditor;

        Tab(const char *name, MemoryInterface &memory) : mName(name), mEditor(std::make_unique<MemoryInterfaceEditor>(memory))
        {
        }
    };

    std::vector<Tab> mTabs;

    RAMWindow() : Window("RAM View")
    {
    }

    void Init()
    {
        if (!gSimCore.mTop)
        {
            mTabs.clear();
            return;
        }

        mTabs.clear();
        mTabs.emplace_back("Work", gSimCore.Memory(MemoryRegion::WORK_RAM));
        mTabs.emplace_back("VRAM A", gSimCore.Memory(MemoryRegion::VRAM_A));
        mTabs.emplace_back("VRAM B", gSimCore.Memory(MemoryRegion::VRAM_B));
        mTabs.emplace_back("Sprite", gSimCore.Memory(MemoryRegion::SPRITE_RAM));
        mTabs.emplace_back("MCU", gSimCore.Memory(MemoryRegion::MCU_RAM));
        mTabs.emplace_back("MCU Shared", gSimCore.Memory(MemoryRegion::MCU_SHARED_RAM));
    }

    void Draw()
    {
        if (ImGui::BeginTabBar("memory_tabs"))
        {
            for (auto &it : mTabs)
            {
                if (ImGui::BeginTabItem(it.mName))
                {
                    it.mEditor->DrawContents();
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
    }
};

RAMWindow gRamWindow;

static void RefreshMemoryWindows()
{
    gRomWindow.Init();
    gRamWindow.Init();
}
