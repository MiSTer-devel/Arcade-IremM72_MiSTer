#include "testrom_gui.h"

#include <algorithm>
#include <array>
#include <cstring>

#include "sim_core.h"

namespace
{
enum GuiType : uint8_t
{
    GUI_TYPE_NULL = 0,
    GUI_TYPE_U8 = 1,
    GUI_TYPE_U16 = 2,
    GUI_TYPE_BITS16 = 3,
    GUI_TYPE_BOOL = 4,
    GUI_TYPE_BUTTON = 5,
};

// V30 is little-endian (the IGSPGM original read 68k big-endian words)
uint16_t ReadLe16(const uint8_t *ptr)
{
    return static_cast<uint16_t>(ptr[0]) | (static_cast<uint16_t>(ptr[1]) << 8);
}

std::string ReadLabel(const uint8_t *ptr, size_t size)
{
    size_t length = 0;
    while (length < size && ptr[length] != 0)
        length++;
    return std::string(reinterpret_cast<const char *>(ptr), length);
}

TestRomGuiWindow gTestRomGuiWindow;
} // namespace

TestRomGuiWindow::TestRomGuiWindow() : Window("TestROM GUI")
{
    Reset();
}

void TestRomGuiWindow::Init()
{
}

void TestRomGuiWindow::Reset()
{
    mState = {};
    mState.mAddress = kWorkRamAddress;
    for (auto &pending : mPending)
        pending = {};
}

const char *TestRomGuiWindow::TypeName(uint8_t type)
{
    switch (type)
    {
    case GUI_TYPE_U8:
        return "u8";
    case GUI_TYPE_U16:
        return "u16";
    case GUI_TYPE_BITS16:
        return "bits16";
    case GUI_TYPE_BOOL:
        return "bool";
    case GUI_TYPE_BUTTON:
        return "button";
    default:
        return "unknown";
    }
}

TestRomGuiState TestRomGuiWindow::GetState() const
{
    return mState;
}

bool TestRomGuiWindow::ReadSafeState(TestRomGuiState &state) const
{
    state = {};
    state.mAddress = kWorkRamAddress;

    std::array<uint8_t, kStateSize> raw{};
    gSimCore.Memory(MemoryRegion::WORK_RAM).Read(kWorkRamAddress, raw.size(), raw.data());

    const uint16_t startMagic = ReadLe16(&raw[0]);
    const uint16_t lock = ReadLe16(&raw[2]);
    const uint16_t count = ReadLe16(&raw[4]);
    const uint16_t endMagic = ReadLe16(&raw[kHeaderSize + (kMaxEntries * kEntrySize)]);

    if (startMagic != kMagic || endMagic != kMagic || lock != 0 || count == 0 || count > kMaxEntries)
        return false;

    state.mAvailable = true;
    state.mEntries.reserve(count);

    for (uint32_t index = 0; index < count; ++index)
    {
        const uint32_t offset = kHeaderSize + (index * kEntrySize);
        TestRomGuiEntry entry;
        entry.mIndex = index;
        entry.mLabel = ReadLabel(&raw[offset], 15);
        entry.mType = raw[offset + 15];
        entry.mValue = ReadLe16(&raw[offset + 16]);
        entry.mOverrideValue = ReadLe16(&raw[offset + 18]);
        state.mEntries.push_back(std::move(entry));
    }

    return true;
}

void TestRomGuiWindow::WriteOverrideValue(uint32_t index, uint16_t value)
{
    const uint32_t offset = kWorkRamAddress + kHeaderSize + (index * kEntrySize) + 18;
    // V30 little-endian
    const uint8_t bytes[2] = {static_cast<uint8_t>(value & 0xff), static_cast<uint8_t>(value >> 8)};
    gSimCore.Memory(MemoryRegion::WORK_RAM).Write(offset, sizeof(bytes), bytes);
}

bool TestRomGuiWindow::RefreshAndApplyPending()
{
    TestRomGuiState state;
    if (!ReadSafeState(state))
    {
        // Keep the last safe snapshot visible in the simulator UI until we can
        // refresh it again on a later vblank.
        return false;
    }

    for (uint32_t index = 0; index < state.mEntries.size(); ++index)
    {
        PendingOverride &pending = mPending[index];
        if (!pending.mActive)
            continue;

        const TestRomGuiEntry &entry = state.mEntries[index];
        if (entry.mLabel != pending.mLabel || entry.mType != pending.mType)
            continue;

        WriteOverrideValue(index, pending.mValue);
        state.mEntries[index].mOverrideValue = pending.mValue;

        if (pending.mPulse)
        {
            pending.mPulse = false;
            pending.mValue = 0;
        }
        else
        {
            pending = {};
        }
    }

    state.mLastSyncTicks = gSimCore.mTotalTicks;
    mState = std::move(state);
    return true;
}

void TestRomGuiWindow::TickVblank()
{
    RefreshAndApplyPending();
}

bool TestRomGuiWindow::QueueOverride(uint32_t index, uint16_t value, bool pulse, std::string *error)
{
    if (!mState.mAvailable)
    {
        if (error)
            *error = "TestROM GUI is not currently available";
        return false;
    }
    if (index >= mState.mEntries.size())
    {
        if (error)
            *error = "GUI entry index out of range";
        return false;
    }

    if (pulse && mState.mEntries[index].mType != GUI_TYPE_BUTTON)
    {
        if (error)
            *error = "GUI pulse writes are only supported for button entries";
        return false;
    }

    PendingOverride &pending = mPending[index];
    pending.mActive = true;
    pending.mLabel = mState.mEntries[index].mLabel;
    pending.mType = mState.mEntries[index].mType;
    pending.mValue = value;
    pending.mPulse = pulse;

    mState.mEntries[index].mOverrideValue = value;
    return true;
}

bool TestRomGuiWindow::SetOverrideByIndex(uint32_t index, uint16_t value, bool pulse, bool *applied, std::string *error)
{
    if (applied)
        *applied = false;

    if (!QueueOverride(index, value, pulse, error))
        return false;

    const bool didApply = RefreshAndApplyPending();
    if (applied)
        *applied = didApply;
    return true;
}

bool TestRomGuiWindow::SetOverrideByLabel(const std::string &label, uint16_t value, bool pulse, bool *applied, std::string *error)
{
    if (!mState.mAvailable)
    {
        if (error)
            *error = "TestROM GUI is not currently available";
        return false;
    }

    auto it = std::find_if(mState.mEntries.begin(), mState.mEntries.end(), [&](const auto &entry) { return entry.mLabel == label; });
    if (it == mState.mEntries.end())
    {
        if (error)
            *error = "GUI entry not found: " + label;
        return false;
    }

    return SetOverrideByIndex(it->mIndex, value, pulse, applied, error);
}

void TestRomGuiWindow::Draw()
{
    if (!mState.mAvailable)
    {
        ImGui::TextUnformatted("No safe TestROM GUI data available.");
        ImGui::TextUnformatted("The simulator checks WORK_RAM[0x0A00] on each vblank.");
        return;
    }

    ImGui::Text("Entries: %zu", mState.mEntries.size());
    ImGui::SameLine();
    ImGui::Text("Last sync tick: %llu", static_cast<unsigned long long>(mState.mLastSyncTicks));
    ImGui::Separator();

    if (ImGui::BeginTable("testrom_gui", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
    {
        ImGui::TableSetupColumn("Label");
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Override");
        ImGui::TableHeadersRow();

        const std::vector<TestRomGuiEntry> entries = mState.mEntries;
        for (const TestRomGuiEntry &entry : entries)
        {
            ImGui::PushID(static_cast<int>(entry.mIndex));
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.mLabel.c_str());

            ImGui::TableNextColumn();
            ImGui::TextUnformatted(TypeName(entry.mType));

            ImGui::TableNextColumn();
            switch (entry.mType)
            {
            case GUI_TYPE_BOOL:
                ImGui::TextUnformatted(entry.mValue ? "ON" : "OFF");
                break;
            case GUI_TYPE_BUTTON:
                ImGui::TextUnformatted(entry.mValue ? "PRESSED" : "-");
                break;
            default:
                ImGui::Text("%04X", entry.mValue);
                break;
            }

            ImGui::TableNextColumn();
            switch (entry.mType)
            {
            case GUI_TYPE_U8:
            {
                uint8_t value = static_cast<uint8_t>(entry.mOverrideValue & 0xff);
                const uint8_t minValue = 0;
                const uint8_t maxValue = 0xff;
                if (ImGui::DragScalar("##u8", ImGuiDataType_U8, &value, 0.5f, &minValue, &maxValue, "%02X"))
                {
                    bool applied = false;
                    SetOverrideByIndex(entry.mIndex, value, false, &applied);
                }
                break;
            }
            case GUI_TYPE_U16:
            {
                uint16_t value = entry.mOverrideValue;
                const uint16_t minValue = 0;
                const uint16_t maxValue = 0xffff;
                if (ImGui::DragScalar("##u16", ImGuiDataType_U16, &value, 0.5f, &minValue, &maxValue, "%04X"))
                {
                    bool applied = false;
                    SetOverrideByIndex(entry.mIndex, value, false, &applied);
                }
                break;
            }
            case GUI_TYPE_BITS16:
            {
                uint16_t value = entry.mOverrideValue;
                if (ImGui::BeginTable("##bits", 16, ImGuiTableFlags_SizingFixedFit))
                {
                    for (int bit = 15; bit >= 0; --bit)
                    {
                        ImGui::TableNextColumn();
                        bool enabled = (value & (1u << bit)) != 0;
                        char id[16];
                        snprintf(id, 16, "##bits%d", bit);
                        if (ImGui::Checkbox(id, &enabled))
                        {
                            if (enabled)
                                value |= (1u << bit);
                            else
                                value &= ~(1u << bit);
                            bool applied = false;
                            SetOverrideByIndex(entry.mIndex, value, false, &applied);
                        }
                    }
                    ImGui::EndTable();
                }
                break;
            }
            case GUI_TYPE_BOOL:
            {
                bool value = entry.mOverrideValue != 0;
                if (ImGui::Checkbox("##bool", &value))
                {
                    bool applied = false;
                    SetOverrideByIndex(entry.mIndex, value ? 1 : 0, false, &applied);
                }
                break;
            }
            case GUI_TYPE_BUTTON:
                if (ImGui::Button("Press"))
                {
                    bool applied = false;
                    SetOverrideByIndex(entry.mIndex, 1, true, &applied);
                }
                break;
            default:
                ImGui::Text("%04X", entry.mOverrideValue);
                break;
            }

            ImGui::PopID();
        }

        ImGui::EndTable();
    }
}

TestRomGuiWindow &GetTestRomGuiWindow()
{
    return gTestRomGuiWindow;
}
