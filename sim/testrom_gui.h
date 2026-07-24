#ifndef TESTROM_GUI_H
#define TESTROM_GUI_H 1

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "imgui_wrap.h"

struct TestRomGuiEntry
{
    uint32_t mIndex = 0;
    std::string mLabel;
    uint8_t mType = 0;
    uint16_t mValue = 0;
    uint16_t mOverrideValue = 0;
};

struct TestRomGuiState
{
    bool mAvailable = false;
    uint32_t mAddress = 0x0A00;
    uint64_t mLastSyncTicks = 0;
    std::vector<TestRomGuiEntry> mEntries;
};

class TestRomGuiWindow : public Window
{
  public:
    TestRomGuiWindow();

    void Init() override;
    void Draw() override;

    void Reset();
    void TickVblank();

    TestRomGuiState GetState() const;
    bool SetOverrideByIndex(uint32_t index, uint16_t value, bool pulse, bool *applied, std::string *error = nullptr);
    bool SetOverrideByLabel(const std::string &label, uint16_t value, bool pulse, bool *applied, std::string *error = nullptr);

    static const char *TypeName(uint8_t type);

  private:
    static constexpr uint32_t kWorkRamAddress = 0x0A00;
    static constexpr uint16_t kMagic = 0xAB7D;
    static constexpr uint32_t kMaxEntries = 32;
    static constexpr uint32_t kEntrySize = 20;
    static constexpr uint32_t kHeaderSize = 6;
    static constexpr uint32_t kFooterSize = 2;
    static constexpr uint32_t kStateSize = kHeaderSize + (kMaxEntries * kEntrySize) + kFooterSize;

    struct PendingOverride
    {
        bool mActive = false;
        std::string mLabel;
        uint8_t mType = 0;
        uint16_t mValue = 0;
        bool mPulse = false;
    };

    TestRomGuiState mState;
    std::array<PendingOverride, kMaxEntries> mPending;

    bool ReadSafeState(TestRomGuiState &state) const;
    bool RefreshAndApplyPending();
    void WriteOverrideValue(uint32_t index, uint16_t value);
    bool QueueOverride(uint32_t index, uint16_t value, bool pulse, std::string *error = nullptr);
};

TestRomGuiWindow &GetTestRomGuiWindow();

#endif
