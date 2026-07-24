#pragma once

#include <cstdint>
#include <fstream>
#include <string>

// Captures the core's AUDIO_L/AUDIO_R outputs to a stereo 16-bit WAV file.
// Tick() is called once per CLK_32M cycle; samples are taken on a fixed
// divider of the simulation clock.
class SimAudioCapture
{
  public:
    SimAudioCapture() = default;
    ~SimAudioCapture();

    bool Start(const std::string &path, uint64_t simClockHz);
    void Stop();
    bool IsActive() const;
    void Tick(uint64_t totalTicks, bool audioValid, int16_t audioLeft, int16_t audioRight);

  private:
    void WriteHeader(uint32_t dataBytes);

    std::ofstream mStream;
    uint64_t mSimClockHz = 0;
    uint32_t mSampleRateHz = 0;
    uint32_t mDivider = 0;
    uint32_t mDividerCount = 0;
    uint32_t mDataBytes = 0;
};
