#include "sim_audio_capture.h"

#include <cstdio>
#include <cstring>

namespace
{
constexpr uint32_t kTargetSampleRate = 50000; // divides 32MHz exactly (640)
}

SimAudioCapture::~SimAudioCapture()
{
    Stop();
}

bool SimAudioCapture::Start(const std::string &path, uint64_t simClockHz)
{
    Stop();

    mStream.open(path, std::ios::binary | std::ios::trunc);
    if (!mStream)
    {
        printf("Audio capture: failed to open %s\n", path.c_str());
        return false;
    }

    mSimClockHz = simClockHz;
    mDivider = static_cast<uint32_t>(simClockHz / kTargetSampleRate);
    if (mDivider == 0)
        mDivider = 1;
    mSampleRateHz = static_cast<uint32_t>(simClockHz / mDivider);
    mDividerCount = 0;
    mDataBytes = 0;

    WriteHeader(0); // placeholder, patched on Stop
    printf("Audio capture started: %s (%u Hz)\n", path.c_str(), mSampleRateHz);
    return true;
}

void SimAudioCapture::Stop()
{
    if (!mStream.is_open())
        return;

    // Patch the header with the final sizes
    mStream.seekp(0);
    WriteHeader(mDataBytes);
    mStream.close();
    printf("Audio capture stopped (%u bytes of samples)\n", mDataBytes);
}

bool SimAudioCapture::IsActive() const
{
    return mStream.is_open();
}

void SimAudioCapture::Tick(uint64_t totalTicks, bool audioValid, int16_t audioLeft, int16_t audioRight)
{
    (void)totalTicks;
    if (!mStream.is_open() || !audioValid)
        return;

    if (++mDividerCount < mDivider)
        return;
    mDividerCount = 0;

    int16_t frame[2] = {audioLeft, audioRight};
    mStream.write(reinterpret_cast<const char *>(frame), sizeof(frame));
    mDataBytes += sizeof(frame);
}

void SimAudioCapture::WriteHeader(uint32_t dataBytes)
{
    struct __attribute__((packed)) WavHeader
    {
        char riff[4];
        uint32_t riffSize;
        char wave[4];
        char fmt[4];
        uint32_t fmtSize;
        uint16_t format;
        uint16_t channels;
        uint32_t sampleRate;
        uint32_t byteRate;
        uint16_t blockAlign;
        uint16_t bitsPerSample;
        char data[4];
        uint32_t dataSize;
    };
    static_assert(sizeof(WavHeader) == 44, "WAV header must be 44 bytes");

    WavHeader header;
    memcpy(header.riff, "RIFF", 4);
    header.riffSize = 36 + dataBytes;
    memcpy(header.wave, "WAVE", 4);
    memcpy(header.fmt, "fmt ", 4);
    header.fmtSize = 16;
    header.format = 1; // PCM
    header.channels = 2;
    header.sampleRate = mSampleRateHz;
    header.byteRate = mSampleRateHz * 4;
    header.blockAlign = 4;
    header.bitsPerSample = 16;
    memcpy(header.data, "data", 4);
    header.dataSize = dataBytes;

    mStream.write(reinterpret_cast<const char *>(&header), sizeof(header));
}
