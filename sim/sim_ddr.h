#ifndef SIM_DDR_H
#define SIM_DDR_H

#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include "file_search.h"
#include "sim_memory.h"

// Class to simulate a 64-bit wide memory device
class SimDDR : public MemoryInterface
{
  public:
    SimDDR(uint32_t base, uint32_t sizeBytes)
    {
        // Initialize memory with size rounded up to multiple of 8 bytes
        mSize = (sizeBytes + 7) & ~7; // Round up to multiple of 8
        mMemory.resize(mSize, 0);
        mBaseAddr = base;

        // Reset state
        mReadComplete = false;
        mBusy = false;
        mBusyCounter = 0;
        mBurstCounter = 0;
        mBurstSize = 0;
    }

    bool LoadData(const std::vector<uint8_t> &data, uint32_t offset = 0, uint32_t stride = 1)
    {
        uint32_t memOffset = offset - mBaseAddr;

        // Check if the file will fit in memory with the stride
        if (memOffset + (data.size() - 1) * stride + 1 > mSize)
        {
            printf("Data too large (%u) to fit in memory at specified offset 0x%08x (0x%08x) with "
                   "stride %u\n",
                   (uint32_t)data.size(), offset, memOffset, stride);
            return false;
        }

        if (stride == 1)
        {
            // Fast path for stride=1 (contiguous data)
            std::copy(data.begin(), data.end(), mMemory.begin() + memOffset);
        }
        else
        {
            // Copy to memory with stride
            for (size_t i = 0; i < data.size(); i++)
            {
                mMemory[memOffset + i * stride] = data[i];
            }
        }
        return true;
    }

    // Load data from a file into memory at specified offset with optional
    // stride
    bool LoadData(const std::string &filename, uint32_t offset = 0, uint32_t stride = 1)
    {
        std::vector<uint8_t> buffer;
        if (!gFileSearch.LoadFile(filename, buffer))
        {
            printf("Failed to find file: %s\n", filename.c_str());
            return false;
        }

        if (LoadData(buffer, offset, stride))
        {
            printf("Loaded %zu bytes from %s at offset 0x%08X with stride %u\n", buffer.size(), filename.c_str(), offset, stride);
            return true;
        }
        return false;
    }

    // Save memory data to a file
    bool SaveData(const std::string &filename, uint32_t offset = 0, size_t length = 0)
    {
        uint32_t memOffset = offset - mBaseAddr;

        if (length == 0)
            length = mSize - memOffset;

        if (memOffset + length > mSize)
        {
            printf("Invalid offset/length for memory save\n");
            return false;
        }

        FILE *fp = fopen(filename.c_str(), "wb");
        if (!fp)
        {
            printf("Failed to open file for saving memory: %s\n", filename.c_str());
            return false;
        }

        size_t bytesWritten = fwrite(&mMemory[memOffset], 1, length, fp);
        fclose(fp);

        if (bytesWritten != length)
        {
            printf("Failed to write entire data to file: %s\n", filename.c_str());
            return false;
        }

        printf("Saved %zu bytes to %s from offset 0x%08X\n", bytesWritten, filename.c_str(), offset);
        return true;
    }

    // Clock the memory, processing read/write operations
    void Clock(uint32_t addr, const uint64_t &wdata, uint64_t &rdata, bool read, bool write, uint8_t &busyOut, uint8_t &readCompleteOut,
               uint8_t burstcnt = 1, uint8_t byteenable = 0xFF)
    {
        // Update busy status - simulate memory with occasional busy cycles
        if (mBusy)
        {
            mBusyCounter--;
            if (mBusyCounter == 0)
            {
                // If we're completing a read operation
                if (mPendingRead)
                {
                    mReadComplete = true;

                    // Prepare read data from the current burst address
                    uint32_t currentBurstAddr = (mPendingAddr & ~0x7) + (mBurstSize - mBurstCounter) * 8;
                    currentBurstAddr -= mBaseAddr;
                    if (currentBurstAddr + 8 <= mSize)
                    {
                        // Assemble 64-bit word from memory
                        mPendingRdata = 0;
                        for (int i = 0; i < 8; i++)
                        {
                            mPendingRdata |= static_cast<uint64_t>(mMemory[currentBurstAddr + i]) << (i * 8);
                        }
                    }
                    else
                    {
                        mPendingRdata = 0;
                    }

                    // Decrement burst counter
                    mBurstCounter--;

                    // If burst is complete, clear pending read flag
                    if (mBurstCounter == 0)
                    {
                        mPendingRead = false;
                        mBusy = false;
                    }
                    else
                    {
                        // Otherwise, set up for next word in burst
                        mBusyCounter = mReadLatency;
                    }
                }
                else if (mBurstCounter > 0)
                {
                    // Writing in burst mode, move to next word
                    mBurstCounter--;

                    if (mBurstCounter == 0)
                    {
                        mBusy = false;
                    }
                    else
                    {
                        // Ready for next write in the burst
                        mBusy = false;
                        mBusyCounter = 0;
                    }
                }
                else
                {
                    // Normal operation completion
                    mBusy = false;
                }
            }
        }
        else
        {
            if (read && !mPendingRead)
            {
                // Start new read operation in burst mode
                mBusy = true;
                mBusyCounter = mReadLatency;
                mPendingRead = true;
                mPendingAddr = addr;
                mReadComplete = false;
                mBurstCounter = burstcnt;
                mBurstSize = burstcnt;
            }
            else if (write && (mBurstCounter == 0 || !mBusy))
            {
                // Handle start of burst write or individual word in burst
                uint32_t currentBurstAddr;

                if (mBurstCounter == 0)
                {
                    // Starting a new burst write
                    mPendingAddr = addr;
                    mBurstCounter = burstcnt - 1; // First word is written now
                    mBurstSize = burstcnt;
                    currentBurstAddr = (addr & ~0x7) - mBaseAddr;
                }
                else
                {
                    // Writing next word in an existing burst
                    currentBurstAddr = (mPendingAddr & ~0x7) + (mBurstSize - mBurstCounter) * 8;
                    currentBurstAddr -= mBaseAddr;
                    mBurstCounter--;
                }

                // Perform write operation
                if (currentBurstAddr + 8 <= mSize)
                {
                    // Write 64-bit word to memory, respecting byte enable
                    // signal
                    for (int i = 0; i < 8; i++)
                    {
                        // Only write byte if corresponding bit in byteenable is
                        // set
                        if (byteenable & (1 << i))
                        {
                            mMemory[currentBurstAddr + i] = (wdata >> (i * 8)) & 0xFF;
                        }
                    }
                }

                // If this is the last word in the burst or not a burst
                // operation
                if (mBurstCounter == 0)
                {
                    // Simulate write latency
                    // TODO - busy usage doesn't match DE-10
                    // busy = true;
                    // busy_counter = write_latency;
                }
            }
        }

        // Set outputs
        busyOut = 0; // busy ? 1 : 0; // TODO - busy_out doesn't match DE-10 DDR
        readCompleteOut = mReadComplete ? 1 : 0;

        if (mReadComplete)
        {
            rdata = mPendingRdata;
            mReadComplete = false; // Clear completion flag after it's been seen
        }
    }

    // Direct access to memory for debugging/testing
    uint8_t &operator[](size_t addr)
    {
        static uint8_t sDummy = 0;
        if (addr < mBaseAddr)
            return sDummy;
        size_t offset = addr - mBaseAddr;
        if (offset >= mSize)
            return sDummy;
        return mMemory[offset];
    }

    // Memory parameters
    void SetReadLatency(int cycles)
    {
        mReadLatency = cycles;
    }
    void SetWriteLatency(int cycles)
    {
        mWriteLatency = cycles;
    }

    // ------------------------------------------------------------------
    // MemoryInterface
    virtual void Read(uint32_t address, uint32_t sz, void *data) const
    {
        if (address < mBaseAddr)
            return;
        address = address - mBaseAddr;
        sz = ClampSize(mSize, address, sz);
        memcpy(data, mMemory.data() + address, sz);
    }

    virtual void Write(uint32_t address, uint32_t sz, const void *data)
    {
        if (address < mBaseAddr)
            return;
        address = address - mBaseAddr;
        sz = ClampSize(mSize, address, sz);
        memcpy(mMemory.data() + address, data, sz);
    }

    virtual uint32_t GetSize() const
    {
        return mSize + mBaseAddr;
    }
    virtual bool IsReadonly() const
    {
        return false;
    }

  private:
    std::vector<uint8_t> mMemory;
    uint32_t mSize;
    uint32_t mBaseAddr;

    // Memory timing parameters
    int mReadLatency = 2;  // Default read latency in clock cycles
    int mWriteLatency = 1; // Default write latency in clock cycles

    // Internal state
    bool mBusy;
    int mBusyCounter;
    bool mReadComplete;
    bool mPendingRead;
    uint32_t mPendingAddr;
    uint64_t mPendingRdata;

    // Burst operation state
    uint8_t mBurstCounter; // Counter for remaining words in burst
    uint8_t mBurstSize;    // Total size of current burst
};

// SimDDR global instance is now provided via sim_core.h

#endif // SIM_DDR_H
