#include <cstdio>
#if !defined(SIM_SDRAM_H)
#define SIM_SDRAM_H 1

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "file_search.h"
#include "sim_memory.h"

class SimSDRAM : public MemoryInterface
{
  public:
    SimSDRAM(uint32_t sz)
    {
        mSize = sz;
        mMask = sz - 1;
        mData = new uint8_t[mSize];
    }

    ~SimSDRAM()
    {
        delete[] mData;
        mData = nullptr;
    }

    void UpdateChannel16(int ch, int dly, uint32_t addr, uint8_t req, uint8_t rw, uint8_t be, uint16_t din, uint16_t *dout, uint8_t *ack)
    {
        if (req == *ack)
            return;

        mDelay[ch]--;
        if (mDelay[ch] > 0)
            return;
        mDelay[ch] = rand() % dly;

        addr &= mMask;
        addr &= 0xfffffffe;

        if (rw)
        {
            *dout = (mData[addr + 1] << 8) | mData[addr];
            *ack = req;
        }
        else
        {
            if (be & 1)
                mData[addr + 0] = din & 0xff;
            if (be & 2)
                mData[addr + 1] = (din >> 8) & 0xff;
            *ack = req;
        }
    }

    void UpdateChannel32(int ch, int dly, uint32_t addr, uint8_t req, uint8_t rw, uint8_t be, uint32_t din, uint32_t *dout, uint8_t *ack)
    {
        if (req == *ack)
            return;

        mDelay[ch]--;
        if (mDelay[ch] > 0)
            return;
        mDelay[ch] = rand() % dly;

        addr &= mMask;
        addr &= 0xfffffffe;

        if (rw)
        {
            *dout = (mData[addr + 3] << 24) | (mData[addr + 2] << 16) | (mData[addr + 1] << 8) | (mData[addr + 0]);
            *ack = req;
        }
        else
        {
            if (be & 1)
                mData[addr + 0] = din & 0xff;
            if (be & 2)
                mData[addr + 1] = (din >> 8) & 0xff;
            if (be & 4)
                mData[addr + 2] = (din >> 16) & 0xff;
            if (be & 8)
                mData[addr + 3] = (din >> 24) & 0xff;
            *ack = req;
        }
    }

    void UpdateChannel64(int ch, uint32_t addr, uint8_t req, uint8_t rw, uint8_t be, uint64_t din, uint64_t *dout, uint8_t *ack)
    {
        if (req == *ack)
            return;

        addr &= mMask;
        addr &= 0xfffffffe;

        if (rw)
        {
            const uint32_t burstBase = addr & ~uint32_t(7);
            const uint32_t startHalfword = (addr >> 1) & 3;
            uint64_t value = 0;
            for (uint32_t beat = 0; beat < 4; beat++)
            {
                const uint32_t halfword = (startHalfword + beat) & 3;
                const uint32_t wordAddr = (burstBase + halfword * 2) & mMask;
                const uint16_t word = static_cast<uint16_t>(mData[wordAddr + 0]) |
                                      (static_cast<uint16_t>(mData[(wordAddr + 1) & mMask]) << 8);
                value |= static_cast<uint64_t>(word) << (beat * 16);
            }
            *dout = value;
            *ack = req;
        }
        else
        {
            if (be & 0x01)
                mData[addr + 0] = din & 0xff;
            if (be & 0x02)
                mData[addr + 1] = (din >> 8) & 0xff;
            if (be & 0x04)
                mData[addr + 2] = (din >> 16) & 0xff;
            if (be & 0x08)
                mData[addr + 3] = (din >> 24) & 0xff;
            if (be & 0x10)
                mData[addr + 4] = (din >> 32) & 0xff;
            if (be & 0x20)
                mData[addr + 5] = (din >> 40) & 0xff;
            if (be & 0x40)
                mData[addr + 6] = (din >> 48) & 0xff;
            if (be & 0x80)
                mData[addr + 7] = (din >> 56) & 0xff;
            *ack = req;
        }
    }

    // ------------------------------------------------------------------
    // Pulse-protocol channels, matching rtl/sdram.sv semantics: the client
    // pulses req for one clk cycle (rising-edge detected), request params are
    // latched at the edge, and ready is pulsed for exactly one clk cycle when
    // the access completes.  Call once per SDRAM-clock posedge, BEFORE eval of
    // that edge, so clients see the ready pulse exactly once.
    struct PulseChannel
    {
        uint8_t prevReq = 0;
        bool pending = false;
        int delay = 0;
        uint32_t addr = 0;
        uint16_t din = 0;
        uint8_t be = 0;
        uint8_t rnw = 1;
    };

    // Latches a new request on the rising edge of req; returns true when the
    // latched access should be performed this cycle.
    bool PulseChannelUpdate(int ch, int dly, uint32_t addr, uint8_t req, uint8_t rnw, uint8_t be, uint16_t din)
    {
        PulseChannel &c = mPulseCh[ch];
        if (req && !c.prevReq)
        {
            c.pending = true;
            c.delay = 2 + (dly > 1 ? rand() % dly : 0);
            c.addr = addr;
            c.din = din;
            c.be = be;
            c.rnw = rnw;
        }
        c.prevReq = req;

        if (!c.pending)
            return false;
        if (--c.delay > 0)
            return false;
        c.pending = false;
        return true;
    }

    uint16_t ReadWord(uint32_t addr) const
    {
        addr &= mMask & ~1u;
        return static_cast<uint16_t>(mData[addr]) | (static_cast<uint16_t>(mData[addr + 1]) << 8);
    }

    // ch3 style: 16-bit read/write with byte enables
    void UpdateChannelPulse16(int ch, int dly, uint32_t addr, uint8_t req, uint8_t rnw, uint8_t be, uint16_t din,
                              uint16_t *dout, uint8_t *rdy)
    {
        *rdy = 0;
        if (!PulseChannelUpdate(ch, dly, addr, req, rnw, be, din))
            return;

        PulseChannel &c = mPulseCh[ch];
        const uint32_t a = c.addr & mMask & ~1u;
        if (c.rnw)
        {
            *dout = ReadWord(a);
        }
        else
        {
            if (c.be & 1)
                mData[a + 0] = c.din & 0xff;
            if (c.be & 2)
                mData[a + 1] = (c.din >> 8) & 0xff;
        }
        *rdy = 1;
    }

    // ch1 style: 32-bit read (2 beats of a wrapped burst-of-4, like the real
    // controller: burst wraps within the aligned 8-byte block)
    void UpdateChannelPulse32(int ch, int dly, uint32_t addr, uint8_t req, uint32_t *dout, uint8_t *rdy)
    {
        *rdy = 0;
        if (!PulseChannelUpdate(ch, dly, addr, req, 1, 0, 0))
            return;

        PulseChannel &c = mPulseCh[ch];
        const uint32_t burstBase = c.addr & mMask & ~7u;
        const uint32_t startHalfword = (c.addr >> 1) & 3;
        uint32_t value = 0;
        for (uint32_t beat = 0; beat < 2; beat++)
        {
            const uint32_t halfword = (startHalfword + beat) & 3;
            value |= static_cast<uint32_t>(ReadWord(burstBase + halfword * 2)) << (beat * 16);
        }
        *dout = value;
        *rdy = 1;
    }

    // ch2 style: 64-bit read (4 beats of a wrapped burst-of-4)
    void UpdateChannelPulse64(int ch, int dly, uint32_t addr, uint8_t req, uint64_t *dout, uint8_t *rdy)
    {
        *rdy = 0;
        if (!PulseChannelUpdate(ch, dly, addr, req, 1, 0, 0))
            return;

        PulseChannel &c = mPulseCh[ch];
        const uint32_t burstBase = c.addr & mMask & ~7u;
        const uint32_t startHalfword = (c.addr >> 1) & 3;
        uint64_t value = 0;
        for (uint32_t beat = 0; beat < 4; beat++)
        {
            const uint32_t halfword = (startHalfword + beat) & 3;
            value |= static_cast<uint64_t>(ReadWord(burstBase + halfword * 2)) << (beat * 16);
        }
        *dout = value;
        *rdy = 1;
    }

    bool LoadData(const char *name, int offset, int stride)
    {
        std::vector<uint8_t> buffer;
        if (!gFileSearch.LoadFile(name, buffer))
        {
            printf("Failed to find file: %s\n", name);
            return false;
        }

        uint32_t addr = offset;
        for (uint8_t byte : buffer)
        {
            mData[addr & mMask] = byte;
            addr += stride;
        }

        printf("Loaded %zu bytes from %s at offset 0x%08X with stride %d\n", buffer.size(), name, offset, stride);
        return true;
    }

    bool LoadData16be(const char *name, int offset, int stride)
    {
        std::vector<uint8_t> buffer;
        if (!gFileSearch.LoadFile(name, buffer))
        {
            printf("Failed to find file: %s\n", name);
            return false;
        }

        // Ensure the buffer mSize is even
        if (buffer.size() % 2 != 0)
        {
            buffer.push_back(0); // Pad with zero if odd
        }

        uint32_t addr = offset;
        for (size_t i = 0; i < buffer.size(); i += 2)
        {
            // Store in big-endian format (swapping bytes)
            mData[addr & mMask] = buffer[i + 1];
            mData[(addr + 1) & mMask] = buffer[i + 0];
            addr += stride;
        }

        printf("Loaded %zu bytes (16-bit BE) from %s at offset 0x%08X\n", buffer.size(), name, offset);
        return true;
    }

    bool SaveData(const char *filename)
    {
        FILE *fp = fopen(filename, "wb");
        if (fp == nullptr)
        {
            return false;
        }

        if (fwrite(mData, 1, mSize, fp) != mSize)
        {
            fclose(fp);
            return false;
        }

        fclose(fp);
        return true;
    }

    // ------------------------------------------------------------------
    // MemoryInterface
    virtual void Read(uint32_t address, uint32_t size, void *data) const
    {
        size = ClampSize(mSize, address, size);
        memcpy(data, mData + address, size);
    }

    virtual void Write(uint32_t address, uint32_t size, const void *data)
    {
        size = ClampSize(mSize, address, size);
        memcpy(mData + address, data, size);
    }

    virtual uint32_t GetSize() const
    {
        return mSize;
    }
    virtual bool IsReadonly() const
    {
        return false;
    }

    uint32_t mSize;
    uint32_t mMask;
    uint8_t *mData;
    int mDelay[8];
    PulseChannel mPulseCh[8];
};

// SimSDRAM global instance is now provided via sim_core.h

#endif
