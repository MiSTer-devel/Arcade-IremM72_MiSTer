#if !defined(SIM_MEMORY_H)
#define SIM_MEMORY_H 1

#include <stdint.h>
#include <algorithm>

class MemoryInterface
{
  public:
    virtual void Read(uint32_t address, uint32_t size, void *data) const = 0;
    virtual void Write(uint32_t address, uint32_t size, const void *data) = 0;

    virtual uint32_t GetSize() const = 0;

    virtual bool IsReadonly() const = 0;

    virtual ~MemoryInterface() {};

  protected:
    static uint32_t ClampSize(uint32_t totalSize, uint32_t offset, uint32_t size)
    {
        if (offset >= totalSize)
            return 0;
        return std::min((totalSize - offset), size);
    }
};

class MemorySlice : public MemoryInterface
{
  public:
    MemorySlice() = delete;
    MemorySlice(MemoryInterface &source, uint32_t offset, uint32_t size) : mSource(source)
    {
        mIsReadonly = source.IsReadonly();
        mSourceSize = source.GetSize();

        mOffset = offset;
        mSize = ClampSize(mSourceSize, mOffset, size);
    }

    virtual void Read(uint32_t address, uint32_t size, void *data) const
    {
        size = ClampSize(mSize, address, size);
        mSource.Read(address + mOffset, size, data);
    }

    virtual void Write(uint32_t address, uint32_t size, const void *data)
    {
        size = ClampSize(mSize, address, size);
        mSource.Write(address + mOffset, size, data);
    }

    virtual uint32_t GetSize() const
    {
        return mSize;
    }
    virtual bool IsReadonly() const
    {
        return mIsReadonly;
    }

    uint32_t mOffset;
    uint32_t mSize;

    bool mIsReadonly;
    uint32_t mSourceSize;
    MemoryInterface &mSource;
};

class Memory16b : public MemoryInterface
{
  public:
    Memory16b(void *lowMem, void *highMem, uint32_t size)
    {
        mLowMem = (uint8_t *)lowMem;
        mHighMem = (uint8_t *)highMem;
        mSize = size;
    }

    virtual void Read(uint32_t address, uint32_t size, void *data) const
    {
        uint32_t endAddress = address + ClampSize(mSize, address, size);
        uint8_t *outPtr = (uint8_t *)data;
        while (address < endAddress)
        {
            if (address & 1)
                *outPtr = mLowMem[address >> 1];
            else
                *outPtr = mHighMem[address >> 1];
            address++;
            outPtr++;
        }
    }

    virtual void Write(uint32_t address, uint32_t size, const void *data)
    {
        uint32_t endAddress = address + ClampSize(mSize, address, size);
        const uint8_t *inPtr = (const uint8_t *)data;
        while (address < endAddress)
        {
            if (address & 1)
                mLowMem[address >> 1] = *inPtr;
            else
                mHighMem[address >> 1] = *inPtr;
            address++;
            inPtr++;
        }
    }

    virtual uint32_t GetSize() const
    {
        return mSize;
    }
    virtual bool IsReadonly() const
    {
        return false;
    }

    uint8_t *mLowMem;
    uint8_t *mHighMem;
    uint32_t mSize;
};

class Memory8b : public MemoryInterface
{
  public:
    Memory8b(void *mem, uint32_t size)
    {
        mMem = (uint8_t *)mem;
        mSize = size;
    }

    virtual void Read(uint32_t address, uint32_t size, void *data) const
    {
        size = ClampSize(mSize, address, size);
        memcpy(data, mMem + address, size);
    }

    virtual void Write(uint32_t address, uint32_t size, const void *data)
    {
        size = ClampSize(mSize, address, size);
        memcpy(mMem + address, data, size);
    }

    virtual uint32_t GetSize() const
    {
        return mSize;
    }
    virtual bool IsReadonly() const
    {
        return false;
    }

    uint8_t *mMem;
    uint32_t mSize;
};

class MemoryNull : public MemoryInterface
{
  public:
    MemoryNull()
    {
    }

    virtual void Read(uint32_t, uint32_t, void *) const
    {
    }

    virtual void Write(uint32_t, uint32_t, const void *)
    {
    }

    virtual uint32_t GetSize() const
    {
        return 0;
    }
    virtual bool IsReadonly() const
    {
        return true;
    }
};


#endif
