#include "imgui_wrap.h"
#include "sim_core.h"
#include "sim_hierarchy.h"
#include "M72.h"
#include "M72___024root.h"

#ifdef HAVE_CAPSTONE
#include <capstone/capstone.h>
#endif

// V30 CPU window: registers/CS:IP/opcode from the core's debug export taps,
// with disassembly of upcoming instructions when capstone is available.

class V30Window : public Window
{
  public:
    V30Window() : Window("V30")
    {
#ifdef HAVE_CAPSTONE
        mCapstoneOk = cs_open(CS_ARCH_X86, CS_MODE_16, &mCapstone) == CS_ERR_OK;
#endif
    }

    ~V30Window()
    {
#ifdef HAVE_CAPSTONE
        if (mCapstoneOk)
            cs_close(&mCapstone);
#endif
    }

    void Init() override
    {
    }

    void Draw() override
    {
        if (!gSimCore.mTop)
            return;

        // Packed 224-bit register file: 0=AX,1=CX,2=DX,3=BX,4=SP,5=BP,6=SI,
        // 7=DI,8=ES,9=CS,10=SS,11=DS,12=IP,13=PSW (each 16-bit, half-word aligned).
        auto reg16 = [](int idx) -> uint16_t {
            const int word = idx / 2;
            const int half = idx % 2;
            return static_cast<uint16_t>((gSimCore.mTop->dbg_cpu_regs[word] >> (half * 16)) & 0xffffu);
        };

        const uint16_t ax = reg16(0), cx = reg16(1), dx = reg16(2), bx = reg16(3);
        const uint16_t sp = reg16(4), bp = reg16(5), si = reg16(6), di = reg16(7);
        const uint16_t es = reg16(8), cs = reg16(9), ss = reg16(10), ds = reg16(11);
        const uint16_t ip = reg16(12), psw = reg16(13);
        const uint32_t linear = ((uint32_t)cs << 4) + ip;

        ImGui::Text("AX %04X  BX %04X  CX %04X  DX %04X", ax, bx, cx, dx);
        ImGui::Text("SP %04X  BP %04X  SI %04X  DI %04X", sp, bp, si, di);
        ImGui::Text("CS %04X  DS %04X  ES %04X  SS %04X", cs, ds, es, ss);
        ImGui::Text("IP %04X  (CS:IP linear %05X)", ip, linear);
        ImGui::Text("PSW %04X  %c%c%c%c%c%c%c%c%c", psw,
                    (psw & 0x0800) ? 'O' : '-', (psw & 0x0400) ? 'D' : '-',
                    (psw & 0x0200) ? 'I' : '-', (psw & 0x0100) ? 'T' : '-',
                    (psw & 0x0080) ? 'S' : '-', (psw & 0x0040) ? 'Z' : '-',
                    (psw & 0x0010) ? 'A' : '-', (psw & 0x0004) ? 'P' : '-',
                    (psw & 0x0001) ? 'C' : '-');

        ImGui::Separator();

#ifdef HAVE_CAPSTONE
        if (mCapstoneOk)
        {
            // Disassemble a window of code at the current linear address.
            // The CPU address space maps low 1MB to the ROM/RAM regions;
            // fetches under 0xA0000 come from CPU ROM in all memory maps.
            uint8_t code[64];
            MemoryRegion region = MemoryRegion::CPU_ROM;
            uint32_t offset = linear;
            if (linear >= 0xA0000)
            {
                // work RAM window (0xA0000+ varies per memory map; show raw)
                region = MemoryRegion::WORK_RAM;
                offset = linear & 0xFFFF;
            }
            gSimCore.Memory(region).Read(offset, sizeof(code), code);

            cs_insn *insn = nullptr;
            size_t count = cs_disasm(mCapstone, code, sizeof(code), linear, 8, &insn);
            for (size_t i = 0; i < count; i++)
            {
                ImGui::Text("%05X  %-10s %s", (uint32_t)insn[i].address, insn[i].mnemonic, insn[i].op_str);
            }
            if (insn)
                cs_free(insn, count);
            if (count == 0)
            {
                ImGui::TextUnformatted("(disassembly unavailable)");
            }
        }
        else
        {
            ImGui::TextUnformatted("capstone init failed");
        }
#else
        ImGui::TextUnformatted("built without capstone - no disassembly");
#endif
    }

  private:
#ifdef HAVE_CAPSTONE
    csh mCapstone = 0;
    bool mCapstoneOk = false;
#endif
};

V30Window gV30Window;
