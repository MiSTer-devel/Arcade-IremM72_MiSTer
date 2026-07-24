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

        const uint16_t cs = gSimCore.mTop->dbg_cpu_cs;
        const uint16_t ip = gSimCore.mTop->dbg_cpu_ip;
        const uint8_t opcode = gSimCore.mTop->dbg_cpu_opcode;
        const uint32_t linear = ((uint32_t)cs << 4) + ip;

        ImGui::Text("CS:IP  %04X:%04X  (linear %05X)", cs, ip, linear);
        ImGui::Text("Opcode %02X", opcode);

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
