#include "games.h"
#include "sim_core.h"
#include "file_search.h"
#include "mra_loader.h"
#include "third_party/pugixml.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace
{

bool GameInitHharryb();
bool GameInitHharryb2();
bool GameInitTestbed();

struct GameEntry
{
    const char *name;
    const char *mraPath;      // MRA-backed loader
    bool (*handLoader)();     // hand-coded loader
    const char *unsupported;  // reason the core can't run it
};

// clang-format off
const GameEntry gGames[N_GAMES] = {
    /* GAME_DBREED   */ {"dbreed",   "../releases/Dragon Breed (Japan, M72 hardware).mra", nullptr, nullptr},
    /* GAME_HHARRY   */ {"hharry",   nullptr, nullptr,
                         "hharry (World) runs on native M81 hardware; the core has no M81 memory map "
                         "(only M72 maps 0-2 and M84 maps 3-4) - use hharryu instead"},
    /* GAME_HHARRYU  */ {"hharryu",  "../releases/Hammerin' Harry (US, M84 hardware).mra", nullptr, nullptr},
    /* GAME_HHARRYB  */ {"hharryb",  nullptr, GameInitHharryb, nullptr},
    /* GAME_HHARRYB2 */ {"hharryb2", nullptr, GameInitHharryb2, nullptr},
    /* GAME_TESTBED  */ {"testbed",  nullptr, GameInitTestbed, nullptr},
};
// clang-format on

std::string gLoadedShortName = "none";
uint16_t gDipDefaults = 0x0000;

std::string RomDir()
{
    const char *romDir = std::getenv("M72_ROM_DIR");
    return romDir ? romDir : "../roms";
}

// ---------------------------------------------------------------------------
// Hand-coded rom_loader stream builders (rtl/rom.sv format):
//   [board_cfg][region idx][size BE24][data...]...
// Mirrors what the release MRAs produce for games that have no MRA.

void AppendRegionHeader(std::vector<uint8_t> &stream, uint8_t region, uint32_t size)
{
    stream.push_back(region);
    stream.push_back((size >> 16) & 0xff);
    stream.push_back((size >> 8) & 0xff);
    stream.push_back(size & 0xff);
}

// A ROM file reference: looked up by name first (so build-dir overrides like
// the testbed take precedence), then by CRC in the zip search paths.
struct RomRef
{
    RomRef(const char *n, uint32_t c = 0) : name(n), crc(c)
    {
    }
    const char *name;
    uint32_t crc;
};

bool LoadRomRef(const RomRef &ref, std::vector<uint8_t> &buffer)
{
    if (gFileSearch.LoadFile(ref.name, buffer))
        return true;
    if (ref.crc != 0 && gFileSearch.LoadFileByCRC(ref.crc, buffer))
        return true;
    printf("Failed to find ROM file: %s (crc %08x)\n", ref.name, ref.crc);
    return false;
}

// Load and concatenate a list of ROM files into a single lane buffer
bool LoadLane(const std::vector<RomRef> &refs, std::vector<uint8_t> &lane)
{
    lane.clear();
    for (const RomRef &ref : refs)
    {
        std::vector<uint8_t> buffer;
        if (!LoadRomRef(ref, buffer))
            return false;
        lane.insert(lane.end(), buffer.begin(), buffer.end());
    }
    return true;
}

// Byte-interleave equal-sized lanes into the stream (lane 0 = byte 0 of each
// group, matching the MRA <interleave> element with map="...01")
bool AppendInterleaved(std::vector<uint8_t> &stream, const std::vector<std::vector<RomRef>> &laneFiles)
{
    std::vector<std::vector<uint8_t>> lanes(laneFiles.size());
    for (size_t i = 0; i < laneFiles.size(); i++)
    {
        if (!LoadLane(laneFiles[i], lanes[i]))
            return false;
        if (lanes[i].size() != lanes[0].size())
        {
            printf("Interleave lane size mismatch\n");
            return false;
        }
    }

    for (size_t offset = 0; offset < lanes[0].size(); offset++)
    {
        for (const auto &lane : lanes)
            stream.push_back(lane[offset]);
    }
    return true;
}

bool AppendLinear(std::vector<uint8_t> &stream, const RomRef &ref)
{
    return AppendInterleaved(stream, {{ref}});
}

bool FinishGameLoad(std::vector<uint8_t> &stream, uint16_t dipDefaults)
{
    if (!gSimCore.SendIOCTLData(0, stream))
    {
        printf("Failed to send ROM data via ioctl\n");
        return false;
    }
    gDipDefaults = dipDefaults;
    return true;
}

// Hammerin' Harry (World, M84 hardware bootleg): identical GFX/sound data to
// hharryu, bootleg-patched main CPU ROMs.  Structure mirrors the hharryu MRA.
bool GameInitHharryb()
{
    gFileSearch.ClearSearchPaths();
    gFileSearch.AddSearchPath(".");
    gFileSearch.AddSearchPath(RomDir());
    gFileSearch.AddSearchPath(RomDir() + "/hharryb.zip");
    gFileSearch.AddSearchPath(RomDir() + "/hharry.zip"); // parent set: GFX/sample ROMs

    std::vector<uint8_t> stream;
    stream.push_back(0x14); // board_cfg: m84, memory_map 4

    // maincpu: h/l pairs; l1/h1 pair repeated to fill/mirror the top of the
    // 512KB window (reset vector), as in the hharryu MRA
    AppendRegionHeader(stream, REGION_CPU_ROM, 0x80000);
    if (!AppendInterleaved(stream, {{"6-a-27c010a.bin"}, {"4-a-27c010a.bin"}}))
        return false;
    if (!AppendInterleaved(stream, {{"5-a-27c512.bin"}, {"3-a-27c512.bin"}}))
        return false;
    if (!AppendInterleaved(stream, {{"5-a-27c512.bin"}, {"3-a-27c512.bin"}}))
        return false;

    AppendRegionHeader(stream, REGION_SPRITE, 0x80000);
    if (!AppendInterleaved(stream,
                           {{{"17-c-27c010a.bin", 0xec5127ef}}, {{"16-c-27c010a.bin", 0xdef65294}}, {{"14-c-27c010a.bin", 0xbb0d6ad4}}, {{"15-c-27c010a.bin", 0x4351044e}}}))
        return false;

    AppendRegionHeader(stream, REGION_BG_A, 0x80000);
    if (!AppendInterleaved(stream, {{{"13-b-27c010a.bin", 0xc577ba5f}}, {{"11-b-27c010a.bin", 0x429d12ab}}, {{"9-b-27c010a.bin", 0xb5b163b0}}, {{"7-b-27c010a.bin", 0x8ef566a1}}}))
        return false;

    AppendRegionHeader(stream, REGION_SAMPLES, 0x20000);
    if (!AppendLinear(stream, {"1-a-27c010a.bin", 0xfaaacaff}))
        return false;

    AppendRegionHeader(stream, REGION_SOUND, 0x10000);
    if (!AppendLinear(stream, {"2-a-27c512.bin", 0x80e210e7}))
        return false;

    return FinishGameLoad(stream, 0x0600);
}

// testroms testbed: the test program's CPU ROMs (built by testroms/Makefile
// into testroms/build/testbed/hharryu/) on the hharryu M84 board, with the
// original hharryu GFX/sound ROMs from the zips.  The build directory is
// searched first, so freshly built test ROMs take precedence by name.
bool GameInitTestbed()
{
    gFileSearch.ClearSearchPaths();
    gFileSearch.AddSearchPath("../testroms/build/testbed/hharryu");
    gFileSearch.AddSearchPath(".");
    gFileSearch.AddSearchPath(RomDir());
    gFileSearch.AddSearchPath(RomDir() + "/hharryu.zip");
    gFileSearch.AddSearchPath(RomDir() + "/hharry.zip");

    std::vector<uint8_t> stream;
    stream.push_back(0x14); // board_cfg: m84, memory_map 4

    AppendRegionHeader(stream, REGION_CPU_ROM, 0x80000);
    if (!AppendInterleaved(stream, {{{"gen_a-l0-u.ic60", 0xdf0726ae}}, {{"gen_a-h0-u.ic54", 0xede7f755}}}))
        return false;
    if (!AppendInterleaved(stream, {{{"gen_a-l1-f.ic59", 0xb23e966c}}, {{"gen_a-h1-f.ic53", 0x31b741c5}}}))
        return false;
    if (!AppendInterleaved(stream, {{{"gen_a-l1-f.ic59", 0xb23e966c}}, {{"gen_a-h1-f.ic53", 0x31b741c5}}}))
        return false;

    AppendRegionHeader(stream, REGION_SPRITE, 0x80000);
    if (!AppendInterleaved(stream, {{{"hh_n0.ic33", 0xec5127ef}},
                                    {{"hh_n1.ic34", 0xdef65294}},
                                    {{"hh_n2.ic35", 0xbb0d6ad4}},
                                    {{"hh_n3.ic36", 0x4351044e}}}))
        return false;

    AppendRegionHeader(stream, REGION_BG_A, 0x80000);
    if (!AppendInterleaved(stream, {{{"hh_a0.ic51", 0xc577ba5f}},
                                    {{"hh_a1.ic52", 0x429d12ab}},
                                    {{"hh_a2.ic53", 0xb5b163b0}},
                                    {{"hh_a3.ic54", 0x8ef566a1}}}))
        return false;

    AppendRegionHeader(stream, REGION_SAMPLES, 0x20000);
    if (!AppendLinear(stream, {"gen_a-v0-0.ic44", 0xfaaacaff}))
        return false;

    AppendRegionHeader(stream, REGION_SOUND, 0x10000);
    if (!AppendLinear(stream, {"gen=84=_a-sp-0-f.ic14", 0x80e210e7}))
        return false;

    return FinishGameLoad(stream, 0x0600);
}

// Hammerin' Harry (Playmark bootleg): M84-style main/gfx layout with split
// ROMs.  The bootleg's sound board is a different Z80+YM2151 design (128KB
// Z80 program, samples embedded); only the first 64KB of Z80 code fits the
// core's sound region, so sound is not expected to be correct (MAME marks
// this set MACHINE_NOT_WORKING).
bool GameInitHharryb2()
{
    gFileSearch.ClearSearchPaths();
    gFileSearch.AddSearchPath(".");
    gFileSearch.AddSearchPath(RomDir());
    gFileSearch.AddSearchPath(RomDir() + "/hharryb2.zip");

    std::vector<uint8_t> stream;
    stream.push_back(0x14); // board_cfg: m84, memory_map 4

    AppendRegionHeader(stream, REGION_CPU_ROM, 0x80000);
    if (!AppendInterleaved(stream, {{"1.ic33"}, {"3.ic30"}}))
        return false;
    if (!AppendInterleaved(stream, {{"2.ic32"}, {"4.ic29"}}))
        return false;
    if (!AppendInterleaved(stream, {{"2.ic32"}, {"4.ic29"}}))
        return false;

    // sprites: 8 x 64KB, sequential pairs per 128KB MAME lane
    AppendRegionHeader(stream, REGION_SPRITE, 0x80000);
    if (!AppendInterleaved(stream, {{"ic1", "ic30"}, {"ic2", "ic31"}, {"ic3", "ic32"}, {"ic4", "ic33"}}))
        return false;

    AppendRegionHeader(stream, REGION_BG_A, 0x80000);
    if (!AppendInterleaved(stream,
                           {{"8.ic139", "7.ic140"}, {"10.ic137", "9.ic138"}, {"12.ic135", "11.ic136"}, {"14.ic133", "13.ic134"}}))
        return false;

    // no sample ROM on this bootleg

    AppendRegionHeader(stream, REGION_SOUND, 0x10000);
    if (!AppendLinear(stream, {"5.ic23"}))
        return false;

    return FinishGameLoad(stream, 0x0600);
}

// Parse the <switches default="xx,yy"> attribute: raw bytes for dip_sw[0]
// and dip_sw[1] as MiSTer would send them via ioctl index 254.
uint16_t ParseMraDipDefaults(const char *mraPath)
{
    pugi::xml_document doc;
    if (!doc.load_file(mraPath))
        return 0;

    auto switches = doc.child("misterromdescription").child("switches");
    const char *def = switches.attribute("default").as_string(nullptr);
    if (!def)
        return 0;

    unsigned b0 = 0, b1 = 0;
    sscanf(def, "%x,%x", &b0, &b1);
    return static_cast<uint16_t>((b1 << 8) | b0);
}

} // namespace

Game GameFind(const char *name)
{
    for (int i = 0; i < N_GAMES; i++)
    {
        if (strcasecmp(gGames[i].name, name) == 0)
            return (Game)i;
    }
    return GAME_INVALID;
}

const char *GameName(Game game)
{
    if (game < N_GAMES)
        return gGames[game].name;
    return "none";
}

const char *GameLoadedShortName()
{
    return gLoadedShortName.c_str();
}

uint16_t GameDipDefaults()
{
    return gDipDefaults;
}

bool GameInitMra(const char *mraPath)
{
    gFileSearch.ClearSearchPaths();
    gFileSearch.AddSearchPath(".");
    gFileSearch.AddSearchPath(RomDir());

    // Add the directory containing the MRA file as a search path
    std::string mraPathStr(mraPath);
    size_t lastSlash = mraPathStr.find_last_of("/\\");
    if (lastSlash != std::string::npos)
    {
        gFileSearch.AddSearchPath(mraPathStr.substr(0, lastSlash));
    }

    MRALoader loader;
    std::vector<uint8_t> romData;
    uint32_t address = 0;

    if (!loader.Load(mraPath, romData, address))
    {
        printf("Failed to load MRA file '%s': %s\n", mraPath, loader.GetLastError().c_str());
        return false;
    }

    if (address != 0)
    {
        printf("MRA '%s' requests a DDR load address (%08x); the M72 core has no DDR load path\n", mraPath, address);
        return false;
    }

    printf("Loaded MRA: %s (%zu bytes)\n", mraPath, romData.size());

    if (!gSimCore.SendIOCTLData(0, romData))
    {
        printf("Failed to send ROM data via ioctl\n");
        return false;
    }

    gDipDefaults = ParseMraDipDefaults(mraPath);

    // Derive a short name from the setname if the file names one we know
    size_t nameStart = (lastSlash == std::string::npos) ? 0 : lastSlash + 1;
    gLoadedShortName = mraPathStr.substr(nameStart);

    printf("Successfully loaded MRA: %s (dips %04x)\n", mraPath, gDipDefaults);
    return true;
}

bool GameInit(Game game)
{
    if (game >= N_GAMES)
    {
        printf("GameInit: invalid game\n");
        return false;
    }

    const GameEntry &entry = gGames[game];
    if (entry.unsupported)
    {
        printf("GameInit: %s\n", entry.unsupported);
        return false;
    }

    if (entry.mraPath)
    {
        if (!GameInitMra(entry.mraPath))
            return false;
    }
    else if (entry.handLoader)
    {
        if (!entry.handLoader())
            return false;
    }
    else
    {
        printf("GameInit: '%s' does not have a loader yet\n", entry.name);
        return false;
    }

    gLoadedShortName = entry.name;
    gSimCore.SetGame(game);
    return true;
}
