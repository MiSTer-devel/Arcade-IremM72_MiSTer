#ifndef GAMES_H
#define GAMES_H 1

#include <stdint.h>

// Directly loadable games (zips expected in ../roms, override with M72_ROM_DIR).
enum Game : uint8_t
{
    GAME_DBREED = 0, // Dragon Breed (Japan, M72) - via releases MRA
    GAME_HHARRY,     // Hammerin' Harry (World, M84) - hand-coded
    GAME_HHARRYU,    // Hammerin' Harry (US, M84) - via releases MRA
    GAME_HHARRYB,    // Hammerin' Harry (bootleg) - hand-coded
    GAME_HHARRYB2,   // Hammerin' Harry (bootleg 2) - hand-coded
    GAME_TESTBED,    // testroms/ testbed CPU ROMs on the hharryu board

    N_GAMES,

    GAME_INVALID = 0xff
};

// SDRAM region base addresses (byte addresses), from rtl/m72_pkg.sv LOAD_REGIONS
static const uint32_t CPU_ROM_SDR_BASE    = 0x0000000;
static const uint32_t SPRITE_ROM_SDR_BASE = 0x0100000; // stored 64-bit reordered
static const uint32_t BG_B_ROM_SDR_BASE   = 0x0200000;
static const uint32_t CPU_RAM_SDR_BASE    = 0x0400000; // work RAM, not loaded
static const uint32_t BG_A_ROM_SDR_BASE   = 0x1000000;

// rom_loader stream region indices (order of LOAD_REGIONS in rtl/m72_pkg.sv)
enum RomRegion : uint8_t
{
    REGION_CPU_ROM = 0,
    REGION_SPRITE,
    REGION_BG_A,
    REGION_BG_B,
    REGION_MCU,
    REGION_SAMPLES,
    REGION_OFFSETS,
    REGION_PROTECT,
    REGION_SOUND,
};

Game GameFind(const char *name);
const char *GameName(Game game);
const char *GameLoadedShortName();

bool GameInit(Game game);
bool GameInitMra(const char *mraPath);

// DIP switch defaults for the loaded game ({dip_sw[1], dip_sw[0]}, active-high
// as presented to sim_top's dipswitch port)
uint16_t GameDipDefaults();

#endif // GAMES_H
